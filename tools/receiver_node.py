#!/usr/bin/env python3
"""ROS2 RPC receiver for completed DataCache event directories.

Usage (after sourcing ROS2 and this package's install space):
  ros2 run datacache upload_receiver_node [inbox_dir] [service_name]

The UploadStore transaction is BEGIN -> FILE_CHUNK* -> END. Each event is
written to a private staging directory and published under inbox/<event>/ only
after END validates the declared file and byte counts. Repeating BEGIN for the
same event resets only its staging transaction, so a failed retry does not
destroy the last complete copy.
"""

from dataclasses import dataclass, field
import hashlib
import os
from pathlib import Path
import re
import shutil
import sys
import time

import rclpy
from rclpy.node import Node

from datacache.srv import UploadStore


EVENT_NAME = re.compile(r"[A-Za-z0-9][A-Za-z0-9_.-]*\Z")
MAX_EVENT_NAME = 160
MAX_RELATIVE_PATH = 1024
MAX_FILES = 10000
MAX_TOTAL_BYTES = 100 * 1024 * 1024 * 1024
MAX_ACTIVE_TRANSFERS = 16
TRANSFER_IDLE_SECONDS = 600
MAX_MANIFEST_BYTES = 16 * 1024 * 1024
WINDOWS_DEVICE_NAMES = {
    "CON", "PRN", "AUX", "NUL", "COM1", "COM2", "COM3", "COM4", "COM5",
    "COM6", "COM7", "COM8", "COM9", "LPT1", "LPT2", "LPT3", "LPT4", "LPT5",
    "LPT6", "LPT7", "LPT8", "LPT9",
}


def validate_event_name(name: str):
    """Return a safe event directory name, or None when the input is unsafe."""
    return name if (len(name) <= MAX_EVENT_NAME and EVENT_NAME.fullmatch(name)
                    and name not in {".", ".."}) else None


def safe_relative(path: str):
    """Return normalized path components, rejecting absolute/traversal paths."""
    if len(path) > MAX_RELATIVE_PATH:
        return None
    normalized = path.replace("\\", "/")
    if (not normalized or "\x00" in normalized or normalized.startswith("/")
            or re.match(r"^[A-Za-z]:", normalized)):
        return None
    parts = normalized.split("/")
    if any(part in {"", ".", ".."} or ":" in part or part.endswith((".", " "))
           or part.split(".", 1)[0].upper() in WINDOWS_DEVICE_NAMES
           for part in parts):
        return None
    return parts


@dataclass
class FileState:
    expected_bytes: int
    received_bytes: int = 0
    hasher: object = field(default_factory=hashlib.sha256)


@dataclass
class TransferState:
    event: str
    transfer_id: str
    staging_dir: Path
    final_dir: Path
    expected_files: int
    expected_bytes: int
    files: dict = field(default_factory=dict)
    canonical_paths: dict = field(default_factory=dict)
    last_activity: float = field(default_factory=time.monotonic)


class ReceiverNode(Node):

    def __init__(self, inbox: str, service_name: str):
        super().__init__("upload_receiver")
        self.inbox = Path(inbox).resolve()
        self.inbox.mkdir(parents=True, exist_ok=True)
        self.transfers = {}
        self.recover_backups()
        self.service = self.create_service(
            UploadStore, service_name, self.handle_request)
        self.get_logger().info(
            "RPC receiver started: service=%s, inbox=%s" % (service_name, self.inbox))

    def handle_request(self, request, response):
        try:
            response.success, response.message, response.received_bytes = self.dispatch(request)
        except Exception as error:  # noqa: BLE001 - keep a bad request from killing the service
            response.success = False
            response.message = "receiver error: %s" % error
            response.received_bytes = 0
            self.get_logger().error("request failed: %s" % error)
        return response

    def dispatch(self, request):
        self.prune_idle_transfers()
        event = validate_event_name(request.event_name)
        transfer_id = validate_event_name(request.transfer_id)
        if event is None or transfer_id is None:
            return False, "unsafe event name or transfer_id", 0

        if request.opcode == UploadStore.Request.BEGIN:
            return self.begin(event, transfer_id, request.file_count, request.total_bytes)

        key = (event, transfer_id)
        state = self.transfers.get(key)
        if state is None:
            return False, "FILE_CHUNK/END before BEGIN for this event", 0

        if request.opcode == UploadStore.Request.FILE_CHUNK:
            state.last_activity = time.monotonic()
            return self.store_chunk(state, request)
        if request.opcode == UploadStore.Request.END:
            state.last_activity = time.monotonic()
            return self.finish(state)
        return False, "unknown opcode %d" % request.opcode, 0

    def begin(self, event: str, transfer_id: str, expected_files: int, expected_bytes: int):
        if expected_files <= 0 or expected_files > MAX_FILES:
            return False, "file count exceeds receiver limit", 0
        if expected_bytes < 0 or expected_bytes > MAX_TOTAL_BYTES:
            return False, "byte count exceeds receiver limit", 0
        if len(event) + len(transfer_id) + len("...part") > 240:
            return False, "event and transfer_id are too long", 0
        key = (event, transfer_id)
        if key not in self.transfers and len(self.transfers) >= MAX_ACTIVE_TRANSFERS:
            return False, "too many active transfers", 0
        if any(state.event == event and state.transfer_id != transfer_id
               for state in self.transfers.values()):
            return False, "another transfer already owns this event", 0
        staging = self.inbox / (".%s.%s.part" % (event, transfer_id))
        final = self.inbox / event
        shutil.rmtree(staging, ignore_errors=True)
        staging.mkdir(parents=True)
        self.transfers[key] = TransferState(
            event, transfer_id, staging, final, expected_files, expected_bytes)
        self.get_logger().info(
            "receiving %s (%d files, %d bytes)"
            % (event, expected_files, expected_bytes))
        return True, "transfer started", 0

    def store_chunk(self, state: TransferState, request):
        parts = safe_relative(request.file_path)
        if parts is None:
            return False, "unsafe file path: %r" % request.file_path, 0

        relative = "/".join(parts)
        canonical = relative.casefold()
        existing_path = state.canonical_paths.get(canonical)
        if existing_path is not None and existing_path != relative:
            return False, "case-insensitive path collision for %s" % relative, 0
        state.canonical_paths[canonical] = relative
        file_state = state.files.get(relative)
        if file_state is None:
            file_state = FileState(request.total_bytes)
            state.files[relative] = file_state
        elif file_state.expected_bytes != request.total_bytes:
            return False, "file size changed for %s" % relative, file_state.received_bytes

        data = bytes(request.data)
        offset = request.offset
        end = offset + len(data)
        if end > file_state.expected_bytes:
            return False, "chunk exceeds declared size for %s" % relative, file_state.received_bytes

        target = state.staging_dir.joinpath(*parts)
        target.parent.mkdir(parents=True, exist_ok=True)

        if offset < file_state.received_bytes:
            # A request may complete at the server while its response times out at the sender.
            # Accept an exact duplicate without counting its bytes twice.
            if end > file_state.received_bytes or not target.exists():
                return False, "overlapping chunk for %s" % relative, file_state.received_bytes
            with target.open("rb") as source:
                source.seek(offset)
                if source.read(len(data)) != data:
                    return False, "conflicting duplicate chunk for %s" % relative, file_state.received_bytes
            return True, "duplicate chunk already stored", file_state.received_bytes

        if offset != file_state.received_bytes:
            return False, "non-contiguous offset for %s: got %d, expected %d" % (
                relative, offset, file_state.received_bytes), file_state.received_bytes

        mode = "r+b" if target.exists() else "wb"
        with target.open(mode) as output:
            output.seek(offset)
            output.write(data)
        file_state.hasher.update(data)
        file_state.received_bytes = end
        return True, "chunk stored", file_state.received_bytes

    def finish(self, state: TransferState):
        received_bytes = sum(item.received_bytes for item in state.files.values())
        if len(state.files) != state.expected_files:
            return False, "file count mismatch: got %d, expected %d" % (
                len(state.files), state.expected_files), received_bytes
        if received_bytes != state.expected_bytes:
            return False, "byte count mismatch: got %d, expected %d" % (
                received_bytes, state.expected_bytes), received_bytes

        for relative, item in state.files.items():
            path = state.staging_dir.joinpath(*relative.split("/"))
            actual = path.stat().st_size if path.exists() else -1
            if item.received_bytes != item.expected_bytes or actual != item.expected_bytes:
                return False, "incomplete file %s: got %d, expected %d" % (
                    relative, actual, item.expected_bytes), received_bytes
            # Windows requires a writable descriptor for fsync; the receiver owns
            # every staging file, so opening it read/write is safe here.
            with path.open("r+b") as handle:
                os.fsync(handle.fileno())

        checksum_error = self.verify_checksums(state)
        if checksum_error:
            return False, checksum_error, received_bytes
        self.sync_directory(state.staging_dir)

        backup = self.inbox / (".%s.backup" % state.event)
        shutil.rmtree(backup, ignore_errors=True)
        self.sync_directory(self.inbox)
        if state.final_dir.exists():
            os.replace(state.final_dir, backup)
        try:
            os.replace(state.staging_dir, state.final_dir)
        except Exception:
            if backup.exists() and not state.final_dir.exists():
                os.replace(backup, state.final_dir)
            raise
        shutil.rmtree(backup, ignore_errors=True)
        self.sync_directory(self.inbox)
        self.transfers.pop((state.event, state.transfer_id), None)
        self.get_logger().info(
            "received event %s: %d files, %d bytes"
            % (state.event, len(state.files), received_bytes))
        return True, "transfer complete", received_bytes

    @staticmethod
    def sync_directory(directory: Path):
        if os.name == "nt":
            return
        descriptor = os.open(directory, os.O_RDONLY | getattr(os, "O_DIRECTORY", 0))
        try:
            os.fsync(descriptor)
        finally:
            os.close(descriptor)

    @staticmethod
    def verify_checksums(state: TransferState):
        manifest_path = state.staging_dir / "manifest.sha256"
        if not manifest_path.is_file():
            return "manifest.sha256 is required"
        if manifest_path.stat().st_size > MAX_MANIFEST_BYTES:
            return "manifest.sha256 exceeds receiver limit"
        expected = {}
        for line_number, line in enumerate(
                manifest_path.read_text(encoding="utf-8").splitlines(), start=1):
            if "  " not in line:
                return "invalid manifest.sha256 line %d" % line_number
            digest, relative = line.split("  ", 1)
            parts = safe_relative(relative)
            if (not re.fullmatch(r"[0-9a-f]{64}", digest)
                    or parts is None or relative in expected):
                return "invalid manifest.sha256 line %d" % line_number
            expected[relative] = digest

        if not expected:
            return "manifest.sha256 contains no data files"

        received = set(state.files) - {"manifest.sha256"}
        if set(expected) != received:
            return "checksum manifest file set mismatch"
        for relative, wanted in expected.items():
            if state.files[relative].hasher.hexdigest() != wanted:
                return "sha256 mismatch for %s" % relative
        return None

    def recover_backups(self):
        for backup in self.inbox.glob(".*.backup"):
            event = backup.name[1:-len(".backup")]
            if validate_event_name(event) is None:
                continue
            final = self.inbox / event
            if final.exists():
                shutil.rmtree(backup, ignore_errors=True)
            else:
                os.replace(backup, final)

    def prune_idle_transfers(self):
        cutoff = time.monotonic() - TRANSFER_IDLE_SECONDS
        for key, state in list(self.transfers.items()):
            if state.last_activity >= cutoff:
                continue
            shutil.rmtree(state.staging_dir, ignore_errors=True)
            self.transfers.pop(key, None)
            self.get_logger().warning("discarded idle transfer %s" % state.event)

def main():
    inbox = sys.argv[1] if len(sys.argv) > 1 else "upload_inbox"
    service_name = sys.argv[2] if len(sys.argv) > 2 else "/upload_store"

    rclpy.init()
    node = ReceiverNode(inbox, service_name)
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
        print("receiver stopped", flush=True)


if __name__ == "__main__":
    main()
