import importlib.util
import hashlib
from pathlib import Path
import sys

import rclpy

from datacache.srv import UploadStore


MODULE_PATH = Path(__file__).parents[1] / "tools" / "receiver_node.py"
SPEC = importlib.util.spec_from_file_location("datacache_receiver_node", MODULE_PATH)
receiver = importlib.util.module_from_spec(SPEC)
sys.modules[SPEC.name] = receiver
SPEC.loader.exec_module(receiver)


def request(opcode, event, path="", offset=0, total_bytes=0, file_count=0, data=b""):
    message = UploadStore.Request()
    message.opcode = opcode
    message.event_name = event
    message.transfer_id = "test-transfer"
    message.file_path = path
    message.offset = offset
    message.total_bytes = total_bytes
    message.file_count = file_count
    message.data = list(data)
    return message


def checksum_manifest(files):
    return "".join(
        "%s  %s\n" % (hashlib.sha256(content).hexdigest(), path)
        for path, content in sorted(files.items())).encode()


def test_transaction_handles_chunks_duplicates_empty_files_and_staging(tmp_path):
    final = tmp_path / "collision_1"
    final.mkdir()
    (final / "old.txt").write_text("last complete copy", encoding="utf-8")

    rclpy.init()
    node = receiver.ReceiverNode(str(tmp_path), "/upload_store_receiver_test")
    try:
        checksums = checksum_manifest({"a.bin": b"abc", "nested/empty.bin": b""})
        assert node.dispatch(request(UploadStore.Request.BEGIN, "collision_1",
                                     total_bytes=3 + len(checksums), file_count=3))[0]
        assert (final / "old.txt").exists()  # BEGIN only resets staging

        first = request(UploadStore.Request.FILE_CHUNK, "collision_1", "a.bin", 0, 3,
                        data=b"ab")
        assert node.dispatch(first)[0]
        assert node.dispatch(first)[0]  # exact duplicate is idempotent
        assert node.dispatch(request(UploadStore.Request.FILE_CHUNK, "collision_1", "a.bin", 2,
                                     3, data=b"c"))[0]
        assert node.dispatch(request(UploadStore.Request.FILE_CHUNK, "collision_1",
                                     "nested/empty.bin"))[0]
        assert node.dispatch(request(UploadStore.Request.FILE_CHUNK, "collision_1",
                                     "manifest.sha256", 0, len(checksums), data=checksums))[0]

        success, _, received = node.dispatch(request(UploadStore.Request.END, "collision_1"))
        assert success
        assert received == 3 + len(checksums)
        assert (final / "a.bin").read_bytes() == b"abc"
        assert (final / "nested" / "empty.bin").read_bytes() == b""
        assert not (final / "old.txt").exists()
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_rejects_unsafe_names_and_supports_interleaved_events(tmp_path):
    rclpy.init()
    node = receiver.ReceiverNode(str(tmp_path), "/upload_store_receiver_test_2")
    try:
        assert not node.dispatch(request(UploadStore.Request.BEGIN, "../escape"))[0]
        checksum_a = checksum_manifest({"a": b"a"})
        checksum_b = checksum_manifest({"b": b"b"})
        assert node.dispatch(request(UploadStore.Request.BEGIN, "event_a",
                                     total_bytes=1 + len(checksum_a), file_count=2))[0]
        request_b = request(UploadStore.Request.BEGIN, "event_b",
                            total_bytes=1 + len(checksum_b), file_count=2)
        request_b.transfer_id = "other-transfer"
        assert node.dispatch(request_b)[0]
        assert not node.dispatch(request(UploadStore.Request.FILE_CHUNK, "event_a", "../x", 0,
                                         1, data=b"x"))[0]
        assert not node.dispatch(request(UploadStore.Request.FILE_CHUNK, "event_a", "dir/a:b", 0,
                                         1, data=b"x"))[0]
        assert not node.dispatch(request(UploadStore.Request.FILE_CHUNK, "event_a", "NUL", 0, 1,
                                         data=b"x"))[0]
        assert node.dispatch(request(UploadStore.Request.FILE_CHUNK, "event_a", "a", 0, 1,
                                     data=b"a"))[0]
        assert not node.dispatch(request(UploadStore.Request.FILE_CHUNK, "event_a", "A", 0, 1,
                                         data=b"a"))[0]
        chunk_b = request(UploadStore.Request.FILE_CHUNK, "event_b", "b", 0, 1, data=b"b")
        chunk_b.transfer_id = "other-transfer"
        assert node.dispatch(chunk_b)[0]
        manifest_b = request(UploadStore.Request.FILE_CHUNK, "event_b", "manifest.sha256", 0,
                             len(checksum_b), data=checksum_b)
        manifest_b.transfer_id = "other-transfer"
        assert node.dispatch(manifest_b)[0]
        assert node.dispatch(request(UploadStore.Request.FILE_CHUNK, "event_a",
                                     "manifest.sha256", 0, len(checksum_a), data=checksum_a))[0]
        end_b = request(UploadStore.Request.END, "event_b")
        end_b.transfer_id = "other-transfer"
        assert node.dispatch(end_b)[0]
        assert node.dispatch(request(UploadStore.Request.END, "event_a"))[0]
        assert (tmp_path / "event_a" / "a").read_bytes() == b"a"
        assert (tmp_path / "event_b" / "b").read_bytes() == b"b"
    finally:
        node.destroy_node()
        rclpy.shutdown()


def test_checksum_failure_keeps_previous_complete_copy(tmp_path):
    final = tmp_path / "event_a"
    final.mkdir()
    (final / "old.txt").write_text("previous", encoding="utf-8")
    invalid_manifest = checksum_manifest({"a": b"different"})

    rclpy.init()
    node = receiver.ReceiverNode(str(tmp_path), "/upload_store_receiver_checksum_test")
    try:
        assert node.dispatch(request(UploadStore.Request.BEGIN, "event_a",
                                     total_bytes=1 + len(invalid_manifest), file_count=2))[0]
        assert node.dispatch(request(UploadStore.Request.FILE_CHUNK, "event_a", "a", 0, 1,
                                     data=b"a"))[0]
        assert node.dispatch(request(UploadStore.Request.FILE_CHUNK, "event_a",
                                     "manifest.sha256", 0, len(invalid_manifest),
                                     data=invalid_manifest))[0]

        success, message, _ = node.dispatch(request(UploadStore.Request.END, "event_a"))
        assert not success
        assert "sha256 mismatch" in message
        assert (final / "old.txt").read_text(encoding="utf-8") == "previous"
    finally:
        node.destroy_node()
        rclpy.shutdown()
