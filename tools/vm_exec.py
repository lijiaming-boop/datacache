#!/usr/bin/env python3
"""宿主机侧的虚拟机操作助手。

用法:
  python tools/vm_exec.py check                 检查连通性与 ROS2 环境
  python tools/vm_exec.py sync                   同步项目源码到虚拟机 ~/datacache
  python tools/vm_exec.py run "<command>"        在虚拟机内执行命令(默认 bash -lc)
  python tools/vm_exec.py run-bg "<command>"     同上, 但以 nohup 后台方式启动
  python tools/vm_exec.py get <remote> <local>   拉取虚拟机文件
  python tools/vm_exec.py put <local> <remote>   推送单个文件

连接参数必须通过环境变量提供: VM_HOST / VM_USER / VM_PASSWORD (可选 VM_ROOT)
"""

import argparse
import os
import posixpath
import sys
import time

import paramiko

HOST = os.environ.get("VM_HOST", "")
USER = os.environ.get("VM_USER", "")
PASSWORD = os.environ.get("VM_PASSWORD", "")
REMOTE_ROOT = os.environ.get("VM_ROOT", "datacache")

# 同步时跳过的目录: 构建产物与本地产物不进虚拟机
EXCLUDE_DIRS = {".git", ".vscode", "__pycache__", "build", "install", "log",
                "records", "upload_inbox"}


def connect():
    client = paramiko.SSHClient()
    client.set_missing_host_key_policy(paramiko.AutoAddPolicy())
    client.connect(HOST, username=USER, password=PASSWORD, timeout=15,
                   allow_agent=False, look_for_keys=False)
    return client


def run(client, command, timeout=900):
    _, stdout, stderr = client.exec_command(command, timeout=timeout)
    out = stdout.read().decode("utf-8", errors="replace")
    err = stderr.read().decode("utf-8", errors="replace")
    code = stdout.channel.recv_exit_status()
    return code, out, err


def sync(client, local_root, remote_root):
    sftp = client.open_sftp()
    copied = 0

    def mkdirs(path):
        parts = path.split("/")
        current = ""
        for part in parts:
            current = posixpath.join(current, part)
            try:
                sftp.stat(current)
            except FileNotFoundError:
                sftp.mkdir(current)

    mkdirs(remote_root)
    for dirpath, dirnames, filenames in os.walk(local_root):
        dirnames[:] = sorted(d for d in dirnames if d not in EXCLUDE_DIRS)
        rel = os.path.relpath(dirpath, local_root)
        rdir = remote_root if rel == "." else posixpath.join(
            remote_root, rel.replace(os.sep, "/"))
        mkdirs(rdir)
        for name in sorted(filenames):
            if name.endswith(".tmp") or name == ".DS_Store":
                continue
            sftp.put(os.path.join(dirpath, name), posixpath.join(rdir, name))
            copied += 1
    sftp.close()
    return copied


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("action",
                        choices=["check", "sync", "run", "run-bg", "get", "put"])
    parser.add_argument("args", nargs="*", help="命令参数")
    parser.add_argument("--timeout", type=int, default=900)
    options = parser.parse_args()

    missing = [name for name, value in
               (("VM_HOST", HOST), ("VM_USER", USER), ("VM_PASSWORD", PASSWORD))
               if not value]
    if missing:
        sys.exit("缺少连接参数: " + ", ".join(missing) + "\n"
                 "请通过环境变量提供, 例如: VM_HOST=192.168.1.10 VM_USER=ubuntu "
                 "VM_PASSWORD=xxx python tools/vm_exec.py check")

    client = connect()
    try:
        if options.action == "check":
            code, out, err = run(client,
                'bash -lc "whoami; ls /opt/ros; nproc; df -h / | tail -1"')
            print(out or err)
            sys.exit(code)

        if options.action == "sync":
            local_root = options.args[0] if options.args else \
                os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
            count = sync(client, local_root, REMOTE_ROOT)
            print(f"同步完成: {count} 个文件 -> {USER}@{HOST}:~/{REMOTE_ROOT}")
            return

        if options.action == "run":
            command = " ".join(options.args)
            code, out, err = run(client,
                f'bash -lc {shquote(command)}', timeout=options.timeout)
            if out:
                print(out, end="")
            if err:
                print(err, end="", file=sys.stderr)
            sys.exit(code)

        if options.action == "run-bg":
            command = " ".join(options.args)
            code, out, err = run(client,
                f'bash -lc {shquote("nohup " + command + " > /tmp/vm_bg.log 2>&1 & echo started")}')
            print(out or err)
            sys.exit(code)

        if options.action == "get":
            sftp = client.open_sftp()
            sftp.get(options.args[0], options.args[1])
            sftp.close()
            print(f"已拉取 {options.args[0]} -> {options.args[1]}")
            return

        if options.action == "put":
            sftp = client.open_sftp()
            sftp.put(options.args[0], options.args[1])
            sftp.close()
            print(f"已推送 {options.args[0]} -> {options.args[1]}")
            return
    finally:
        client.close()


def shquote(text):
    return "'" + text.replace("'", "'\\''") + "'"


if __name__ == "__main__":
    main()
