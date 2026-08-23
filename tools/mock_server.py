#!/usr/bin/env python3
"""接收 UploadWorker 回传数据的 mock 服务端(纯标准库), 用于验证上传闭环。

用法: python3 mock_server.py [port] [inbox_dir]
  port      默认 8080
  inbox_dir 默认 upload_inbox

POST /upload?event=<name>   接收 multipart 事件目录, 存入 inbox/<event>/
POST /upload?event=x&fail=1 模拟服务端故障(返回 500), 用于验证重试
GET  /health                健康检查

每个收到的文件会打印相对路径、字节数与 md5, 便于与本地记录比对。
"""

import hashlib
import http.server
import json
import os
import re
import sys
import urllib.parse


PORT = int(sys.argv[1]) if len(sys.argv) > 1 else 8080
INBOX = sys.argv[2] if len(sys.argv) > 2 else "upload_inbox"


def parse_multipart(body: bytes, boundary: bytes):
    """按 boundary 切分 multipart, 返回 [(name, filename|None, content)]"""
    parts = []
    for segment in body.split(b"--" + boundary):
        segment = segment.strip(b"\r\n")
        if not segment or segment == b"--":
            continue
        if b"\r\n\r\n" not in segment:
            continue
        raw_headers, content = segment.split(b"\r\n\r\n", 1)
        headers = raw_headers.decode("utf-8", errors="replace")
        name_match = re.search(r'name="([^"]*)"', headers)
        file_match = re.search(r'filename="([^"]*)"', headers)
        parts.append((
            name_match.group(1) if name_match else "",
            file_match.group(1) if file_match else None,
            content,
        ))
    return parts


class Handler(http.server.BaseHTTPRequestHandler):
    protocol_version = "HTTP/1.1"

    def log_message(self, fmt, *args):
        print("[%s] %s" % (self.log_date_time_string(), fmt % args), flush=True)

    def _respond(self, code, payload):
        body = json.dumps(payload, ensure_ascii=False).encode()
        self.send_response(code)
        self.send_header("Content-Type", "application/json")
        self.send_header("Content-Length", str(len(body)))
        self.end_headers()
        self.wfile.write(body)

    def do_GET(self):
        if urllib.parse.urlparse(self.path).path == "/health":
            self._respond(200, {"status": "ok"})
        else:
            self._respond(404, {"status": "not found"})

    def do_POST(self):
        parsed = urllib.parse.urlparse(self.path)
        if parsed.path != "/upload":
            self._respond(404, {"status": "not found"})
            return
        query = urllib.parse.parse_qs(parsed.query)
        event = query.get("event", ["unknown"])[0]
        # 过滤路径穿越, 事件名只保留安全字符
        event = re.sub(r"[^A-Za-z0-9_.\-]", "_", event)

        length = int(self.headers.get("Content-Length", "0"))
        body = self.rfile.read(length)

        if query.get("fail", ["0"])[0] == "1":
            self._respond(500, {"status": "error", "reason": "simulated failure"})
            return

        boundary_match = re.search(r'boundary="?([^";]+)"?',
                                   self.headers.get("Content-Type", ""))
        if not boundary_match:
            self._respond(400, {"status": "error", "reason": "no boundary"})
            return

        target_root = os.path.join(INBOX, event)
        saved = []
        for name, filename, content in parse_multipart(
                body, boundary_match.group(1).encode()):
            if filename is None:
                continue  # 普通表单字段(如 event 名), 不落盘
            relative = os.path.normpath(filename).lstrip(".\\/")
            target = os.path.join(target_root, relative)
            os.makedirs(os.path.dirname(target), exist_ok=True)
            with open(target, "wb") as output:
                output.write(content)
            saved.append({
                "file": relative,
                "bytes": len(content),
                "md5": hashlib.md5(content).hexdigest(),
            })

        print("收到事件 %s: %d 个文件" % (event, len(saved)), flush=True)
        for item in saved:
            print("  %-50s %8d bytes  md5=%s" %
                  (item["file"], item["bytes"], item["md5"]), flush=True)
        self._respond(200, {"status": "ok", "event": event, "files": saved})


def main():
    os.makedirs(INBOX, exist_ok=True)
    server = http.server.ThreadingHTTPServer(("0.0.0.0", PORT), Handler)
    print("mock 接收端启动: 0.0.0.0:%d, 落盘目录: %s" % (PORT, INBOX), flush=True)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        print("\n退出", flush=True)


if __name__ == "__main__":
    main()
