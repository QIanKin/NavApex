#!/usr/bin/env python3
"""Local-only log player: python3 tools/log_viewer/main.py [recording.log ...]."""
import argparse
import json
import mimetypes
import tempfile
import threading
import webbrowser
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer
from pathlib import Path
from urllib.parse import parse_qs, urlparse

from replay_log import ReplayLog

ASSETS = Path(__file__).resolve().parent / "dist"


def make_server(port=8765, paths=()):
    model = ReplayLog()
    for path in paths:
        model.add_file(path)
    model.finalize()
    temporary = tempfile.TemporaryDirectory(prefix="navapex-log-viewer-")
    lock = threading.RLock()

    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *_):
            pass

        def respond(self, data, status=200, mime="application/json; charset=utf-8"):
            if not isinstance(data, bytes):
                data = json.dumps(data, ensure_ascii=False, allow_nan=False).encode()
            self.send_response(status)
            self.send_header("Content-Type", mime)
            self.send_header("Content-Length", str(len(data)))
            self.send_header("Cache-Control", "no-store")
            self.send_header("X-Content-Type-Options", "nosniff")
            self.end_headers()
            try:
                self.wfile.write(data)
            except (BrokenPipeError, ConnectionResetError):
                pass

        def do_GET(self):
            request = urlparse(self.path)
            args = parse_qs(request.query)
            try:
                with lock:
                    if request.path == "/api/summary":
                        return self.respond(model.summary())
                    if request.path == "/api/frame":
                        return self.respond(model.frame(args["session"][0], int(args["index"][0]), args.get("mode", ["plans"])[0]))
                    if request.path == "/api/map":
                        return self.respond(model.map_bytes(args["session"][0], args["id"][0]), mime="application/octet-stream")
                assets = {"/": "index.html", "/app.js": "app.js", "/style.css": "style.css"}
                if request.path in assets:
                    file = ASSETS / assets[request.path]
                    return self.respond(file.read_bytes(), mime=mimetypes.guess_type(file.name)[0] + "; charset=utf-8")
                self.respond({"error": "Not found"}, 404)
            except (KeyError, ValueError, OSError, TypeError) as error:
                self.respond({"error": str(error)}, 400)

        def do_POST(self):
            nonlocal model
            request = urlparse(self.path)
            args = parse_qs(request.query)
            try:
                # Files only enter through explicit user upload; no arbitrary path-reading API.
                if request.path != "/api/import":
                    return self.respond({"error": "Not found"}, 404)
                origin = self.headers.get("Origin")
                if origin and origin != "http://" + self.headers.get("Host", ""):
                    return self.respond({"error": "Only same-origin uploads are accepted"}, 403)
                length = int(self.headers.get("Content-Length", "0"))
                if length <= 0 or length > 1024 * 1024 * 1024:
                    raise ValueError("每个日志文件应在 1 B 到 1 GiB 之间")
                with lock:
                    if args.get("reset") == ["1"]:
                        model = ReplayLog()
                        for file in Path(temporary.name).iterdir():
                            file.unlink()
                    path = Path(temporary.name) / str(len(model.files))
                    with path.open("wb") as stream:
                        remaining = length
                        while remaining:
                            chunk = self.rfile.read(min(1024 * 1024, remaining))
                            if not chunk:
                                raise ValueError("上传中断")
                            stream.write(chunk)
                            remaining -= len(chunk)
                    model.add_file(path, args.get("name", ["recording.log"])[0])
                    model.finalize()
                    self.respond({"files": len(model.files)})
            except (ValueError, OSError) as error:
                self.respond({"error": str(error)}, 400)

    server = ThreadingHTTPServer(("127.0.0.1", port), Handler)
    server.temporary = temporary
    return server


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("logs", nargs="*", type=Path)
    parser.add_argument("--port", type=int, default=8765)
    parser.add_argument("--no-browser", action="store_true")
    args = parser.parse_args()
    try:
        server = make_server(args.port, args.logs)
    except (OSError, ValueError) as error:
        parser.exit(1, f"无法启动播放器: {error}\n")
    url = f"http://127.0.0.1:{server.server_port}"
    print(f"NavApex 日志播放器: {url}\n拖入 .log 和轮转文件即可回放。Ctrl+C 退出。", flush=True)
    if not args.no_browser:
        webbrowser.open(url)
    try:
        server.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        server.server_close()
        server.temporary.cleanup()


if __name__ == "__main__":
    main()
