"""Mission-control web dashboard: a small stdlib HTTP server between Zenoh and the browser.

    ros2 run mesh_dashboard dashboard            ->  open http://localhost:8080

GET  /                 the page (static/index.html, offline: no external URLs)
GET  /events           Server-Sent Events: "snapshot" 4x a second, "log" for each mesh event
POST /api/mesh         {"on": true|false}             mesh ON/OFF
POST /api/failure      {"drop_probability": 0.1, "corrupt_probability": 0.2, "kill": "drone_2", "revive": ...}
POST /api/photo        {}                             rover takes a photo, sends it to the base
Listens on 127.0.0.1 only by default: the controls have no login.
"""
import argparse
import json
import mimetypes
import os
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from mesh_common import UNITS

STATIC = os.path.join(os.path.dirname(os.path.abspath(__file__)), "static")
PERIOD = 0.25          # s between snapshots
MAX_BODY = 64 * 1024   # bytes


def check_failure(cfg):
    """Validated failure settings, or None if anything is wrong."""
    if not isinstance(cfg, dict) or not cfg or ("kill" in cfg and "revive" in cfg):
        return None                                 # (one of kill / revive per request)
    for k, v in cfg.items():
        if k in ("drop_probability", "corrupt_probability"):
            if isinstance(v, bool) or not isinstance(v, (int, float)) or not 0 <= v <= 1:
                return None
        elif k in ("kill", "revive"):
            if v not in UNITS:
                return None
        else:
            return None
    return cfg


def make_server(bridge, host="127.0.0.1", port=8080):
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *a):          # keep the terminal quiet
            pass

        def _reply(self, code, body=b"", ctype="application/json"):
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            path = self.path.split("?")[0]
            if path == "/events":
                return self._events()
            name = "index.html" if path == "/" else path[len("/static/"):] if path.startswith("/static/") else None
            full = os.path.realpath(os.path.join(STATIC, name)) if name else None
            if not full or os.path.dirname(full) != os.path.realpath(STATIC) or not os.path.isfile(full):
                return self._reply(404, b"not found", "text/plain")
            with open(full, "rb") as f:
                self._reply(200, f.read(), mimetypes.guess_type(full)[0] or "application/octet-stream")

        def _events(self):
            self.send_response(200)
            self.send_header("Content-Type", "text/event-stream")
            self.send_header("Cache-Control", "no-store")
            self.end_headers()
            seen = 0
            try:
                while True:
                    log, seen = bridge.events(seen)
                    msg = "".join(f"event: log\ndata: {json.dumps(e)}\n\n" for e in log)
                    msg = f"event: snapshot\ndata: {json.dumps(bridge.snapshot())}\n\n" + msg
                    self.wfile.write(msg.encode())
                    self.wfile.flush()
                    time.sleep(PERIOD)
            except (BrokenPipeError, ConnectionResetError, ValueError):
                pass                                # browser went away

        def do_POST(self):
            # JSON only: a cross-site page can send text/plain without a CORS preflight, never application/json
            if (self.headers.get("Content-Type") or "").split(";")[0].strip() != "application/json":
                return self._reply(415, b'{"error": "send application/json"}')
            try:
                n = int(self.headers.get("Content-Length") or 0)
            except ValueError:
                n = -1
            if not 0 <= n <= MAX_BODY:
                return self._reply(413, b'{"error": "body too large"}')
            try:
                body = json.loads(self.rfile.read(n) or b"{}")
            except (ValueError, RecursionError):
                return self._reply(400, b'{"error": "bad json"}')
            if self.path.split("?")[0] == "/api/mesh" and isinstance(body, dict) and isinstance(body.get("on"), bool):
                bridge.set_mesh(body["on"])
            elif self.path.split("?")[0] == "/api/failure" and check_failure(body) is not None:
                bridge.set_failure(body)
            elif self.path.split("?")[0] == "/api/photo":
                bridge.take_photo()
            else:
                return self._reply(400, b'{"error": "bad request"}')
            self._reply(200, b'{"ok": true}')

    srv = ThreadingHTTPServer((host, port), Handler)
    srv.daemon_threads = True
    return srv


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--port", type=int, default=8080)
    args, _ = ap.parse_known_args()
    from mesh_dashboard.bridge import Bridge
    bridge = Bridge()
    srv = make_server(bridge, args.host, args.port)
    print(f"[dashboard] open http://{'localhost' if args.host == '127.0.0.1' else args.host}:{args.port}",
          flush=True)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        bridge.close()


if __name__ == "__main__":
    main()
