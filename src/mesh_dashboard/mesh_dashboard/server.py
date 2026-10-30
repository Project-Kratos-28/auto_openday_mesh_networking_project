"""Mission-control web dashboard: a small stdlib HTTP server between Zenoh and the browser.

    ros2 run mesh_dashboard dashboard            ->  open http://localhost:8080

GET  /                 the page (static/index.html, offline: no external URLs)
GET  /events           Server-Sent Events: "snapshot" 4x a second, "log" for each mesh event
POST /api/mesh         {"on": true|false}             mesh ON/OFF
POST /api/failure      {"drop_probability": 0.1, "corrupt_probability": 0.2, "kill": "drone_2", "revive": ...}
POST /api/photo        {}                             rover takes a photo, sends it to the base

Watching is public; the three POST routes need the header `X-Passcode`. Listens on 127.0.0.1
by default. `--host 0.0.0.0` shares the page on the LAN, and `--passcode ""` turns the check off
for a loopback-only demo. The passcode travels in clear text: it keeps visitors out, it is not
real authentication (put TLS in front if the network is untrusted).
"""
import argparse
import hmac
import json
import mimetypes
import socket
import threading
import os
import time
from http.server import BaseHTTPRequestHandler, ThreadingHTTPServer

from mesh_common import UNITS, PHONE_PORT

STATIC = os.path.join(os.path.dirname(os.path.abspath(__file__)), "static")
PERIOD = 0.25          # s between snapshots
MAX_BODY = 64 * 1024   # bytes
MAX_UPLOAD = 15 * 1024 * 1024
PASSCODE = os.environ.get("MESH_PASSCODE", "Kr4tos-Rover-Mesh!28")   # --passcode or MESH_PASSCODE overrides; "" disables


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


def phone_url():
    """Where a phone should go. PHONE_URL overrides (use the address the phone can reach)."""
    if os.environ.get("PHONE_URL"):
        return os.environ["PHONE_URL"]
    with socket.socket(socket.AF_INET, socket.SOCK_DGRAM) as sock:
        try:
            sock.connect(("10.255.255.255", 1))     # no packet is sent; it only picks our LAN address
            ip = sock.getsockname()[0]
        except OSError:
            ip = "127.0.0.1"
    return f"http://{ip}:{PHONE_PORT}/phone"


def make_phone_server(bridge, port=PHONE_PORT):
    """The only thing exposed to the network: the upload page and one POST. Nothing controls the rover."""
    class Handler(BaseHTTPRequestHandler):
        def log_message(self, *a):
            pass

        def _reply(self, code, body=b"", ctype="text/plain"):
            self.send_response(code)
            self.send_header("Content-Type", ctype)
            self.send_header("Content-Length", str(len(body)))
            self.end_headers()
            self.wfile.write(body)

        def do_GET(self):
            if self.path.split("?")[0] != "/phone":
                return self._reply(404, b"not found")
            with open(os.path.join(STATIC, "phone.html"), "rb") as f:
                self._reply(200, f.read(), "text/html")

        def do_POST(self):
            try:
                n = int(self.headers.get("Content-Length") or 0)
            except ValueError:
                n = -1
            if self.path.split("?")[0] != "/upload" or not 0 < n <= MAX_UPLOAD:
                return self._reply(400, b"bad request")
            try:
                bridge.phone_photo(self.rfile.read(n))
            except ValueError:
                return self._reply(415, b"not an image")
            self._reply(200, b"sent")

    srv = ThreadingHTTPServer(("0.0.0.0", port), Handler)
    srv.daemon_threads = True
    return srv


def make_server(bridge, host="127.0.0.1", port=8080, passcode=PASSCODE):
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
            if path == "/phone_url":
                return self._reply(200, phone_url().encode(), "text/plain")
            if path == "/qr.svg":
                try:
                    import io
                    import segno
                    out = io.BytesIO()
                    segno.make(phone_url()).save(out, kind="svg", scale=8, border=2, dark="#000", light="#fff")
                    return self._reply(200, out.getvalue(), "image/svg+xml")
                except ImportError:
                    return self._reply(404, b"pip install segno", "text/plain")
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
            # JSON only: a cross-site page can send text/plain without a CORS preflight, never application/json.
            # The passcode header does the same job: a custom header also forces a preflight.
            if (self.headers.get("Content-Type") or "").split(";")[0].strip() != "application/json":
                return self._reply(415, b'{"error": "send application/json"}')
            supplied = self.headers.get("X-Passcode") or ""
            if passcode and not hmac.compare_digest(supplied.encode(), passcode.encode()):
                time.sleep(1)                       # a wrong guess costs a second: brute force is slow
                return self._reply(401, b'{"error": "passcode required"}')
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
    ap.add_argument("--passcode", default=PASSCODE,
                    help='controls passcode (default: $MESH_PASSCODE or the built-in one, "" disables the check)')
    args, _ = ap.parse_known_args()
    from mesh_dashboard.bridge import Bridge
    bridge = Bridge()
    srv = make_server(bridge, args.host, args.port, args.passcode)
    threading.Thread(target=make_phone_server(bridge).serve_forever, daemon=True).start()
    print(f"[dashboard] open http://{'localhost' if args.host == '127.0.0.1' else args.host}:{args.port}",
          flush=True)
    print(f"[dashboard] controls {'locked (passcode required)' if args.passcode else 'unlocked'}", flush=True)
    try:
        srv.serve_forever()
    except KeyboardInterrupt:
        pass
    finally:
        bridge.close()


if __name__ == "__main__":
    main()
