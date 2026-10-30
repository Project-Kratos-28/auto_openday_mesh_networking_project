"""Photo transfer (like NASA's CFDP file delivery, much simplified): the rover cuts a file into
1 KB chunks that travel as bulk-priority bundles with custody transfer; the base puts them back
together and checks the whole file's sha256. Pure logic (test/test_photo.py).
"""
import base64
import hashlib
import io
import os
import threading
import time

from mesh_common import CHUNK_BYTES


def photo_bytes():
    """The rover's 'camera': a fixed picture shipped with the package (installed share, else source)."""
    paths = [os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "resource", "photo.png")]
    try:
        from ament_index_python.packages import get_package_share_directory
        paths.insert(0, os.path.join(get_package_share_directory("mesh_protocol"), "photo.png"))
    except Exception:
        pass
    for p in paths:
        if os.path.isfile(p):
            with open(p, "rb") as f:
                return f.read()
    raise FileNotFoundError("photo.png not found")


class Camera:
    """The rover camera: keeps the newest frame Gazebo publishes on /rover/camera."""

    def __init__(self, topic="/rover/camera"):
        self.frame, self.stamp, self.lock = None, 0.0, threading.Lock()
        from gz.transport13 import Node
        from gz.msgs10.image_pb2 import Image
        self._node = Node()
        self._node.subscribe(Image, topic, self._on_image)

    def _on_image(self, msg):
        with self.lock:
            self.frame, self.stamp = (msg.width, msg.height, bytes(msg.data)), time.time()

    def encode(self, fmt="PNG", size=None, quality=90):
        """The newest frame as image bytes, or None if there is none from the last 2 s."""
        with self.lock:
            if self.frame is None or time.time() - self.stamp > 2.0:
                return None
            w, h, raw = self.frame
        from PIL import Image as PILImage
        img = PILImage.frombytes("RGB", (w, h), raw)
        out = io.BytesIO()
        (img.resize(size) if size else img).save(out, fmt, quality=quality)
        return out.getvalue()


def chunks(data, photo_id, size=CHUNK_BYTES):
    """bytes -> list of PHOTO_CHUNK payloads."""
    total = max(1, -(-len(data) // size))
    digest = hashlib.sha256(data).hexdigest()
    return [{"photo_id": photo_id, "index": i, "total": total, "sha256": digest,
             "data": base64.b64encode(data[i * size:(i + 1) * size]).decode()} for i in range(total)]


class Reassembler:
    def __init__(self, partial=False):
        """partial=True: unfinished progress also carries `partial_b64`, the gap-free start of the file (a
        baseline JPEG cut short still shows its top part)."""
        self.partial = partial
        self._start(None)

    def _start(self, photo_id):
        self.photo_id, self.parts, self.result = photo_id, {}, None
        self._prefix, self._next, self._prefix_b64 = bytearray(), 0, None      # gap-free start of the file

    def _grow_prefix(self):
        """Extend the gap-free start with the chunks that now follow it (each chunk is decoded once)."""
        grown = False
        while self._next in self.parts:
            self._prefix += base64.b64decode(self.parts[self._next])
            self._next += 1
            grown = True
        if grown:
            self._prefix_b64 = base64.b64encode(bytes(self._prefix)).decode()
        return self._prefix_b64

    def add(self, p):
        """Add one chunk payload. Returns progress; when done, also ok (sha256 match) and the image."""
        pid = p.get("photo_id")
        if self.photo_id is not None and pid < self.photo_id:
            return self.result or {"photo_id": self.photo_id, "received": len(self.parts),
                                   "total": p["total"], "done": False}   # late chunk of an older photo
        if pid != self.photo_id:
            self._start(pid)
        if self.result is not None:
            return self.result
        self.parts[p["index"]] = p["data"]
        out = {"photo_id": self.photo_id, "received": len(self.parts), "total": p["total"], "done": False}
        if len(self.parts) == p["total"]:
            try:
                data = b"".join(base64.b64decode(self.parts[i]) for i in range(p["total"]))
            except (KeyError, ValueError):
                data = b""
            ok = hashlib.sha256(data).hexdigest() == p["sha256"]
            out.update(done=True, ok=ok, image_b64=base64.b64encode(data).decode() if ok else None)
            self.result = out
        elif self.partial:
            out["partial_b64"] = self._grow_prefix()
        return out
