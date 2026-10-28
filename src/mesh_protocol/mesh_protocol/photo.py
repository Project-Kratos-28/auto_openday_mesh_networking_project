"""Photo transfer (like NASA's CFDP file delivery, much simplified): the rover cuts a file into
1 KB chunks that travel as bulk-priority bundles with custody transfer; base B puts them back
together and checks the whole file's sha256. Pure logic (test/test_photo.py).
"""
import base64
import hashlib
import os


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


def chunks(data, photo_id, size=1024):
    """bytes -> list of PHOTO_CHUNK payloads."""
    total = max(1, -(-len(data) // size))
    digest = hashlib.sha256(data).hexdigest()
    return [{"photo_id": photo_id, "index": i, "total": total, "sha256": digest,
             "data": base64.b64encode(data[i * size:(i + 1) * size]).decode()} for i in range(total)]


class Reassembler:
    def __init__(self):
        self.photo_id, self.parts, self.result = None, {}, None

    def add(self, p):
        """Add one chunk payload. Returns progress; when done, also ok (sha256 match) and the image."""
        if p.get("photo_id") != self.photo_id:
            self.photo_id, self.parts, self.result = p.get("photo_id"), {}, None
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
        return out
