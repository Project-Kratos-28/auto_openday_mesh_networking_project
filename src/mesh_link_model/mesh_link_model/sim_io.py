"""Teleport a unit: in Gazebo (set_pose service) or in fake mode (Zenoh message).

Gazebo: calls the set_pose service through the Gazebo Python bindings (takes milliseconds).
If the bindings are missing it falls back to the `gz service` command (slower, ~0.3 s).
"""
import json
import math
import subprocess

from mesh_common import GZ_NAMES, WORLD, KEY_FAKE_POSE

_gz = {}          # cached Gazebo node + message classes (made on first use)


def _gz_request(name, x, y, z, qz, qw):
    """set_pose via the Python bindings. Returns True/False, or None if bindings are missing."""
    try:
        if not _gz:
            from gz.transport13 import Node
            from gz.msgs10.pose_pb2 import Pose
            from gz.msgs10.boolean_pb2 import Boolean
            _gz.update(node=Node(), Pose=Pose, Boolean=Boolean)
    except Exception:
        return None
    req = _gz["Pose"]()
    req.name = name
    req.position.x, req.position.y, req.position.z = x, y, z
    req.orientation.z, req.orientation.w = qz, qw
    try:
        ok, rep = _gz["node"].request(f"/world/{WORLD}/set_pose", req, _gz["Pose"], _gz["Boolean"], 1000)
        return bool(ok and rep.data)
    except Exception:
        return None


def fast_available():
    """True if Gazebo's Python bindings are installed (teleports take milliseconds)."""
    try:
        import gz.transport13  # noqa: F401
        import gz.msgs10  # noqa: F401
        return True
    except Exception:
        return False


def teleport(unit, x, y, z, yaw=0.0, fake=False, session=None):
    """Move `unit` to (x, y, z) facing `yaw` (radians). Returns True if it worked."""
    if fake:
        session.put(KEY_FAKE_POSE, json.dumps({"unit": unit, "pos": [x, y, z], "yaw": yaw}))
        return True
    qz, qw = math.sin(yaw / 2), math.cos(yaw / 2)
    ok = _gz_request(GZ_NAMES[unit], float(x), float(y), float(z), qz, qw)
    if ok is not None:
        return ok
    req = (f'name: "{GZ_NAMES[unit]}", position: {{x: {x}, y: {y}, z: {z}}}, '
           f'orientation: {{z: {qz:.5f}, w: {qw:.5f}}}')
    cmd = ["gz", "service", "-s", f"/world/{WORLD}/set_pose", "--reqtype", "gz.msgs.Pose",
           "--reptype", "gz.msgs.Boolean", "--timeout", "2000", "--req", req]
    try:
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=5)
        return "true" in r.stdout
    except Exception:
        return False
