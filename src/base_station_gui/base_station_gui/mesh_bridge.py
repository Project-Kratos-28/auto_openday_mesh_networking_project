"""MeshBridge: the ONLY place the GUI talks to the mesh. No Qt in here.

The GUI calls snapshot() a few times a second to draw, and set_mesh(True/False) when the
toggle is clicked. Keep this file if you rebuild the GUI with other widgets.

    bridge = MeshBridge()
    bridge.set_mesh(True)          # same as: ros2 run mesh_protocol mesh_ctl on
    s = bridge.snapshot()          # dict with everything below
    bridge.close()
"""
import json
import threading
import time

from mesh_common import (KEY_COORD_STATE, KEY_COORD_CMD, KEY_ACKS_A, KEY_ODOM_B, KEY_MISSION,
                         KEY_LINKS, KEY_POSITIONS, open_session)

CONNECTED_AGE = 3.0         # s: newer data than this = "connected"


class MeshBridge:
    def __init__(self):
        self._lock = threading.Lock()
        self._d = {"coord": None, "coord_t": 0.0, "ack": None, "ack_t": 0.0, "acks": 0,
                   "odom": None, "odom_t": 0.0, "odoms": 0, "odom_times": [],
                   "mission": None, "links": [], "positions": {}}
        self._s = open_session()
        self._subs = [self._s.declare_subscriber(KEY_COORD_STATE, self._on("coord")),
                      self._s.declare_subscriber(KEY_ACKS_A, self._on("ack", "acks")),
                      self._s.declare_subscriber(KEY_ODOM_B, self._on("odom", "odoms")),
                      self._s.declare_subscriber(KEY_MISSION, self._on("mission")),
                      self._s.declare_subscriber(KEY_LINKS, self._on_links),
                      self._s.declare_subscriber(KEY_POSITIONS, self._on_pos)]

    # ---------------------------------------------------------- incoming data
    def _on(self, key, count=None):
        def cb(sample):
            m = json.loads(sample.payload.to_bytes())
            now = time.time()
            with self._lock:
                self._d[key], self._d[key + "_t"] = m, now
                if count:
                    self._d[count] += 1
                if key == "odom":
                    self._d["odom_times"] = [t for t in self._d["odom_times"] if now - t < 600] + [now]
        return cb

    def _on_links(self, sample):
        with self._lock:
            self._d["links"] = json.loads(sample.payload.to_bytes())["links"]

    def _on_pos(self, sample):
        with self._lock:
            self._d["positions"] = json.loads(sample.payload.to_bytes())["positions"]

    # ---------------------------------------------------------- for the GUI
    def set_mesh(self, on):
        """The toggle: switch the mesh network ON (True) or OFF (False)."""
        self._s.put(KEY_COORD_CMD, "on" if on else "off")

    def snapshot(self):
        """Everything the GUI needs, as plain Python values."""
        now = time.time()
        with self._lock:
            d = {k: (list(v) if isinstance(v, list) else dict(v) if isinstance(v, dict) else v)
                 for k, v in self._d.items()}
        co = d["coord"] if now - d["coord_t"] < 2.0 else None
        since = co.get("on_since") if co else None
        ts = [t for t in d["odom_times"] if since and t >= since]
        return {
            "coordinator_running": co is not None,
            "mesh_mode": co["mode"] if co else None,               # "on" | "off" | "auto" | None
            "mesh_state": co["state"] if co else None,             # "STANDBY" | "DEPLOYED"
            "mesh_reason": co["reason"] if co else "",
            "chains": co["chains"] if co else {},
            "drone_targets": co["targets"] if co else {},
            "base_a_connected": now - d["ack_t"] < CONNECTED_AGE,
            "base_a_last_ack": d["ack"],                           # rtt_ms, cmd_route, ack_route
            "base_a_acks": d["acks"],
            "base_b_receiving": now - d["odom_t"] < CONNECTED_AGE,
            "base_b_last_odom": d["odom"],                         # route, delay_ms, payload
            "base_b_msgs": d["odoms"],
            "odom_longest_gap_since_on": max((b - a for a, b in zip(ts, ts[1:])), default=None),
            "mission": d["mission"],
            "links": d["links"],
            "positions": d["positions"],
        }

    def close(self):
        self._s.close()
