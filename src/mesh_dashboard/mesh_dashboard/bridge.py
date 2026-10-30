"""Bridge: the ONLY place the dashboard talks to the mesh (Zenoh). The web server calls
snapshot() / events() to draw, and set_mesh / set_failure / take_photo for the controls.
"""
import io
import json
import threading
import time
from collections import deque

from mesh_common import (KEY_COORD_STATE, KEY_COORD_CMD, KEY_ACKS, KEY_ODOM, KEY_MISSION, KEY_LINKS,
                         KEY_POSITIONS, KEY_EVENTS, KEY_FAIL_CONTROL, KEY_PHOTO, KEY_PHOTO_CMD, KEY_PHONE_PHOTO, KEY_FEED, TARGET_XY,
                         open_session)

TYPES_SHOWN = ("COMMAND", "TELEMETRY", "PHOTO_CHUNK")
ODOM_KEEP = 600          # odometry samples kept for the track (5 min at 2/s)
EVENTS_KEEP = 500


def delivery(stats):
    """Per bundle type: sent / delivered / expired summed over every node's counters."""
    out = {t: {"sent": 0, "delivered": 0, "expired": 0} for t in TYPES_SHOWN}
    for st in stats.values():
        for t, c in (st.get("by_type") or {}).items():
            if t in out:
                for k in out[t]:
                    out[t][k] += c.get(k, 0)
    return out


def terrain_profile():
    """Ground height along the route (y = 0), one point per metre, or [] if the terrain can't load."""
    try:
        from mesh_link_model.terrain import Terrain
        t = Terrain.load()
        return [[x, round(t.ground(x, 0.0), 2)] for x in range(-56, 57)]
    except Exception as e:
        print(f"[dashboard] no terrain profile: {e}", flush=True)
        return []


class Bridge:
    def __init__(self, session=None):
        self._lock = threading.Lock()
        self._started = time.time()
        self._d = {"coord": None, "ack": None, "ack_t": 0.0, "odom": None, "odom_t": 0.0,
                   "mission": None, "links": [], "positions": {}, "stats": {}, "photo": None, "feed": None,
                   "failure": {"drop_probability": 0.0, "corrupt_probability": 0.0}}
        self._odom = deque(maxlen=ODOM_KEEP)
        self._events, self._n_events = deque(maxlen=EVENTS_KEEP), 0
        self._terrain = terrain_profile()
        self._s = session or open_session()
        S = self._s
        self._subs = [S.declare_subscriber(KEY_COORD_STATE, self._set("coord")),
                      S.declare_subscriber(KEY_ACKS, self._set("ack")),
                      S.declare_subscriber(KEY_ODOM, self._on_odom),
                      S.declare_subscriber(KEY_MISSION, self._set("mission")),
                      S.declare_subscriber(KEY_LINKS, self._on_links),
                      S.declare_subscriber(KEY_POSITIONS, self._on_pos),
                      S.declare_subscriber(KEY_PHOTO, self._set("photo")),
                      S.declare_subscriber(KEY_FEED, self._set("feed")),
                      S.declare_subscriber(KEY_EVENTS, self._on_event),
                      S.declare_subscriber("mesh/stats/*", self._on_stats)]

    # ---------------------------------------------------------- incoming
    def _set(self, key):
        def cb(sample):
            try:
                m = json.loads(sample.payload.to_bytes())
            except ValueError:
                return
            with self._lock:
                self._d[key], self._d[key + "_t"] = m, time.time()
        return cb

    @staticmethod
    def _json(sample):
        try:
            return json.loads(sample.payload.to_bytes())
        except ValueError:
            return None

    def _on_odom(self, sample):
        m = self._json(sample)
        if not isinstance(m, dict) or "seq" not in m:
            return
        truth = (m.get("payload") or {}).get("truth") or {}
        odom = (m.get("payload") or {}).get("odom") or {}
        x = (truth.get("pos") or [odom.get("x")])[0]
        with self._lock:
            self._d["odom"], self._d["odom_t"] = m, time.time()
            if x is not None:
                self._odom.append({"seq": m["seq"], "t": m.get("sent_time", m["time"]), "x": x,
                                   "late": bool(m.get("late"))})

    def _on_links(self, sample):
        m = self._json(sample)
        if isinstance(m, dict) and "links" in m:
            with self._lock:
                self._d["links"] = m["links"]

    def _on_pos(self, sample):
        m = self._json(sample)
        if isinstance(m, dict) and "positions" in m:
            with self._lock:
                self._d["positions"] = m["positions"]

    def _on_stats(self, sample):
        unit = str(sample.key_expr).split("/")[-1]
        m = self._json(sample)
        if isinstance(m, dict):
            with self._lock:
                self._d["stats"][unit] = m

    def _on_event(self, sample):
        m = self._json(sample)
        if isinstance(m, dict):
            with self._lock:
                self._events.append(m)
                self._n_events += 1

    # ---------------------------------------------------------- for the server
    def snapshot(self):
        now = time.time()
        with self._lock:
            d = dict(self._d)
            stats = {u: dict(s) for u, s in d["stats"].items()}
            odom = sorted(self._odom, key=lambda o: o["t"])     # late samples go back in place
        return {
            "now": now, "started": self._started,
            "coord": d["coord"], "positions": d["positions"], "links": d["links"], "stats": stats,
            "last_ack": d["ack"], "ack_age": now - d["ack_t"] if d["ack"] else None,
            "last_odom": d["odom"], "odom_age": now - d["odom_t"] if d["odom"] else None,
            "odom": odom, "delivery": delivery(stats), "mission": d["mission"], "photo": d["photo"], "feed": d["feed"],
            "failure": d["failure"], "terrain": self._terrain, "target": list(TARGET_XY),
        }

    def events(self, after):
        """New events since index `after`: (list, new index)."""
        with self._lock:
            n = self._n_events
            new = list(self._events)[-(n - after):] if n > after else []
        return new[-EVENTS_KEEP:], n

    def set_mesh(self, on):
        self._s.put(KEY_COORD_CMD, "on" if on else "off")

    def set_failure(self, cfg):
        with self._lock:
            self._d["failure"].update({k: v for k, v in cfg.items() if k.endswith("probability")})
        self._s.put(KEY_FAIL_CONTROL, json.dumps(cfg))

    def phone_photo(self, data):
        """A phone picture (any size) -> small JPEG -> the rover. ValueError if it is not an image."""
        from PIL import Image, ImageOps, UnidentifiedImageError
        try:
            img = ImageOps.exif_transpose(Image.open(io.BytesIO(data))).convert("RGB")
        except (UnidentifiedImageError, OSError, Image.DecompressionBombError) as e:
            raise ValueError("not an image") from e
        img.thumbnail((480, 480))                   # ~30 KB: about 30 chunks over the mesh
        out = io.BytesIO()
        img.save(out, "JPEG", quality=70)
        self._s.put(KEY_PHONE_PHOTO, out.getvalue())

    def take_photo(self):
        self._s.put(KEY_PHOTO_CMD, json.dumps({"time": time.time()}))

    def close(self):
        self._s.close()
