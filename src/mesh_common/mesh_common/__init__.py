"""mesh_common: names, layout, Zenoh keys and the packet format shared by EVERY package.

Import from here instead of copying names into your own code, e.g.
    from mesh_common import UNITS, key_tx, encode, decode
"""
import json
import os
import time
import zlib

# ------------------------------------------------------------------ units
DRONES = ["drone_1", "drone_2", "drone_3", "drone_4"]
UNITS = ["rover"] + DRONES + ["base"]                         # mesh names
GZ_NAMES = {                                                  # mesh name -> Gazebo model name
    "rover": "primary_rover", "base": "base_station",
    **{d: d for d in DRONES},
}
WORLD = "mars_world"
COMMANDER = "base"            # sends commands to the rover
RECEIVER = "base"             # receives the rover's odometry and photos (same unit: there is one base)

# ------------------------------------------------------------------ layout (x, y)
# Small demo scale. Same numbers as mars_sim/scripts/build_relay_world.py - change both together.
START_XY = {
    "base": (-40.0, 0.0), "rover": (-32.0, 0.0),
    "drone_1": (-48.0, 3.0), "drone_2": (-48.0, -3.0),
    "drone_3": (-51.0, 3.0), "drone_4": (-51.0, -3.0),
}
CRATER_RADIUS = 18.0          # rim radius (generate_crater.py RIM_RADIUS)
DRONE_HOVER = 3.0             # parked drones hover this high above the ground
TARGET_XY = (30.0, 0.0)       # science target, past the far crater rim (marker in build_relay_world.py)
TARGET_REACH = 2.0            # the rover counts as arrived this many metres before the marker

# ------------------------------------------------------------------ movement (demo scale)
ROVER_SPEED = 1.0             # m/s, mission default (whole trip ~70 s)
RELAY_HEIGHTS = (3.0, 6.0, 10.0, 15.0)   # m above ground the planner tries for relay drones
DRONE_SPEED = 15.0            # m/s drones glide at (20 steps a second, mesh_protocol/mover.py)
DRONE_ACCEL = 10.0            # m/s^2: drones speed up / slow down gently (full speed after 1.5 s)


def flight_time(distance):
    """Seconds a drone needs to fly `distance` m, speeding up and slowing down at DRONE_ACCEL."""
    ramp = DRONE_SPEED ** 2 / DRONE_ACCEL                    # metres spent speeding up + slowing down
    if distance >= ramp:
        return distance / DRONE_SPEED + DRONE_SPEED / DRONE_ACCEL
    return 2 * (distance / DRONE_ACCEL) ** 0.5               # short hop: never reaches full speed

# ------------------------------------------------------------------ radio
RADIO_RANGE = 20.0            # metres: further than this = no link (base <-> target is 70 m)
LOS_CLEARANCE = 0.5           # line must pass this far above the ground
# antenna height above each model's origin, from the real models:
# rover antenna ball 1.27 m, base station dish 10.3 m / beacon 10.8 m, drone antenna under the body
ANTENNA_HEIGHT = {"rover": 1.3, "base": 10.5}


def antenna_height(unit):
    """Height of a unit's antenna above its model origin (drones: 0 m)."""
    return ANTENNA_HEIGHT.get(unit, 0.0)


# ------------------------------------------------------------------ zenoh keys
def key_tx(u):    return f"mesh/{u}/tx"      # unit -> radio channel
def key_air(u):   return f"mesh/{u}/air"     # radio channel -> failure injector
def key_rx(u):    return f"mesh/{u}/rx"      # failure injector -> unit
def key_stats(u): return f"mesh/stats/{u}"   # unit counters -> GUI


KEY_LINKS = "mesh/links"                       # link table (link model -> channel, GUI)
KEY_POSITIONS = "mesh/positions"               # true x, y, z (+ yaw) of every unit
KEY_CMD = "mesh/base/cmd"                      # mission / GUI -> base: velocity command
KEY_ACKS = "mesh/base/acks"                    # base feed: rover confirmed a command
KEY_ODOM = "mesh/base/telemetry"               # base feed: rover odometry
KEY_COORD_STATE = "mesh/coordinator/state"     # coordinator: mode, relay on/off, drone plan
KEY_COORD_CMD = "mesh/coordinator/cmd"         # mesh_ctl -> coordinator: on / off / deploy
KEY_MISSION = "mesh/mission/status"            # mission autopilot progress
KEY_FAIL_CONTROL = "mesh/failure/control"
KEY_FAIL_EVENTS = "mesh/failure/events"
KEY_FAKE_POSE = "mesh/fake/pose"               # fake mode only: teleport a unit
KEY_FAKE_CMD_VEL = "mesh/fake/cmd_vel"         # fake mode only: rover wheel command
KEY_EVENTS = "mesh/events"                     # every node -> dashboard: one notable event each
KEY_PHOTO = "mesh/base/photo"                  # base -> dashboard: photo progress / image
KEY_PHOTO_CMD = "mesh/base/photo_cmd"          # dashboard -> base: take a photo

# All our programs meet at one local "hub" address (the link model). Only 127.0.0.1,
# multicast off: we never talk to teammates' laptops on hostel Wi-Fi, and IPv4 avoids
# the "Address family not supported" error on machines without IPv6.
HUB = os.environ.get("MESH_HUB", "tcp/127.0.0.1:7447")


def zenoh_config(hub=False):
    """Zenoh settings. hub=True for the one program that listens on HUB (the link model)."""
    import zenoh
    c = zenoh.Config()
    c.insert_json5("mode", '"peer"')
    c.insert_json5("scouting/multicast/enabled", "false")
    if hub:
        c.insert_json5("listen/endpoints", json.dumps([HUB]))
    else:
        c.insert_json5("listen/endpoints", '["tcp/127.0.0.1:0"]')
        c.insert_json5("connect/endpoints", json.dumps([HUB]))
    return c


def open_session(hub=False):
    """Open a Zenoh session with our settings."""
    import zenoh
    return zenoh.open(zenoh_config(hub))


# ------------------------------------------------------------------ packets
# v3 = DTN "bundles": every packet has a lifetime and a priority; routed ones go hop by hop
# with custody transfer (see docs/V3_CHANGES.md).
TYPES = ("HELLO", "CUSTODY_ACK", "COMMAND", "ACK", "TELEMETRY", "PHOTO_REQ", "PHOTO_CHUNK",
         "STATUS", "LEADER", "MESH_CMD")
FLOODED = ("STATUS", "LEADER", "MESH_CMD")     # control: flooded with duplicate filter, no custody
ROUTED = ("COMMAND", "ACK", "TELEMETRY", "PHOTO_REQ", "PHOTO_CHUNK")   # custody transfer
BROADCAST = "*"
REQUIRED = ("src", "dst", "next_hop", "seq", "ttl", "path", "type", "sent_time", "payload",
            "lifetime", "priority")
# seconds a bundle lives after sent_time. COMMAND < the rover's 2 s watchdog: never act on old orders
LIFETIME = {"HELLO": 2, "CUSTODY_ACK": 2, "COMMAND": 1.0, "ACK": 2, "TELEMETRY": 120, "PHOTO_REQ": 10,
            "PHOTO_CHUNK": 300, "STATUS": 2, "LEADER": 2, "MESH_CMD": 10}
PRIORITY = {t: 0 for t in TYPES}               # 0 expedited, 1 normal, 2 bulk
PRIORITY.update(TELEMETRY=1, PHOTO_CHUNK=2)
SEND_RATE = 40          # bundles a node may send per second (makes priority matter)
STORE_LIMIT = 500       # bundles a node can hold
RTO_START = 0.3         # s before the first resend; doubles each try
RTO_MAX = 2.0           # s, longest wait between resends
LEASE = 2.0             # s a leader's heartbeat stays valid
LATE_AFTER = 1.0        # s: odometry older than this on arrival is marked late


def make_packet(src, dst, seq, ptype, payload, ttl=8, next_hop=BROADCAST):
    """Build a new bundle (dict). It gets its crc32 when encoded."""
    return {"src": src, "dst": dst, "next_hop": next_hop, "seq": seq, "ttl": ttl,
            "path": [src], "type": ptype, "sent_time": time.time(), "payload": payload,
            "lifetime": LIFETIME[ptype], "priority": PRIORITY[ptype]}


def bundle_id(pkt):
    """A bundle's identity: (src, type, seq)."""
    return (pkt["src"], pkt["type"], pkt["seq"])


def expired(pkt, now):
    return now > pkt["sent_time"] + pkt["lifetime"]


def _crc(pkt):
    """CRC32 of the packet without its crc32 field (keys sorted, so it is repeatable)."""
    body = {k: v for k, v in pkt.items() if k != "crc32"}
    return zlib.crc32(json.dumps(body, sort_keys=True, separators=(",", ":")).encode())


def encode(pkt):
    """dict -> bytes ready to send (adds crc32)."""
    p = dict(pkt)
    p["crc32"] = _crc(p)
    return json.dumps(p, separators=(",", ":")).encode()


def decode(data):
    """bytes -> (packet, None) if good, or (None, reason) if broken. Never raises."""
    try:
        pkt = json.loads(bytes(data).decode("utf-8"))
    except Exception:
        return None, "unparseable"          # bit flips usually break the JSON itself
    if not isinstance(pkt, dict) or "crc32" not in pkt:
        return None, "bad_format"
    crc = pkt.pop("crc32")
    try:
        if crc != _crc(pkt):
            return None, "bad_crc"
    except Exception:
        return None, "bad_format"
    if any(k not in pkt for k in REQUIRED):
        return None, "bad_format"
    path, seq = pkt["path"], pkt["seq"]
    if not all(isinstance(pkt[k], str) for k in ("src", "dst", "next_hop")) \
            or not isinstance(seq, int) or isinstance(seq, bool) or pkt["priority"] not in (0, 1, 2) \
            or not isinstance(path, list) or not path or not all(isinstance(u, str) for u in path):
        return None, "bad_format"
    if pkt["type"] not in TYPES \
            or not isinstance(pkt["path"], list) or not isinstance(pkt["ttl"], int) \
            or not isinstance(pkt["payload"], dict) or not isinstance(pkt["priority"], int) \
            or not isinstance(pkt["lifetime"], (int, float)) or not isinstance(pkt["sent_time"], (int, float)):
        return None, "bad_format"
    return pkt, None
