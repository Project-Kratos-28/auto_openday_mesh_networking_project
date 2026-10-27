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
UNITS = ["rover"] + DRONES + ["base_a", "base_b"]             # mesh names
GZ_NAMES = {                                                  # mesh name -> Gazebo model name
    "rover": "primary_rover", "base_a": "base_station", "base_b": "base_station_2",
    **{d: d for d in DRONES},
}
WORLD = "mars_world"
COMMANDER = "base_a"          # sends velocity commands to the rover
RECEIVER = "base_b"           # receives the rover's odometry

# ------------------------------------------------------------------ layout (x, y)
# Small demo scale. Same numbers as mars_sim/scripts/build_relay_world.py - change both together.
START_XY = {
    "base_a": (-40.0, 0.0), "base_b": (40.0, 0.0), "rover": (-32.0, 0.0),
    "drone_1": (-48.0, 3.0), "drone_2": (-48.0, -3.0),
    "drone_3": (-51.0, 3.0), "drone_4": (-51.0, -3.0),
}
CRATER_RADIUS = 18.0          # rim radius (generate_crater.py RIM_RADIUS)
DRONE_HOVER = 3.0             # parked drones hover this high above the ground
GOAL_X = 33.0                 # mission ends when the rover reaches this x (in front of base B's pad)

# ------------------------------------------------------------------ movement (demo scale)
ROVER_SPEED = 1.0             # m/s, mission default (whole trip ~70 s)
RELAY_HEIGHTS = (3.0, 6.0, 10.0, 15.0)   # m above ground the planner tries for relay drones
REPLAN_DISTANCE = 8.0         # m the rover may move before drones are re-placed anyway

# ------------------------------------------------------------------ radio
RADIO_RANGE = 32.0            # metres: further than this = no link (base A <-> base B is 80 m)
LOS_CLEARANCE = 0.5           # line must pass this far above the ground
# antenna height above each model's origin, from the real models:
# rover antenna ball 1.27 m, base station dish 10.3 m / beacon 10.8 m, drone antenna under the body
ANTENNA_HEIGHT = {"rover": 1.3, "base_a": 10.5, "base_b": 10.5}


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
KEY_CMD_A = "mesh/base_a/cmd"                  # mission / GUI -> base A: velocity command
KEY_ACKS_A = "mesh/base_a/acks"                # base A feed: rover confirmed a command
KEY_ODOM_B = "mesh/base_b/telemetry"           # base B feed: rover odometry
KEY_COORD_STATE = "mesh/coordinator/state"     # coordinator: mode, relay on/off, drone plan
KEY_COORD_CMD = "mesh/coordinator/cmd"         # mesh_ctl -> coordinator: on / off / deploy
KEY_MISSION = "mesh/mission/status"            # mission autopilot progress
KEY_FAIL_CONTROL = "mesh/failure/control"
KEY_FAIL_EVENTS = "mesh/failure/events"
KEY_FAKE_POSE = "mesh/fake/pose"               # fake mode only: teleport a unit
KEY_FAKE_CMD_VEL = "mesh/fake/cmd_vel"         # fake mode only: rover wheel command

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
TYPES = ("TELEMETRY", "COMMAND", "ACK", "HELLO")
BROADCAST = "*"
REQUIRED = ("src", "dst", "next_hop", "seq", "ttl", "path", "type", "sent_time", "payload")


def make_packet(src, dst, seq, ptype, payload, ttl=8):
    """Build a new packet (dict). It gets its crc32 when encoded."""
    return {"src": src, "dst": dst, "next_hop": BROADCAST, "seq": seq, "ttl": ttl,
            "path": [src], "type": ptype, "sent_time": time.time(), "payload": payload}


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
    if any(k not in pkt for k in REQUIRED) or pkt["type"] not in TYPES \
            or not isinstance(pkt["path"], list) or not isinstance(pkt["ttl"], int) \
            or not isinstance(pkt["payload"], dict):
        return None, "bad_format"
    return pkt, None
