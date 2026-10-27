"""Mesh protocol node, v2. Run ONE copy per unit:
    python3 -m mesh_protocol.mesh_node --unit drone_1

Roles
  base_a (commander): turns mesh/base_a/cmd into COMMAND packets for the rover, sends a
                      'ping' COMMAND every second when idle, and publishes every ACK the
                      rover sends back on mesh/base_a/acks  (= "is the rover reachable?")
  rover:              applies COMMANDs (Gazebo /rover/cmd_vel), answers each with an ACK,
                      stops by itself if no drive command for 2 s (safety watchdog),
                      sends its odometry as TELEMETRY to base_b twice a second
  base_b (receiver):  publishes arriving odometry on mesh/base_b/telemetry
  drones:             only relay. Relaying is OFF until the coordinator switches the mesh
                      on (mesh/coordinator/state "relay": true), or with --always-relay
Forwarding (flooding): duplicate filter (src, type, seq), ttl -1 per hop, path list,
crc32 + "can't parse" check -> broken packets are dropped, the node never crashes.
Bases and the rover never forward other units' packets: only drones are relays.
"""
import argparse
import json
import threading
import time
from collections import OrderedDict

from mesh_common import (UNITS, BROADCAST, COMMANDER, RECEIVER, KEY_POSITIONS, KEY_CMD_A,
                         KEY_ACKS_A, KEY_ODOM_B, KEY_COORD_STATE, KEY_FAKE_CMD_VEL,
                         key_tx, key_rx, key_stats, make_packet, encode, decode, open_session)

NEIGHBOUR_TIMEOUT = 3.0        # seconds without HELLO -> neighbour gone
SEEN_MEMORY = 5000             # packet ids remembered by the duplicate filter
WATCHDOG = 2.0                 # rover stops if no drive command for this long
PING_PERIOD = 1.0              # base_a pings the rover this often when idle


class MeshNode:
    def __init__(self, unit, ttl=8, fake=False, always_relay=False, telemetry_period=0.5):
        self.unit, self.ttl, self.fake = unit, ttl, fake
        self.is_drone = unit.startswith("drone")
        self.relay = always_relay or not self.is_drone   # endpoints don't relay anyway
        self.always_relay = always_relay
        self.telemetry_period = telemetry_period
        self.lock = threading.Lock()
        self.seq = {}                                   # packet type -> last sequence number
        self.seen = OrderedDict()                       # (src, type, seq) -> time first seen
        self.neighbours = {}                            # unit -> time of last HELLO
        self.stats = dict(sent=0, received=0, forwarded=0, delivered=0, duplicates=0,
                          bad_crc=0, ttl_expired=0)
        self.truth = None                               # rover: true pose from the link model
        self.odom = None                                # rover: wheel odometry from Gazebo
        self.last_drive = 0.0                           # rover: time of last drive command
        self.moving = False
        self.last_cmd_sent = 0.0                        # base_a
        self.session = open_session()
        self.pub_tx = self.session.declare_publisher(key_tx(unit))
        self.pub_stats = self.session.declare_publisher(key_stats(unit))
        self.subs = [self.session.declare_subscriber(key_rx(unit), self.on_rx)]
        if self.is_drone:
            self.subs.append(self.session.declare_subscriber(KEY_COORD_STATE, self.on_coord))
        if unit == "rover":
            self.subs.append(self.session.declare_subscriber(KEY_POSITIONS, self.on_positions))
            self._setup_gazebo()
        if unit == COMMANDER:
            self.pub_acks = self.session.declare_publisher(KEY_ACKS_A)
            self.subs.append(self.session.declare_subscriber(KEY_CMD_A, self.on_cmd_request))
        if unit == RECEIVER:
            self.pub_odom = self.session.declare_publisher(KEY_ODOM_B)
        role = {"rover": "rover", COMMANDER: "commander base", RECEIVER: "receiver base"}.get(unit, "relay drone")
        self.log(f"started as {role}" + (" (relay OFF until the mesh is activated)"
                                         if self.is_drone and not self.relay else ""))

    # -------------------------------------------------------------- helpers
    def log(self, text):
        print(f"[{self.unit}] {text}", flush=True)

    def _setup_gazebo(self):
        """Rover: publisher for /rover/cmd_vel and subscriber for /rover/odom."""
        self.gz_pub = None
        if self.fake:
            self.fake_pub = self.session.declare_publisher(KEY_FAKE_CMD_VEL)
            return
        try:
            from gz.transport13 import Node
            from gz.msgs10.twist_pb2 import Twist
            from gz.msgs10.odometry_pb2 import Odometry
            self._gz_node, self._Twist = Node(), Twist
            self.gz_pub = self._gz_node.advertise("/rover/cmd_vel", Twist)

            def on_odom(msg: Odometry):
                import math
                q = msg.pose.orientation
                yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
                self.odom = {"x": round(msg.pose.position.x, 2), "y": round(msg.pose.position.y, 2),
                             "yaw": round(yaw, 3), "v": round(msg.twist.linear.x, 2)}
            self._gz_node.subscribe(Odometry, "/rover/odom", on_odom)
        except Exception as e:
            self.log(f"(no Gazebo python bindings, commands will only be printed: {e})")

    def drive(self, linear, angular):
        """Rover: send a wheel command to Gazebo (or to the fake rover)."""
        if self.fake:
            self.fake_pub.put(json.dumps({"linear": linear, "angular": angular}))
        elif self.gz_pub is not None:
            msg = self._Twist()
            msg.linear.x, msg.angular.z = float(linear), float(angular)
            self.gz_pub.publish(msg)

    def _remember(self, key):
        """Duplicate filter. Returns True if this packet id is new."""
        if key in self.seen:
            return False
        self.seen[key] = time.time()
        if len(self.seen) > SEEN_MEMORY:
            self.seen.popitem(last=False)
        return True

    def originate(self, dst, ptype, payload, ttl=None):
        """Create and send a brand-new packet from this unit."""
        with self.lock:
            self.seq[ptype] = self.seq.get(ptype, 0) + 1
            seq = self.seq[ptype]
            pkt = make_packet(self.unit, dst, seq, ptype, payload, ttl or self.ttl)
            if ptype != "HELLO":
                self._remember((self.unit, ptype, seq))
            self.stats["sent"] += 1
        self.pub_tx.put(encode(pkt))
        return pkt

    # -------------------------------------------------------------- receiving
    def on_rx(self, sample):
        pkt, err = decode(sample.payload.to_bytes())
        fwd = None
        with self.lock:
            if err:                                    # corrupted -> drop quietly, count it
                self.stats["bad_crc"] += 1
                return
            self.stats["received"] += 1
            if pkt["type"] == "HELLO":
                self.neighbours[pkt["src"]] = time.time()
                return
            if pkt["src"] == self.unit or self.unit in pkt["path"]:
                return                                 # our own packet coming back
            if not self._remember((pkt["src"], pkt["type"], pkt["seq"])):
                self.stats["duplicates"] += 1
                return
            if pkt["dst"] == self.unit:
                self.stats["delivered"] += 1
            elif not (self.is_drone and self.relay):
                return                                 # not for us and we are not a relay
            elif pkt["ttl"] <= 1:
                self.stats["ttl_expired"] += 1
                return
            else:
                fwd = dict(pkt, ttl=pkt["ttl"] - 1, path=pkt["path"] + [self.unit], next_hop=BROADCAST)
                self.stats["forwarded"] += 1
        if fwd is not None:
            self.pub_tx.put(encode(fwd))               # re-encode = new crc32 for new ttl/path
        else:
            self.deliver(pkt)

    def deliver(self, pkt):
        """A packet addressed to us arrived."""
        route = pkt["path"] + [self.unit]
        delay_ms = round((time.time() - pkt["sent_time"]) * 1000, 1)
        p = pkt["payload"]
        if self.unit == "rover" and pkt["type"] == "COMMAND":
            if "linear" in p or "angular" in p:
                lin, ang = float(p.get("linear", 0.0)), float(p.get("angular", 0.0))
                self.drive(lin, ang)
                self.last_drive, self.moving = time.time(), (lin != 0.0 or ang != 0.0)
            self.originate(pkt["src"], "ACK", {"ack_seq": pkt["seq"], "cmd_sent_time": pkt["sent_time"],
                                               "cmd_route": route, "moving": self.moving})
        elif self.unit == COMMANDER and pkt["type"] == "ACK":
            rtt = round((time.time() - p.get("cmd_sent_time", time.time())) * 1000, 1)
            self.pub_acks.put(json.dumps({"time": time.time(), "ack_seq": p.get("ack_seq"),
                                          "rtt_ms": rtt, "cmd_route": p.get("cmd_route"),
                                          "ack_route": route, "moving": p.get("moving")}))
        elif self.unit == RECEIVER and pkt["type"] == "TELEMETRY":
            self.pub_odom.put(json.dumps({"time": time.time(), "src": pkt["src"], "seq": pkt["seq"],
                                          "route": route, "hops": len(route) - 1,
                                          "delay_ms": delay_ms, "payload": p}))
            if pkt["seq"] % 10 == 0:
                self.log(f"odometry #{pkt['seq']} via {' -> '.join(route)} ({delay_ms} ms)")
        else:
            self.log(f"{pkt['type']} #{pkt['seq']} from {pkt['src']} via {' -> '.join(route)}")

    # -------------------------------------------------------------- other inputs
    def on_coord(self, sample):
        """Drones: the coordinator tells us if the mesh (relaying) is on."""
        if self.always_relay:
            return
        on = bool(json.loads(sample.payload.to_bytes()).get("relay", False))
        if on != self.relay:
            self.relay = on
            self.log(f"relay {'ON' if on else 'OFF'}")

    def on_positions(self, sample):
        m = json.loads(sample.payload.to_bytes())
        if "rover" in m["positions"]:
            self.truth = {"pos": m["positions"]["rover"], "yaw": m.get("yaw", {}).get("rover")}

    def on_cmd_request(self, sample):
        """base_a: a velocity command from the mission / GUI -> COMMAND packet."""
        try:
            cmd = json.loads(sample.payload.to_bytes())
        except Exception:
            self.log("ignored a command that isn't JSON")
            return
        self.originate("rover", "COMMAND", cmd)
        self.last_cmd_sent = time.time()

    # -------------------------------------------------------------- main loop
    def run(self):
        next_hello = next_telem = next_stats = time.time()
        known, battery = set(), 100.0
        try:
            while True:
                now = time.time()
                if now >= next_hello:
                    self.originate(BROADCAST, "HELLO", {}, ttl=1)
                    next_hello = now + 1.0
                if self.unit == "rover":
                    if self.moving and now - self.last_drive > WATCHDOG:
                        self.drive(0.0, 0.0)
                        self.moving = False
                        self.log("no command for 2 s -> stopping (safety watchdog)")
                    if now >= next_telem:
                        battery = max(0.0, battery - 0.005)
                        self.originate(RECEIVER, "TELEMETRY",
                                       {"odom": self.odom, "truth": self.truth,
                                        "moving": self.moving, "battery": round(battery, 2)})
                        next_telem = now + self.telemetry_period
                if self.unit == COMMANDER and now - self.last_cmd_sent > PING_PERIOD:
                    self.originate("rover", "COMMAND", {"ping": True})
                    self.last_cmd_sent = now
                if now >= next_stats:
                    with self.lock:
                        nb = sorted(u for u, t in self.neighbours.items() if now - t < NEIGHBOUR_TIMEOUT)
                        stats = dict(self.stats, neighbours=nb, relay=self.relay if self.is_drone else None, time=now)
                    self.pub_stats.put(json.dumps(stats))
                    if set(nb) != known:
                        self.log(f"neighbours: {nb if nb else 'NONE'}")
                        known = set(nb)
                    next_stats = now + 1.0
                time.sleep(0.02)
        except KeyboardInterrupt:
            pass
        finally:
            if self.unit == "rover":
                self.drive(0.0, 0.0)
            self.session.close()


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--unit", required=True, choices=UNITS)
    ap.add_argument("--ttl", type=int, default=8, help="max hops a packet may travel")
    ap.add_argument("--fake", action="store_true", help="rover: drive the fake rover (no Gazebo)")
    ap.add_argument("--always-relay", action="store_true", help="drones relay without the coordinator")
    args, _ = ap.parse_known_args()
    MeshNode(args.unit, ttl=args.ttl, fake=args.fake, always_relay=args.always_relay).run()


if __name__ == "__main__":
    main()
