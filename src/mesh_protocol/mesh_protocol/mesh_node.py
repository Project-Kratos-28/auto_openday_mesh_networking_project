"""Mesh protocol node, v3 (DTN). Run ONE copy per unit:
    python3 -m mesh_protocol.mesh_node --unit drone_1

Roles
  base_a (commander): turns mesh/base_a/cmd into COMMAND bundles for the rover, pings the rover
                      every second when idle, publishes every ACK on mesh/base_a/acks
  rover:              applies COMMANDs (Gazebo /rover/cmd_vel), answers each with an ACK,
                      stops by itself if no drive command for 2 s (safety watchdog),
                      sends its odometry as TELEMETRY to base_b twice a second
  base_b (receiver):  publishes arriving odometry on mesh/base_b/telemetry
  drones:             relays
DTN rules (docs/V3_CHANGES.md):
  routed bundles (COMMAND, ACK, TELEMETRY, ...) go hop by hop with CUSTODY TRANSFER: a node keeps
  a bundle in its store and resends it (0.3 s, doubling to 2 s) until the next hop answers with a
  CUSTODY_ACK. No route = the bundle waits. Every bundle has a lifetime; expired ones are dropped
  (a COMMAND lives 1 s, so the rover never acts on old orders). Best priority is sent first, at
  most SEND_RATE bundles per second. Next hop = the route table (mesh/coordinator/state
  "routes"), or the destination itself if there is no route and it is a neighbour.
  crc32 + "can't parse" check -> broken packets are dropped, the node never crashes.
"""
import argparse
import json
import threading
import time
from collections import OrderedDict

from mesh_common import (UNITS, BROADCAST, COMMANDER, RECEIVER, ROUTED, SEND_RATE, LATE_AFTER,
                         KEY_POSITIONS, KEY_CMD_A, KEY_ACKS_A, KEY_ODOM_B, KEY_COORD_STATE,
                         KEY_FAKE_CMD_VEL, KEY_EVENTS, KEY_FAIL_CONTROL, key_tx, key_rx, key_stats,
                         make_packet, encode, decode, expired, bundle_id, open_session)
from mesh_protocol.store import BundleStore

NEIGHBOUR_TIMEOUT = 1.5        # seconds without HELLO -> neighbour gone
HELLO_PERIOD = 0.5
SEEN_MEMORY = 5000             # bundle ids remembered by the duplicate filter
WATCHDOG = 2.0                 # rover stops if no drive command for this long
PING_PERIOD = 1.0              # base_a pings the rover this often when idle


class MeshNode:
    def __init__(self, unit, ttl=8, fake=False, telemetry_period=0.5, session=None):
        self.unit, self.ttl, self.fake = unit, ttl, fake
        self.is_drone = unit.startswith("drone")
        self.telemetry_period = telemetry_period
        self.lock = threading.Lock()
        self.seq = {}                                   # bundle type -> last sequence number
        self.seen = OrderedDict()                       # bundle id -> time first seen
        self.neighbours = {}                            # unit -> time of last HELLO
        self.store = BundleStore()
        self.routes = {}                                # dst -> next hop (from the coordinator)
        self.killed = False                             # failure injection: node silent
        self.budget, self.last_tick = float(SEND_RATE), time.time()
        self.stats = dict(sent=0, received=0, forwarded=0, delivered=0, duplicates=0, bad_crc=0,
                          ttl_expired=0, custody_sent=0, custody_acked=0, retransmits=0, expired=0,
                          dropped_full=0)
        self.stats["by_type"] = {t: {"sent": 0, "delivered": 0, "expired": 0} for t in ROUTED}
        self.truth = None                               # rover: true pose from the link model
        self.odom = None                                # rover: wheel odometry from Gazebo
        self.last_drive = 0.0                           # rover: time of last drive command
        self.moving = False
        self.last_cmd_sent = 0.0                        # base_a
        self.session = session or open_session()
        s = self.session
        self.pub_tx = s.declare_publisher(key_tx(unit))
        self.pub_stats = s.declare_publisher(key_stats(unit))
        self.pub_events = s.declare_publisher(KEY_EVENTS)
        self.subs = [s.declare_subscriber(key_rx(unit), self.on_rx),
                     s.declare_subscriber(KEY_COORD_STATE, self.on_coord),
                     s.declare_subscriber(KEY_FAIL_CONTROL, self.on_failure)]
        if unit == "rover":
            self.subs.append(s.declare_subscriber(KEY_POSITIONS, self.on_positions))
            self._setup_gazebo()
        if unit == COMMANDER:
            self.pub_acks = s.declare_publisher(KEY_ACKS_A)
            self.subs.append(s.declare_subscriber(KEY_CMD_A, self.on_cmd_request))
        if unit == RECEIVER:
            self.pub_odom = s.declare_publisher(KEY_ODOM_B)
        role = {"rover": "rover", COMMANDER: "commander base", RECEIVER: "receiver base"}.get(unit, "relay drone")
        self.log(f"started as {role}")

    # -------------------------------------------------------------- helpers
    def log(self, text):
        print(f"[{self.unit}] {text}", flush=True)

    def event(self, name, **kw):
        self.pub_events.put(json.dumps({"time": time.time(), "unit": self.unit, "event": name, **kw}))

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
        """Duplicate filter. Returns True if this bundle id is new."""
        if key in self.seen:
            return False
        self.seen[key] = time.time()
        if len(self.seen) > SEEN_MEMORY:
            self.seen.popitem(last=False)
        return True

    def _alive(self, now):
        return {u for u, t in self.neighbours.items() if now - t < NEIGHBOUR_TIMEOUT}

    def next_hop(self, dst, now=None):
        """The coordinator's route; without one, the destination itself if we can hear it; else None (hold).

        The route table wins: it is rebuilt twice a second, while a neighbour stays "alive" for up to
        NEIGHBOUR_TIMEOUT after its last HELLO, even if the link just broke.
        """
        nh = self.routes.get(dst)
        if nh is None and dst in self._alive(now or time.time()):
            return dst
        return nh

    def _take(self, pkt, now):
        """Take custody of a bundle (own or forwarded)."""
        dropped = self.store.add(pkt, now)
        if dropped is not None:
            self.stats["dropped_full"] += 1
            self.event("dropped_full", type=dropped["type"], src=dropped["src"], seq=dropped["seq"])

    def _send(self, pkts):
        for p in pkts:
            self.pub_tx.put(encode(p))

    # -------------------------------------------------------------- sending
    def originate(self, dst, ptype, payload, ttl=None):
        """Create a brand-new bundle from this unit. Routed ones go into the store first."""
        with self.lock:
            self.seq[ptype] = self.seq.get(ptype, 0) + 1
            pkt = make_packet(self.unit, dst, self.seq[ptype], ptype, payload, ttl or self.ttl)
            self.stats["sent"] += 1
            if ptype in ROUTED:
                self.stats["by_type"][ptype]["sent"] += 1
                self._remember(bundle_id(pkt))
                self._take(pkt, time.time())
                return pkt
        if not self.killed:
            self._send([pkt])
        return pkt

    def tick(self, now):
        """Drop expired bundles, then (re)send what is due, within the send budget."""
        if self.killed:
            return []
        out = []
        with self.lock:
            for p in self.store.expire(now):
                self.stats["expired"] += 1
                self.stats["by_type"][p["type"]]["expired"] += 1
                self.event("expired", type=p["type"], src=p["src"], seq=p["seq"], where="store")
            self.budget = min(SEND_RATE, self.budget + SEND_RATE * max(0.0, now - self.last_tick))
            self.last_tick = now
            for p, nh in self.store.due(now, lambda d: self.next_hop(d, now), int(self.budget)):
                if self.store.tries(bundle_id(p)) > 1:
                    self.stats["retransmits"] += 1
                    self.event("retransmit", type=p["type"], src=p["src"], seq=p["seq"], to=nh)
                out.append(dict(p, next_hop=nh))
                self.budget -= 1
        return out

    # -------------------------------------------------------------- receiving
    def on_rx(self, sample):
        if self.killed:
            return
        pkt, err = decode(sample.payload.to_bytes())
        if err:                                        # corrupted -> drop quietly, count it
            with self.lock:
                self.stats["bad_crc"] += 1
            return
        self._send(self.handle(pkt, time.time()))

    def handle(self, pkt, now):
        """Apply the receive rules to one good bundle. Returns the bundles to send right away."""
        if self.killed:
            return []
        deliver = None
        with self.lock:
            self.stats["received"] += 1
            t, src = pkt["type"], pkt["src"]
            if t == "HELLO":
                if src not in self._alive(now):
                    self.store.kick(src)                # link back: retry its bundles now
                self.neighbours[src] = now
                return []
            if pkt["next_hop"] != self.unit:
                return []                               # unicast for someone else
            if t == "CUSTODY_ACK":
                if self.store.ack(tuple(pkt["payload"].get("id", ()))):
                    self.stats["custody_acked"] += 1
                return []
            if t not in ROUTED:
                return []
            # routed bundle for us as next hop: always confirm custody, even for duplicates
            ack = make_packet(self.unit, pkt["path"][-1], 0, "CUSTODY_ACK",
                              {"id": list(bundle_id(pkt))}, ttl=1, next_hop=pkt["path"][-1])
            self.stats["custody_sent"] += 1
            if expired(pkt, now):
                self.stats["expired"] += 1
                self.stats["by_type"][t]["expired"] += 1
                self.event("expired", type=t, src=src, seq=pkt["seq"], where="rx")
            elif not self._remember(bundle_id(pkt)):
                self.stats["duplicates"] += 1
            elif pkt["dst"] == self.unit:
                self.stats["delivered"] += 1
                self.stats["by_type"][t]["delivered"] += 1
                deliver = pkt
            elif pkt["ttl"] <= 1 or self.unit in pkt["path"]:
                self.stats["ttl_expired"] += 1
            else:
                self._take(dict(pkt, ttl=pkt["ttl"] - 1, path=pkt["path"] + [self.unit]), now)
                self.stats["forwarded"] += 1
        if deliver is not None:
            self.deliver(deliver, now)
        return [ack]

    def deliver(self, pkt, now):
        """A bundle addressed to us arrived."""
        route = pkt["path"] + [self.unit]
        delay_ms = round((now - pkt["sent_time"]) * 1000, 1)
        p = pkt["payload"]
        if self.unit == "rover" and pkt["type"] == "COMMAND":
            if "linear" in p or "angular" in p:
                lin, ang = float(p.get("linear", 0.0)), float(p.get("angular", 0.0))
                self.drive(lin, ang)
                self.last_drive, self.moving = now, (lin != 0.0 or ang != 0.0)
            self.originate(pkt["src"], "ACK", {"ack_seq": pkt["seq"], "cmd_sent_time": pkt["sent_time"],
                                               "cmd_route": route, "moving": self.moving,
                                               "cmd_age_ms": delay_ms})
        elif self.unit == COMMANDER and pkt["type"] == "ACK":
            rtt = round((now - p.get("cmd_sent_time", now)) * 1000, 1)
            self.pub_acks.put(json.dumps({"time": now, "ack_seq": p.get("ack_seq"), "rtt_ms": rtt,
                                          "cmd_route": p.get("cmd_route"), "ack_route": route,
                                          "moving": p.get("moving"), "cmd_age_ms": p.get("cmd_age_ms")}))
        elif self.unit == RECEIVER and pkt["type"] == "TELEMETRY":
            late = delay_ms > LATE_AFTER * 1000
            self.pub_odom.put(json.dumps({"time": now, "src": pkt["src"], "seq": pkt["seq"],
                                          "sent_time": pkt["sent_time"], "route": route,
                                          "hops": len(route) - 1, "delay_ms": delay_ms, "late": late,
                                          "payload": p}))
            if pkt["seq"] % 10 == 0 or late and pkt["seq"] % 5 == 0:
                self.log(f"odometry #{pkt['seq']} via {' -> '.join(route)} ({delay_ms} ms)")
        else:
            self.log(f"{pkt['type']} #{pkt['seq']} from {pkt['src']} via {' -> '.join(route)}")

    # -------------------------------------------------------------- other inputs
    def on_coord(self, sample):
        """Route table from the coordinator."""
        try:
            routes = json.loads(sample.payload.to_bytes()).get("routes", {}).get(self.unit, {})
        except Exception:
            return
        with self.lock:
            changed = routes != self.routes
            self.routes = routes
            if changed:
                self.store.kick(None)
        if changed:
            self.event("route_change", routes=routes)

    def on_failure(self, sample):
        try:
            m = json.loads(sample.payload.to_bytes())
        except Exception:
            return
        if m.get("kill") == self.unit and not self.killed:
            self.killed = True
            self.log("KILLED (failure injection): silent until revived")
            self.event("killed")
        elif m.get("revive") == self.unit and self.killed:
            self.killed = False
            self.log("revived")
            self.event("revived")

    def on_positions(self, sample):
        m = json.loads(sample.payload.to_bytes())
        if "rover" in m["positions"]:
            self.truth = {"pos": m["positions"]["rover"], "yaw": m.get("yaw", {}).get("rover")}

    def on_cmd_request(self, sample):
        """base_a: a velocity command from the mission / GUI -> COMMAND bundle."""
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
                    next_hello = now + HELLO_PERIOD
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
                self._send(self.tick(now))
                if now >= next_stats:
                    with self.lock:
                        nb = sorted(self._alive(now))
                        stats = dict(self.stats, held=self.store.held(), neighbours=nb, routes=self.routes,
                                     killed=self.killed, leader=None, term=None, time=now)
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
    ap.add_argument("--ttl", type=int, default=8, help="max hops a bundle may travel")
    ap.add_argument("--fake", action="store_true", help="fake rover (no Gazebo)")
    args, _ = ap.parse_known_args()
    MeshNode(args.unit, ttl=args.ttl, fake=args.fake).run()


if __name__ == "__main__":
    main()
