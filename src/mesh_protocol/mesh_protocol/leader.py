"""The coordinator, now inside the drones: a lease-based leader election, and the leader's job
(where the drones go, and the route table). Pure logic, no Zenoh (test/test_leader.py).

Election: the leader floods a LEADER heartbeat every 0.5 s. A drone that hears no valid one for
LEASE (2 s) + a random 0-0.5 s, and is the lowest-id drone among the drones it heard in the last
2 s, becomes leader with term + 1. A leader steps down when it hears a higher term, or the same
term from a lower id, so a split mesh that heals ends up with one leader again.
"""
import itertools
import math
import random

from mesh_common import DRONES, UNITS, START_XY, DRONE_HOVER, RADIO_RANGE, LEASE, flight_time
from mesh_link_model.planner import plan, link_ok, antenna
from mesh_protocol.routing import compute_routes

HEARD_WINDOW = 2.0        # s: drones heard this recently take part in an election
JITTER = 0.5              # s: random extra wait, so two drones rarely start at once
LOOK_AHEAD = 5.0          # s: re-place drones if a chain would break within this time (20 m radio: start early)
CHECK_PERIOD = 0.5        # s between chain checks
MOVE_SLACK = 2.0          # s on top of the flight time to wait for a drone to report it arrived
ARRIVED = 0.5             # m
ROVER_STALE = 1.0         # s: a rover report older than this is old news (the rover is out of reach)
ROVER_GUESS_MAX = 4.0     # s: guess where the silent rover is by its last speed, for at most this long


class Election:
    def __init__(self, unit, now, rng=random):
        self.unit, self.rng = unit, rng
        self.leader, self.term, self.is_leader = None, 0, False
        self.leader_seen = now                 # last valid LEADER heard from another drone
        self.heard = {}                        # drone -> time of its last STATUS
        self.jitter = rng.uniform(0, JITTER)

    @staticmethod
    def _key(leader, term):
        return (term, -DRONES.index(leader))   # higher term wins, then the lower id

    def on_status(self, src, now):
        if src in DRONES:
            self.heard[src] = now

    def on_leader(self, msg, now):
        """A LEADER heartbeat from another drone."""
        lead, term = msg.get("leader"), msg.get("term")
        if lead == self.unit or lead not in DRONES or not isinstance(term, int):
            return
        mine = self._key(self.leader, self.term) if self.leader else None
        lease_over = now - self.leader_seen > LEASE
        if mine is None or self._key(lead, term) >= mine or (not self.is_leader and lease_over):
            self.leader, self.term, self.leader_seen = lead, term, now
            self.is_leader = False

    def tick(self, now):
        """True when this drone has just become leader."""
        if self.is_leader or now - self.leader_seen <= LEASE + self.jitter:
            return False
        alive = {d for d, t in self.heard.items() if now - t < HEARD_WINDOW} | {self.unit}
        if min(alive, key=DRONES.index) != self.unit:
            return False
        self.term += 1
        self.leader, self.is_leader, self.leader_seen = self.unit, True, now
        self.jitter = self.rng.uniform(0, JITTER)
        return True


# ---------------------------------------------------------------- the leader's job
def reachable(terrain, pos, start, goal, relays):
    """Is there a path start -> goal that only uses `relays` in between?"""
    todo, seen = [start], {start}
    while todo:
        u = todo.pop()
        for v in list(relays) + [goal]:
            if v in seen or v not in pos:
                continue
            if link_ok(terrain, antenna(u, pos[u]), antenna(v, pos[v])):
                if v == goal:
                    return True
                seen.add(v)
                todo.append(v)
    return False


def connected(terrain, pos, drones=DRONES):
    """True if the rover can reach the base through `drones`."""
    return reachable(terrain, pos, "rover", "base", drones)


def assign(spots, here):
    """Give each spot to a drone so the longest flight (then the total) is as short as possible.
    spots: [(x, y, z)]; here: {drone: (x, y, z)} with at least len(spots) drones. Returns {drone: spot}."""
    best = min(itertools.permutations(here, len(spots)),
               key=lambda ds: (max(math.dist(here[d], s) for d, s in zip(ds, spots)),
                               sum(math.dist(here[d], s) for d, s in zip(ds, spots))))
    return dict(zip(best, spots))


class Coordinator:
    """Plans from positions REPORTED over the mesh (STATUS bundles), not true positions."""

    def __init__(self, terrain, mode="off", log=lambda text: None):
        self.terrain, self.mode, self.log = terrain, mode, log
        self.park = {d: (*START_XY[d], round(terrain.ground(*START_XY[d]) + DRONE_HOVER, 2)) for d in DRONES}
        self.bases = {"base": (*START_XY["base"], terrain.ground(*START_XY["base"]))}
        self.targets, self.goal = dict(self.park), dict(self.park)
        self.order, self.moving = [], {}       # drones still to move; drone -> deadline while in flight
        self.chains, self.state, self.reason, self.on_since = {}, "STANDBY", "mesh off", None
        self.next_check, self.vel, self.last_rover = 0.0, [0.0, 0.0], None
        self.drones = list(DRONES)             # drones we have a STATUS from (alive and in reach)

    def adopt(self, msg):
        """A new leader carries on from the last heartbeat it heard."""
        self.mode = msg.get("mode", self.mode)
        if msg.get("targets"):
            self.targets = {d: tuple(p) for d, p in msg["targets"].items()}
            self.goal = dict(self.targets)
        self.chains, self.state = msg.get("chains", {}), msg.get("state", self.state)
        self.on_since = msg.get("on_since")

    def set_mode(self, mode):
        if mode in ("on", "off"):
            self.mode = mode

    def _ahead(self, p, t):
        x, y = p[0] + self.vel[0] * t, p[1] + self.vel[1] * t
        return (x, y, self.terrain.ground(x, y))

    def _deploy(self, pos, why, now, lead=0.0):
        new = ch = None
        if lead > 0:                                   # plan for slightly ahead first
            new, ch = plan(self.terrain, dict(pos, rover=self._ahead(pos["rover"], lead)), self.drones)
            if new is not None and not connected(self.terrain, {**pos, **new}, self.drones):
                new = None                             # it must also work where the rover is NOW
        if new is None:
            new, ch = plan(self.terrain, pos, self.drones)
        if new is None:
            self.reason = f"cannot plan: {ch}"
            return
        if self.state != "DEPLOYED":
            self.on_since = now
        # the planner names drones in list order; re-name so no drone crosses the crater to swap places
        names = list(new)
        cur = self._where(pos)
        got = assign([new[n] for n in names], {d: cur[d] for d in names})
        owner = {names.index(n): d for d, spot in got.items() for n in names if new[n] is spot}
        new = {owner[i]: new[n] for i, n in enumerate(names)}
        ch = {b: [owner[names.index(n)] for n in chain] for b, chain in ch.items()}
        self.goal, self.chains = {**self.goal, **new}, ch
        self.state, self.reason = "DEPLOYED", why
        self.log(f"t={now:.1f} plan ({why}): chain to base {ch.get('base')}")
        self._queue()

    def _queue(self):
        self.order = [d for d in self.drones if math.dist(self.targets[d], self.goal[d]) > ARRIVED]

    def _where(self, pos):
        """Best guess of where every unit is: drones in flight where they REPORT, the rest at their targets."""
        return {**pos, **{d: t for d, t in self.targets.items() if d not in self.moving or d not in pos}}

    def _send(self, d, pos, now):
        self.order.remove(d)
        start = pos.get(d, self.targets[d])
        self.log(f"t={now:.1f} fly {d} {math.dist(start, self.goal[d]):.0f} m")
        self.targets[d] = self.goal[d]
        self.moving[d] = now + flight_time(math.dist(start, self.goal[d])) + MOVE_SLACK

    def _move(self, pos, now):
        """Hand out drone targets. While the chain works: one drone at a time, in an order that keeps
        them working. When nothing works yet (fresh deploy) or the mesh is OFF (parking): every drone
        at once, nothing to protect. Parking one at a time would cut off the far drones before their
        turn: they would never hear their 'fly home' target."""
        for d, deadline in list(self.moving.items()):
            if d in pos and math.dist(pos[d], self.targets[d]) < ARRIVED or now > deadline:
                del self.moving[d]
        if not self.order:
            return
        if self.mode == "off":
            for d in list(self.order):
                self._send(d, pos, now)
            return
        if self.moving:
            return
        cur = self._where(pos)
        if not connected(self.terrain, cur, self.drones):
            for d in list(self.order):
                self._send(d, pos, now)
            return
        # best: a drone the chain doesn't need while it flies (treated as gone), and fine once there;
        # else one whose new spot keeps the chain up; else just the next one
        lands_ok = [d for d in self.order if connected(self.terrain, {**cur, d: self.goal[d]}, self.drones)]
        spare = [d for d in lands_ok
                 if connected(self.terrain, cur, [x for x in self.drones if x != d])]
        self._send((spare or lands_ok or self.order)[0], pos, now)

    def update(self, statuses, now):
        """statuses: {unit: {"pos": [x, y, z], "neighbours": [...]}} from units heard recently (a drone
        missing here is treated as dead). Returns the LEADER payload."""
        pos = {u: tuple(s["pos"]) for u, s in statuses.items() if s.get("pos")}
        pos.update(self.bases)
        self.drones = [d for d in DRONES if d in statuses]
        age = now - statuses.get("rover", {}).get("time", now)
        if "rover" in pos and age <= ROVER_STALE:      # rover speed from successive FRESH reports
            if self.last_rover and now - self.last_rover[0] > 0.2:
                dt = now - self.last_rover[0]
                for i in (0, 1):
                    self.vel[i] = 0.5 * self.vel[i] + 0.5 * (pos["rover"][i] - self.last_rover[1][i]) / dt
            if self.last_rover is None or now - self.last_rover[0] > 0.2:
                self.last_rover = (now, pos["rover"])
        if "rover" in pos and age > ROVER_STALE:       # no report (cut off): it has driven on since
            pos["rover"] = self._ahead(pos["rover"], min(age, ROVER_GUESS_MAX))
        if self.mode == "off" and self.state == "DEPLOYED":
            self.goal, self.chains, self.state, self.reason, self.on_since = dict(self.park), {}, "STANDBY", "mesh switched OFF", None
            self._queue()
        elif (self.mode == "off" and not self.order
              and any(math.dist(self.targets[d], self.park[d]) > ARRIVED for d in self.drones)):
            # mesh OFF but a drone isn't sent home (e.g. a new leader carried on from a half-parked plan)
            self.goal = dict(self.park)
            self._queue()
        elif self.mode == "on" and self.state == "STANDBY" and "rover" in pos:
            self._deploy(pos, "mesh switched ON", now)
        elif (self.mode == "on" and self.state == "DEPLOYED" and "rover" in pos and not self.order
              and not self.moving and now >= self.next_check):
            self.next_check = now + CHECK_PERIOD
            cur = self._where(pos)
            soon = dict(cur, rover=self._ahead(pos["rover"], LOOK_AHEAD))
            ok = all(reachable(self.terrain, p, "rover", "base", self.drones) for p in (cur, soon))
            # re-plan only when the chain is about to break: every move is a flight that can open a gap
            if not ok:
                self._deploy(pos, "chain to base will break soon", now, lead=LOOK_AHEAD / 2)
        self._move(pos, now)
        return {"mode": self.mode, "state": self.state, "reason": self.reason, "chains": self.chains,
                "targets": {d: list(p) for d, p in self.targets.items()}, "on_since": self.on_since,
                "routes": self.routes(statuses, pos)}

    def routes(self, statuses, pos):
        """Route table from the link model the leader can work out itself: reported positions (drones at
        the targets it handed out) + range + line of sight over the terrain map. This is right the
        moment a move is ordered; nodes still refuse a next hop they can't hear (mesh_node.next_hop)."""
        cur = self._where(pos)
        units = [u for u in UNITS if u in cur and (u not in DRONES or u in self.drones)]
        edges = {}
        for i, u in enumerate(units):
            for v in units[i + 1:]:
                a, b = antenna(u, cur[u]), antenna(v, cur[v])
                if link_ok(self.terrain, a, b):
                    edges[(u, v)] = round(max(0.0, 1 - math.dist(a, b) / RADIO_RANGE), 2)
        relays = self.drones + ["rover"] if self.state == "DEPLOYED" else []
        return compute_routes(edges, UNITS, relays)
