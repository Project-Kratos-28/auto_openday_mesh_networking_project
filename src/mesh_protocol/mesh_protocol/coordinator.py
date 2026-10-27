"""Coordinator: the mesh ON/OFF switch, and where the drones go.

Modes (set by the GUI toggle, or: ros2 run mesh_protocol mesh_ctl on|off|auto|deploy):
  off   (default at start)  drones parked next to base A, relaying OFF. Only direct links:
        the rover loses base A in the crater and base B never hears it.
  on    drones deploy IMMEDIATELY (wherever the rover is) and stay deployed: base A keeps
        commanding and base B keeps receiving odometry for the rest of the journey.
  auto  old behaviour: stay parked until base A loses the rover, then deploy.

While deployed it re-checks the chains every 0.5 s, for where the rover is now AND where it
will be in 3 s, and re-places the drones BEFORE a link breaks (or after REPLAN_DISTANCE m). Drones are moved one at a time
in an order that keeps both chains working at every step (when such an order exists).
"""
import argparse
import json
import math
import threading
import time

from mesh_common import (DRONES, START_XY, DRONE_HOVER, REPLAN_DISTANCE, KEY_POSITIONS, KEY_ACKS_A,
                         KEY_ODOM_B, KEY_COORD_STATE, KEY_COORD_CMD, open_session)
from mesh_link_model.terrain import Terrain
from mesh_link_model.planner import plan, link_ok, antenna
from mesh_link_model.sim_io import teleport

LOST_AFTER = 3.0          # s without an ACK at base A = rover unreachable (auto mode)
LOOK_AHEAD = 3.0          # s: re-place drones if a chain would break within this time
CHECK_PERIOD = 0.5        # s between chain checks
MODES = ("off", "on", "auto")


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


def connected(terrain, pos):
    """True if the rover can reach BOTH bases through the drones."""
    return (reachable(terrain, pos, "rover", "base_a", DRONES)
            and reachable(terrain, pos, "rover", "base_b", DRONES))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--fake", action="store_true", help="teleport in fake mode (no Gazebo)")
    ap.add_argument("--start", choices=MODES, default="off", help="mesh mode at start (default off)")
    args, _ = ap.parse_known_args()

    terrain = Terrain.load()
    lock = threading.Lock()
    st = {"pos": {}, "last_ack": 0.0, "last_odom": 0.0, "request": None}
    s = open_session()
    park = {d: (*START_XY[d], round(terrain.ground(*START_XY[d]) + DRONE_HOVER, 2)) for d in DRONES}

    def on_pos(x):
        m = json.loads(x.payload.to_bytes())
        with lock:
            st["pos"] = {u: tuple(p) for u, p in m["positions"].items()}

    def on_ack(x):
        with lock:
            st["last_ack"] = time.time()

    def on_odom(x):
        with lock:
            st["last_odom"] = time.time()

    def on_cmd(x):
        c = x.payload.to_string().strip().strip('"').lower()
        with lock:
            st["request"] = c

    subs = [s.declare_subscriber(KEY_POSITIONS, on_pos), s.declare_subscriber(KEY_ACKS_A, on_ack),  # noqa
            s.declare_subscriber(KEY_ODOM_B, on_odom), s.declare_subscriber(KEY_COORD_CMD, on_cmd)]
    pub = s.declare_publisher(KEY_COORD_STATE)
    log = lambda t: print(f"[coordinator] {t}", flush=True)   # noqa: E731

    S = {"mode": args.start, "state": "STANDBY", "relay": False, "targets": dict(park), "chains": {},
         "plan_at": None, "moved_at": 0.0, "reason": "mesh off" if args.start == "off" else "starting",
         "vel": [0.0, 0.0], "last_rover": None, "on_since": None}

    def ahead(p, t):
        """Where the rover will be in t seconds at its current speed."""
        x, y = p[0] + S["vel"][0] * t, p[1] + S["vel"][1] * t
        return (x, y, terrain.ground(x, y))

    def place(new, pos):
        """Move drones to `new`, one at a time, never breaking a working connection if possible."""
        cur = {**pos, **S["targets"]}
        todo = [d for d in new if d not in S["targets"] or math.dist(S["targets"][d], new[d]) > 0.5]
        keep = connected(terrain, cur)                 # only worth protecting if it works now
        while todo:
            pick = todo[0]
            if keep:
                for d in todo:
                    if connected(terrain, {**cur, d: new[d]}):
                        pick = d
                        break
            ok = teleport(pick, *new[pick], fake=args.fake, session=s)
            log(f"  {pick} -> ({new[pick][0]:.0f}, {new[pick][1]:.0f}, {new[pick][2]:.0f})"
                f"{'' if ok else '  (teleport FAILED)'}")
            cur[pick] = new[pick]
            todo.remove(pick)
        S["targets"] = {**S["targets"], **new}

    def deploy(why, lead=0.0):
        """Plan + place drones. lead = plan for where the rover will be in `lead` seconds."""
        with lock:
            pos = dict(st["pos"])
        if not all(u in pos for u in ("rover", "base_a", "base_b")):
            S["reason"] = "waiting for positions"
            return
        new = ch = None
        if lead > 0:                                   # plan for slightly ahead first
            new, ch = plan(terrain, dict(pos, rover=ahead(pos["rover"], lead)), DRONES)
            if new is not None and not connected(terrain, {**pos, **new}):
                new = None                             # it must also work where the rover is NOW
        if new is None:
            new, ch = plan(terrain, pos, DRONES)
        if new is None:
            S["reason"] = f"cannot plan: {ch}"
            log(S["reason"])
            return
        log(f"{why} -> base_a chain: rover -> {' -> '.join(ch['base_a']) or '(direct)'} -> base_a | "
            f"base_b chain: rover -> {' -> '.join(ch['base_b']) or '(direct)'} -> base_b")
        place(new, pos)
        if not S["relay"]:
            S["on_since"] = time.time()
        S.update(chains=ch, plan_at=pos["rover"], moved_at=time.time(), relay=True,
                 state="DEPLOYED", reason=why)
        publish()                                      # drones switch relay on right away

    def stand_down(why):
        log(f"{why} -> drones back to base A, relaying OFF")
        S.update(relay=False, state="STANDBY", chains={}, reason=why, on_since=None)
        publish()                                      # relay off first, then move
        with lock:
            pos = dict(st["pos"])
        place(park, pos)

    def publish():
        now = time.time()
        with lock:
            ack_age = now - st["last_ack"] if st["last_ack"] else None
            odom_age = now - st["last_odom"] if st["last_odom"] else None
        pub.put(json.dumps({"time": now, "mode": S["mode"], "state": S["state"], "relay": S["relay"],
                            "reason": S["reason"], "chains": S["chains"], "on_since": S["on_since"],
                            "targets": {d: list(p) for d, p in S["targets"].items()},
                            "ack_age": ack_age, "odom_age": odom_age}))

    log(f"started, mesh mode = {S['mode'].upper()}. Terrain: {terrain.describe()}")
    t_start, next_check, next_pub = time.time(), 0.0, 0.0
    try:
        while True:
            now = time.time()
            with lock:
                req, st["request"] = st["request"], None
                ack_age = now - st["last_ack"] if st["last_ack"] else None
                pos = dict(st["pos"])
            if req:
                if req in MODES and req != S["mode"]:
                    log(f"mesh mode {S['mode'].upper()} -> {req.upper()}")
                    S["mode"] = req
                    if req == "off" and S["state"] == "DEPLOYED":
                        stand_down("mesh switched OFF")
                    elif req == "on" and S["state"] == "STANDBY":
                        deploy("mesh switched ON")
                    next_pub = 0.0
                elif req == "deploy":
                    deploy("deploy requested")
                elif req not in MODES:
                    log(f"unknown command '{req}' (use on, off, auto, deploy)")
            if now >= next_check:
                next_check = now + CHECK_PERIOD
                if "rover" in pos:                         # rover speed estimate
                    lr = S["last_rover"]
                    if lr is not None and now - lr[0] > 0.2:
                        dt = now - lr[0]
                        for i in (0, 1):
                            S["vel"][i] = 0.5 * S["vel"][i] + 0.5 * (pos["rover"][i] - lr[1][i]) / dt
                    if lr is None or now - lr[0] > 0.2:
                        S["last_rover"] = (now, pos["rover"])
                lost = (ack_age is None and now - t_start > 10) or (ack_age is not None and ack_age > LOST_AFTER)
                if S["state"] == "STANDBY" and S["mode"] == "on":
                    deploy("mesh ON")                      # e.g. positions weren't ready yet
                elif S["state"] == "STANDBY" and S["mode"] == "auto" and lost:
                    deploy(f"LINK LOST: no answer from the rover for "
                           f"{'>10' if ack_age is None else f'{ack_age:.0f}'} s")
                elif S["state"] == "DEPLOYED" and "rover" in pos and now - S["moved_at"] > 1.0:
                    now_pos = {**pos, **S["targets"]}
                    soon = dict(now_pos, rover=ahead(pos["rover"], LOOK_AHEAD))
                    ok_a = all(reachable(terrain, p, "rover", "base_a", DRONES) for p in (now_pos, soon))
                    ok_b = all(reachable(terrain, p, "rover", "base_b", DRONES) for p in (now_pos, soon))
                    far = S["plan_at"] is None or math.dist(S["plan_at"][:2], pos["rover"][:2]) > REPLAN_DISTANCE
                    if not (ok_a and ok_b):
                        deploy(f"chain to {'base_a' if not ok_a else 'base_b'} will break soon",
                               lead=LOOK_AHEAD / 2)
                    elif far:
                        deploy(f"rover moved {REPLAN_DISTANCE:.0f} m", lead=LOOK_AHEAD / 2)
            if now >= next_pub:
                next_pub = now + 0.5
                publish()
            time.sleep(0.05)
    except KeyboardInterrupt:
        pass
    finally:
        s.close()


if __name__ == "__main__":
    main()
