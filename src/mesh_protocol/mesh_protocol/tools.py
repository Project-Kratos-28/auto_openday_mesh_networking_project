"""Small helper commands.

  ros2 run mesh_protocol mesh_monitor            # live view: both bases, leader drone, links
  ros2 run mesh_protocol mesh_ctl on|off         # the mesh toggle (same as the dashboard switch)
  ros2 run mesh_protocol send_cmd 1.0 0.0        # one drive command from base A (0 = stop)
  ros2 run mesh_protocol move_unit drone_1 0 0 12   # teleport a unit in Gazebo
  ros2 run mesh_protocol move_unit home             # drones back to their parking spot
  (add --fake to move_unit when running without Gazebo)
"""
import argparse
import json
import sys
import threading
import time

from mesh_common import (UNITS, DRONES, START_XY, DRONE_HOVER, KEY_LINKS, KEY_ACKS_A, KEY_ODOM_B,
                         KEY_CMD_A, KEY_COORD_STATE, KEY_COORD_CMD, KEY_MISSION, open_session)


# ------------------------------------------------------------------ move_unit
def move_unit():
    from mesh_link_model.sim_io import teleport
    from mesh_link_model.terrain import Terrain
    ap = argparse.ArgumentParser(prog="move_unit")
    ap.add_argument("unit", help="unit name, or 'home' (all drones to parking)")
    ap.add_argument("xyz", nargs="*", type=float)
    ap.add_argument("--fake", action="store_true", help="move in fake mode (no Gazebo)")
    args, _ = ap.parse_known_args()
    if args.unit == "home":
        t = Terrain.load()
        moves = [(d, (*START_XY[d], round(t.ground(*START_XY[d]) + DRONE_HOVER, 2))) for d in DRONES]
    elif args.unit in UNITS and len(args.xyz) == 3:
        moves = [(args.unit, tuple(args.xyz))]
    else:
        sys.exit(f"usage: move_unit <{'|'.join(UNITS)}> X Y Z   or   move_unit home")
    s = open_session() if args.fake else None
    if s:
        time.sleep(1.0)                                         # let Zenoh connect
    for u, p in moves:
        ok = teleport(u, *p, fake=args.fake, session=s)
        print(f"{u} -> {p}: {'OK' if ok else 'FAILED (is Gazebo running?)'}")
    if s:
        time.sleep(0.5)
        s.close()


# ------------------------------------------------------------------ send_cmd / mesh_ctl
def _put_once(key, value):
    s = open_session()
    time.sleep(1.0)                                             # let Zenoh connect
    s.put(key, value)
    time.sleep(0.5)
    s.close()


def send_cmd():
    ap = argparse.ArgumentParser(prog="send_cmd")
    ap.add_argument("linear", type=float, help="forward speed m/s (0 = stop)")
    ap.add_argument("angular", type=float, nargs="?", default=0.0, help="turn rate rad/s")
    args, _ = ap.parse_known_args()
    _put_once(KEY_CMD_A, json.dumps({"linear": args.linear, "angular": args.angular}))
    print(f"base_a will send: linear {args.linear}, angular {args.angular} "
          f"(the rover's safety watchdog stops it after 2 s without new commands)")


def mesh_ctl():
    ap = argparse.ArgumentParser(prog="mesh_ctl")
    ap.add_argument("what", choices=["on", "off", "auto"])
    args, _ = ap.parse_known_args()
    if args.what == "auto":
        print("auto mode was removed in v3: use mesh_ctl on / off")
        return
    _put_once(KEY_COORD_CMD, args.what)
    print({"off": "mesh OFF: base A tells the leader drone; drones park, no relaying",
           "on": "mesh ON: base A tells the leader drone; drones deploy and keep both bases connected"}[args.what])


# ------------------------------------------------------------------ mesh_monitor
def mesh_monitor():
    argparse.ArgumentParser().parse_known_args()
    lock = threading.Lock()
    st = {"links": [], "ack": None, "acks": 0, "odom": None, "odoms": 0, "coord": None,
          "mission": None, "stats": {}, "odom_times": []}
    s = open_session()

    def setter(key, count=None):
        def cb(x):
            with lock:
                st[key] = json.loads(x.payload.to_bytes())
                if count:
                    st[count] += 1
        return cb

    def on_links(x):
        with lock:
            st["links"] = json.loads(x.payload.to_bytes())["links"]

    def on_odom(x):
        m = json.loads(x.payload.to_bytes())
        with lock:
            st["odom"], st["odoms"] = m, st["odoms"] + 1
            st["odom_times"] = [t for t in st["odom_times"] if m["time"] - t < 600] + [m["time"]]

    def on_stats(x):
        with lock:
            st["stats"][str(x.key_expr).split("/")[-1]] = json.loads(x.payload.to_bytes())

    subs = [s.declare_subscriber(KEY_LINKS, on_links),                          # noqa: F841
            s.declare_subscriber(KEY_ACKS_A, setter("ack", "acks")),
            s.declare_subscriber(KEY_ODOM_B, on_odom),
            s.declare_subscriber(KEY_COORD_STATE, setter("coord")),
            s.declare_subscriber(KEY_MISSION, setter("mission")),
            s.declare_subscriber("mesh/stats/*", on_stats)]
    G, R, Y, N = "\033[32m", "\033[31m", "\033[33m", "\033[0m"
    try:
        while True:
            now = time.time()
            with lock:
                c = {k: (v.copy() if isinstance(v, (dict, list)) else v) for k, v in st.items()}
            out = ["\033[2J\033[H=== MARS RELAY MESH MONITOR (Ctrl+C to quit) ===", ""]
            co = c["coord"]
            if co:
                col = G if co["state"] == "DEPLOYED" else Y
                out.append(f"MESH: {col}{co['mode'].upper()}{N}  state={co['state']}  leader={co.get('leader')} "
                           f"(term {co.get('term')})   ({co['reason']})")
                for b in ("base_a", "base_b"):
                    if co["chains"].get(b) is not None:
                        out.append(f"  chain {b}: rover -> {' -> '.join(co['chains'][b]) or '(direct)'} -> {b}")
            else:
                out.append(f"MESH: {Y}base A has heard no leader drone yet{N}")
            m = c["mission"]
            if m:
                out.append(f"MISSION: {m['state']}  rover x = {m['x']} m  ({m['progress']} % to base B)"
                           f"  assists: {m['assists']}")
            out.append("")
            a = c["ack"]
            if a and now - a["time"] < 3:
                out.append(f"BASE A (commander): {G}CONNECTED{N}  {c['acks']} acks  rtt {a['rtt_ms']} ms")
                out.append(f"  command went: {' -> '.join(a['cmd_route'] or [])}")
                out.append(f"  ack came back: {' -> '.join(a['ack_route'])}   rover moving: {a['moving']}")
            else:
                age = f"no ack for {now - a['time']:.0f} s" if a else "no ack yet"
                out.append(f"BASE A (commander): {R}NO CONNECTION{N} to the rover ({age})")
            o = c["odom"]
            if o and now - o["time"] < 3:
                p = o["payload"]
                od, tr = p.get("odom"), p.get("truth")
                out.append(f"BASE B (receiver):  {G}RECEIVING{N}  {c['odoms']} msgs  delay {o['delay_ms']} ms")
                out.append(f"  route: {' -> '.join(o['route'])}")
                since = (co or {}).get("on_since")
                ts = [t for t in c["odom_times"] if since and t >= since]
                if since and len(ts) > 1:
                    gap = max(b - a for a, b in zip(ts, ts[1:]))
                    out.append(f"  since mesh ON: {len(ts)} msgs, longest gap {gap:.1f} s "
                               f"({'flow OK' if gap < 1.5 else 'there was a gap'})")
                if od:
                    out.append(f"  wheel odometry: x {od['x']} y {od['y']} yaw {od['yaw']} v {od['v']} m/s")
                if tr:
                    out.append(f"  true position:  {tr['pos']}")
            else:
                age = f"nothing for {now - o['time']:.0f} s" if o else "nothing received yet"
                out.append(f"BASE B (receiver):  {R}NO DATA{N} from the rover ({age})")
            out += ["", "LINKS UP (not counting drone<->drone):"]
            up = [f"{l['a']}<->{l['b']} {l['distance']:.0f}m" for l in c["links"]
                  if l["up"] and not (l["a"].startswith("drone") and l["b"].startswith("drone"))]
            for i in range(0, len(up), 4):
                out.append("  " + "   ".join(up[i:i + 4]))
            if not up:
                out.append("  (none)")
            out += ["", f"{'unit':>8} {'held':>5} {'sent':>6} {'recv':>6} {'fwd':>6} {'deliv':>6} "
                        f"{'resend':>6} {'expd':>5} {'dup':>5} {'badCRC':>6}  neighbours"]
            for u in UNITS:
                x = c["stats"].get(u)
                if x:
                    held = 'DEAD' if x.get('killed') else sum(x.get('held', {}).values())
                    out.append(f"{u:>8} {held:>5} {x['sent']:6} {x['received']:6} {x['forwarded']:6} "
                               f"{x['delivered']:6} {x['retransmits']:6} {x['expired']:5} {x['duplicates']:5} "
                               f"{x['bad_crc']:6}  "
                               f"{', '.join(x['neighbours']) or '-'}")
                else:
                    out.append(f"{u:>8}   (no stats yet - is its mesh_node running?)")
            print("\n".join(out), flush=True)
            time.sleep(1.0)
    except KeyboardInterrupt:
        pass
    finally:
        s.close()
