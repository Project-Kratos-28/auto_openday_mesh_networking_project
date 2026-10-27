"""Link model: reads where every unit is (from Gazebo) and decides who can reach whom.

A link is UP when two units are within RADIO_RANGE *and* the terrain doesn't block
the straight line between their antennas. Publishes mesh/links and mesh/positions
5 times a second. This program is also the Zenoh "hub" the others connect to.

Run:  python3 -m mesh_link_model.link_model            (positions from Gazebo)
      python3 -m mesh_link_model.link_model --fake     (no Gazebo: a simple fake rover
                                                       that drives on mesh/fake/cmd_vel)
"""
import argparse
import itertools
import json
import math
import threading
import time

from mesh_common import (UNITS, GZ_NAMES, WORLD, START_XY, DRONE_HOVER, RADIO_RANGE,
                         LOS_CLEARANCE, KEY_LINKS, KEY_POSITIONS, KEY_FAKE_POSE,
                         KEY_FAKE_CMD_VEL, antenna_height, open_session)
from mesh_link_model.terrain import Terrain


def yaw_of(q):
    """Heading (radians) from a quaternion."""
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def start_gazebo_poses(state, lock, world):
    """Subscribe to Gazebo's pose list and keep state['pos'] / state['yaw'] up to date."""
    from gz.transport13 import Node, SubscribeOptions
    from gz.msgs10.pose_v_pb2 import Pose_V
    to_unit = {gz: u for u, gz in GZ_NAMES.items()}

    def on_poses(msg: Pose_V):
        found = {}
        for p in msg.pose:                         # list also has links/visuals: keep model names
            u = to_unit.get(p.name)
            if u and u not in found:               # first entry with that name = the model itself
                found[u] = ((p.position.x, p.position.y, p.position.z), yaw_of(p.orientation))
        with lock:
            for u, (xyz, yaw) in found.items():
                state["pos"][u], state["yaw"][u] = xyz, yaw

    node = Node()
    opts = SubscribeOptions()
    try:
        opts.msgs_per_sec = 10                     # we don't need 60 updates a second
    except Exception:
        pass
    topic = f"/world/{world}/pose/info"
    if not node.subscribe(Pose_V, topic, on_poses, opts):
        raise RuntimeError(f"could not subscribe to {topic}")
    print(f"[link_model] reading positions from Gazebo topic {topic}", flush=True)
    return node


def compute_links(positions, terrain, radio_range=RADIO_RANGE):
    """List of link dicts for every pair of units we have a position for."""
    links = []
    for a, b in itertools.combinations(UNITS, 2):
        if a not in positions or b not in positions:
            continue
        pa = list(positions[a][:3]); pa[2] += antenna_height(a)
        pb = list(positions[b][:3]); pb[2] += antenna_height(b)
        dist = math.dist(pa, pb)
        in_range = dist <= radio_range
        los = terrain.line_of_sight(pa, pb, LOS_CLEARANCE) if in_range else False
        up = in_range and los
        links.append({"a": a, "b": b, "up": up, "distance": round(dist, 1),
                      "in_range": in_range, "los": los,
                      "quality": round(max(0.0, 1 - dist / radio_range), 2) if up else 0.0})
    return links


def fake_start(terrain):
    """Start positions on the ground (fake mode)."""
    pos = {}
    for u, (x, y) in START_XY.items():
        up = DRONE_HOVER if u.startswith("drone") else 0.0
        pos[u] = (x, y, round(terrain.ground(x, y) + up, 2))
    return pos


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--fake", action="store_true", help="no Gazebo: fake rover + start poses")
    ap.add_argument("--terrain", default=None, help="path to crater.obj (found automatically)")
    ap.add_argument("--range", type=float, default=RADIO_RANGE, help="radio range in metres")
    ap.add_argument("--rate", type=float, default=5.0, help="link updates per second")
    args, _ = ap.parse_known_args()

    terrain = Terrain.load(args.terrain)
    print(f"[link_model] terrain: {terrain.describe()}", flush=True)
    lock = threading.Lock()
    state = {"pos": {}, "yaw": {}, "cmd": (0.0, 0.0)}
    session = open_session(hub=True)
    keep = []                                          # keeps subscribers / gz node alive

    if args.fake:
        state["pos"] = fake_start(terrain)
        state["yaw"] = {u: 0.0 for u in UNITS}

        def on_fake_pose(sample):                      # {"unit": "drone_1", "pos": [x,y,z], "yaw": 0}
            try:
                m = json.loads(sample.payload.to_bytes())
                with lock:
                    state["pos"][m["unit"]] = tuple(float(v) for v in m["pos"])
                    state["yaw"][m["unit"]] = float(m.get("yaw", state["yaw"].get(m["unit"], 0.0)))
            except Exception as e:
                print(f"[link_model] bad fake pose message: {e}", flush=True)

        def on_fake_cmd(sample):                       # {"linear": v, "angular": w}
            m = json.loads(sample.payload.to_bytes())
            with lock:
                state["cmd"] = (float(m.get("linear", 0)), float(m.get("angular", 0)))

        keep += [session.declare_subscriber(KEY_FAKE_POSE, on_fake_pose),
                 session.declare_subscriber(KEY_FAKE_CMD_VEL, on_fake_cmd)]
        print("[link_model] FAKE mode: start positions, fake rover drives on mesh/fake/cmd_vel",
              flush=True)
    else:
        keep.append(start_gazebo_poses(state, lock, WORLD))

    pub_links = session.declare_publisher(KEY_LINKS)
    pub_pos = session.declare_publisher(KEY_POSITIONS)
    last_state, t_start, warned = {}, time.time(), False
    dt, next_pub, last = 0.05, 0.0, time.time()
    try:
        while True:
            now = time.time()
            if args.fake:                              # fake rover: drive along the ground
                with lock:
                    v, w = state["cmd"]
                    x, y, _ = state["pos"]["rover"]
                    yaw = state["yaw"]["rover"] + w * (now - last)
                    x += v * math.cos(yaw) * (now - last)
                    y += v * math.sin(yaw) * (now - last)
                    state["pos"]["rover"] = (x, y, terrain.ground(x, y))
                    state["yaw"]["rover"] = yaw
            last = now
            if now >= next_pub:
                next_pub = now + 1.0 / args.rate
                with lock:
                    pos = {u: state["pos"][u] for u in UNITS if u in state["pos"]}
                    yaws = {u: state["yaw"].get(u, 0.0) for u in pos}
                links = compute_links(pos, terrain, args.range)
                pub_links.put(json.dumps({"time": now, "links": links}))
                pub_pos.put(json.dumps({"time": now,
                                        "positions": {u: [round(c, 2) for c in p[:3]] for u, p in pos.items()},
                                        "yaw": {u: round(a, 3) for u, a in yaws.items()}}))
                for l in links:                        # print only when a link changes
                    k = (l["a"], l["b"])
                    if last_state.get(k) != l["up"]:
                        why = "" if l["up"] else (" (terrain blocks)" if l["in_range"] else " (out of range)")
                        if k in last_state or l["up"]:
                            print(f"[link_model] LINK {'UP  ' if l['up'] else 'DOWN'} "
                                  f"{l['a']} <-> {l['b']}  {l['distance']} m{why}", flush=True)
                        last_state[k] = l["up"]
                missing = [u for u in UNITS if u not in pos]
                if missing and not warned and now - t_start > 5:
                    warned = True
                    print(f"[link_model] WARNING: no position yet for {missing}. Is Gazebo running "
                          f"and playing (press the play button)? Check: gz topic -l | grep pose", flush=True)
            time.sleep(dt)
    except KeyboardInterrupt:
        pass
    finally:
        session.close()


if __name__ == "__main__":
    main()
