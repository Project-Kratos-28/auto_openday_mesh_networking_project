"""Mission autopilot at the base: drive the rover from the base to the science target.

  ros2 run mesh_protocol mission                 # 1 m/s, assist on
  ros2 run mesh_protocol mission --speed 0.5 --no-assist

Twice a second it sends a velocity command to the base (mesh/base/cmd); the base sends it
over the mesh. Steering: aim at a point 4 m ahead on the line y = 0.
(Steering uses the simulator's true rover position, like a localisation system, so the
demo doesn't drift. The odometry the rover sends goes to the base over the mesh.)
Stops TARGET_REACH metres before the target marker (TARGET_XY).

Assist (backup): if the rover IS getting the commands (ACKs arrive) but hasn't moved 0.5 m
in 6 s (stuck on a slope or a bump), it is lifted 1.5 m forward along the path.
If commands are NOT getting through, it does nothing and doesn't count that time as stuck: that's
the comms problem the mesh fixes (the rover waits while the drones fly into place).
"""
import argparse
import json
import math
import threading
import time

from mesh_common import (TARGET_XY, TARGET_REACH, START_XY, ROVER_SPEED, KEY_POSITIONS, KEY_ACKS, KEY_CMD,
                         KEY_MISSION, KEY_PHOTO_CMD, open_session)
from mesh_link_model.terrain import Terrain
from mesh_link_model.sim_io import teleport

LOOKAHEAD = 4.0          # m ahead on the line y = 0 that the rover steers towards
ASSIST_STEP = 1.5        # m the rover is lifted forward when stuck
STUCK_MOVE = 0.5         # m: moved less than this in --stuck-time seconds = stuck


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--speed", type=float, default=ROVER_SPEED, help="forward speed m/s")
    ap.add_argument("--no-assist", action="store_true", help="never lift the rover when stuck")
    ap.add_argument("--fake", action="store_true", help="fake mode (no Gazebo)")
    ap.add_argument("--stuck-time", type=float, default=6.0)
    args, _ = ap.parse_known_args()

    terrain = Terrain.load()
    lock = threading.Lock()
    st = {"pos": None, "yaw": 0.0, "last_ack": 0.0, "acks": 0}
    s = open_session()

    def on_pos(x):
        m = json.loads(x.payload.to_bytes())
        if "rover" in m["positions"]:
            with lock:
                st["pos"] = m["positions"]["rover"]
                st["yaw"] = m.get("yaw", {}).get("rover", 0.0)

    def on_ack(x):
        with lock:
            st["last_ack"] = time.time()
            st["acks"] += 1

    subs = [s.declare_subscriber(KEY_POSITIONS, on_pos), s.declare_subscriber(KEY_ACKS, on_ack)]  # noqa
    pub_cmd = s.declare_publisher(KEY_CMD)
    pub_status = s.declare_publisher(KEY_MISSION)
    log = lambda t: print(f"[mission] {t}", flush=True)   # noqa: E731
    start_x, goal = START_XY["rover"][0], TARGET_XY[0] - TARGET_REACH

    log(f"waiting for the rover to answer the base ... (goal x = {goal:.0f} m, speed {args.speed} m/s)")
    while True:
        with lock:
            ok = st["pos"] is not None and time.time() - st["last_ack"] < 2.0
        if ok:
            break
        time.sleep(0.5)
    log("rover answers -> driving to the target")

    history, assists, state, last_print = [], 0, "DRIVING", 0.0
    try:
        while True:
            now = time.time()
            with lock:
                x, y, z = st["pos"]
                yaw = st["yaw"]
                link_ok = now - st["last_ack"] < 2.0
            if x >= goal:
                for _ in range(5):
                    pub_cmd.put(json.dumps({"linear": 0.0, "angular": 0.0}))
                    time.sleep(0.2)
                state = "ARRIVED"
                pub_status.put(json.dumps({"time": now, "state": state, "x": x, "y": y,
                                           "progress": 100.0, "assists": assists}))
                log(f"ARRIVED at the target, x = {x:.1f} m ({assists} assists). Rover stopped.")
                s.put(KEY_PHOTO_CMD, "arrived")        # ask the rover for a photo of the target
                log("photo requested (it travels back over the mesh when the links allow)")
                break
            heading = math.atan2(-y, LOOKAHEAD)                    # aim back to y = 0
            err = wrap(heading - yaw)
            angular = max(-0.8, min(0.8, 1.5 * err))
            linear = args.speed * max(0.2, math.cos(err))
            pub_cmd.put(json.dumps({"linear": round(linear, 2), "angular": round(angular, 2)}))

            if link_ok:
                history.append((now, x))
                history = [h for h in history if now - h[0] <= args.stuck_time]
            else:
                history = []            # stopped because there's no link (waiting for drones), not stuck
            stuck = (link_ok and history and now - history[0][0] >= args.stuck_time - 0.6
                     and x - history[0][1] < STUCK_MOVE)
            if stuck and not args.no_assist:
                nx, ny = x + ASSIST_STEP, y * 0.5
                nz = terrain.ground(nx, ny) + 0.3
                assists += 1
                log(f"ASSIST #{assists}: rover stuck at x = {x:.1f} m -> lifting it to x = {nx:.1f} m")
                teleport("rover", round(nx, 2), round(ny, 2), round(nz, 2), 0.0, fake=args.fake, session=s)
                history = []
            state = "DRIVING" if link_ok else "NO LINK (rover stopped, waiting for mesh)"
            progress = 100.0 * (x - start_x) / (goal - start_x)
            pub_status.put(json.dumps({"time": now, "state": state, "x": round(x, 1), "y": round(y, 1),
                                       "progress": round(progress, 1), "assists": assists}))
            if now - last_print > 5:
                log(f"x = {x:7.1f} m  ({progress:4.1f} %)  {state}")
                last_print = now
            time.sleep(0.5)
    except KeyboardInterrupt:
        pub_cmd.put(json.dumps({"linear": 0.0, "angular": 0.0}))
        log("stopped by user")
    finally:
        time.sleep(0.3)
        s.close()


if __name__ == "__main__":
    main()
