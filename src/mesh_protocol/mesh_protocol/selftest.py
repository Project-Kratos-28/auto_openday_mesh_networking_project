"""Self-test of the relay demo WITHOUT Gazebo (fake rover), about 3 minutes.

  ros2 run mesh_protocol selftest

Starts everything on a separate Zenoh port (so it doesn't disturb a running sim), with the
mesh OFF like a normal start, and drives the fake rover at demo speed (ROVER_SPEED, 1 m/s). Checks:
  1. start (mesh OFF): a leader drone is elected, base A talks to the rover directly, base B gets nothing
  2. mesh OFF, rover drives into the crater -> base A loses it, nothing deploys
  3. toggle ON -> all 4 drones deploy, base A connected again, base B receives odometry
  4. rover drives on to base B -> odometry flows the whole way, no gaps
  5. second journey (demo speed, 1 m/s) with the mesh ON from the start -> base B gets odometry from the
     first second to the end, base A never loses the rover
  6. the web dashboard streams the state and its toggle works
"""
import json
import os
import subprocess
import sys
import threading
import time
import urllib.request

PORT = "tcp/127.0.0.1:7457"
os.environ["MESH_HUB"] = PORT                          # must be set before mesh_common is imported

from mesh_common import (UNITS, DRONES, START_XY, ROVER_SPEED, CRATER_RADIUS, KEY_ACKS_A, KEY_ODOM_B, KEY_COORD_STATE,  # noqa: E402
                         KEY_COORD_CMD, KEY_MISSION, KEY_FAKE_POSE, KEY_FAKE_CMD_VEL, KEY_FAIL_CONTROL,
                         KEY_PHOTO_B, KEY_PHOTO_CMD,
                         open_session)

MAX_GAP = 1.5      # s: odometry is sent every 0.5 s, so a gap above this means the flow broke


def main():
    env = dict(os.environ, MESH_HUB=PORT, PYTHONUNBUFFERED="1")
    log = open("/tmp/mesh_selftest.log", "w")
    procs, results = [], []

    def start(module, *args, extra_env=None):
        p = subprocess.Popen([sys.executable, "-u", "-m", module, *args], env={**env, **(extra_env or {})},
                             stdout=log, stderr=subprocess.STDOUT)
        procs.append(p)
        return p

    def check(name, ok, detail=""):
        results.append(bool(ok))
        print(f"  [{'PASS' if ok else 'FAIL'}] {name} {detail}", flush=True)

    lock = threading.Lock()
    st = {"acks": [], "odoms": [], "coord": None, "mission": None, "stats": {}, "max_held": 0, "photo": None}

    def on_stats(x):
        m = json.loads(x.payload.to_bytes())
        with lock:
            st["stats"][str(x.key_expr).split("/")[-1]] = m
            st["max_held"] = max(st["max_held"], sum(m.get("held", {}).values()))

    def total(field):
        with lock:
            return sum(m.get(field, 0) for m in st["stats"].values())

    def missing_seqs(t0=0.0, t1=None):
        """Telemetry seqs sent in [t0, t1] that base B never got (between the first and last it got)."""
        with lock:
            got = {m["seq"] for m in st["odoms"] if m["sent_time"] >= t0 and (t1 is None or m["sent_time"] <= t1)}
        return sorted(set(range(min(got), max(got) + 1)) - got) if got else None

    def rec(key):
        def cb(x):
            m = json.loads(x.payload.to_bytes())
            m["rx"] = time.time()
            with lock:
                if key in ("acks", "odoms"):
                    st[key].append(m)
                else:
                    st[key] = m
        return cb

    def times(key, t0, t1=None):
        with lock:
            return [m["rx"] for m in st[key] if m["rx"] >= t0 and (t1 is None or m["rx"] <= t1)]

    def age(key):
        with lock:
            return time.time() - st[key][-1]["rx"] if st[key] else None

    def coord(field):
        with lock:
            return (st["coord"] or {}).get(field)

    def wait_for(cond, timeout):
        end = time.time() + timeout
        while time.time() < end:
            if cond():
                return True
            time.sleep(0.2)
        return cond()

    def longest_gap(ts, t0, t1):
        pts = [t0] + ts + [t1]
        return max(b - a for a, b in zip(pts, pts[1:]))

    def gap_start(ts, t0, t1):
        """When the longest gap began (epoch seconds), to find it in the leader's log (t=...)."""
        pts = [t0] + ts + [t1]
        return max(zip(pts, pts[1:]), key=lambda p: p[1] - p[0])[0]

    try:
        print("Starting link model, radio, injector, 7 mesh nodes (mesh OFF, fake mode)...")
        start("mesh_link_model.link_model", "--fake")
        time.sleep(1.5)
        start("mesh_link_model.radio_channel")
        start("mesh_link_model.passthrough_injector")
        for u in UNITS:
            start("mesh_protocol.mesh_node", "--unit", u, "--fake", "--mesh", "off")
        s = open_session()
        subs = [s.declare_subscriber(KEY_ACKS_A, rec("acks")), s.declare_subscriber(KEY_ODOM_B, rec("odoms")),  # noqa
                s.declare_subscriber(KEY_COORD_STATE, rec("coord")), s.declare_subscriber(KEY_MISSION, rec("mission")),
                s.declare_subscriber("mesh/stats/*", on_stats), s.declare_subscriber(KEY_PHOTO_B, rec("photo"))]

        print("\nTest 1: start, mesh OFF")
        check("base A talks to the rover directly", wait_for(lambda: age("acks") is not None and age("acks") < 1.5, 20))
        check("base B receives nothing", len(st["odoms"]) == 0)
        check("a leader drone was elected: mode OFF, drones parked",
              wait_for(lambda: coord("leader") in DRONES and coord("mode") == "off" and coord("state") == "STANDBY", 15),
              f"(leader {coord('leader')})")

        print("\nTest 2: mesh OFF, rover drives into the crater")
        mission = start("mesh_protocol.mission", "--fake", "--speed", str(ROVER_SPEED))
        ok = wait_for(lambda: age("acks") is not None and age("acks") > 4, 60)
        with lock:
            x = (st["mission"] or {}).get("x")
        check("base A lost the rover inside the crater", ok and x is not None and -CRATER_RADIUS < x < CRATER_RADIUS, f"(x = {x} m)")
        time.sleep(3)
        check("mesh stays OFF: nothing deployed", coord("state") == "STANDBY" and not coord("relay"))
        check("base B still receives nothing", len(st["odoms"]) == 0)
        check("DTN: odometry is held during the outage", wait_for(lambda: st["max_held"] > 5, 5),
              f"(max held {st['max_held']})")
        check("DTN: old commands expired instead of being sent", total("expired") > 0, f"({total('expired')})")

        print("\nTest 3: toggle ON")
        t_on = time.time()
        s.put(KEY_COORD_CMD, "on")
        ok = wait_for(lambda: coord("state") == "DEPLOYED", 5)
        ch = coord("chains") or {}
        used = sorted(ch.get("base_a", []) + ch.get("base_b", []))
        check("drones deployed straight away", ok, f"({time.time() - t_on:.1f} s)")
        check("all 4 drones in the relay chains", used == DRONES, f"({ch})")
        check("base A connected again", wait_for(lambda: age("acks") is not None and age("acks") < 1.5, 8))
        check("base B receives odometry", wait_for(lambda: age("odoms") is not None and age("odoms") < 1.5, 8))
        time.sleep(3)
        with lock:
            n_late = sum(1 for m in st["odoms"] if m["late"])
        miss = missing_seqs()
        check("DTN: every odometry sample from the outage reached base B", miss == [] and n_late > 0,
              f"({n_late} late, missing {miss if miss is None else len(miss)})")

        print("\nTest 4: rover drives on to base B with the mesh ON")
        t_flow = time.time()
        ok = wait_for(lambda: mission.poll() is not None, 150)
        t_end = time.time()
        with lock:
            mis = st["mission"] or {}
        check("rover arrived next to base B", ok and mis.get("state") == "ARRIVED", f"(x = {mis.get('x')} m)")
        g_b = longest_gap(times("odoms", t_flow), t_flow, t_end)
        g_a = longest_gap(times("acks", t_flow), t_flow, t_end)
        check("odometry reached base B the whole way", g_b < MAX_GAP, f"(longest gap {g_b:.1f} s)")
        check("base A kept its connection", g_a < 2.5,
              f"(longest ACK gap {g_a:.1f} s from t={gap_start(times('acks', t_flow), t_flow, t_end):.1f})")

        with lock:
            ages = [m.get("cmd_age_ms") or 0 for m in st["acks"]]
        check("DTN: the rover never applied a command older than 1 s", max(ages) <= 1000,
              f"(oldest {max(ages):.0f} ms)")

        print("\nTest 4b: 10 % of packets lost (custody transfer resends)")
        r0, t0 = total("retransmits"), time.time()
        s.put(KEY_FAIL_CONTROL, json.dumps({"drop_probability": 0.1}))
        time.sleep(20)
        s.put(KEY_FAIL_CONTROL, json.dumps({"drop_probability": 0.0}))
        t1 = time.time()
        time.sleep(4)
        miss = missing_seqs(t0, t1)
        check("odometry complete despite 10 % loss", miss == [], f"(missing {miss})")
        check("lost bundles were resent", total("retransmits") > r0, f"({total('retransmits') - r0} resends)")

        print("\nTest 4c: 20 % of packets corrupted")
        b0 = total("bad_crc")
        s.put(KEY_FAIL_CONTROL, json.dumps({"corrupt_probability": 0.2}))
        time.sleep(15)
        s.put(KEY_FAIL_CONTROL, json.dumps({"corrupt_probability": 0.0}))
        time.sleep(2)
        check("corrupted packets caught by the CRC", total("bad_crc") > b0, f"({total('bad_crc') - b0})")
        check("no program crashed under corruption", all(p.poll() is None for p in procs if p is not mission))

        print("\nTest 4d: kill the leader drone")
        old = coord("leader")
        t_kill = time.time()
        s.put(KEY_FAIL_CONTROL, json.dumps({"kill": old}))
        ok = wait_for(lambda: coord("leader") not in (None, old), 8)
        took = time.time() - t_kill
        check("a new leader took over within 3 s", ok and took <= 3.0, f"({old} -> {coord('leader')}, {took:.1f} s)")
        check("the new leader kept the mesh ON", coord("mode") == "on")
        check("both bases connected again within 5 s of the takeover",
              wait_for(lambda: age("acks") < 1.5 and age("odoms") < 1.5, 5))

        print("\nTest 4e: switch the mesh OFF while the leader is dead, then ON again")
        s.put(KEY_FAIL_CONTROL, json.dumps({"revive": old}))     # back to 4 drones, then lose the leader again
        time.sleep(3)
        old2 = coord("leader")
        s.put(KEY_FAIL_CONTROL, json.dumps({"kill": old2}))
        s.put(KEY_COORD_CMD, "off")
        check("mesh OFF reached the next leader", wait_for(lambda: coord("mode") == "off" and coord("leader") not in (None, old2), 10),
              f"(leader {coord('leader')})")
        s.put(KEY_COORD_CMD, "on")
        check("and ON again", wait_for(lambda: coord("mode") == "on" and coord("state") == "DEPLOYED", 8))
        s.put(KEY_FAIL_CONTROL, json.dumps({"revive": old2}))
        check("revived drones accept the current leader",
              wait_for(lambda: all(st["stats"].get(u, {}).get("leader") == coord("leader") for u in (old, old2)), 8))

        print("\nTest 4f: rover photo to base B with 10 % packet loss")
        wait_for(lambda: coord("state") == "DEPLOYED" and age("odoms") < 1.0, 10)
        t0 = time.time()
        s.put(KEY_FAIL_CONTROL, json.dumps({"drop_probability": 0.1}))
        s.put(KEY_PHOTO_CMD, json.dumps({"time": t0}))
        ok = wait_for(lambda: (st["photo"] or {}).get("done"), 90)
        t1 = time.time()
        s.put(KEY_FAIL_CONTROL, json.dumps({"drop_probability": 0.0}))
        ph = st["photo"] or {}
        check("photo arrived complete with a matching sha256", ok and ph.get("ok"),
              f"({ph.get('received')}/{ph.get('total')} chunks in {t1 - t0:.0f} s)")
        g_b = longest_gap(times("odoms", t0, t1), t0, t1)
        check("odometry kept flowing during the photo (priority)", g_b < MAX_GAP, f"(longest gap {g_b:.1f} s)")

        print("\nTest 5: second journey, mesh ON from the start")
        s.put(KEY_COORD_CMD, "off")
        wait_for(lambda: coord("state") == "STANDBY", 5)
        time.sleep(1)
        s.put(KEY_FAKE_CMD_VEL, json.dumps({"linear": 0, "angular": 0}))
        s.put(KEY_FAKE_POSE, json.dumps({"unit": "rover", "pos": [*START_XY["rover"], 0.0], "yaw": 0.0}))
        time.sleep(2)
        s.put(KEY_COORD_CMD, "on")
        wait_for(lambda: coord("state") == "DEPLOYED", 5)
        ch = coord("chains") or {}
        check("at the start: chains to both bases", "base_b" in ch and len(ch.get("base_b", [])) > 0, f"({ch})")
        # the drones fly there (DRONE_SPEED m/s): up to ~85 m from parking to the far end of the chain
        check("drones flew into place and base B receives odometry",
              wait_for(lambda: age("odoms") is not None and age("odoms") < 1.0, 20))
        t0 = time.time()
        # demo speed (1 m/s): with flying drones, 2.5 m/s outruns a 4-drone reshuffle, the demo never does
        mission2 = start("mesh_protocol.mission", "--fake", "--speed", str(ROVER_SPEED))
        ok = wait_for(lambda: mission2.poll() is not None, 200)
        t1 = time.time()
        with lock:
            mis = st["mission"] or {}
        check("rover arrived again", ok and mis.get("state") == "ARRIVED")
        g_b = longest_gap(times("odoms", t0), t0, t1)
        g_a = longest_gap(times("acks", t0), t0, t1)
        n_b = len(times("odoms", t0, t1))
        check("base B got odometry from start to end", g_b < MAX_GAP, f"({n_b} msgs, longest gap {g_b:.1f} s)")
        check("base A never lost the rover", g_a < 2.5, f"(longest ACK gap {g_a:.1f} s)")
        check("no program crashed", all(p.poll() is None for p in procs if p not in (mission, mission2)))

        print("\nTest 6: dashboard (web)")
        start("mesh_dashboard.server", "--port", "18080")
        base = "http://127.0.0.1:18080"

        def dash_snapshot():
            try:
                with urllib.request.urlopen(base + "/events", timeout=3) as r:
                    for line in r:
                        if line.startswith(b"data: "):
                            return json.loads(line[6:])
            except Exception:
                return None

        def dash_post(path, body):
            req = urllib.request.Request(base + path, data=json.dumps(body).encode(), method="POST",
                                         headers={"Content-Type": "application/json"})
            try:
                return urllib.request.urlopen(req, timeout=3).status
            except Exception:
                return None

        snap = None
        for _ in range(60):                       # it loads the terrain first: allow 30 s
            snap = dash_snapshot()
            if snap and snap.get("coord") and snap.get("stats"):     # counters come once a second
                break
            time.sleep(0.5)
        check("dashboard streams the mesh state", bool(snap and snap.get("coord") and snap.get("stats")),
              "(no snapshot from the dashboard)" if snap is None else
              f"(coord {bool(snap.get('coord'))}, stats {len(snap.get('stats') or {})})")
        ok_off = dash_post("/api/mesh", {"on": False}) == 200 and wait_for(lambda: coord("mode") == "off", 8)
        ok_on = dash_post("/api/mesh", {"on": True}) == 200 and wait_for(lambda: coord("mode") == "on", 8)
        check("dashboard toggle switched the mesh OFF and ON", ok_off and ok_on)
        s.close()
    finally:
        for p in procs:
            if p.poll() is None:
                p.terminate()
        for p in procs:
            try:
                p.wait(timeout=5)
            except subprocess.TimeoutExpired:
                p.kill()
        log.close()

    print(f"\n{sum(results)}/{len(results)} checks passed. Full log: /tmp/mesh_selftest.log")
    sys.exit(0 if results and all(results) else 1)


if __name__ == "__main__":
    main()
