"""STAND-IN failure injector, so the mesh works before the real one exists.

Copies mesh/<unit>/air -> mesh/<unit>/rx. By default it changes nothing.
For testing it can also drop packets or flip random bits:
    python3 -m mesh_link_model.passthrough_injector --corrupt 0.2 --drop 0.05
or live, from another terminal (Zenoh key mesh/failure/control):
    {"corrupt_probability": 0.3}
When the team's real failure_injection node is ready, launch with injector:=none.
"""
import argparse
import json
import random
import threading
import time

from mesh_common import UNITS, KEY_FAIL_CONTROL, KEY_FAIL_EVENTS, key_rx, open_session


def flip_bits(data, n):
    """Flip n random bits in the raw bytes (like radio noise)."""
    b = bytearray(data)
    for _ in range(n):
        i = random.randrange(len(b) * 8)
        b[i // 8] ^= 1 << (i % 8)
    return bytes(b)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--corrupt", type=float, default=0.0, help="chance a packet gets bit errors")
    ap.add_argument("--drop", type=float, default=0.0, help="chance a packet disappears")
    ap.add_argument("--bits", type=int, nargs=2, default=[1, 8], help="min max bits to flip")
    args, _ = ap.parse_known_args()
    cfg = {"corrupt_probability": args.corrupt, "drop_probability": args.drop, "bit_flips": args.bits}
    lock = threading.Lock()
    counts = {"passed": 0, "corrupted": 0, "dropped": 0}
    session = open_session()
    pubs = {u: session.declare_publisher(key_rx(u)) for u in UNITS}
    events = session.declare_publisher(KEY_FAIL_EVENTS)

    def on_air(sample):
        unit = str(sample.key_expr).split("/")[1]
        if unit not in pubs:
            return
        data = sample.payload.to_bytes()
        with lock:
            c = dict(cfg)
        if random.random() < c["drop_probability"]:
            with lock:
                counts["dropped"] += 1
            events.put(json.dumps({"time": time.time(), "event": "drop", "unit": unit}))
            return
        if random.random() < c["corrupt_probability"]:
            data = flip_bits(data, random.randint(*c["bit_flips"]))
            with lock:
                counts["corrupted"] += 1
            events.put(json.dumps({"time": time.time(), "event": "corrupt", "unit": unit}))
        else:
            with lock:
                counts["passed"] += 1
        pubs[unit].put(data)

    def on_control(sample):
        try:
            m = json.loads(sample.payload.to_bytes())
            with lock:
                cfg.update({k: m[k] for k in cfg if k in m})
            print(f"[injector] settings now {cfg}", flush=True)
        except Exception as e:
            print(f"[injector] bad control message: {e}", flush=True)

    subs = [session.declare_subscriber("mesh/*/air", on_air),          # noqa: F841 (keep alive)
            session.declare_subscriber(KEY_FAIL_CONTROL, on_control)]
    print(f"[injector] stand-in running, settings {cfg}", flush=True)
    try:
        while True:
            time.sleep(10)
            with lock:
                print(f"[injector] last 10 s: {counts}", flush=True)
                counts.update({"passed": 0, "corrupted": 0, "dropped": 0})
    except KeyboardInterrupt:
        pass
    finally:
        session.close()


if __name__ == "__main__":
    main()
