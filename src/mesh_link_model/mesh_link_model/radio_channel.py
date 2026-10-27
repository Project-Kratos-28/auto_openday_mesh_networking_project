"""Radio channel: the fake "air" between units.

A packet a unit sends on mesh/<unit>/tx is copied to mesh/<other>/air for every
unit whose link to the sender is UP (like a real radio: every neighbour hears it).
No link = the packet is simply lost. It never reads or changes the packet bytes.
"""
import argparse
import json
import threading
import time

from mesh_common import UNITS, KEY_LINKS, key_air, open_session


def main():
    argparse.ArgumentParser().parse_known_args()
    lock = threading.Lock()
    neighbours = {u: set() for u in UNITS}       # unit -> units it can reach right now
    counts = {"in": 0, "delivered": 0, "no_link": 0}
    session = open_session()
    pubs = {u: session.declare_publisher(key_air(u)) for u in UNITS}

    def on_links(sample):
        table = json.loads(sample.payload.to_bytes())["links"]
        new = {u: set() for u in UNITS}
        for l in table:
            if l["up"]:
                new[l["a"]].add(l["b"])
                new[l["b"]].add(l["a"])
        with lock:
            neighbours.clear()
            neighbours.update(new)

    def on_tx(sample):
        sender = str(sample.key_expr).split("/")[1]      # "mesh/rover/tx" -> "rover"
        if sender not in neighbours:
            return
        data = sample.payload.to_bytes()
        with lock:
            hearers = list(neighbours[sender])
            counts["in"] += 1
            counts["no_link"] += 0 if hearers else 1
            counts["delivered"] += len(hearers)
        for v in hearers:
            pubs[v].put(data)

    subs = [session.declare_subscriber(KEY_LINKS, on_links),      # noqa: F841 (keep alive)
            session.declare_subscriber("mesh/*/tx", on_tx)]
    print("[radio_channel] running: mesh/<unit>/tx -> mesh/<neighbour>/air", flush=True)
    try:
        while True:
            time.sleep(10)
            with lock:
                print(f"[radio_channel] last 10 s: {counts['in']} sent, {counts['delivered']} "
                      f"copies delivered, {counts['no_link']} heard by nobody", flush=True)
                counts.update({"in": 0, "delivered": 0, "no_link": 0})
    except KeyboardInterrupt:
        pass
    finally:
        session.close()


if __name__ == "__main__":
    main()
