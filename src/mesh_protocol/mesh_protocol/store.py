"""Bundle store: the bundles a node has custody of, until the next hop confirms it has them.

Pure logic, no Zenoh, so it can be tested on its own (test/test_store.py).
"""
from mesh_common import RTO_MAX, RTO_START, STORE_LIMIT, bundle_id, expired


class BundleStore:
    def __init__(self, limit=STORE_LIMIT):
        self.limit = limit
        self._b = {}            # bundle id -> {"pkt", "tries", "retry_at", "hop"}

    def __len__(self):
        return len(self._b)

    def add(self, pkt, now):
        """Take custody of pkt. If the store is full, returns the bundle that was dropped."""
        self._b[bundle_id(pkt)] = {"pkt": pkt, "tries": 0, "retry_at": now, "hop": None}
        if len(self._b) <= self.limit:
            return None
        # drop the oldest bundle of the lowest priority (largest number) present
        bid = max(self._b, key=lambda k: (self._b[k]["pkt"]["priority"], -self._b[k]["pkt"]["sent_time"]))
        return self._b.pop(bid)["pkt"]

    def due(self, now, next_hop_of, budget):
        """Bundles to (re)send now, as [(pkt, next_hop)], best priority first, at most `budget`.

        next_hop_of(dst) -> unit or None. None (or a hop the bundle already passed) = hold it.
        """
        out = []
        for e in sorted(self._b.values(), key=lambda e: (e["pkt"]["priority"], e["pkt"]["sent_time"])):
            if len(out) >= budget:
                break
            if e["retry_at"] > now:
                continue
            nh = next_hop_of(e["pkt"]["dst"])
            if nh is None or nh in e["pkt"]["path"]:
                continue
            if nh != e["hop"]:                   # new route: start the back-off again
                e["hop"], e["tries"] = nh, 0
            e["tries"] += 1
            e["retry_at"] = now + min(RTO_START * 2 ** (e["tries"] - 1), RTO_MAX)
            out.append((e["pkt"], nh))
        return out

    def tries(self, bid):
        e = self._b.get(bid)
        return e["tries"] if e else 0

    def ack(self, bid):
        """The next hop confirmed custody: forget the bundle. True if we held it."""
        return self._b.pop(bid, None) is not None

    def expire(self, now):
        """Remove and return every bundle past its lifetime."""
        old = [k for k, e in self._b.items() if expired(e["pkt"], now)]
        return [self._b.pop(k)["pkt"] for k in old]

    def held(self):
        n = {0: 0, 1: 0, 2: 0}
        for e in self._b.values():
            n[e["pkt"]["priority"]] += 1
        return n

    def kick(self, hop):
        """A link to `hop` came back (or routes changed): retry its bundles now."""
        for e in self._b.values():
            if hop is None or e["hop"] == hop:
                e["retry_at"] = 0.0
