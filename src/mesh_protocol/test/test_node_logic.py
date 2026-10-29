"""Receive-path rules of MeshNode, driven without Zenoh (a fake session records publishes)."""
import json
import time

from mesh_common import encode, decode, make_packet, bundle_id
from mesh_protocol.mesh_node import MeshNode


class FakePub:
    def __init__(self, key, log):
        self.key, self.log = key, log

    def put(self, data):
        self.log.append((self.key, data))


class FakeSession:
    def __init__(self):
        self.log = []

    def declare_publisher(self, key):
        return FakePub(key, self.log)

    def declare_subscriber(self, key, cb):
        return None

    def put(self, key, data):
        self.log.append((key, data))

    def published(self, key):
        return [json.loads(d) for k, d in self.log if k == key]


def node(unit, fake=True):
    s = FakeSession()
    return MeshNode(unit, fake=fake, session=s), s


def telemetry(seq=1, next_hop="drone_1", path=("rover",), sent=None):
    p = make_packet("rover", "base", seq, "TELEMETRY", {"odom": None}, next_hop=next_hop)
    p["path"] = list(path)
    if sent is not None:
        p["sent_time"] = sent
    return decode(encode(p))[0]


def test_custody_ack_sent_on_receive():
    n, _ = node("drone_1")
    out = n.handle(telemetry(), time.time())
    assert len(out) == 1
    ack = out[0]
    assert ack["type"] == "CUSTODY_ACK" and ack["next_hop"] == "rover" and ack["dst"] == "rover"
    assert tuple(ack["payload"]["id"]) == ("rover", "TELEMETRY", 1)
    assert len(n.store) == 1
    held = n.store.due(time.time(), lambda d: "drone_2", 10)[0][0]
    assert held["path"] == ["rover", "drone_1"] and held["ttl"] == 7


def test_duplicate_reacked_delivered_once():
    n, s = node("base")
    p = telemetry(next_hop="base", path=("rover", "drone_3"))
    now = time.time()
    out1, out2 = n.handle(p, now), n.handle(p, now)
    assert [o["type"] for o in out1 + out2] == ["CUSTODY_ACK", "CUSTODY_ACK"]
    assert len(s.published("mesh/base/telemetry")) == 1
    assert n.stats["duplicates"] == 1


def test_late_odometry_marked():
    n, s = node("base")
    n.handle(telemetry(next_hop="base", sent=time.time() - 5), time.time())
    assert s.published("mesh/base/telemetry")[0]["late"] is True


def test_expired_command_not_applied():
    n, s = node("rover")
    p = make_packet("base", "rover", 1, "COMMAND", {"linear": 1.0}, next_hop="rover")
    p["sent_time"] = time.time() - 1.5
    out = n.handle(decode(encode(p))[0], time.time())
    assert [o["type"] for o in out] == ["CUSTODY_ACK"]
    assert n.stats["expired"] == 1
    assert s.published("mesh/fake/cmd_vel") == []


def test_fresh_command_applied_and_acked_with_age():
    n, s = node("rover")
    p = make_packet("base", "rover", 1, "COMMAND", {"linear": 1.0}, next_hop="rover")
    n.handle(decode(encode(p))[0], time.time())
    assert s.published("mesh/fake/cmd_vel")[0]["linear"] == 1.0
    ack = n.store.due(time.time(), lambda d: "base", 10)[0][0]
    assert ack["type"] == "ACK" and ack["payload"]["cmd_age_ms"] < 1000


def test_ignores_other_next_hop():
    n, _ = node("drone_2")
    assert n.handle(telemetry(next_hop="drone_1"), time.time()) == []
    assert len(n.store) == 0


def test_custody_ack_clears_store():
    n, _ = node("drone_1")
    n.handle(telemetry(), time.time())
    ack = make_packet("drone_2", "drone_1", 1, "CUSTODY_ACK", {"id": ["rover", "TELEMETRY", 1]},
                      next_hop="drone_1")
    n.handle(decode(encode(ack))[0], time.time())
    assert len(n.store) == 0 and n.stats["custody_acked"] == 1


def test_tick_sends_held_bundle_to_route():
    n, _ = node("drone_1")
    n.handle(telemetry(), time.time())
    n.routes = {"base": "drone_2"}
    n.neighbours["drone_2"] = time.time() + 1
    out = n.tick(time.time() + 1)
    assert [(o["type"], o["next_hop"]) for o in out] == [("TELEMETRY", "drone_2")]


def test_tick_holds_without_route():
    n, _ = node("drone_1")
    n.handle(telemetry(), time.time())
    assert n.tick(time.time() + 1) == []
    assert len(n.store) == 1


def test_killed_node_ignores_everything():
    n, _ = node("drone_1")
    n.killed = True
    assert n.handle(telemetry(), time.time()) == []
    assert n.tick(time.time()) == []


def test_counts_by_type():
    r, _ = node("rover")
    r.originate("base", "TELEMETRY", {})
    assert r.stats["by_type"]["TELEMETRY"]["sent"] == 1
    b, _ = node("base")
    b.handle(telemetry(next_hop="base"), time.time())
    assert b.stats["by_type"]["TELEMETRY"]["delivered"] == 1
    old = make_packet("base", "rover", 1, "COMMAND", {}, next_hop="rover")
    old["sent_time"] = time.time() - 5
    r.handle(decode(encode(old))[0], time.time())
    assert r.stats["by_type"]["COMMAND"]["expired"] == 1


def test_route_table_beats_stale_neighbour():
    n, _ = node("base")
    now = time.time()
    n.neighbours["rover"] = now - 1.0          # heard a second ago, but the link may be gone
    n.neighbours["drone_2"] = now - 0.2
    n.routes = {"rover": "drone_2"}
    assert n.next_hop("rover", now) == "drone_2"


def test_neighbour_used_when_no_route():
    n, _ = node("base")
    now = time.time()
    n.neighbours["rover"] = now - 0.2
    assert n.next_hop("rover", now) == "rover"


def test_route_through_silent_hop_is_held():
    n, _ = node("base")
    now = time.time()
    n.neighbours["drone_3"] = now - 5.0        # dead: no HELLO for 5 s
    n.routes = {"rover": "drone_3"}
    assert n.next_hop("rover", now) is None


def test_route_through_live_hop_used():
    n, _ = node("base")
    now = time.time()
    n.neighbours["drone_3"] = now - 0.2
    n.routes = {"rover": "drone_3"}
    assert n.next_hop("rover", now) == "drone_3"


def leader_flood(src="drone_1", seq=1):
    p = make_packet(src, "*", seq, "LEADER", {"leader": src, "term": 1, "routes": {}, "targets": {}})
    return decode(encode(p))[0]


def test_rover_passes_on_control_floods():
    """The rover links the two drone chains, so control traffic (one leader for everyone) must cross it."""
    n, _ = node("rover")
    out = n.handle(leader_flood(), time.time())
    assert [(o["type"], o["path"]) for o in out] == [("LEADER", ["drone_1", "rover"])]


def test_control_flood_passed_on_once():
    n, _ = node("base")
    p = leader_flood()
    assert len(n.handle(p, time.time())) == 1
    assert n.handle(p, time.time()) == []


def test_leader_with_bad_routes_is_ignored():
    n, _ = node("base")
    p = make_packet("drone_1", "*", 1, "LEADER", {"leader": "drone_1", "term": 1, "routes": {"base": ["x"]}})
    n.handle(decode(encode(p))[0], time.time())
    assert n.routes == {}
    n.routes = {"rover": "drone_1"}
    n.tick(time.time())                                   # must not raise


def test_status_with_bad_pos_is_ignored():
    n, _ = node("drone_2")
    p = make_packet("drone_1", "*", 1, "STATUS", {"pos": "here", "neighbours": 3})
    n.handle(decode(encode(p))[0], time.time())
    assert "drone_1" not in n.statuses


class _Sample:
    def __init__(self, m):
        self.payload = type("P", (), {"to_bytes": lambda s: json.dumps(m).encode()})()


def test_revived_leader_forgets_old_role_and_mode():
    n, _ = node("drone_1")
    n.election.is_leader, n.election.term, n.election.leader = True, 3, "drone_1"
    n.coord.set_mode("on")
    n.on_failure(_Sample({"kill": "drone_1"}))
    assert n.killed and not n.election.is_leader and n.election.leader is None
    n.coord.set_mode("on")                              # (changed while it was dead: nothing reaches it)
    n.on_failure(_Sample({"revive": "drone_1"}))
    assert not n.killed and not n.election.is_leader and n.election.term == 0
    assert n.coord.mode == "off"                        # back to the start mode, not the old ON
    assert n.election.tick(time.time()) is False        # must wait a full lease before leading again


def test_killed_node_makes_no_new_bundles():
    n, _ = node("rover")
    n.killed = True
    assert n.originate("base", "TELEMETRY", {}) is None
    assert len(n.store) == 0


def test_restarted_unit_does_not_reuse_old_sequence_numbers():
    n, _ = node("rover")
    pkt = n.originate("base", "TELEMETRY", {})
    assert pkt["seq"] > 10**9                           # starts from the clock, not from 1


def test_rover_ignores_nan_and_clamps_speed():
    n, _ = node("rover")
    driven = []
    n.drive = lambda lin, ang: driven.append((lin, ang))
    for lin, ang in ((float("nan"), 0.0), (99.0, -99.0), ("x", 1)):
        cmd = make_packet("base", "rover", 1, "COMMAND", {"linear": lin, "angular": ang}, next_hop="rover")
        n.deliver(decode(encode(cmd))[0], time.time())
    assert len(driven) == 3 and all(abs(l) <= 2.0 and abs(a) <= 3.0 for l, a in driven)


def test_base_asks_again_when_the_leader_it_follows_is_in_another_mode():
    n, _ = node("base")
    n.on_mesh_request(type("S", (), {"payload": type("P", (), {"to_string": lambda s: "on"})()})())
    assert n.want_mode == "on"
    n._apply_leader({"mode": "on", "routes": {}, "targets": {}}, time.time())
    assert n.want_mode is None                          # a leader shows ON: stop asking
    n._apply_leader({"mode": "off", "routes": {}, "targets": {}}, time.time())
    assert n.want_mode == "on"                          # another leader (after a split) is OFF: ask again
