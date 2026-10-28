from mesh_common import make_packet, bundle_id
from mesh_protocol.store import BundleStore


def pkt(ptype, seq=1, t=0.0, dst="base_b"):
    p = make_packet("rover", dst, seq, ptype, {})
    p["sent_time"] = t
    return p


def hop(dst):
    return "drone_1"


def test_priority_order():
    s = BundleStore()
    for p in (pkt("TELEMETRY"), pkt("COMMAND"), pkt("PHOTO_CHUNK")):
        s.add(p, 0.0)
    assert [p["type"] for p, _ in s.due(0.0, hop, 10)] == ["COMMAND", "TELEMETRY", "PHOTO_CHUNK"]


def test_backoff():
    s = BundleStore()
    s.add(pkt("TELEMETRY"), 0.0)
    assert len(s.due(0.0, hop, 10)) == 1
    assert s.due(0.29, hop, 10) == []
    assert len(s.due(0.3, hop, 10)) == 1          # 2nd try, next wait 0.6
    assert s.due(0.89, hop, 10) == []
    assert len(s.due(0.9, hop, 10)) == 1          # 3rd, next 1.2
    assert len(s.due(2.1, hop, 10)) == 1          # 4th, next 2.4 -> capped at 2.0
    assert s.due(4.0, hop, 10) == []
    assert len(s.due(4.1, hop, 10)) == 1


def test_due_returns_next_hop():
    s = BundleStore()
    s.add(pkt("TELEMETRY"), 0.0)
    assert s.due(0.0, hop, 10)[0][1] == "drone_1"


def test_ack_removes():
    s = BundleStore()
    p = pkt("TELEMETRY")
    s.add(p, 0.0)
    assert s.ack(bundle_id(p)) is True
    assert len(s) == 0 and s.ack(bundle_id(p)) is False


def test_expire_removes():
    s = BundleStore()
    s.add(pkt("COMMAND", t=0.0), 0.0)
    assert [p["type"] for p in s.expire(1.1)] == ["COMMAND"]
    assert len(s) == 0


def test_no_route_holds():
    s = BundleStore()
    s.add(pkt("TELEMETRY"), 0.0)
    assert s.due(0.0, lambda d: None, 10) == []
    assert len(s) == 1


def test_never_sends_back_along_path():
    s = BundleStore()
    p = pkt("TELEMETRY")
    p["path"] = ["rover", "drone_1"]
    s.add(p, 0.0)
    assert s.due(0.0, hop, 10) == []


def test_full_store_drops_bulk_first():
    s = BundleStore(limit=3)
    s.add(pkt("PHOTO_CHUNK", 1, t=1.0), 0.0)
    s.add(pkt("PHOTO_CHUNK", 2, t=2.0), 0.0)
    s.add(pkt("TELEMETRY", 3), 0.0)
    dropped = s.add(pkt("COMMAND", 4), 0.0)
    assert dropped["type"] == "PHOTO_CHUNK" and dropped["seq"] == 1
    assert len(s) == 3


def test_full_store_drops_new_bundle_if_it_is_lowest():
    s = BundleStore(limit=1)
    s.add(pkt("COMMAND", 1), 0.0)
    dropped = s.add(pkt("PHOTO_CHUNK", 2), 0.0)
    assert dropped["seq"] == 2 and len(s) == 1


def test_budget():
    s = BundleStore()
    for i in range(10):
        s.add(pkt("TELEMETRY", i), 0.0)
    assert len(s.due(0.0, hop, 4)) == 4


def test_held_by_priority():
    s = BundleStore()
    s.add(pkt("COMMAND", 1), 0.0)
    s.add(pkt("TELEMETRY", 2), 0.0)
    s.add(pkt("TELEMETRY", 3), 0.0)
    assert s.held() == {0: 1, 1: 2, 2: 0}


def test_kick_makes_due_now():
    s = BundleStore()
    s.add(pkt("TELEMETRY"), 0.0)
    s.due(0.0, hop, 10)
    assert s.due(0.1, hop, 10) == []
    s.kick("drone_1")
    assert len(s.due(0.1, hop, 10)) == 1
