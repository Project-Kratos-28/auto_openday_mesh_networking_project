from mesh_common import LIFETIME, decode, encode, expired, make_packet, bundle_id


def test_roundtrip_v3_fields():
    p = make_packet("rover", "base_b", 3, "TELEMETRY", {"x": 1}, next_hop="drone_1")
    q, err = decode(encode(p))
    assert err is None
    assert q["lifetime"] == LIFETIME["TELEMETRY"] and q["priority"] == 1 and q["next_hop"] == "drone_1"
    assert bundle_id(q) == ("rover", "TELEMETRY", 3)


def test_decode_rejects_missing_lifetime():
    p = make_packet("rover", "base_b", 3, "TELEMETRY", {})
    del p["lifetime"]
    assert decode(encode(p)) == (None, "bad_format")


def test_expired():
    p = make_packet("base_a", "rover", 1, "COMMAND", {})
    p["sent_time"], p["lifetime"] = 100.0, 1
    assert expired(p, 100.9) is False
    assert expired(p, 101.1) is True


def test_command_lifetime_below_watchdog():
    assert LIFETIME["COMMAND"] < 2.0
