import base64
import os
import random

from mesh_protocol.photo import Reassembler, chunks

DATA = os.urandom(5000)


def test_chunk_sizes():
    c = chunks(DATA, 7, size=1024)
    assert len(c) == 5 and all(p["total"] == 5 and p["photo_id"] == 7 for p in c)
    assert len(base64.b64decode(c[0]["data"])) == 1024


def test_roundtrip_any_order():
    c = chunks(DATA, 1)
    random.Random(3).shuffle(c)
    r = Reassembler()
    for p in c[:-1]:
        assert r.add(p)["done"] is False
    out = r.add(c[-1])
    assert out["done"] and out["ok"] and base64.b64decode(out["image_b64"]) == DATA


def test_bad_chunk_not_ok():
    c = chunks(DATA, 2)
    c[1] = dict(c[1], data=base64.b64encode(b"x" * 1024).decode())
    r = Reassembler()
    for p in c:
        out = r.add(p)
    assert out["done"] is True and out["ok"] is False


def test_progress_counts_duplicates_once():
    c = chunks(DATA, 3)
    r = Reassembler()
    r.add(c[0])
    assert r.add(c[0])["received"] == 1


def test_new_photo_resets():
    r = Reassembler()
    r.add(chunks(DATA, 4)[0])
    assert r.add(chunks(b"abc", 5)[0])["photo_id"] == 5


def test_rover_sends_photo_on_request_and_base_rebuilds_it():
    import time
    from mesh_common import decode, encode, make_packet
    from mesh_protocol.photo import photo_bytes
    from test_node_logic import node

    rover, _ = node("rover")
    req = make_packet("base", "rover", 1, "PHOTO_REQ", {}, next_hop="rover")
    rover.handle(decode(encode(req))[0], time.time())
    held = [p for p, _ in rover.store.due(time.time(), lambda d: "base", 1000) if p["type"] == "PHOTO_CHUNK"]
    assert held and all(p["priority"] == 2 for p in held)

    base, s = node("base")
    for p in held:
        q = dict(p, next_hop="base")
        base.handle(decode(encode(q))[0], time.time())
    final = s.published("mesh/base/photo")[-1]
    assert final["done"] and final["ok"] and base64.b64decode(final["image_b64"]) == photo_bytes()


def test_late_chunk_of_older_photo_does_not_wipe_newer():
    """Visitor double-clicks: photo #2 starts while a resend of a photo #1 chunk is still on its way."""
    r = Reassembler()
    new = chunks(DATA, 2)
    r.add(new[0])
    r.add(chunks(DATA, 1)[3])                             # late chunk of the older photo
    for p in new[1:]:
        out = r.add(p)
    assert out["done"] and out["ok"] and out["photo_id"] == 2


def test_partial_reassembler_gives_the_gap_free_start():
    import base64
    from mesh_protocol.photo import Reassembler, chunks
    data = bytes(range(256)) * 12
    parts = chunks(data, 1)
    r = Reassembler(partial=True)
    assert base64.b64decode(r.add(parts[0])["partial_b64"]) == data[:1024]
    assert base64.b64decode(r.add(parts[2])["partial_b64"]) == data[:1024]       # chunk 1 missing: still just the start
    done = r.add(parts[1])
    assert done["done"] and done["ok"] and "partial_b64" not in done
    assert "partial_b64" not in Reassembler().add(parts[0])                      # off by default
