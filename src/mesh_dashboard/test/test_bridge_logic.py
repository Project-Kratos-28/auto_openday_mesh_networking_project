from mesh_dashboard.bridge import delivery


def test_delivery_sums_counters_over_nodes():
    stats = {
        "base": {"by_type": {"COMMAND": {"sent": 10, "delivered": 0, "expired": 2}}},
        "drone_1": {"by_type": {"COMMAND": {"sent": 0, "delivered": 0, "expired": 1}}},
        "rover": {"by_type": {"COMMAND": {"sent": 0, "delivered": 7, "expired": 0},
                              "TELEMETRY": {"sent": 20, "delivered": 0, "expired": 0}}},
    }
    d = delivery(stats)
    assert d["COMMAND"] == {"sent": 10, "delivered": 7, "expired": 3}
    assert d["TELEMETRY"] == {"sent": 20, "delivered": 0, "expired": 0}


class _FakeSession:
    def __init__(self):
        self.puts = []

    def declare_subscriber(self, key, cb):
        return None

    def put(self, key, data):
        self.puts.append((key, data))


def test_phone_picture_goes_to_the_base_as_a_baseline_jpeg_for_the_rover():
    import io
    from PIL import Image
    from mesh_common import KEY_UPLINK
    from mesh_dashboard.bridge import Bridge
    s = _FakeSession()
    b = Bridge(session=s)
    buf = io.BytesIO()
    Image.new("RGB", (900, 600), (200, 80, 40)).save(buf, "PNG")
    b.phone_photo(buf.getvalue())
    key, data = s.puts[-1]
    assert key == KEY_UPLINK                                   # to the base, not to the rover directly
    img = Image.open(io.BytesIO(data))
    assert img.format == "JPEG" and max(img.size) <= 480
    assert "progressive" not in img.info and "progression" not in img.info    # baseline: partial data still shows
    sent = b.snapshot()["uplink_sent"]
    assert sent["bytes"] == len(data) and sent["chunks"] >= 1
    assert b.snapshot()["uplink"] is None


def test_simulation_restart_clears_the_whole_history():
    from mesh_dashboard.bridge import Bridge
    b = Bridge(session=_FakeSession())
    b._d["photo"] = {"photo_id": 1}
    b._d["stats"]["base"] = {"killed": False}
    b._odom.append({"seq": 1, "t": 0, "x": 1.0, "late": False})
    b._events.append({"event": "photo"})
    b._n_events = 7
    b.set_failure({"drop_probability": 0.2})
    first = b.snapshot()
    assert first["epoch"] == 0 and first["failure"]["drop_probability"] == 0.2

    b._on_reset(None)
    snap = b.snapshot()
    assert snap["epoch"] == first["epoch"] + 1 and snap["photo"] is None and snap["stats"] == {} and snap["odom"] == []
    assert snap["failure"]["drop_probability"] == 0.0 and snap["started"] > first["started"] - 1
    events, n = b.events(7)                                # an open event stream stays in step
    assert events == [] and n == 7


def test_a_finished_picture_is_sent_to_a_page_only_once():
    from mesh_dashboard.server import trim_repeats
    sent = {}
    snap = {"photo": {"photo_id": 7, "done": True, "image_b64": "AAAA"}, "uplink": {"photo_id": 9, "done": True, "image_b64": "BBBB"}}
    first = trim_repeats({k: dict(v) for k, v in snap.items()}, sent)
    assert first["photo"]["image_b64"] == "AAAA" and first["uplink"]["image_b64"] == "BBBB"
    again = trim_repeats({k: dict(v) for k, v in snap.items()}, sent)
    assert "image_b64" not in again["photo"] and again["photo"]["photo_id"] == 7 and again["photo"]["done"]
    assert "image_b64" not in again["uplink"]
    newer = {"photo": {"photo_id": 8, "done": True, "image_b64": "CCCC"}, "uplink": None}
    assert trim_repeats(newer, sent)["photo"]["image_b64"] == "CCCC"              # a new picture is sent again
    assert trim_repeats({"photo": None, "uplink": None}, {}) == {"photo": None, "uplink": None}
