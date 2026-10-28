from mesh_dashboard.bridge import delivery


def test_delivery_sums_counters_over_nodes():
    stats = {
        "base_a": {"by_type": {"COMMAND": {"sent": 10, "delivered": 0, "expired": 2}}},
        "drone_1": {"by_type": {"COMMAND": {"sent": 0, "delivered": 0, "expired": 1}}},
        "rover": {"by_type": {"COMMAND": {"sent": 0, "delivered": 7, "expired": 0},
                              "TELEMETRY": {"sent": 20, "delivered": 0, "expired": 0}}},
        "base_b": {},
    }
    d = delivery(stats)
    assert d["COMMAND"] == {"sent": 10, "delivered": 7, "expired": 3}
    assert d["TELEMETRY"] == {"sent": 20, "delivered": 0, "expired": 0}
