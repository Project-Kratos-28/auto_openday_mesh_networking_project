from mesh_protocol.routing import compute_routes

UNITS = ["rover", "drone_1", "drone_2", "drone_3", "base_a", "base_b"]
DRONES = ["drone_1", "drone_2", "drone_3"]


def test_route_chain():
    edges = {("rover", "drone_1"): 0.5, ("drone_1", "drone_2"): 0.5, ("drone_2", "base_a"): 0.5}
    r = compute_routes(edges, UNITS, DRONES)
    assert r["base_a"]["rover"] == "drone_2"
    assert r["rover"]["base_a"] == "drone_1"
    assert r["drone_1"]["base_a"] == "drone_2"


def test_route_not_through_non_relay():
    edges = {("rover", "base_b"): 0.5, ("base_b", "base_a"): 0.5}
    r = compute_routes(edges, UNITS, DRONES)
    assert "base_a" not in r["rover"]
    assert r["rover"]["base_b"] == "base_b"


def test_route_prefers_quality():
    edges = {("rover", "drone_1"): 0.2, ("drone_1", "base_a"): 0.2,
             ("rover", "drone_2"): 0.8, ("drone_2", "base_a"): 0.8}
    assert compute_routes(edges, UNITS, DRONES)["rover"]["base_a"] == "drone_2"


def test_route_prefers_fewer_hops():
    edges = {("rover", "drone_1"): 0.9, ("drone_1", "drone_2"): 0.9, ("drone_2", "base_a"): 0.9,
             ("rover", "drone_3"): 0.1, ("drone_3", "base_a"): 0.1}
    assert compute_routes(edges, UNITS, DRONES)["rover"]["base_a"] == "drone_3"
