import random

from mesh_common import DRONES, START_XY
from mesh_link_model.terrain import Terrain
from mesh_protocol.leader import Coordinator, Election

TERRAIN = Terrain.load()


def el(unit):
    return Election(unit, now=0.0, rng=random.Random(1))


def test_lowest_id_takes_over():
    d2, d3 = el("drone_2"), el("drone_3")
    for e in (d2, d3):
        e.on_status("drone_2", 2.0)
        e.on_status("drone_3", 2.0)
    assert d2.tick(2.6) is True and d2.is_leader and d2.term == 1
    assert d3.tick(2.6) is False and not d3.is_leader


def test_no_election_while_lease_valid():
    d2 = el("drone_2")
    d2.on_leader({"leader": "drone_1", "term": 4}, 10.0)
    assert d2.tick(11.0) is False and d2.leader == "drone_1" and d2.term == 4


def test_takeover_after_lease_runs_out():
    d2 = el("drone_2")
    d2.on_leader({"leader": "drone_1", "term": 4}, 10.0)
    d2.on_status("drone_2", 12.0)
    assert d2.tick(12.6) is True and d2.term == 5 and d2.leader == "drone_2"


def test_steps_down_on_higher_term():
    d1 = el("drone_1")
    d1.tick(3.0)
    assert d1.is_leader
    d1.on_leader({"leader": "drone_3", "term": 7}, 3.5)
    assert not d1.is_leader and d1.leader == "drone_3" and d1.term == 7


def test_two_leaders_converge():
    d1, d2 = el("drone_1"), el("drone_2")
    d1.term = d2.term = 2
    d1.tick(3.0), d2.tick(3.0)                       # both alone: both lead, term 3
    assert d1.is_leader and d2.is_leader and d1.term == d2.term == 3
    d1.on_leader({"leader": "drone_2", "term": 3}, 3.5)
    d2.on_leader({"leader": "drone_1", "term": 3}, 3.5)
    assert d1.is_leader and not d2.is_leader and d2.leader == "drone_1"


def test_ignores_older_leader():
    d2 = el("drone_2")
    d2.on_leader({"leader": "drone_3", "term": 5}, 1.0)
    d2.on_leader({"leader": "drone_1", "term": 4}, 1.5)
    assert d2.leader == "drone_3"


def statuses(rover_x):
    st = {u: {"pos": [*START_XY[u], TERRAIN.ground(*START_XY[u]) + (3.0 if u in DRONES else 0.0)], "time": 0.0}
          for u in START_XY}
    st["rover"]["pos"] = [rover_x, 0.0, TERRAIN.ground(rover_x, 0.0)]
    return st


def test_coordinator_off_parks_drones():
    c = Coordinator(TERRAIN, "off")
    out = c.update(statuses(0.0), 0.0)
    for d in DRONES:
        assert abs(out["targets"][d][0] - START_XY[d][0]) < 0.01
    # mesh OFF: drones relay for nobody (a drone is only a next hop when it is the destination)
    assert all(nh == dst for r in out["routes"].values() for dst, nh in r.items() if nh in DRONES)
    assert "rover" not in out["routes"]["base_a"]             # rover in the crater, out of sight: no route



def test_coordinator_on_chains_both_bases():
    c = Coordinator(TERRAIN, "on")
    out = c.update(statuses(0.0), 0.0)
    ch = out["chains"]
    assert ch["base_a"] and ch["base_b"]
    assert sorted(ch["base_a"] + ch["base_b"]) == DRONES
    assert out["state"] == "DEPLOYED"


def test_coordinator_moves_one_drone_at_a_time():
    c = Coordinator(TERRAIN, "on")
    st = statuses(0.0)
    out = c.update(st, 0.0)
    moved = [d for d in DRONES if abs(out["targets"][d][0] - START_XY[d][0]) > 0.5]
    assert len(moved) == 1
    st[moved[0]]["pos"] = list(out["targets"][moved[0]])     # it got there
    out = c.update(st, 0.5)
    assert len([d for d in DRONES if abs(out["targets"][d][0] - START_XY[d][0]) > 0.5]) == 2


def test_coordinator_skips_silent_drone():
    c = Coordinator(TERRAIN, "on")
    st = statuses(0.0)
    del st["drone_3"]                                   # no STATUS from it: dead or out of reach
    out = c.update(st, 0.0)
    used = out["chains"]["base_a"] + out["chains"]["base_b"]
    assert "drone_3" not in used and len(used) == 3
    assert all("drone_3" not in r.values() for r in out["routes"].values())


def test_routes_follow_drone_targets_at_once():
    """Routes come from where drones were TOLD to be (line of sight + range on the terrain map),
    so they are right the moment a move is ordered, not 1-2 s later when neighbour lists catch up."""
    c = Coordinator(TERRAIN, "on")
    st = statuses(0.0)                                  # no neighbour lists reported at all
    for _ in range(8):                                  # let every drone get its target
        out = c.update(st, 0.0)
        for d in DRONES:
            st[d]["pos"] = list(out["targets"][d])
    r = out["routes"]
    hop, seen = "base_a", []
    while hop != "rover" and len(seen) < 6:             # walk base_a -> rover
        seen.append(hop)
        hop = r[hop]["rover"]
    assert hop == "rover" and all(h in DRONES for h in seen[1:])
