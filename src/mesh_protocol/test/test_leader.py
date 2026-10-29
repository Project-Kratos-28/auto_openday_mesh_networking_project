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
    assert "rover" not in out["routes"]["base"]             # rover in the crater, out of sight: no route



def test_coordinator_on_chains_the_base():
    c = Coordinator(TERRAIN, "on")
    out = c.update(statuses(0.0), 0.0)
    ch = out["chains"]
    assert sorted(ch["base"]) == DRONES
    assert out["state"] == "DEPLOYED"


def _settle(c, st, t0):
    """Run the leader with every drone arriving where it is sent. Returns the last payload."""
    out = None
    for i in range(12):
        out = c.update(st, t0 + i * 0.5)
        for d in DRONES:
            st[d]["pos"] = list(out["targets"][d])
    return out


def test_coordinator_skips_silent_drone():
    c = Coordinator(TERRAIN, "on")
    st = statuses(0.0)
    del st["drone_3"]                                   # no STATUS from it: dead or out of reach
    out = c.update(st, 0.0)
    used = out["chains"]["base"]
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
    hop, seen = "base", []
    while hop != "rover" and len(seen) < 6:             # walk base -> rover
        seen.append(hop)
        hop = r[hop]["rover"]
    assert hop == "rover" and all(h in DRONES for h in seen[1:])


def test_deploy_from_nothing_moves_all_drones_at_once():
    """Nothing connected yet = nothing to protect: all drones fly together."""
    c = Coordinator(TERRAIN, "on")
    out = c.update(statuses(0.0), 0.0)
    moved = [d for d in DRONES if abs(out["targets"][d][0] - START_XY[d][0]) > 0.5]
    assert len(moved) == 4


def test_drone_in_flight_routed_from_where_it_is():
    """A drone still on its way is not used as a relay at its target yet."""
    c = Coordinator(TERRAIN, "on")
    st = statuses(0.0)
    out = c.update(st, 0.0)                                   # all four told to fly, none moved yet
    r = out["routes"]
    assert "rover" not in r["base"]


def test_assign_spots_to_nearest_drones():
    """A re-plan must not send drones across the crater to swap places: shortest longest-flight wins."""
    from mesh_protocol.leader import assign
    spots = [(0.0, 0.0, 5.0), (20.0, 0.0, 5.0), (40.0, 0.0, 5.0)]
    here = {"drone_1": (41.0, 0.0, 5.0), "drone_2": (19.0, 0.0, 5.0), "drone_3": (1.0, 0.0, 5.0)}
    got = assign(spots, here)
    assert got == {"drone_3": spots[0], "drone_2": spots[1], "drone_1": spots[2]}


class FlatTerrain:
    """No hills: only the 20 m radio range decides links."""
    def ground(self, x, y):
        return 0.0

    def line_of_sight(self, a, b, clearance=0.5, step=1.0):
        return True


def test_moves_a_spare_drone_before_one_the_chain_needs():
    """While the chain works, drones move one at a time, and a drone in flight is treated as gone:
    first move one whose absence keeps the chain up."""
    c = Coordinator(FlatTerrain(), "on")
    pos = {"rover": (0.0, 0.0, 0.0), "base": (-40.0, 0.0, 0.0),
           "drone_1": (-26.0, 0.0, 3.0), "drone_2": (-13.0, 0.0, 3.0),     # the chain: base - d1 - d2 - rover
           "drone_3": (-30.0, 5.0, 3.0), "drone_4": (-45.0, 5.0, 3.0)}    # spares
    c.targets = {d: pos[d] for d in DRONES}
    c.goal = dict(c.targets, drone_2=(-12.0, 0.0, 3.0), drone_3=(-31.0, 5.0, 3.0))
    c.order, c.drones = ["drone_2", "drone_3"], list(DRONES)
    c._move(pos, 0.0)
    assert list(c.moving) == ["drone_3"]                     # drone_2 carries the link: not first


def test_no_reshuffle_while_the_chain_holds():
    """Flying costs a relay gap, so the leader only re-plans when the chain is about to break,
    not every time the rover has moved a few metres."""
    c = Coordinator(TERRAIN, "on")
    st = statuses(0.0)
    from mesh_protocol.leader import connected
    settled = _settle(c, st, 0.0)["targets"]
    pos = {**{u: tuple(v["pos"]) for u, v in st.items()}, **{d: tuple(p) for d, p in settled.items()}}
    x = next(x for x in (12.0, 11.0, 10.0, 9.0)         # > 8 m on, both chains still fine
             if connected(TERRAIN, dict(pos, rover=(x, 0.0, TERRAIN.ground(x, 0.0)))))
    st["rover"]["pos"] = [x, 0.0, TERRAIN.ground(x, 0.0)]
    out = c.update(st, 100.0)
    assert out["targets"] == settled


def test_mesh_off_sends_every_drone_home_at_once():
    """Parking one at a time cut the far drone off before its turn: it never heard 'fly home'."""
    c = Coordinator(TERRAIN, "on")
    st = statuses(5.0)
    _settle(c, st, 0.0)                                     # deployed, every drone at its relay spot
    c.set_mode("off")
    out = c.update(st, 100.0)
    assert out["state"] == "STANDBY"
    for d in DRONES:                                        # all in the FIRST heartbeat after OFF
        assert abs(out["targets"][d][0] - START_XY[d][0]) < 0.01, d


def test_new_leader_in_mesh_off_parks_a_drone_left_out():
    """A drone that ends up leading itself carries on from a half-parked plan: it must still go home."""
    c = Coordinator(TERRAIN, "off")
    relay_spot = [20.0, 0.0, 8.0]
    c.adopt({"mode": "off", "state": "STANDBY",
             "targets": {**{d: [*START_XY[d], 3.0] for d in DRONES}, "drone_4": relay_spot}})
    st = {"drone_4": {"pos": relay_spot, "time": 0.0}}         # alone: hears nobody else
    out = c.update(st, 0.0)
    assert abs(out["targets"]["drone_4"][0] - START_XY["drone_4"][0]) < 0.01
