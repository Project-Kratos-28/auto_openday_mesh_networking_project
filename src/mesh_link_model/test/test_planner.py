"""Step 1: with the 20 m radio, the rover needs 0, 1, 2, 3 drones as it drives from the base to the target."""
import pytest

from mesh_common import START_XY, TARGET_XY, DRONES
from mesh_link_model.planner import antenna, min_chain
from mesh_link_model.terrain import Terrain

TERRAIN = Terrain.load()
BASE = antenna("base", (*START_XY["base"], TERRAIN.ground(*START_XY["base"])))


@pytest.mark.parametrize("x, drones", [(-24.0, 0), (-12.0, 1), (0.0, 2), (TARGET_XY[0], 3)])
def test_drones_needed_along_the_route(x, drones):
    rover = antenna("rover", (x, 0.0, TERRAIN.ground(x, 0.0)))
    k, _ = min_chain(TERRAIN, rover, BASE, len(DRONES))
    assert k == drones
