"""Relay planner: where should the drones go so the rover reaches BOTH bases?

Idea (simple, no fancy maths):
  * For one base, try 0, 1, 2 ... drones spaced evenly on the straight line rover -> base.
    Each drone hovers H metres above the highest ground near its spot. If any hop is too
    long or blocked by terrain, try higher (RELAY_HEIGHTS in mesh_common), then one more drone.
  * Do that for base A and base B. That gives the minimum number of drones per chain.
  * Spare drones are added to the chain with the longest hop (shorter hops = stronger
    links), so all 4 drones always help.
"""
import math

from mesh_common import RADIO_RANGE, LOS_CLEARANCE, RELAY_HEIGHTS, antenna_height

HEIGHTS = RELAY_HEIGHTS


def link_ok(terrain, a, b, radio_range=RADIO_RANGE):
    """Would a radio link between antenna points a and b work?"""
    return math.dist(a, b) <= radio_range and terrain.line_of_sight(a, b, LOS_CLEARANCE)


def place_chain(terrain, start, end, k, radio_range=RADIO_RANGE):
    """k drone positions (model origins) between antenna points start and end, or None."""
    for h in HEIGHTS:
        drones = []
        for i in range(1, k + 1):
            f = i / (k + 1)
            x = start[0] + f * (end[0] - start[0])
            y = start[1] + f * (end[1] - start[1])
            half = 0.5 / (k + 1)                       # look at the ground around this spot
            ground = max(terrain.ground(start[0] + g * (end[0] - start[0]),
                                        start[1] + g * (end[1] - start[1]))
                         for g in [f - half + j * (2 * half) / 8 for j in range(9)])
            drones.append((round(x, 1), round(y, 1), round(ground + h, 1)))
        nodes = [start] + [(x, y, zz + antenna_height("drone_1")) for x, y, zz in drones] + [end]
        if all(link_ok(terrain, nodes[j], nodes[j + 1], radio_range) for j in range(len(nodes) - 1)):
            return drones
    return None


def antenna(unit, pos):
    return (pos[0], pos[1], pos[2] + antenna_height(unit))


def min_chain(terrain, rover_ant, base_ant, max_k, radio_range=RADIO_RANGE):
    for k in range(0, max_k + 1):
        d = place_chain(terrain, rover_ant, base_ant, k, radio_range)
        if d is not None:
            return k, d
    return None, None


def plan(terrain, positions, drones, bases=("base_a", "base_b"), radio_range=RADIO_RANGE):
    """Return ({drone: (x, y, z)}, {base: [drones in its chain, rover side first]}) or (None, reason)."""
    r = antenna("rover", positions["rover"])
    ends = {b: antenna(b, positions[b]) for b in bases}
    need = {}
    for b in bases:
        k, d = min_chain(terrain, r, ends[b], len(drones), radio_range)
        if k is None:
            return None, f"no chain to {b} possible even with {len(drones)} drones"
        need[b] = [k, d]
    if sum(v[0] for v in need.values()) > len(drones):
        return None, f"needs {sum(v[0] for v in need.values())} drones, only {len(drones)}"
    spare = len(drones) - sum(v[0] for v in need.values())
    while spare > 0:                                     # spare drone -> chain with longest hop
        def longest_hop(b):
            k, d = need[b]
            pts = [r] + [(x, y, zz + 0.3) for x, y, zz in d] + [ends[b]]
            return max(math.dist(pts[i], pts[i + 1]) for i in range(len(pts) - 1))
        for b in sorted(bases, key=longest_hop, reverse=True):
            d = place_chain(terrain, r, ends[b], need[b][0] + 1, radio_range)
            if d is not None:
                need[b] = [need[b][0] + 1, d]
                break
        else:
            break
        spare -= 1
    targets, chains, free = {}, {}, list(drones)
    for b in bases:
        chains[b] = []
        for p in need[b][1]:
            name = free.pop(0)
            targets[name] = p
            chains[b].append(name)
    return targets, chains
