"""Route table: for every unit, the next hop towards every destination it can reach.

Shortest path by hop count, ties broken by higher total link quality. Only units in `relays`
may be in the middle of a path. Pure logic (test/test_routing.py).
"""
import heapq


def compute_routes(edges, units, relays):
    """edges: {(a, b): quality} for up links (either order). Returns {unit: {dst: next_hop}}."""
    nb = {u: {} for u in units}
    for (a, b), q in edges.items():
        if a in nb and b in nb:
            nb[a][b] = nb[b][a] = q
    relays = set(relays)
    routes = {}
    for src in units:
        best, first, todo = {src: (0, 0.0)}, {}, [(0, 0.0, src, None)]
        while todo:
            hops, negq, u, fh = heapq.heappop(todo)
            if best.get(u, (hops, negq)) < (hops, negq):
                continue                              # stale entry
            if u != src and u not in relays:
                continue                              # can be reached, but can't pass traffic on
            for v, q in nb[u].items():
                cost = (hops + 1, negq - q)
                if v not in best or cost < best[v]:
                    best[v] = cost
                    first[v] = fh or v
                    heapq.heappush(todo, (*cost, v, fh or v))
        routes[src] = first
    return routes
