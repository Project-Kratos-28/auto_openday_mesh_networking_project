# Mesh v3: DTN protocol, coordinator on a drone, mission-control dashboard

Status: design, waiting for approval · 2026-10-28

## Why

The current mesh (v2) is "everyone shouts": drones re-broadcast every packet, there is no per-hop
confirmation, nothing is resent, a packet with no link is lost, and all traffic is treated the same.
The coordinator that places the drones is an all-seeing process outside the radio: it reads true
positions and its orders can never be lost. The base station GUI is a placeholder.

We want the mesh to work the way NASA keeps links to Mars reliable: **Delay/Disruption Tolerant
Networking (DTN)**. Every hop takes responsibility for a packet until the next hop confirms it, and
packets wait out an outage instead of being lost. The coordinator becomes a drone in the mesh, and
a mission-control web dashboard shows that all of this is really happening.

This is for an open-day demo: visitors watch a projector screen.

## Deadline and order

Open day is **2026-10-30**. v3 is built on its own branch; `main` keeps the working v2 demo until v3
passes the self-test. Build order, and the cut line if time runs out (each step leaves a working demo):

1. DTN protocol: custody, store, expiry, priority, routes (routes computed by the v2 coordinator
   process at first, sent in a `LEADER`-shaped bundle).
2. Web dashboard.
3. Coordinator on a drone (election), replacing the coordinator process.
4. Photo transfer.

The dashboard is built from scratch. The teammate's unmerged PyQt branch (`origin/Ronak_GUI`) is
dropped, by the team's decision.

## Goals

1. **Reliable hop-by-hop delivery (custody transfer).** A node keeps a packet until the next hop
   confirms it, and resends it otherwise.
2. **Store and forward.** With no usable link, packets are held and sent once a link returns.
3. **Expiry.** Every packet has a lifetime. Drive commands expire fast, so the rover never acts on
   an old order. Odometry lives long, so base B still gets the full track, late.
4. **Assigned routes.** Packets follow next hops chosen by the coordinator, not flooding.
5. **Priority.** Commands and control traffic go first, odometry next, bulk data (photo) last.
6. **Coordinator on a drone**, chosen by a simple leader election with a lease. Kill the leader
   drone and another one takes over. Its orders travel over the mesh like any other packet.
7. **Photo transfer.** The rover sends a "photo" to base B in chunks. The link drops partway through
   and the image still arrives intact.
8. **Mission-control web dashboard** that replaces the PyQt GUI and shows all of the above.

## Non-goals

- Wire compatibility with the real standards (Bundle Protocol RFC 9171, LTP, CFDP, BPSec). We copy
  the ideas, not the byte formats.
- Raft or any other consensus protocol. The coordinator's state can be recomputed from positions,
  so one leader at a time is enough.
- Clock sync. All processes run on one machine and share its clock.
- Drones actually flying. They are still teleported (each drone now moves itself).
- A phone view for visitors. The dashboard is for the projector only.
- A "start mission" button. The mission is still started from a terminal (`mission`).

## Anti-goals (a failure even if everything else works)

- The rover applies a drive command older than its lifetime.
- A corrupted or unexpected packet crashes any node.
- The dashboard needs internet access (venue Wi-Fi is not reliable): no CDN scripts or fonts.
- The demo stops working without Gazebo: fake mode (`fake:=true`) must keep working for everything.

## Constraints

- ROS 2 Jazzy, Gazebo Harmonic, Python 3.12, Zenoh, as now. No new Python dependencies.
- Keep the simulated radio as it is: `mesh/<u>/tx` → `radio_channel` → `mesh/<v>/air` → injector →
  `mesh/<v>/rx`. The radio still delivers a packet to every unit with an up link. Unicast is done
  by the receiver ignoring packets whose `next_hop` isn't itself.
- Shared names, constants, keys and the packet format stay in `mesh_common`.
- The self-test (`ros2 run mesh_protocol selftest`, fake mode) remains the main proof and must pass.

---

## Part 1: DTN protocol (`mesh_protocol/mesh_node.py`, `mesh_common`)

### Bundle format

The packet keeps its JSON + CRC32 form and gains three fields:

| Field | Meaning |
|---|---|
| `lifetime` | seconds after `sent_time` when the bundle expires |
| `priority` | `0` expedited, `1` normal, `2` bulk |
| `next_hop` | now a real unit name for routed bundles; `"*"` only for flooded control bundles and HELLO |

`ttl` stays as a hop-count safety limit (default 8). A bundle's id is `(src, type, seq)`, as now.

### Bundle types

| Type | From → to | How it travels | Priority | Lifetime |
|---|---|---|---|---|
| `HELLO` | every unit, 1/s | broadcast, TTL 1, never forwarded | – | – |
| `CUSTODY_ACK` | receiver → previous hop | unicast, one hop, never forwarded, no custody | 0 | 2 s |
| `COMMAND` | base A → rover (drive or ping) | routed, custody | 0 | **1.0 s** |
| `ACK` | rover → base A (end-to-end, as now) | routed, custody | 0 | 2 s |
| `TELEMETRY` | rover → base B, every 0.5 s | routed, custody | 1 | 120 s |
| `PHOTO_REQ` | base B → rover | routed, custody | 0 | 10 s |
| `PHOTO_CHUNK` | rover → base B | routed, custody | 2 | 300 s |
| `STATUS` | every unit → leader, every 0.5 s | flooded, duplicate filter | 0 | 2 s |
| `LEADER` | leader drone → all, every 0.5 s | flooded, duplicate filter | 0 | 2 s |
| `MESH_CMD` | base A → leader (`on`/`off`) | flooded, duplicate filter | 0 | 10 s |

The 1.0 s command lifetime is below the rover's 2 s watchdog, so an expired command can never keep
the rover driving.

**Flooded control bundles** (`STATUS`, `LEADER`, `MESH_CMD`) work like v2 flooding: duplicate filter,
TTL, re-broadcast by every unit that relays (see "who forwards"). They have no custody: they repeat
often, so a lost one is replaced by the next. Flooding them avoids needing routes to find the leader
before there are routes.

### Custody transfer (routed bundles)

Sending node:
1. Puts the bundle in its **store**, picks `next_hop` from the route table (below), sends it.
2. Waits for a `CUSTODY_ACK` from `next_hop` naming the bundle id.
3. No ack in time: resend. Timeout starts at 0.3 s and doubles each try, capped at 2 s.
4. After 5 tries to the same next hop, it looks the route up again (the table may have changed). If
   there is no usable next hop, the bundle stays in the store and is retried when a new `LEADER`
   route table arrives or the next hop's HELLO is heard again.
5. On `CUSTODY_ACK`: removes the bundle from its store. Custody has moved on.

Receiving node (bundle with `next_hop == me`, CRC good):
1. Always sends `CUSTODY_ACK` to the previous hop (the last entry in `path`), **even for duplicates**,
   so the sender stops resending.
2. Expired? Drop it, count `expired`, emit an event. (An expired bundle is still acked: we don't want
   it resent.)
3. Duplicate (seen id)? Count `duplicates`, stop.
4. `dst == me`: deliver (as in v2). Otherwise add itself to `path`, lower `ttl`, take custody (put it
   in its own store) and forward as above.

Expiry is also checked whenever a bundle is taken from the store; expired bundles are dropped there too.

### Store and priority

- One store per node, ordered by priority, then age. The node sends at most `SEND_RATE` bundles per
  second (new constant in `mesh_common`, default 40) so priority is visible when a photo is moving.
- Bounded at `STORE_LIMIT` bundles (default 500). When full, drop the oldest lowest-priority bundle
  (count `dropped_full`).
- The store logic (insert, pop by priority, expiry, retry timers, custody ack) is a **pure class with
  no Zenoh**, so it can be tested with plain asserts.

### Routing

- The leader publishes a route table inside each `LEADER` bundle: `routes[unit][dst] = next_hop`.
- It computes the table from the link graph it builds from `STATUS` reports (each unit lists the
  neighbours it hears via HELLO, with link quality). Shortest path by hop count, ties broken by
  higher link quality.
- When the mesh is OFF, drones are left out of the graph as relays, so only direct links get routes.
- **Fallback** (no `LEADER` yet, or no entry for `dst`): if `dst` is a current neighbour, send to it
  directly; otherwise hold the bundle in the store.
- **Who forwards:** any unit the route table names as a next hop. In practice the leader only routes
  through drones and, if needed, the rover (the rover links the two chains). For flooded control
  bundles: every drone forwards always (even when the mesh is OFF, parked drones are near base A and
  must keep the election working), bases and rover never.

### Delivery and what the endpoints publish

As in v2, plus:
- Base B publishes odometry on `mesh/base_b/telemetry` with `late: true` when it arrived more than
  1 s after `sent_time`. The dashboard uses `seq`/`sent_time` to put late samples back in place.
- Each node publishes on `mesh/stats/<unit>` (1/s): v2 counters plus `held` (store size, by
  priority), `custody_sent`, `custody_acked`, `retransmits`, `expired`, `dropped_full`, `routes`
  (its current next hops), `leader`, `term`.
- Each node publishes notable events on `mesh/events` (new key): retransmit, expired, custody
  handed over, election, leader change, route change, photo progress. One JSON object each, with
  `time`, `unit`, `event`, and details.

`mesh/stats/*`, `mesh/events` and the endpoint feeds are **instrumentation**: they go straight to
Zenoh, not over the radio, the same as today. They let the dashboard see the true state.

### Photo transfer

- Trigger: dashboard → `mesh/base_b/photo_cmd` → base B sends `PHOTO_REQ` to the rover.
- Rover sends a bundled image file (`docs/images/relay_world_preview.png`, copied into the package)
  as `PHOTO_CHUNK` bundles: `{photo_id, index, total, sha256, data(base64, 1 KB raw)}`.
- Base B reassembles, publishes progress on `mesh/base_b/photo` (`received/total`), and when
  complete checks the sha256 and publishes the image (base64) with `ok: true/false`.

---

## Part 2: Coordinator on a drone

The separate `coordinator.py` process goes away. Every drone's node includes the coordinator logic,
active only while that drone is leader.

### Election (lease, lowest id wins)

- The leader floods `LEADER` every 0.5 s: `{term, leader, lease: 2.0, mode, routes, targets, chains}`.
- A drone that has heard no valid `LEADER` for the lease time (2 s) plus a random 0–0.5 s, and is the
  lowest-id drone among the drones it has heard `STATUS` from in the last 2 s, makes itself leader
  with `term + 1`.
- A leader steps down when it hears a `LEADER` with a higher term, or the same term from a lower id.
- A split mesh can briefly have two leaders, one per side. When they hear each other, the rule above
  leaves one.
- The leader's `mode` and `targets` are in every heartbeat, so a new leader inherits the mesh mode and
  the current drone positions.

### Planning and moving drones

- The leader plans with the existing planner (`mesh_link_model/planner.py`), the same check / replan
  rules as v2 (every 0.5 s, look-ahead 3 s, replan after 8 m), but using positions **reported in
  `STATUS`**, not true positions. The rover's speed comes from successive `STATUS` reports.
- It moves drones one at a time in the safe order v2 uses: it changes one drone's target in
  `targets`, and waits until that drone's `STATUS` shows it there (or 2 s) before the next.
- Each drone teleports **itself** when its target in the latest `LEADER` differs from where it is
  (`sim_io.teleport`). Drone nodes therefore need `--fake` in fake mode too (launch change).
- Mesh ON/OFF: the dashboard (or `mesh_ctl`) publishes on `mesh/coordinator/cmd`; base A's node turns
  that into a `MESH_CMD` bundle; the leader applies it. OFF parks the drones as now.
- Base A republishes the latest `LEADER` it has heard on `mesh/coordinator/state`, so the dashboard
  sees what base A knows.
- **The `auto` mode is dropped** (rarely used, not in the GUI). `mesh_ctl auto` prints that it's gone.

### Positions

Each unit knows only its own position (like GPS), from `mesh/positions` (simulation instrumentation),
and shares it in `STATUS`. Nothing in the protocol reads other units' true positions.

### Failure controls

`mesh/failure/control` gains `{"kill": "<unit>"}` and `{"revive": "<unit>"}`. A killed node goes
silent (sends and receives nothing, keeps running) until revived. The node handles this itself.
The existing `corrupt_probability` / `drop_probability` controls stay in the injector.

---

## Part 3: Mission-control web dashboard (new package `mesh_dashboard`)

Replaces `base_station_gui`, which is deleted. Look: dark mission-control style, as in the chosen
mockup (`.superpowers/brainstorm/*/content/dashboard-platform.html`, option A).

### Server

- Python standard library only: `http.server.ThreadingHTTPServer`.
- Zenoh side reuses the approach of `base_station_gui/mesh_bridge.py` (moved into the new package):
  subscribe to everything, keep a snapshot.
- `GET /` serves the page (HTML/CSS/JS files in the package, no build step, no external URLs).
- `GET /events`: Server-Sent Events, a snapshot JSON 4 times a second plus each `mesh/events` item.
- `POST /api/mesh` `{on: bool}`, `POST /api/failure` `{corrupt, drop, kill, revive}`,
  `POST /api/photo`.
- Binds to `127.0.0.1:8080` by default (the controls have no login); `--host` to change.
- Run: `ros2 run mesh_dashboard dashboard`, or `ros2 launch mesh_protocol mesh.launch.py
  dashboard:=true` (replaces `gui:=true`).

### Panels

1. **Top bar:** time since start, mesh ON/OFF toggle, leader drone, term, lease age.
2. **Map:** side profile along the route (x vs height, terrain from `Terrain`), all units, links
   coloured by state (up / down), the current COMMAND route and TELEMETRY route highlighted, and a
   "held" badge on any node holding bundles.
3. **Delivery:** per type (COMMAND, TELEMETRY, PHOTO): sent / delivered / expired, delivery %, and
   recent delay.
4. **Held per node:** bars of store size, coloured by priority.
5. **Odometry at base B:** the rover's track; samples that arrived late are drawn in a different
   colour, so a gap visibly fills in.
6. **Event log:** scrolling list from `mesh/events` (retransmits, expiries, custody hand-overs,
   elections, route changes).
7. **Photo:** "take photo" button, progress bar, the image when complete, sha256 OK / FAIL.
8. **Failure controls:** corrupt %, drop %, kill / revive a drone (default target: current leader).

Charts are hand-drawn SVG; no chart library.

---

## Other changes

- `launch/mesh.launch.py`: no coordinator process; `--fake` passed to drones too; `dashboard:=true`
  replaces `gui:=true`; `coordinator` argument removed; `mesh:=on` now sets the initial mode inside
  the drones (passed as a node argument).
- `tools.py`: `mesh_ctl` still publishes `on`/`off`; `mesh_monitor` shows the new counters and leader.
- `mission.py`: unchanged (it is the "operator" outside the mesh and may keep using true positions).
- README and `docs/RELAY_DEMO.md` updated for v3. `base_station_gui` removed from the README.

## Proof (self-test, fake mode)

Keep the existing checks that still apply, and add:

1. **Outage:** with the mesh OFF the rover drives into the crater, then the mesh goes ON: telemetry
   bundles generated during the outage were held (non-zero `held` seen) and **every** telemetry `seq`
   reaches base B, some marked `late`.
2. **Stale commands:** no command older than 1.0 s is ever applied by the rover (rover logs the age of
   each applied command; the test reads it), and `expired` > 0 for COMMAND during the outage.
3. **Custody under loss:** with `drop_probability` 0.1, telemetry delivery stays 100% and
   `retransmits` > 0.
4. **Corruption:** with `corrupt_probability` 0.2, no node crashes.
5. **Leader takeover:** kill the leader: a new leader within 3 s, both chains working again within
   5 s, mesh mode kept.
6. **Photo:** with `drop_probability` 0.1 the photo arrives with a matching sha256.
7. **Dashboard:** server starts, `/events` streams a snapshot, `POST /api/mesh` switches the mesh.

Plus a small plain-assert test for the store class (priority order, expiry, retry back-off, custody
ack removes the bundle).

## Calls I made (not discussed; check these)

- Lifetimes, priorities, timeouts, `SEND_RATE` 40/s and `STORE_LIMIT` 500 as in the tables above.
- Control traffic (`STATUS`, `LEADER`, `MESH_CMD`) is flooded, not routed, so the election works
  before any routes exist.
- The rover may forward if the route table says so.
- The `auto` mesh mode is dropped.
- The photo is the existing `relay_world_preview.png` render.
- Stats and events go to the dashboard over plain Zenoh, outside the simulated radio
  (instrumentation, like now).
- The dashboard only listens on localhost.
- The leader takes the bases' positions from `START_XY` (bases never move, and base B can't be heard
  while the mesh is OFF).
- The election uses a short window: a drone counts drones heard in the last 2 s, so a takeover
  lands well under the 3 s test limit.

Checked by the reviewer (no change needed): 3 drones still find both chains along the whole route
after a leader is killed; a plan made from slightly old rover positions still works at mesh ON;
the 4 drones can always reach each other for control traffic; the photo (~105 KB, ~103 chunks)
fits in the store.

## Left to the builder

Exact JSON layouts of `STATUS`, `LEADER` and events beyond the fields named here; module split
inside `mesh_protocol`; dashboard styling details within the mockup's look; how the self-test stages
the outage.
