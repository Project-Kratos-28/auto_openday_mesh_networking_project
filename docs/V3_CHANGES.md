# Mesh v3: what changes from v2, and where the work stands

Read this first if you're picking up v3. Full design (the why, every number, every decision):
[`docs/superpowers/specs/2026-10-28-dtn-mesh-design.md`](superpowers/specs/2026-10-28-dtn-mesh-design.md).

- **Branch:** `v3-dtn`. `main` keeps the working v2 demo until v3 passes the self-test.
- **Deadline:** open day, 2026-10-30.
- **Idea in one line:** v2 floods every packet and hopes. v3 works like NASA's Delay/Disruption
  Tolerant Networking (DTN): each hop keeps a packet until the next hop confirms it, packets wait out
  an outage, old commands expire, and the coordinator is a drone chosen by election.

## Status

Update this table when you finish or start a step. Steps are in build order. If time runs out,
stop after any step: each one leaves a working demo.

| # | Step | Status | Notes |
|---|---|---|---|
| 1 | DTN protocol: custody, store, expiry, priority, routes | done | routes computed by the old coordinator process (until step 3). Self-test 26/26 in fake mode |
| 2 | Web dashboard (`mesh_dashboard`), PyQt GUI removed | done | `ros2 run mesh_dashboard dashboard` → http://localhost:8080. Self-test 28/28 |
| 3 | Coordinator on a drone (election), old coordinator process removed | todo | |
| 4 | Photo transfer | todo | |
| 5 | README / RELAY_DEMO updated for v3 | todo | |

## v2 → v3 at a glance

| | v2 | v3 |
|---|---|---|
| Forwarding | drones re-broadcast everything (flooding) | routed: next hop from the leader's route table. Only control traffic (`STATUS`, `LEADER`, `MESH_CMD`) still floods |
| Reliability | none | custody transfer: hold until the next hop sends `CUSTODY_ACK`, resend after 0.3 s, doubling to 2 s |
| Outage | packet lost | packet held in the node's store, sent when a link or route comes back |
| Old data | TTL (hop limit) only | `lifetime` per packet: COMMAND 1 s, TELEMETRY 120 s, PHOTO_CHUNK 300 s. Expired = dropped |
| Priority | none | 0 expedited (commands, control), 1 normal (odometry), 2 bulk (photo). Send rate capped at 40/s per node |
| Coordinator | separate process, reads true positions, orders can't be lost | runs on the leader drone (lease election, lowest id wins), plans from positions reported over the mesh |
| Moving drones | coordinator teleports them | each drone teleports itself to its target from the `LEADER` bundle |
| Mesh modes | off / on / auto | off / on (`auto` removed) |
| Failure controls | corrupt / drop | + `kill` / `revive` a unit |
| GUI | PyQt placeholder (`base_station_gui`) | web dashboard (`mesh_dashboard`), `http://localhost:8080` |

## What changes, file by file

| Where | Change |
|---|---|
| `mesh_common` | bundle fields `lifetime`, `priority`, real `next_hop`; new types; constants `SEND_RATE`, `STORE_LIMIT`, lifetimes; new keys `mesh/events`, `mesh/base_b/photo`, `mesh/base_b/photo_cmd` |
| `mesh_protocol/mesh_node.py` | custody transfer, store, expiry, routing, `STATUS`; drones also run the election and leader logic; kill / revive |
| `mesh_protocol/` (new module) | the store as a pure class with a plain-assert test |
| `mesh_protocol/coordinator.py` | step 1: also publishes routes. Step 3: deleted (logic moves into the leader drone) |
| `mesh_protocol/launch/mesh.launch.py` | `dashboard:=true` replaces `gui:=true`; drones get `--fake` too; coordinator process removed in step 3 |
| `mesh_protocol/tools.py` | `mesh_ctl auto` removed; `mesh_monitor` shows new counters and the leader |
| `mesh_protocol/selftest.py` | new checks: outage, stale commands, 10 % loss, 20 % corruption, leader takeover, photo, dashboard |
| `mesh_dashboard` (new) | stdlib HTTP server + Server-Sent Events, offline HTML/JS/SVG, mission-control look |
| `base_station_gui` | deleted (the unmerged `origin/Ronak_GUI` branch is dropped too) |
| `mesh_link_model`, `mars_sim`, `mission.py` | unchanged |

## Zenoh keys: new or changed in v3

| Key | From → to | Contents |
|---|---|---|
| `mesh/events` | every node → dashboard | one event: retransmit, expired, custody, election, leader, route change, photo progress |
| `mesh/stats/<unit>` | node → dashboard | v2 counters + `held`, `custody_sent`, `custody_acked`, `retransmits`, `expired`, `dropped_full`, `routes`, `leader`, `term` |
| `mesh/coordinator/state` | step 1: coordinator process; step 3: base A (last `LEADER` it heard) | mode, routes, targets, chains, leader, term |
| `mesh/base_b/telemetry` | base B → dashboard | as v2 + `late: true` when it arrived > 1 s after sending |
| `mesh/base_b/photo_cmd` | dashboard → base B | take a photo |
| `mesh/base_b/photo` | base B → dashboard | progress, then the image (base64) + sha256 `ok` |
| `mesh/failure/control` | dashboard → injector, nodes | as v2 + `{"kill": unit}`, `{"revive": unit}` |

## How to check your work

```bash
ros2 run mesh_protocol selftest          # fake mode, no Gazebo; must end with all checks passed
```
