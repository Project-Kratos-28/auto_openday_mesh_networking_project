# Mesh v3: what changes from v2, and where the work stands

Read this first if you're picking up v3. Full design (the why, every number, every decision):
[`docs/superpowers/specs/2026-10-28-dtn-mesh-design.md`](superpowers/specs/2026-10-28-dtn-mesh-design.md).
Ideas for after the demo: [`OPTIMIZATIONS.md`](OPTIMIZATIONS.md).

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
| 3 | Coordinator on a drone (election), old coordinator process removed | done | Self-test 34/34 incl. leader kill + takeover |
| 4 | Photo transfer | done | 103 chunks under 10 % loss, sha256 OK. Self-test 36/36 |
| 5 | README / RELAY_DEMO updated for v3 | done | |
| 5b | Smooth drone flight (glide instead of jump) | done | `mesh_protocol/mover.py`. Self-test 37/37 at demo speed |
| 6 | Run on Ubuntu 24.04 + Jazzy + Gazebo | done | build OK, 83/83 unit tests, self-test 37/37, headless Gazebo demo: rover arrived at base B. 4 drone nodes at once: ~18 `set_pose`/s each |
| 7 | Integrate the unpublished v2 work from `main` | done | mission false-assist fix, smooth flight (speed-up / slow-down, facing), missing-bindings warning, 2 new self-test checks. 86 unit tests, self-test 39/39 |

## v2 → v3 at a glance

| | v2 | v3 |
|---|---|---|
| Forwarding | drones re-broadcast everything (flooding) | routed: next hop from the leader's route table (or the destination itself if there is no route and it is in range). Never to a next hop not heard for 1.5 s. Control traffic (`STATUS`, `LEADER`, `MESH_CMD`) still floods, passed on by **every** unit |
| Reliability | none | custody transfer: hold until the next hop sends `CUSTODY_ACK`, resend after 0.3 s, doubling to 2 s |
| Outage | packet lost | packet held in the node's store, sent when a link or route comes back |
| Old data | TTL (hop limit) only | `lifetime` per packet: COMMAND 1 s, TELEMETRY 120 s, PHOTO_CHUNK 300 s. Expired = dropped |
| Priority | none | 0 expedited (commands, control), 1 normal (odometry), 2 bulk (photo). Send rate capped at 40/s per node |
| Coordinator | separate process, reads true positions, orders can't be lost | runs on the leader drone (lease election, lowest id wins, takeover ~2 s), plans from positions reported over the mesh; routes from those positions + line of sight on its terrain map |
| Moving drones | coordinator teleports them | each drone **glides** to its target from the `LEADER` bundle: up to 15 m/s, speeding up / slowing down at `DRONE_ACCEL` (10 m/s²), facing the way it flies, 20 steps a second, ≥ 2 m above ground, in a background thread; spots go to the nearest drones |
| Mesh modes | off / on / auto | off / on (`auto` removed) |
| Neighbours | HELLO 1 s, gone after 3 s | HELLO 0.5 s, gone after 1.5 s |
| Failure controls | corrupt / drop | + `kill` / `revive` a unit |
| Big data | – | photo in 1 KB chunks (bulk priority), sha256 checked at base B |
| GUI | PyQt placeholder (`base_station_gui`) | web dashboard (`mesh_dashboard`), `http://localhost:8080` |

## What changes, file by file

| Where | Change |
|---|---|
| `mesh_common` | bundle fields `lifetime`, `priority`, real `next_hop`; new types; constants `SEND_RATE`, `STORE_LIMIT`, lifetimes; new keys `mesh/events`, `mesh/base_b/photo`, `mesh/base_b/photo_cmd` |
| `mesh_protocol/mesh_node.py` | wires everything to Zenoh: custody transfer, store, expiry, routing, `STATUS`; drones run the election and leader logic; base A relays mesh ON/OFF as `MESH_CMD`; photo; kill / revive |
| `mesh_protocol/store.py`, `routing.py`, `leader.py`, `photo.py` (new) | pure logic, no Zenoh, each with tests in `mesh_protocol/test/` |
| `mesh_protocol/coordinator.py` | deleted (logic moved to `leader.py`, run by the leader drone) |
| `mesh_protocol/launch/mesh.launch.py` | `dashboard:=true` replaces `gui:=true`; every node gets `--fake` and `--mesh`; no coordinator process |
| `mesh_protocol/resource/photo.png` | the rover's "camera" picture |
| `mesh_protocol/tools.py` | `mesh_ctl auto` removed; `mesh_monitor` shows new counters and the leader |
| `mesh_protocol/selftest.py` | new checks: outage, stale commands, 10 % loss, 20 % corruption, leader takeover, photo, dashboard |
| `mesh_dashboard` (new) | stdlib HTTP server + Server-Sent Events, offline HTML/JS/SVG, mission-control look |
| `base_station_gui` | deleted (the unmerged `origin/Ronak_GUI` branch is dropped too) |
| `mesh_link_model/terrain.py` | also finds `crater.obj` in a source checkout outside `~/mars_ws` |
| `mission.py` | waiting during an outage no longer counts as "stuck" (no false lift-forward assist when the link returns) |
| `mesh_link_model/sim_io.py` | `fast_available()`: drones log a warning if the Gazebo Python bindings are missing |
| `mars_sim` | unchanged |

## Zenoh keys: new or changed in v3

| Key | From → to | Contents |
|---|---|---|
| `mesh/events` | every node → dashboard | one event: retransmit, expired, election, route change, photo progress, killed / revived, dropped_full |
| `mesh/stats/<unit>` | node → dashboard | v2 counters + `held`, `custody_sent`, `custody_acked`, `retransmits`, `expired`, `dropped_full`, `by_type` (sent / delivered / expired per bundle type), `routes`, `leader`, `term`, `killed` |
| `mesh/coordinator/state` | base A (newest `LEADER` it heard) → dashboard | mode, state, routes, targets, chains, leader, term |
| `mesh/base_b/telemetry` | base B → dashboard | as v2 + `late: true` when it arrived > 1 s after sending |
| `mesh/base_b/photo_cmd` | dashboard → base B | take a photo |
| `mesh/base_b/photo` | base B → dashboard | progress, then the image (base64) + sha256 `ok` |
| `mesh/failure/control` | dashboard → injector, nodes | as v2 + `{"kill": unit}`, `{"revive": unit}` |

## How to check your work

```bash
python3 -m pytest src/*/test             # 86 unit tests, seconds
ros2 run mesh_protocol selftest          # fake mode, no Gazebo, ~8 min; must end with 39/39 checks passed
```

## Decisions made while building (differ from the design doc)

- Next hop: the route table wins over "destination is a neighbour" (a neighbour stays alive 1.5 s after
  its link breaks; preferring it caused 3 s ACK gaps). A next hop that sent no HELLO for 1.5 s is never used.
- The leader computes routes from positions + line of sight on its terrain map, not from reported
  neighbour lists (those lag ~1.5 s behind a drone move, long enough for 1 s commands to expire).
- Every unit, not only drones, passes on control floods. Otherwise drones split into two groups during
  re-deploys and each elected a leader.
- No re-route-after-5-tries limit: the store looks up the route on every send.
- Drones glide instead of jumping (`mover.py`). Because every move is now a flight that can open a short
  relay gap, the leader re-plans **only when a chain is about to break** (v2 also re-planned every 8 m:
  that caused 3-5 s base A gaps with flying drones). New spots go to the nearest drones, a spare drone
  moves first, and when nothing is connected yet all drones fly at once.
- The self-test drives at demo speed (1 m/s) instead of 2.5 m/s.
- No per-bundle "custody" events in `mesh/events` (counters only): 8+ a second would flood the log.

## Known limits

- Tested at demo speed (1 m/s). Much faster rovers can outrun a drone reshuffle and base A briefly loses
  the rover; ideas 1-3 in [`OPTIMIZATIONS.md`](OPTIMIZATIONS.md) address that.
- Two drones can occasionally start an election at the same moment (both term N); the lower id wins
  within a heartbeat, as designed.
- In Gazebo each drone calls `set_pose` 20 times a second while it flies. That needs the Gazebo Python
  bindings (`python3-gz-transport13`); the `gz service` command-line fallback is far too slow for it
  (each drone logs a WARNING at start if the bindings are missing).

Small issues found in the final review and left for later (none breaks the demo):
- A killed node stops expiring its store, so on revive it reports a burst of expired pings.
- `MESH_CMD` has no ordering: two quick `mesh_ctl on`/`off` reordered in flight could leave the other mode.
- Sequence numbers restart at 1 when a single node is restarted: other nodes take its new bundles for
  duplicates for a while. Restart the whole launch instead of one node.
- A photo chunk dropped at a full relay store is not resent end to end (photo stuck at N-1/N).
- The dashboard re-sends the photo in every snapshot (~140 KB, 4 times a second, localhost only).

- Not run on 24.04 / Jazzy / Gazebo yet (see step 6). In Gazebo the drones teleport themselves with the
  `set_pose` service from inside their own node: check that works with several nodes at once.
- With only 2 drones left and the rover beside base B, no plan covers both bases (3 drones do).
- The dashboard has no login: it listens on 127.0.0.1 only. `--host 0.0.0.0` exposes the controls.
