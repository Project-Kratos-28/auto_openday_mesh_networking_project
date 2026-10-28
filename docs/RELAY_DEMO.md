# Two-base relay demo (mesh v3, DTN)

The rover drives from **base A** to **base B** through the crater. Base A sends the velocity
commands and base B receives the rover's odometry. The **mesh toggle** (dashboard or
`mesh_ctl on|off`) tells base A, which asks the **leader drone** (elected among the 4) to deploy
or park the drones:

- **OFF** (default at start): direct links only. In the crater the rover loses base A, and base B never hears it.
  The rover's odometry is **held** in its store meanwhile (DTN), and old commands expire.
- **ON**: the leader deploys all 4 drones at once, wherever the rover is, and follows it. Held odometry
  reaches base B late (amber on the dashboard) instead of being lost; then it flows live.

What changed from v2: [`V3_CHANGES.md`](V3_CHANGES.md).

Demo scale (small, so everything is visible on screen at once): terrain 110 × 60 m.
```
 drones ── base A (-40) ── rim (-18) ── crater floor (0), 6 m deep ── rim (+18) ── base B (+40)
            ▲ rover starts at -32 (facing the crater), drives at 1 m/s (~70 s to base B)
   radio range 32 m: the rim blocks base A's line to the rover at about x = -12 (6 m past the rim)
   base A <-> base B (80 m) can never talk directly
```
Models keep their real size: rover 1 m, drones 1 m, base station 10 × 10 m pad with a 10 m mast.

## Run it (4 terminals, each: `cd ~/mars_ws && source install/setup.bash`)

| Terminal | Command | What it is |
|---|---|---|
| T1 | `ros2 launch mars_sim sim.launch.py` | Gazebo (press ▶) |
| T2 | `ros2 launch mesh_protocol mesh.launch.py` | link model, radio, injector, 7 mesh nodes (drones elect a leader) |
| T3 | `ros2 run mesh_protocol mesh_monitor` | text view of both bases (optional) |
| T4 | `ros2 run mesh_dashboard dashboard` | mission-control dashboard: **http://localhost:8080** |
| T5 | `ros2 run mesh_protocol mission` | base A drives the rover to base B |

Other controls: `mesh_ctl on` / `mesh_ctl off` (same as the dashboard switch), `send_cmd 1.0` (one command).
Dashboard extras: TAKE PHOTO (rover → base B in 1 KB chunks, sha256 checked), drop / corrupt sliders,
KILL / REVIVE a unit (kill the leader: another drone takes over in about 2 s).
Mesh ON from the start: `ros2 launch mesh_protocol mesh.launch.py mesh:=on`.
Dashboard started together with the mesh: `ros2 launch mesh_protocol mesh.launch.py dashboard:=true`.
No Gazebo: `ros2 launch mesh_protocol mesh.launch.py fake:=true mission:=true dashboard:=true`.

## What happens (demo: start OFF, flip ON)

1. **Mesh OFF, direct link.** Rover next to base A: commands go `base_a -> rover`, ACKs come back. Base B: NO DATA.
2. **Link lost.** The rover goes over the rim: no ACK at base A. The rover's watchdog stops it after 2 s.
   Base A's commands expire after 1 s (red in the event log); the rover holds its odometry (held badge).
3. **Flip the toggle ON.** Base A sends MESH_CMD to the leader drone, which plans 2 chains
   (rover→A, rover→B) and hands out drone targets one at a time. Base A is connected again, base B
   receives the held odometry (late) and then live odometry, and the rover drives on.
4. **Following.** Every 0.5 s the leader checks the chains for where the rover is now and in 3 s,
   and re-places the drones before a link would break. Near base B, all 4 drones form one long chain
   back to base A.
5. **Break things.** 10-20 % packet loss: resends go up, nothing is lost. Kill the leader: a new one
   within about 2 s, mode kept. Take a photo: chunks arrive at bulk priority, odometry keeps flowing.

To show odometry for the **whole** journey, switch ON before starting the mission (or `mesh:=on`).

## Zenoh keys

The full list is in the README (section 6). New in v3: `mesh/events`, `mesh/base_b/photo_cmd`,
`mesh/base_b/photo`; `mesh/coordinator/state` is now base A's copy of the newest `LEADER` heartbeat.
Bundle types: `HELLO`, `CUSTODY_ACK`, `COMMAND`, `ACK`, `TELEMETRY`, `PHOTO_REQ`, `PHOTO_CHUNK`,
and the control floods `STATUS`, `LEADER`, `MESH_CMD`.

## Change the layout

- Terrain size / crater shape: numbers at the top of `src/mars_sim/scripts/generate_crater.py`.
- Base / rover / drone parking positions: `src/mars_sim/scripts/build_relay_world.py` **and** `START_XY` in `mesh_common`.
- Then: `cd ~/mars_ws/src/mars_sim && python3 scripts/generate_crater.py && python3 scripts/build_relay_world.py && cd ~/mars_ws && colcon build --symlink-install`
- Radio range: `RADIO_RANGE` in `mesh_common` (32 m now; it was picked so the centre of the crater needs 2 drones per base).
- Other demo-scale numbers in `mesh_common`: `ROVER_SPEED`, `RELAY_HEIGHTS` (drone heights the planner tries), `REPLAN_DISTANCE`, `CRATER_RADIUS`, `GOAL_X`.
- Back to the big 800 m world: restore the backup made by `setup_small_scale.sh` (`~/scale_backup_<date>/`).
