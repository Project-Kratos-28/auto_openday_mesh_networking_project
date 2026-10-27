# Two-base relay demo (mesh v2)

The rover drives from **base A** to **base B** through the crater. Base A sends the velocity
commands and base B receives the rover's odometry. The **mesh toggle** (GUI switch or
`mesh_ctl on|off`) decides whether the 4 drones relay:

- **OFF** (default at start): direct links only. In the crater the rover loses base A, and base B never hears it.
- **ON**: all 4 drones deploy at once, wherever the rover is, and follow it. Base A keeps commanding and
  base B keeps receiving odometry for the rest of the journey (no gaps: the drones are moved one
  at a time, in an order that keeps both chains working).
- **AUTO** (`mesh_ctl auto`, terminal only): the old behaviour, deploy only once base A loses the rover.

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
| T2 | `ros2 launch mesh_protocol mesh.launch.py` | link model, radio, 7 mesh nodes, coordinator |
| T3 | `ros2 run mesh_protocol mesh_monitor` | live view of both bases |
| T4 | `ros2 run base_station_gui gui` | base station window with the **mesh toggle** |
| T5 | `ros2 run mesh_protocol mission` | base A drives the rover to base B |

Other controls: `mesh_ctl on` / `mesh_ctl off` (same as the GUI switch), `mesh_ctl auto`,
`send_cmd 1.0` (one command), `move_unit home` (park drones).
Mesh ON from the start: `ros2 launch mesh_protocol mesh.launch.py mesh:=on`.
GUI started together with the mesh: `ros2 launch mesh_protocol mesh.launch.py gui:=true`.
No Gazebo: `ros2 launch mesh_protocol mesh.launch.py fake:=true mission:=true gui:=true`.

## What happens (demo: start OFF, flip ON)

1. **Mesh OFF, direct link.** Rover next to base A: commands go `base_a -> rover`, ACKs come back. Base B: NO DATA.
2. **Link lost.** The rover goes over the rim: no ACK at base A. The rover's watchdog stops it after 2 s.
3. **Flip the toggle ON.** The coordinator plans 2 chains (rover→A, rover→B) using all 4 drones,
   teleports them, and switches relaying on. Base A is connected again, base B receives odometry,
   and the rover drives on.
4. **Following.** Every 0.5 s the coordinator checks the chains for where the rover is now and
   in 3 s, and re-places the drones before a link would break. Near base B, all 4 drones form one
   long chain back to base A. Base B's card shows the longest odometry gap since ON (normally 0.5 s).

To show odometry for the **whole** journey, switch ON before starting the mission (or `mesh:=on`).

## Zenoh keys added in v2

| Key | From → To | Contents |
|---|---|---|
| `mesh/base_a/cmd` | mission / GUI → base A | `{"linear": v, "angular": w}` |
| `mesh/base_a/acks` | base A → GUI | each ACK: rtt, command route, ACK route |
| `mesh/base_b/telemetry` | base B → GUI | rover odometry + true pose, route, delay |
| `mesh/coordinator/state` | coordinator → drones, GUI | mode (on/off/auto), state, relay on/off, chains, drone targets, on_since |
| `mesh/coordinator/cmd` | GUI toggle / mesh_ctl → coordinator | `on` / `off` / `auto` / `deploy` |
| `mesh/mission/status` | mission → GUI | rover x, progress %, assists |

Units are now `rover`, `drone_1`…`drone_4`, `base_a` (Gazebo `base_station`) and `base_b` (Gazebo `base_station_2`).
Packet types used: `HELLO`, `COMMAND` (base A → rover), `ACK` (rover → base A), `TELEMETRY` (rover → base B).

## Change the layout

- Terrain size / crater shape: numbers at the top of `src/mars_sim/scripts/generate_crater.py`.
- Base / rover / drone parking positions: `src/mars_sim/scripts/build_relay_world.py` **and** `START_XY` in `mesh_common`.
- Then: `cd ~/mars_ws/src/mars_sim && python3 scripts/generate_crater.py && python3 scripts/build_relay_world.py && cd ~/mars_ws && colcon build --symlink-install`
- Radio range: `RADIO_RANGE` in `mesh_common` (32 m now; it was picked so the centre of the crater needs 2 drones per base).
- Other demo-scale numbers in `mesh_common`: `ROVER_SPEED`, `RELAY_HEIGHTS` (drone heights the planner tries), `REPLAN_DISTANCE`, `CRATER_RADIUS`, `GOAL_X`.
- Back to the big 800 m world: restore the backup made by `setup_small_scale.sh` (`~/scale_backup_<date>/`).

## GUI placeholder (`src/base_station_gui`, PyQt6)

- `mesh_bridge.py`: all Zenoh (`snapshot()`, `set_mesh(True/False)`). There's no Qt in it, so keep it.
- `app.py`: the toggle switch and 3 status cards work. 4 dashed TODO boxes are left for the GUI teammate
  (map, packet routes, telemetry plots, failure controls). See `src/base_station_gui/README.md`.
