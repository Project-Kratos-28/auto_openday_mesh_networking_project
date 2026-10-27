# base_station_gui (placeholder framework, PyQt6)

Run: `ros2 run base_station_gui gui` (the mesh must be running: `ros2 launch mesh_protocol mesh.launch.py`).
Or start it with the mesh: `ros2 launch mesh_protocol mesh.launch.py gui:=true`.

| File | What it does |
|---|---|
| `base_station_gui/mesh_bridge.py` | All Zenoh in/out. `snapshot()` returns the data as a plain dict, and `set_mesh(True/False)` is the toggle. No Qt here. |
| `base_station_gui/app.py` | PyQt6 window: the mesh toggle (working), 3 status cards (working), 4 dashed TODO boxes. |

## Working now
- **Mesh toggle.** It sends `on` / `off` to `mesh/coordinator/cmd`, then waits up to 4 s for the coordinator to confirm.
  The switch always shows the coordinator's real mode, so a `mesh_ctl` from a terminal moves it too.
- **Status cards.** Mesh state and chains, base A connection (ACK routes, round trip), and base B odometry feed (route, delay, longest gap).

## To do (GUI teammate)
Replace the dashed boxes in `app.py`, in `BaseStationWindow.__init__`, the `grid.addWidget(placeholder(...))` lines:
1. Map / link view: `snapshot()["positions"]` and `["links"]`
2. Packet route animation: `snapshot()["base_b_last_odom"]["route"]`
3. Telemetry plots: `snapshot()["base_b_last_odom"]["payload"]`
4. Failure controls: publish JSON like `{"corrupt_probability": 0.2}` to `mesh/failure/control` (add a method to MeshBridge)

Redraw in `refresh()`, which runs every 250 ms. Never call Zenoh from widgets; add a method to `MeshBridge` instead.

Needs: `sudo apt install python3-pyqt6`
