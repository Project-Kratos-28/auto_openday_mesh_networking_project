# Errors and fixes log
(Claude Code appends here. Upload to git when the project/session ends.)

| Date | Problem | Fix |
|------|---------|-----|
| 2026-10-26 | Code from earlier chats wasn't visible, and linking the GitHub repo to Claude failed ("link your GitHub account") | Linked the chat to the laptop and read `~/mars_ws` directly. Set up the GitHub link before the next session |
| 2026-10-26 | Zenoh default discovery (multicast) can find teammates' laptops on hostel Wi-Fi; IPv6-off machines fail with `Address family not supported` | All mesh programs connect only on `tcp/127.0.0.1:7447` (one hub = the link model), multicast off. Set in `mesh_common.zenoh_config()` |
| 2026-10-26 | `pip install eclipse-zenoh` → `externally-managed-environment` on Ubuntu 24.04 | `pip install --user --break-system-packages eclipse-zenoh` |
| 2026-10-26 | Zenoh tutorials online use the old 0.x Python API (`AttributeError` on `payload`) | Use Zenoh 1.x only (`sample.payload.to_bytes()` / `.to_string()`); setup scripts check the version |
| 2026-10-26 | Telemetry sequence numbers jumped by 2: HELLO and TELEMETRY shared one `seq` counter | One counter per packet type (`self.seq[ptype]`) in `mesh_node.py` |
| 2026-10-26 | `colcon build` → `AttributeError: install_layout` when colcon came from pip (seen while testing the setup scripts) | Use colcon from apt (`sudo apt install ros-dev-tools`), not pip |
| 2026-10-26 | Two-base demo: the rover lost contact for several seconds while drones were re-placed (watchdog stopped it) | Coordinator checks the chains for where the rover is now **and in 3 s**, and re-places drones before a link breaks |
| 2026-10-26 | Odometry gaps at base B while drones moved, and drones were only sent out after the link was already lost | Mesh toggle: ON deploys at once and follows the rover; drones move one at a time in an order that keeps both chains working; teleport via the Gazebo Python service (ms) instead of `gz service` (~0.3 s each); link updates 2 → 5 per second. Result: longest odometry gap 0.5 s (= send interval) |
| 2026-10-26 | Self-test failed at the start ("mode OFF") because it checked before the coordinator's first status message | Tests wait up to 10 s for the first status instead of checking once |
| 2026-10-27 | In the 800 × 400 m world the rover (1 m), drones (1 m) and bases were invisible in Gazebo | Small demo scale: terrain 110 × 60 m, crater 36 m wide, bases 80 m apart, models unchanged |
| 2026-10-27 | Shrinking everything equally doesn't work: the base's 10 m mast could see over a shallow rim into the crater, so the link was never lost | Tuned crater depth (6 m, plain 1.5 m) and radio range (32 m) against the real mast height; checked with the real mesh: link to base A lost at x ≈ -11.5, bases never see each other |
| 2026-10-27 | Risk: the GUI placeholder (`base_station_gui`) could overwrite the team's existing GUI with the same folder name | `PLACEHOLDER_MARKER` file: setup scripts only replace the folder if the marker is there. All Zenoh is in `mesh_bridge.py` so it can move into the real GUI |
| 2026-10-27 | Re-running older setup scripts (`setup_relay_demo.sh`, `setup_mesh_toggle.sh`) would bring back the big world | Don't re-run them; the repo now has the current version. Each script backs up to `~/*_backup_<date>/` first |
