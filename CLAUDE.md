# Mars Rover Relay Simulation (robotics club, team of 4)

## Project idea
A primary rover loses line-of-sight to Earth (crater / behind mountains). Drones/rovers spread out and
form a mesh network so data hops until one reaches the base station. We simulate terrain, rover + drones,
deep-space telemetry (delay, corruption), a custom network protocol, and a GUI.

## My role (the user: a beginner in code/software)
I own the Gazebo simulation: world, rovers, drones, sensors, launch files.
Teammates own: network protocol, corruption module, GUI.

## How to talk to me
- Use simple language. Explain each new file/line in one short line.
- Keep code minimal and working first, then improve. Small steps; test each step.
- If my request is unclear, ask before writing lots of code.
- Tell me which files you created and where.

## Environment (IMPORTANT)
- Ubuntu 24.04, ROS 2 **Jazzy**, Gazebo **Harmonic** (command: `gz sim`).
- NEVER use Gazebo Classic (`gazebo`, `gazebo_ros`, `gazebo_ros_pkgs`) or Humble/Foxy tutorials.
- Use `ros_gz_sim`, `ros_gz_bridge`, SDF format. Many online tutorials are for old versions - double check.
- Never run sudo. If something needs sudo, give me the command and I will run it.

## Workspace layout
- Workspace root: this folder (ROS 2 workspace). Packages live in `src/`.
- Main package: `src/mars_sim/` -> `worlds/` (SDF worlds), `models/` (rover, drone), `launch/`, `config/`.
- After changing files: `colcon build --symlink-install`, then `source install/setup.bash`.

## Running commands (each command runs in a fresh shell!)
- Start commands with: `source /opt/ros/jazzy/setup.bash && source install/setup.bash && <command>`
- GUI / never-ending commands (`gz sim`, `ros2 launch`) block forever. For quick checks use a timeout and no window:
  `timeout 20 gz sim -s -r src/mars_sim/worlds/empty_mars.sdf`
  For the real window, ask me to run it in my own terminal.

## Rules for logging problems
- Whenever we hit an error and fix it, append to `ERRORS.md`: date, what failed, the fix (2-3 lines).

## Interfaces with teammates (fill in as we agree)
- Topics the sim publishes / subscribes: TBD
- Message types: TBD

## Current progress
- [x] Workspace + mars_sim package + flat Mars world (`ros2 launch mars_sim sim.launch.py`)
- [ ] Terrain with crater / mountains
- [ ] Primary rover model (drives)
- [ ] Drone/relay models
- [ ] Sensors (position, signal strength)
- [ ] Bridge to ROS topics for teammates
