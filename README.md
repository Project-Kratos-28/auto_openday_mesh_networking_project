# Mars Rover Relay Simulation
Gazebo (Harmonic) + ROS 2 (Jazzy) simulation of rovers/drones forming a relay mesh when line-of-sight to Earth is lost.

## Run
```bash
source /opt/ros/jazzy/setup.bash
cd <this folder> && colcon build --symlink-install
source install/setup.bash
ros2 launch mars_sim sim.launch.py
```
See `docs/CLAUDE_CODE_MANUAL.md` for how we use Claude Code, and `ERRORS.md` for problems we hit and fixes.
