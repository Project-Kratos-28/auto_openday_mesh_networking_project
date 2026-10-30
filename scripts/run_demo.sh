#!/bin/bash
# run_demo.sh start|stop|status
#   start  : Gazebo (3D window on the VM desktop) + mesh + dashboard.  Then run the mission yourself.
#   start nogui : same without the 3D window
#   stop   : kill everything
source /opt/ros/jazzy/setup.bash; source "$(dirname "$0")/../install/setup.bash"
mkdir -p ~/demo_logs
case $1 in
 start)
  export GZ_SIM_RESOURCE_PATH=$(ros2 pkg prefix mars_sim)/share/mars_sim/models:$GZ_SIM_RESOURCE_PATH
  W=$(ros2 pkg prefix mars_sim)/share/mars_sim/worlds/mars_crater.sdf
  if [ "$2" = nogui ]; then
    setsid nohup gz sim -s -r $W > ~/demo_logs/sim.log 2>&1 < /dev/null &
  else
    export XDG_RUNTIME_DIR=/run/user/$(id -u) DISPLAY=:0
    export XAUTHORITY=$(ls /run/user/$(id -u)/.mutter-Xwaylandauth.* | head -1)
    # the VM's virtual GPU can't run Gazebo's default ogre2 window; classic ogre works
    setsid nohup gz sim -r --render-engine-gui ogre $W > ~/demo_logs/sim.log 2>&1 < /dev/null &
  fi
  sleep 12
  setsid nohup ros2 launch mesh_protocol mesh.launch.py dashboard:=true > ~/demo_logs/mesh.log 2>&1 < /dev/null &
  echo "started. dashboard: http://localhost:8080  (in the VM browser)"
  ;;
 stop)
  pkill -f "[g]z sim"; pkill -f "[r]os2 launch"; pkill -f "[m]esh_node"; pkill -f "[l]ink_model"
  pkill -f "[r]adio_channel"; pkill -f "[p]assthrough_injector"; pkill -f "[m]esh_dashboard"; pkill -f "[m]esh_protocol/mission"
  echo stopped ;;
 *) echo "usage: run_demo.sh start [nogui] | stop" ;;
esac
