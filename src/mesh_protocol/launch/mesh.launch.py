# Starts the whole mesh: link model, radio channel, stand-in injector,
# and one mesh node per unit (rover, 4 drones, base_a, base_b).
#   ros2 launch mesh_protocol mesh.launch.py                 # Gazebo must already be running
#   ros2 launch mesh_protocol mesh.launch.py sim:=true       # also start Gazebo (mars_sim)
#   ros2 launch mesh_protocol mesh.launch.py mission:=true   # also start the A -> B drive
#   ros2 launch mesh_protocol mesh.launch.py fake:=true      # no Gazebo, fake rover
#   ros2 launch mesh_protocol mesh.launch.py injector:=none  # teammate's real injector instead
#   ros2 launch mesh_protocol mesh.launch.py mesh:=on      # mesh ON from the start (default off)
#   ros2 launch mesh_protocol mesh.launch.py dashboard:=true  # also start the web dashboard (localhost:8080)
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription,
                            OpaqueFunction, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration

UNITS = ["rover", "drone_1", "drone_2", "drone_3", "drone_4", "base_a", "base_b"]


def py(name, module, *args):
    """Run one of our Python programs; 'name' is the prefix shown in the terminal."""
    return ExecuteProcess(cmd=["python3", "-u", "-m", module, *args], name=name, output="screen")


def build(context):
    arg = lambda n: LaunchConfiguration(n).perform(context)    # noqa: E731
    fake = arg("fake") == "true"
    f = ["--fake"] if fake else []
    actions = []
    if arg("sim") == "true" and not fake:
        sim_launch = os.path.join(get_package_share_directory("mars_sim"), "launch", "sim.launch.py")
        actions.append(IncludeLaunchDescription(PythonLaunchDescriptionSource(sim_launch)))
    actions.append(py("link_model", "mesh_link_model.link_model", *f))
    later = [py("radio_channel", "mesh_link_model.radio_channel")]
    if arg("injector") == "passthrough":
        later.append(py("injector", "mesh_link_model.passthrough_injector"))
    # the coordinator runs inside the drones (elected leader); drones move themselves, so they need --fake too
    later += [py(u, "mesh_protocol.mesh_node", "--unit", u, *f, "--mesh", arg("mesh")) for u in UNITS]
    if arg("dashboard") == "true":
        later.append(py("dashboard", "mesh_dashboard.server"))
    actions.append(TimerAction(period=2.0, actions=later))          # let the hub start first
    if arg("mission") == "true":
        actions.append(TimerAction(period=8.0, actions=[py("mission", "mesh_protocol.mission", *f)]))
    return actions


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument("sim", default_value="false", description="also start Gazebo"),
        DeclareLaunchArgument("fake", default_value="false", description="fake rover, no Gazebo"),
        DeclareLaunchArgument("mission", default_value="false", description="start the A -> B drive"),
        DeclareLaunchArgument("injector", default_value="passthrough",
                              description="passthrough = stand-in injector, none = use the real one"),
        DeclareLaunchArgument("mesh", default_value="off", description="mesh mode at start: off | on"),
        DeclareLaunchArgument("dashboard", default_value="false",
                              description="also start the web dashboard on http://localhost:8080"),
        OpaqueFunction(function=build),
    ])
