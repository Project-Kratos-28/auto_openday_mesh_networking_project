# Starts Gazebo with a Mars world.
#   ros2 launch mars_sim sim.launch.py                          -> crater world (default)
#   ros2 launch mars_sim sim.launch.py world:=empty_mars.sdf    -> the old flat test world
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import AppendEnvironmentVariable, DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution


def generate_launch_description():
    pkg = get_package_share_directory('mars_sim')              # where our package is installed
    ros_gz_sim = get_package_share_directory('ros_gz_sim')     # ROS-Gazebo helper package

    world_arg = DeclareLaunchArgument(                          # lets you pick the world file
        'world', default_value='mars_crater.sdf',
        description='World file inside mars_sim/worlds')
    world_path = PathJoinSubstitution([pkg, 'worlds', LaunchConfiguration('world')])

    return LaunchDescription([
        world_arg,
        # tell Gazebo where our models are, so "model://primary_rover" etc. can be found
        AppendEnvironmentVariable('GZ_SIM_RESOURCE_PATH', os.path.join(pkg, 'models')),
        # start Gazebo; "-r" = start the simulation running immediately
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')),
            launch_arguments={'gz_args': ['-r ', world_path]}.items(),
        ),
    ])
