# Starts Gazebo with our Mars world. Run: ros2 launch mars_sim sim.launch.py
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, SetEnvironmentVariable
from launch.launch_description_sources import PythonLaunchDescriptionSource


def generate_launch_description():
    pkg = get_package_share_directory('mars_sim')              # where our package was installed
    ros_gz_sim = get_package_share_directory('ros_gz_sim')     # the ROS-Gazebo helper package
    world = os.path.join(pkg, 'worlds', 'empty_mars.sdf')      # our world file
    models = os.path.join(pkg, 'models')                       # our rover/drone models (later)

    return LaunchDescription([
        # tells Gazebo where to look for our models
        SetEnvironmentVariable('GZ_SIM_RESOURCE_PATH', models),
        # starts Gazebo; "-r" = start running immediately
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(ros_gz_sim, 'launch', 'gz_sim.launch.py')),
            launch_arguments={'gz_args': f'-r {world}'}.items(),
        ),
    ])
