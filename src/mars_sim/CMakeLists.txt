cmake_minimum_required(VERSION 3.8)
project(mars_sim)

find_package(ament_cmake REQUIRED)

# Copy these folders into the install area so ROS/Gazebo can find them
install(DIRECTORY worlds models launch config DESTINATION share/${PROJECT_NAME})

ament_package()
