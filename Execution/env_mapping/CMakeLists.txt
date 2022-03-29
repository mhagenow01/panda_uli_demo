cmake_minimum_required(VERSION 3.0)
project(env_mapping)

SET(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS roscpp tf2_ros geometry_msgs sensor_msgs std_msgs genmsg message_generation)

catkin_package(
  CATKIN_DEPENDS 
    tf
  LIBRARIES 
)

catkin_install_python(PROGRAMS
    nodes/env_mapping
    nodes/map_odometry
    nodes/scanning_pan
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})