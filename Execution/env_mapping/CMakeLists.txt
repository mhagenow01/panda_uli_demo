cmake_minimum_required(VERSION 3.0)
project(env_mapping)

SET(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS roscpp tf2_ros geometry_msgs sensor_msgs std_msgs genmsg message_generation)

catkin_package(
  CATKIN_DEPENDS message_runtime
)