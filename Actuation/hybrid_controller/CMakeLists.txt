cmake_minimum_required(VERSION 3.0)
project(hybrid_controller)

SET(CMAKE_BUILD_TYPE Release)
SET(CMAKE_SKIP_RPATH False)
SET(CMAKE_INSTALL_RPATH ..)

find_package(catkin REQUIRED COMPONENTS roscpp tf2_ros geometry_msgs sensor_msgs std_msgs genmsg message_generation)

add_message_files(
  FILES
  HybridPose.msg
)
generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)

catkin_package(
  CATKIN_DEPENDS message_runtime
)

catkin_install_python(PROGRAMS
   nodes/hybrid_controller
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})