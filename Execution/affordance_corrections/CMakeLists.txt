cmake_minimum_required(VERSION 3.0)
project(affordance_corrections)

SET(CMAKE_BUILD_TYPE Release)
SET(CMAKE_SKIP_RPATH False)
SET(CMAKE_INSTALL_RPATH ..)


find_package(catkin REQUIRED COMPONENTS roscpp rospy roslib tf2_ros geometry_msgs sensor_msgs std_msgs genmsg message_generation geometry_msgs)

catkin_python_setup()

add_message_files(
  FILES
  AffordanceFits.msg
  AffordanceFit.msg
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)
catkin_package(
  CATKIN_DEPENDS message_runtime
)

find_package(Eigen3 REQUIRED)


