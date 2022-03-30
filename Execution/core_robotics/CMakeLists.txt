cmake_minimum_required(VERSION 3.0)
project(core_robotics)

SET(CMAKE_BUILD_TYPE Release)
SET(CMAKE_SKIP_RPATH False)
SET(CMAKE_INSTALL_RPATH ..)

find_package(catkin REQUIRED COMPONENTS roscpp rospy roslib tf2_ros geometry_msgs sensor_msgs std_msgs genmsg)
catkin_package()
catkin_python_setup()
find_package(Eigen3 REQUIRED)


