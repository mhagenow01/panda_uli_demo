cmake_minimum_required(VERSION 3.0)
project(kinect_ros)

SET(CMAKE_BUILD_TYPE Release)
SET(CMAKE_SKIP_RPATH False)
SET(CMAKE_INSTALL_RPATH ..)

find_package(catkin REQUIRED COMPONENTS roscpp tf2_ros geometry_msgs sensor_msgs std_msgs)
catkin_package()


catkin_install_python(PROGRAMS
    nodes/kinect_toggler
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
