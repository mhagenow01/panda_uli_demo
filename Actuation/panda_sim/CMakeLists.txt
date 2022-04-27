cmake_minimum_required(VERSION 3.0)
project(panda_sim)

SET(CMAKE_BUILD_TYPE Release)
SET(CMAKE_SKIP_RPATH False)
SET(CMAKE_INSTALL_RPATH ..)

find_package(catkin)

catkin_package(
  CATKIN_DEPENDS 
  LIBRARIES 
)

catkin_install_python(PROGRAMS
    nodes/simulator
    nodes/static_publisher
    nodes/rviz_manager
    nodes/joint_republisher
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
