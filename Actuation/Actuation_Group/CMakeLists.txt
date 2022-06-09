cmake_minimum_required(VERSION 3.2)

project(actuation_group)

find_package(catkin REQUIRED)

catkin_package(
  CATKIN_DEPENDS 
    tf
  LIBRARIES 
)

catkin_install_python(PROGRAMS
    nodes/actuation_group_launcher
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

install(DIRECTORY launch
  DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}
)
