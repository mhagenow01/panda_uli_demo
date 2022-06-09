cmake_minimum_required(VERSION 3.0)
project(diagnostics)

SET(CMAKE_BUILD_TYPE Release)
SET(CMAKE_SKIP_RPATH False)
SET(CMAKE_INSTALL_RPATH ..)

find_package(catkin REQUIRED COMPONENTS roscpp rospy roslib tf2_ros geometry_msgs sensor_msgs std_msgs genmsg hybrid_controller message_generation urdf kdl_parser)
find_package(orocos_kdl REQUIRED)

catkin_package()

catkin_install_python(PROGRAMS
   nodes/ee_pose_checker
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

add_executable(kdlfk src/KDLfk.cpp)
target_include_directories(kdlfk PUBLIC ${catkin_INCLUDE_DIRS} ${orocos_kdl_INCLUDE_DIRS})
target_link_libraries(kdlfk ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})

install(TARGETS kdlfk
  RUNTIME DESTINATION lib/kdlfk
)