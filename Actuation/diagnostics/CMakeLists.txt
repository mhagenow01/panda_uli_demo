cmake_minimum_required(VERSION 3.0)
project(diagnostics)

SET(CMAKE_BUILD_TYPE Release)
SET(CMAKE_SKIP_RPATH False)
SET(CMAKE_INSTALL_RPATH ..)

find_package(catkin REQUIRED COMPONENTS roscpp rospy roslib tf2_ros geometry_msgs sensor_msgs std_msgs genmsg hybrid_controller message_generation urdf kdl_parser)

catkin_package()

catkin_install_python(PROGRAMS
   nodes/ee_pose_checker
   nodes/paper_status
   DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
