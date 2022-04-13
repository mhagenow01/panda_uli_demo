cmake_minimum_required(VERSION 3.0)
project(ft_sensor)

SET(CMAKE_BUILD_TYPE Release)
SET(CMAKE_SKIP_RPATH False)
SET(CMAKE_INSTALL_RPATH ..)

find_package(catkin REQUIRED COMPONENTS roscpp tf2_ros geometry_msgs sensor_msgs std_msgs)
catkin_package()


#catkin_install_python(PROGRAMS
#    nodes/kinect_toggler
#    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

# Build C++ Executables
add_executable(ftsocket src/ForceTorqueSocket.cpp)
target_include_directories(ftsocket PUBLIC ${catkin_INCLUDE_DIRS})
target_link_libraries(ftsocket ${catkin_LIBRARIES})

install(TARGETS ftsocket
  RUNTIME DESTINATION lib/ftsocket
)
