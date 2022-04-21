cmake_minimum_required(VERSION 3.0)
project(corrective_input)

SET(CMAKE_BUILD_TYPE Release)
SET(CMAKE_SKIP_RPATH False)
SET(CMAKE_INSTALL_RPATH ..)

find_package(catkin REQUIRED COMPONENTS roscpp rospy roslib tf2_ros geometry_msgs sensor_msgs std_msgs genmsg)
catkin_package()
catkin_python_setup()
find_package(Eigen3 REQUIRED)
find_library(DHDC libdhd.so.3.8.0 REQUIRED)

# Build C++ Executables
add_executable(fdonedzdcontroller src/ForceDimensionOneDZDController.cpp)
target_include_directories(fdonedzdcontroller PUBLIC ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/include ${CMAKE_CURRENT_SOURCE_DIR}/../../devel/include)
target_link_libraries(fdonedzdcontroller ${catkin_LIBRARIES} ${DHDC})

install(TARGETS fdonedzdcontroller
  RUNTIME DESTINATION lib/fdonedzdcontroller
)