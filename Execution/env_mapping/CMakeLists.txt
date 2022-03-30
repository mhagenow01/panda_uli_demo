cmake_minimum_required(VERSION 3.0)
project(env_mapping)

SET(CMAKE_BUILD_TYPE Release)

find_package(catkin REQUIRED COMPONENTS roscpp tf2_ros geometry_msgs sensor_msgs std_msgs genmsg message_generation pcl_conversions pcl_ros)

find_package(PCL REQUIRED)

include_directories(${PCL_INCLUDE_DIRS})
link_directories(${PCL_LIBRARY_DIRS})
add_definitions(${PCL_DEFINITIONS})


catkin_package(
  CATKIN_DEPENDS 
    tf
  LIBRARIES 
)

catkin_install_python(PROGRAMS
    nodes/env_mapping
    nodes/map_odometry
    nodes/scanning_pan
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
    
add_executable(concatenatePC src/ConcatenatePC.cpp)
target_include_directories(concatenatePC PUBLIC ${catkin_INCLUDE_DIRS})
target_link_libraries(concatenatePC ${catkin_LIBRARIES} ${PCL_LIBRARIES})

install(TARGETS concatenatePC
  RUNTIME DESTINATION lib/concatenatePC
)

