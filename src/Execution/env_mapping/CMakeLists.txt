cmake_minimum_required(VERSION 2.8.3)
project(env_mapping)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS 
        roscpp 
        tf 
        cv_bridge 
        image_transport 
        visualization_msgs
        image_geometry 
        rviz_camera_stream
        sensor_msgs
        pcl_conversions
        pcl_ros
        drone_ros_msgs
	rviz
        )


###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES 
  CATKIN_DEPENDS 
    tf
    sensor_msgs
  DEPENDS OpenCV
  LIBRARIES 
)



###########
## Build ##
###########

add_definitions(-DQT_NO_KEYWORDS)

include_directories(include ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR})

message(STATUS test)
message(STATUS ${CMAKE_CURRENT_SOURCE_DIR})

add_definitions(-std=c++11)

set(LIBS 
    ${catkin_LIBRARIES}
    ${OpenCV_LIBRARIES}
    )


set(SOURCE_FILES src/dronepanel.cpp)
add_library(${PROJECT_NAME} ${SOURCE_FILES})

target_link_libraries(${PROJECT_NAME} ${QT_LIBRARIES} ${catkin_LIBRARIES})


add_executable(cloudplane src/PCLPlanes.cpp)
target_link_libraries(cloudplane ${catkin_LIBRARIES} ${PCL_LIBRARIES})

#############
## Install ##
#############

install(DIRECTORY
    launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
install(DIRECTORY
    config/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
)
install(DIRECTORY
    rviz/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
)
install(DIRECTORY
    meshes/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/meshes
)
install(DIRECTORY
    urdfs/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/urdfs
)

catkin_install_python(PROGRAMS
    nodes/simulator
    nodes/static_publisher
    nodes/virtual_drone_manager
    nodes/pc_publish
    nodes/rviz_manager
    nodes/drone_controller
    nodes/tello_driver
    nodes/tello_driver_launcher
    nodes/velocity_controller
    nodes/joy_driver
    nodes/spacemouse_driver
    nodes/map_odometry
    nodes/env_mapping
    nodes/pc_record
    nodes/pcl_plane_to_bounding
    nodes/alternate_view_manager
    nodes/autonomous_mapping
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})

if(DHDC)
  install(TARGETS fd
    RUNTIME DESTINATION lib/fd
  )
endif()

