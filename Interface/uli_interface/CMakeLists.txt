cmake_minimum_required(VERSION 2.8.3)
project(uli_interface)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)
SET(CMAKE_CXX_FLAGS  "-O3 -fopenmp")

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED COMPONENTS 
        # roscpp 
        tf 
        # cv_bridge 
        image_transport 
        # visualization_msgs
        # image_geometry 
        # rviz_camera_stream
        # sensor_msgs
        # pcl_conversions
        # pcl_ros
        # drone_ros_msgs
	rviz
        )

## System dependencies
# find_package(OpenCV REQUIRED)

# find_package(Eigen3 REQUIRED)

# find_package(PCL REQUIRED)

# include_directories(${PCL_INCLUDE_DIRS})
# link_directories(${PCL_LIBRARY_DIRS})
# add_definitions(${PCL_DEFINITIONS})


# message(STATUS "OpenCV version: ${OpenCV_VERSION}")
# if(2.9.0 VERSION_GREATER ${OpenCV_VERSION})
#     message(FATAL "OpenCV3 required!")
# endif()

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
#   DEPENDS OpenCV
  LIBRARIES 
)



###########
## Build ##
###########

add_definitions(-DQT_NO_KEYWORDS)

include_directories(include ${catkin_INCLUDE_DIRS})

# add_definitions(-std=c++11)

set(LIBS 
    ${catkin_LIBRARIES}
    # ${OpenCV_LIBRARIES}
    )

#############
## Install ##
#############

install(DIRECTORY
    launch/
    DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/launch
)
# install(DIRECTORY
#     config/
#     DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/config
# )
# install(DIRECTORY
#     rviz/
#     DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/rviz
# )
# install(DIRECTORY
#     meshes/
#     DESTINATION ${CATKIN_PACKAGE_SHARE_DESTINATION}/meshes
# )

catkin_install_python(PROGRAMS
    nodes/spliner
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})


