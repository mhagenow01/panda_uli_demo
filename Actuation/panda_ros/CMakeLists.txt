cmake_minimum_required(VERSION 3.0)
project(panda_ros)

SET(CMAKE_BUILD_TYPE Release)
SET(CMAKE_SKIP_RPATH False)
SET(CMAKE_INSTALL_RPATH ..)


find_package(catkin REQUIRED COMPONENTS roscpp tf2_ros geometry_msgs sensor_msgs std_msgs)
catkin_package()

find_package(Eigen3 REQUIRED)
find_library(franka libfranka.so.0.8.0 REQUIRED)
find_library(
    pandaController
    NAMES PandaController
    PATHS ${CMAKE_CURRENT_SOURCE_DIR}/../../../../devel/lib
)

message(STATUS ${pandaController})
find_library(
    pandaCommon
    NAMES common 
    PATHS ${CMAKE_CURRENT_SOURCE_DIR}/../../../../devel/lib
)
message(STATUS ${pandaCommon})

add_executable(pandaRosWrapper src/panda_ros.cpp)
target_include_directories(pandaRosWrapper PUBLIC ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/../../../../devel/include)
target_link_libraries(pandaRosWrapper ${catkin_LIBRARIES} ${pandaController} ${pandaCommon} ${franka})

install(TARGETS pandaRosWrapper
  RUNTIME DESTINATION lib/panda_ros
)

catkin_install_python(PROGRAMS
    test/velocity_control_test
    test/path_control_test
    test/mocap_calib
    DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION})
