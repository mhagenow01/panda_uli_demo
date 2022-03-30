cmake_minimum_required(VERSION 3.0)
project(corrective_shared_autonomy)

SET(CMAKE_BUILD_TYPE Release)
SET(CMAKE_SKIP_RPATH False)
SET(CMAKE_INSTALL_RPATH ..)

find_package(catkin REQUIRED COMPONENTS roscpp rospy roslib tf2_ros geometry_msgs sensor_msgs std_msgs genmsg panda_ros_msgs)
catkin_package()
catkin_python_setup()
find_package(Eigen3 REQUIRED)
find_library(DHDC libdhd.so.3.8.0 REQUIRED)
find_library(DRDC libdrd.so.3.8.0 REQUIRED)
find_library(LIBNIFALCON nifalcon REQUIRED)
find_library(franka libfranka.so.0.7.1 REQUIRED)
find_library(
    pandaController
    NAMES PandaController
    PATHS ${CMAKE_CURRENT_SOURCE_DIR}/../../devel/lib
)
find_library(BSpline libBSpline.so REQUIRED)
find_library(NLopt libnlopt.so REQUIRED)

message(STATUS ${pandaController})
find_library(
    pandaCommon
    NAMES common 
    PATHS ${CMAKE_CURRENT_SOURCE_DIR}/../../devel/lib
)

# Build C++ Executables
add_executable(fdzdcontroller src/ForceDimensionZDController.cpp)
target_include_directories(fdzdcontroller PUBLIC ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/include ${CMAKE_CURRENT_SOURCE_DIR}/../../devel/include)
target_link_libraries(fdzdcontroller ${catkin_LIBRARIES} ${DHDC})

install(TARGETS fdzdcontroller
  RUNTIME DESTINATION lib/fdzdcontroller
)

add_executable(fddivisiblecontroller src/ForceDimension_Divisible.cpp)
target_include_directories(fddivisiblecontroller PUBLIC ${catkin_INCLUDE_DIRS} ${EIGEN3_INCLUDE_DIR} ${CMAKE_CURRENT_SOURCE_DIR}/include ${CMAKE_CURRENT_SOURCE_DIR}/../../devel/include)
target_link_libraries(fddivisiblecontroller ${catkin_LIBRARIES} ${DHDC})

install(TARGETS fddivisiblecontroller
  RUNTIME DESTINATION lib/fddivisiblecontroller
)

