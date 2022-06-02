cmake_minimum_required(VERSION 3.0)
project(corrective_shared_autonomy)

SET(CMAKE_BUILD_TYPE Release)
SET(CMAKE_SKIP_RPATH False)
SET(CMAKE_INSTALL_RPATH ..)

find_package(catkin REQUIRED COMPONENTS roscpp rospy roslib tf2_ros geometry_msgs sensor_msgs std_msgs genmsg hybrid_controller message_generation urdf kdl_parser)
find_package(orocos_kdl REQUIRED)

catkin_python_setup()
find_package(Eigen3 REQUIRED)
find_library(DHDC libdhd.so.3.8.0 REQUIRED)

add_service_files(
  FILES
  IK.srv
)

generate_messages(
  DEPENDENCIES
  std_msgs
  geometry_msgs
)
   

catkin_package(
  CATKIN_DEPENDS 
    tf
    message_runtime
  LIBRARIES 
)

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

add_executable(kdlik src/KDLik.cpp)
target_include_directories(kdlik PUBLIC ${catkin_INCLUDE_DIRS} ${orocos_kdl_INCLUDE_DIRS})
target_link_libraries(kdlik ${catkin_LIBRARIES} ${orocos_kdl_LIBRARIES})

install(TARGETS kdlik
  RUNTIME DESTINATION lib/kdlik
)
