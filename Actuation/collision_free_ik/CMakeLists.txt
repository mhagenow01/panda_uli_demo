cmake_minimum_required(VERSION 3.2)

project(collision_free_ik)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++17 -O3")

find_package(catkin REQUIRED COMPONENTS roscpp rospy std_msgs genmsg franka_core_msgs)
catkin_package()

# Begin rust compile stuff

# Set the location of the library output so we can link against it at runtime
set(IK_LIB ${CATKIN_DEVEL_PREFIX}/${CATKIN_PACKAGE_LIB_DESTINATION}/libcollision_free_ik.so)

# Set up commnads for actually generating the library
include(ExternalProject)
ExternalProject_Add(
  panda_ik_rust
  PREFIX ${CMAKE_CURRENT_SOURCE_DIR} # These two set up the path for the build commands
  SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR}
  BUILD_COMMAND cargo build --release # This is the pat that does something
  BUILD_ALWAYS TRUE   # Should have this, since you might want to change the rust code and recompile.
  INSTALL_COMMAND ${CMAKE_COMMAND} -E copy ${CMAKE_CURRENT_SOURCE_DIR}/target/release/libcollision_free_ik.so ${IK_LIB} # Move (install) the library to the target location
  UPDATE_COMMAND ""
  LOG_BUILD ON
)

# Specify some "CMake objects" to link against to esablish dependencies
add_library(panda_ik_rust_lib SHARED IMPORTED)
set_target_properties(panda_ik_rust_lib PROPERTIES IMPORTED_LOCATION ${IK_LIB})
# End rust compile

include_directories(${catkin_INCLUDE_DIRS})
add_executable(collision_free_ik cpp_src/main.cpp)

add_dependencies(
    collision_free_ik
    panda_ik_rust
    panda_ik_rust_lib
)

target_include_directories(collision_free_ik PUBLIC include)
target_link_libraries(collision_free_ik ${catkin_LIBRARIES} panda_ik_rust_lib)


# install(TARGETS collision_free_ik
#   ARCHIVE DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   LIBRARY DESTINATION ${CATKIN_PACKAGE_LIB_DESTINATION}
#   RUNTIME DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
# )
# install(DIRECTORY include/${PROJECT_NAME}/
#   DESTINATION ${CATKIN_PACKAGE_INCLUDE_DESTINATION}
# )
catkin_install_python(PROGRAMS test/ik_test.py
  DESTINATION ${CATKIN_PACKAGE_BIN_DESTINATION}
)

