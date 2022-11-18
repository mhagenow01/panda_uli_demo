cmake_minimum_required(VERSION 3.10)
set (CMAKE_CXX_STANDARD 11)
set(CMAKE_FIND_DEBUG_MODE 1)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS_RELEASE "-O3")

project(cppregistration)

find_package(Eigen3 EXACT 3.4.0 REQUIRED PATHS /home/nitzan/Documents/lib_installs/eigen/)
find_package(PicoTree REQUIRED)
find_package(OpenMP)
find_library(NLopt libnlopt.so REQUIRED)


add_library(cppregistration SHARED cppregistration.cpp Model.cpp)

target_link_libraries(cppregistration PUBLIC
    PicoTree::PicoTree
    Eigen3::Eigen
    OpenMP::OpenMP_CXX
    ${NLopt}
   )
