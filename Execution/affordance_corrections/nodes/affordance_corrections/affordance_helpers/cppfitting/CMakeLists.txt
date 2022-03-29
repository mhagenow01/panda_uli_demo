cmake_minimum_required(VERSION 3.10)

if(NOT CMAKE_BUILD_TYPE)
  set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS_RELEASE "-O3")

project(cppregistration)

find_package(Eigen3 REQUIRED)
message(STATUS ${Eigen3})
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
