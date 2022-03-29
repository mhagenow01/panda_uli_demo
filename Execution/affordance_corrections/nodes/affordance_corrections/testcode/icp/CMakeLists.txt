cmake_minimum_required(VERSION 3.12)

project(icp)

find_package(Eigen3 QUIET)
if(Eigen3_FOUND)
    message(STATUS "Eigen3 found. Building Eigen example.")
else()
    message(STATUS "Eigen3 not found. Eigen example skipped.")
endif()

find_package(PicoTree REQUIRED)
find_package(NumCpp 2.6.2 REQUIRED)

add_executable(icp icp.cpp)

target_link_libraries(icp PUBLIC
    PicoTree::PicoTree
    Eigen3::Eigen
    NumCpp::NumCpp
   )