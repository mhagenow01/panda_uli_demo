cmake_minimum_required(VERSION 3.12)

project(regstrationunit)

find_package(Eigen3 REQUIRED)
find_package(PicoTree REQUIRED)
find_package(OpenMP)
find_library(NLopt libnlopt.so REQUIRED)

add_library(registration_test SHARED registrationunit.cpp ../../affordance_helpers/cppfitting/Model.cpp)

target_link_libraries(registration_test PUBLIC
    PicoTree::PicoTree
    Eigen3::Eigen
    OpenMP::OpenMP_CXX
    ${NLopt}
   )