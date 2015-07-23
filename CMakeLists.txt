cmake_minimum_required(VERSION 3.2)
project(DrakeJointMockup)

if (APPLE)
  set(CMAKE_MACOSX_RPATH ON)
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/cmake")

find_package(Eigen3 3.2)
include_directories(${EIGEN3_INCLUDE_DIR})

add_executable(Option1 Option1.cpp)
add_executable(Option2 Option2.cpp)
