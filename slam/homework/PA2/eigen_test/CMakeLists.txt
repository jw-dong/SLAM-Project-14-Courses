cmake_minimum_required(VERSION 2.8)
project(Eigen_test)

set(CMAKE_CXX_FLAGS "-O3")
set(CMAKE_BUILD_TYPE "Debug")

include_directories( "/usr/include/eigen3")

add_executable(eigen_test eigen_test.cpp)
