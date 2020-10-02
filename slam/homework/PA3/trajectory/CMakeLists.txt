cmake_minimum_required(VERSION 2.8.0)
project(trajectory)

set(CMAKE_CXX_FLAGS "-O3")
set(CMAKE_BUILD_TYPE "Debug")

include_directories("/usr/local/include/sophus")
include_directories("/usr/local/include/eigen3")
include_directories("/usr/local/include/pangolin")

add_executable(trajectory src/draw_trajectory.cpp)
target_link_libraries(trajectory ${Pangoline_INCLUDE_DIRS})
