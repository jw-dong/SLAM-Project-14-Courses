cmake_minimum_required(VERSION 2.8.0)
project(trajectory)

set(CMAKE_CXX_FLAGS "-O3")
set(CMAKE_BUILD_TYPE "Debug")
set(CMAKE_CXX_LINK_FLAGS "-std=c++11")

find_package(Pangolin REQUIRED)
find_package(Sophus REQUIRED)

set(Sophus_LIBRARIES libSophus.so)
include_directories(${Sophus_INCLUDE_DIRS})
include_directories(${Pangolin_INCLUDE_DIRS})
include_directories("/usr/local/include/eigen3")

add_executable(trajectory src/draw_trajectory.cpp)
target_link_libraries(trajectory ${Pangolin_LIBRARIES} ${Sophus_LIBRARIES})
