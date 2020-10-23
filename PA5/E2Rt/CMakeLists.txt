cmake_minimum_required(VERSION 2.8.0)
project(E2Rt)

set(CMAKE_CXX_LINK_FLAGS "-std=c++11")

find_package(Sophus REQUIRED)
set(Sophus_LIBRARIES libSophus.so)
include_directories(${Sophus_INCLUDE_DIRS})
include_directories("/usr/local/include/eigen3")

add_executable(E2Rt E2Rt.cpp)
target_link_libraries(E2Rt ${Sophus_LIBRARIES})
