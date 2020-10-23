cmake_minimum_required(VERSION 2.8.0)
project(GN_BA)

set(CMAKE_CXX_LINK_FLAGS "-std=c++11")

find_package(Sophus REQUIRED)
set(Sophus_LIBRARIES libSophus.so)
include_directories(${Sophus_INCLUDE_DIRS})
include_directories("/usr/local/include/eigen3")

add_executable(GN_BA GN-BA.cpp)
target_link_libraries(GN_BA ${Sophus_LIBRARIES})
