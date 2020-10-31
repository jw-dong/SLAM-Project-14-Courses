cmake_minimum_required(VERSION 2.8.0)
project(direct_method)

set(CMAKE_CXX_FLAGS "-std=c++11 ${SSE_FLAGS} -g -O3 -march=native")

find_package(OpenCV REQUIRED)
find_package(Pangolin REQUIRED)
find_package(Sophus REQUIRED)
set(Sophus_LIBRARIES libSophus.so)
include_directories(${OpenCV_INCLUDE_DIRS} ${Pangolin_INCLUDE_DIRS} ${Sophus_INCLUDE_DIRS})
include_directories("/usr/local/include/eigen3")

add_executable(direct_method direct_method.cpp)
target_link_libraries(direct_method ${OpenCV_LIBS} ${Pangolin_LIBRARIES} ${Sophus_LIBRARIES})
