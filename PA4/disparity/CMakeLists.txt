cmake_minimum_required(VERSION 2.8.0)
project(disparity)

find_package(OpenCV REQUIRED)
find_package(Pangolin REQUIRED)
include_directories("/usr/local/include/eigen3")
include_directories(${Pangolin_INCLUDE_DIRS})

add_executable(disparity disparity.cpp)
target_link_libraries(disparity ${OpenCV_LIBS} ${Pangolin_LIBRARIES})
