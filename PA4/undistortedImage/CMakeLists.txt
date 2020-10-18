cmake_minimum_required(VERSION 2.8.0)
project(undistortedImage)
set(CMAKE_CXX_LINK_FLAGS "-std=c++11")

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDE_DIRS})

add_executable(undistortedImage undistort_image.cpp)
target_link_libraries(undistortedImage ${OpenCV_LIBS})
