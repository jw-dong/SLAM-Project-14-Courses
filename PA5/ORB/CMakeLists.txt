cmake_minimum_required(VERSION 2.8.0)
project(computeORB)

find_package(OpenCV REQUIRED)

add_executable(computeORB computeORB.cpp)
target_link_libraries(computeORB ${OpenCV_LIBS})
