cmake_minimum_required(VERSION 2.8.0)
project(gaussnewton)

find_package(OpenCV REQUIRED)
include_directories("/usr/local/include/eigen3")

add_executable(gaussnewton gaussnewton.cpp)
target_link_libraries(gaussnewton ${OpenCV_LIBS})
