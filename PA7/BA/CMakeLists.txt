cmake_minimum_required(VERSION 2.8.0)
project(ba_g2o)

set(CMAKE_CXX_FLAGS "-O3 -std=c++11")

find_package(g2o REQUIRED)
find_package(Sophus REQUIRED)
include_directories("/usr/local/include/eigen3" "/usr/include/suitesparse" ${g2o_INCLUDE_DIRS} ${Sophus_INCLUDE_DIRS})
SET(g2o_LIBS g2o_csparse_extension g2o_stuff g2o_core cxsparse)
add_library(bal_common common.cpp)

add_executable(${PROJECT_NAME} ba_g2o.cpp)
target_link_libraries(${PROJECT_NAME} ${g2o_LIBS} ${Sophus_LIBRARIES} bal_common)
