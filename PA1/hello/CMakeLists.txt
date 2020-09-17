cmake_minimum_required(VERSION 2.8)
project (hello)

set(CMAKE_BUILD_TYPE "Release")

include_directories(${PROJECT_SOURCE_DIR} ${PROJECT_SOURCE_DIR}/include)

add_library(hello SHARED src/hello.cpp)

add_executable(sayHello useHello.cpp)

target_link_libraries(sayHello PUBLIC hello)

install(TARGETS hello
LIBRARY DESTINATION /usr/local/lib
)

install(FILES include/hello.h DESTINATION /usr/local/include)

