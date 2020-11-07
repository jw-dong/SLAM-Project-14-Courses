cmake_minimum_required(VERSION 2.8.0)
project(ba_direct)

set(CMAKE_CXX_FLAGS "-O3 -std=c++11")

find_package(g2o REQUIRED)
find_package(Sophus REQUIRED)
find_package(Pangolin REQUIRED)
find_package(OpenCV REQUIRED)

include_directories("/usr/local/include/eigen3" 
					"/usr/include/suitesparse" 
					${g2o_INCLUDE_DIRS} 
					${Sophus_INCLUDE_DIRS} 
					${Pangolin_INCLUDE_DIRS}
					${OpenCV_INCLUDE_DIRS})

set(Sophus_LIBRARIES libSophus.so)					
set(g2o_LIBS g2o_cli g2o_ext_freeglut_minimal g2o_simulator g2o_solver_slam2d_linear 
    g2o_types_icp g2o_types_slam2d g2o_types_sba g2o_types_slam3d g2o_core g2o_interface 
    g2o_solver_csparse g2o_solver_structure_only g2o_csparse_extension g2o_opengl_helper g2o_solver_dense 
    g2o_stuff g2o_types_sclam2d g2o_parser g2o_solver_pcg g2o_types_data g2o_types_sim3 cxsparse )

add_executable(${PROJECT_NAME} directBA.cpp)
target_link_libraries(${PROJECT_NAME} 
					  ${g2o_LIBS} 
					  ${Sophus_LIBRARIES} 
					  ${Pangolin_LIBRARIES}
					  ${OpenCV_LIBRARIES})
