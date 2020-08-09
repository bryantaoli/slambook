cmake_minimum_required(VERSION 2.6)
project(pose_estimation_3d2d)

SET(CMAKE_CXX_FLAGS "-std=c++11")

# g2o不是常用库，需要使用FINDG2O.cmake
list(APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules)
SET( G2O_ROOT /usr/local/include/g2o )
find_package(G2O)

# CSparse
find_package(CSparse)
INCLUDE_DIRECTORIES(${G2O_INCLUDE_DIR} ${CSPARSE_INCLUDE_DIR})

find_package(CSparse REQUIRED)

find_package(OpenCV REQUIRED)
include_directories(${OpenCV_INCLUDED_DIRS})
include_directories("/usr/local/eigen3")
include_directories(${CSPARSE_INCLUDE_DIR})

SET(G2O_LIBS g2o_cli g2o_ext_freeglut_minimal g2o_simulator g2o_solver_slam2d_linear g2o_types_icp g2o_types_slam2d g2o_core g2o_interface g2o_solver_csparse g2o_solver_structure_only g2o_types_sba g2o_types_slam3d g2o_csparse_extension g2o_opengl_helper g2o_solver_dense g2o_stuff g2o_types_sclam2d g2o_parser g2o_solver_pcg g2o_types_data g2o_types_sim3 cxsparse)

add_executable(pose_estimation_3d2d pose_estimation_3d2d.cpp)

# 链接OpenCV库
target_link_libraries(pose_estimation_3d2d ${OpenCV_LIBS} )
# 链接g2o库
target_link_libraries(pose_estimation_3d2d ${G2O_LIBS})
# 链接CSparse库
target_link_libraries(pose_estimation_3d2d ${CSPARSE_LIBRARY})

install(TARGETS pose_estimation_3d2d RUNTIME DESTINATION bin)