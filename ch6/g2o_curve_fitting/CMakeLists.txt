cmake_minimum_required(VERSION 2.6)
project(g2o_curve_fitting)

SET(CMAKE_CXX_FLAGS "-std=c++11")
# 因为g2o不是常用库，要添加它的findg2o.cmake文件
LIST( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules ) 

find_package(OpenCV REQUIRED)
SET( G2O_ROOT /usr/local/include/g2o ) 
FIND_PACKAGE(G2O)

include_directories(${OpenCV_INCLUDED_DIRS})
include_directories( ${G2O_INCLUDE_DIRS} )
include_directories("/usr/include/eigen3/Eigen")

add_executable(g2o_curve_fitting main.cpp)

# 链接OpenCV库
target_link_libraries(g2o_curve_fitting ${OpenCV_LIBS})
# 链接g2o库
target_link_libraries(g2o_curve_fitting g2o_core g2o_stuff)

install(TARGETS g2o_curve_fitting RUNTIME DESTINATION bin)
