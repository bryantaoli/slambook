cmake_minimum_required(VERSION 2.6)
project(ceres_curve_fitting)

SET(CMAKE_CXX_FLAGS "-std=c++11")

list( APPEND CMAKE_MODULE_PATH ${PROJECT_SOURCE_DIR}/cmake_modules )
# 寻找OpenCV库
find_package(OpenCV REQUIRED)
find_package(Ceres REQUIRED)
# 添加头文件
include_directories(${OpenCV_INCLUDED_DIRS})
include_directories( ${CERES_INCLUDE_DIRS})

add_executable(ceres_curve_fitting main.cpp)
# 链接OpenCV库
target_link_libraries(ceres_curve_fitting ${OpenCV_LIBS})
# 链接Ceres库
target_link_libraries(ceres_curve_fitting ${CERES_LIBRARIES})

install(TARGETS ceres_curve_fitting RUNTIME DESTINATION bin)
