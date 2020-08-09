cmake_minimum_required(VERSION 2.6)
project(pose_estimation_2d2d)

SET(CMAKE_CXX_FLAGS "-std=c++11")

find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDED_DIRS})

add_executable(pose_estimation_2d2d pose_estimation_2d2d.cpp)

# 链接OpenCV库
target_link_libraries(pose_estimation_2d2d ${OpenCV_LIBS})

install(TARGETS pose_estimation_2d2d RUNTIME DESTINATION bin)