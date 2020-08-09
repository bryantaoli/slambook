cmake_minimum_required(VERSION 2.6)
project(triangulation)

SET(CMAKE_CXX_FLAGS "-std=c++11")

find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDED_DIRS})

add_executable(triangulation triangulation.cpp)

# 链接OpenCV库
target_link_libraries(triangulation ${OpenCV_LIBS})

install(TARGETS triangulation RUNTIME DESTINATION bin)