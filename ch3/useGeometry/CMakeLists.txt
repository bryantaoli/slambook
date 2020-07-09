cmake_minimum_required(VERSION 2.6)
project(usegeometry)

add_executable(usegeometry useGeometry.cpp)
include_directories("/usr/include/eigen3")

install(TARGETS usegeometry RUNTIME DESTINATION bin)
