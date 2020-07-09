cmake_minimum_required(VERSION 2.6)
project(useeigen)

add_executable(useeigen main.cpp)
include_directories("/usr/include/eigen3")

install(TARGETS useeigen RUNTIME DESTINATION bin)
