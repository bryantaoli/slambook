cmake_minimum_required(VERSION 2.6)
project(useSophus)

find_package(Sophus REQUIRED)
include_directories(${Sophus_INCLUDE_DIRS})


add_executable(usesophus useSophus.cpp)
include_directories("/usr/include/eigen3")
target_link_libraries(usesophus ${Sophus_LIBRARIES})
