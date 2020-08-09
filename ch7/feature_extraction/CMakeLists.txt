cmake_minimum_required(VERSION 2.6)
project(feature_extraction)

find_package(OpenCV REQUIRED)

include_directories(${OpenCV_INCLUDED_DIRS})

add_executable(feature_extraction feature_extraction.cpp)

# 链接OpenCV库
target_link_libraries(feature_extraction ${OpenCV_LIBS})

install(TARGETS feature_extraction RUNTIME DESTINATION bin)