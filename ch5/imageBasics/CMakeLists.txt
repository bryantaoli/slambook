cmake_minimum_required(VERSION 2.6)
project(imageBasics)
set(CMAKE_CXX_FLAGS "-std=c++11")

# 寻找OpenCV库
find_package(OpenCV REQUIRED)
# 添加头文件
include_directories(${OpenCV_INCLUDED_DIRS})
# include_directories("/usr/local/include/opencv2")
add_executable(imageBasics imageBasics.cpp)

# 链接OpenCV库
target_link_libraries(imageBasics ${OpenCV_LIBS})

install(TARGETS imageBasics RUNTIME DESTINATION bin)
