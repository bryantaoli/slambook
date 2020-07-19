cmake_minimum_required(VERSION 2.6)
project(joinMap)

# 寻找OpenCV库
find_package(OpenCV REQUIRED)
# 添加头文件
include_directories(${OpenCV_INCLUDED_DIRS})

# Eigen库
include_directories( "/usr/include/eigen3/" )

# PCL库
find_package(PCL REQUIRED COMPONENT common io)
include_directories(${PCL_INCLUDE_DIRS})
add_definitions(${PCL_DEFINITIONS})

add_executable(joinMap joinMap.cpp)
target_link_libraries( joinMap ${OpenCV_LIBS} ${PCL_LIBRARIES} )

install(TARGETS joinMap RUNTIME DESTINATION bin)

