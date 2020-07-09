# 声明要求cmake的最低版本
cmake_minimum_required(VERSION 2.8)
# 声明一个cmake工程
project(HelloSLAM)

# 添加一个可执行程序
add_executable(helloSLAM helloSLAM.cpp)  
# 添加库文件
add_library(hello libHelloSLAM.cpp)
