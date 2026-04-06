cmake_minimum_required(VERSION 3.31)
project(DeepSpace VERSION 0.1.0 LANGUAGES CXX)

set(CMAKE_CXX_STANDARD 20)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# 收集 DeepSpace 源文件
file(GLOB_RECURSE SOURCES "src/*.cpp" "src/*.h")

add_executable(DeepSpace ${SOURCES})

# 添加包含目录
# src: 用于 DeepSpaceApp.h 等
# sdk/mock: 用于 PrismaEngine.h 和 EntryPoint.h
target_include_directories(DeepSpace PRIVATE 
    src
    sdk/mock
)
