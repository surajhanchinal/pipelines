cmake_minimum_required(VERSION 2.8)
project(pipelines)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/build)

add_executable(PipeTest src/test.cpp src/some.cpp)

target_include_directories(PipeTest PRIVATE ${CMAKE_SOURCE_DIR}/include)
