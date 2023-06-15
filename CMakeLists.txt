cmake_minimum_required(VERSION 2.8)
project(pipelines)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/build)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED On)
set(CMAKE_CXX_EXTENSIONS Off)


add_executable(PipeTest src/test.cpp src/some.cpp)

target_include_directories(PipeTest PRIVATE ${CMAKE_SOURCE_DIR}/include)
