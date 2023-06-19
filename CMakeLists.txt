cmake_minimum_required(VERSION 2.8)
project(pipelines)
find_package( OpenCV REQUIRED )
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/build)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED On)
set(CMAKE_CXX_EXTENSIONS Off)

include_directories( ${OpenCV_INCLUDE_DIRS} )

add_executable(PipeTest src/test.cpp src/some.cpp)

target_link_libraries( PipeTest ${OpenCV_LIBS} ${CURSES_LIBRARIES})

target_include_directories(PipeTest PRIVATE ${CMAKE_SOURCE_DIR}/include)
