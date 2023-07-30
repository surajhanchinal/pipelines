cmake_minimum_required(VERSION 3.2)
project(pipelines)
find_package( OpenCV REQUIRED )
find_package(glfw3 REQUIRED)
find_package(GLEW REQUIRED)
find_package(OpenGL REQUIRED)
find_package (Eigen3 3.3 REQUIRED NO_MODULE)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/build)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED On)
set(CMAKE_CXX_EXTENSIONS Off)


set(IMGUI_DIR ${CMAKE_SOURCE_DIR}/external/imgui)

add_library(IMGUI STATIC)

target_include_directories( IMGUI
                            PUBLIC ${IMGUI_DIR}
                            PUBLIC ${IMGUI_DIR}/backends
                            PUBLIC ${IMGUI_DIR}/include
                            )

target_sources(IMGUI PRIVATE ${IMGUI_DIR}/imgui_demo.cpp ${IMGUI_DIR}/imgui_draw.cpp ${IMGUI_DIR}/imgui_tables.cpp ${IMGUI_DIR}/imgui_widgets.cpp ${IMGUI_DIR}/imgui.cpp
	PRIVATE
	${IMGUI_DIR}/backends/imgui_impl_opengl3.cpp
	${IMGUI_DIR}/backends/imgui_impl_glfw.cpp
        src/imconfig.cpp
	)

include_directories( ${OpenCV_INCLUDE_DIRS} )


add_executable(PipeTest src/test.cpp)
target_link_libraries( PipeTest ${OpenCV_LIBS} OpenGL glfw IMGUI GLEW Eigen3::Eigen)
target_include_directories(PipeTest PRIVATE ${CMAKE_SOURCE_DIR}/include)

add_executable(MultiTest src/multi_file.cpp)
target_link_libraries( MultiTest ${OpenCV_LIBS} OpenGL glfw IMGUI GLEW Eigen3::Eigen)
target_include_directories(MultiTest PRIVATE ${CMAKE_SOURCE_DIR}/include)

add_executable(SingleTest src/single_cam.cpp)
target_link_libraries( SingleTest ${OpenCV_LIBS} OpenGL glfw IMGUI GLEW Eigen3::Eigen)
target_include_directories(SingleTest PRIVATE ${CMAKE_SOURCE_DIR}/include)


add_executable(RecorderTest src/stereo_record.cpp)
target_link_libraries( RecorderTest ${OpenCV_LIBS} OpenGL glfw IMGUI GLEW Eigen3::Eigen)
target_include_directories(RecorderTest PRIVATE ${CMAKE_SOURCE_DIR}/include)
