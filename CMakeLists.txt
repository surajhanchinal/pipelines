cmake_minimum_required(VERSION 3.2)
project(pipelines)
find_package( OpenCV REQUIRED )
find_package(glfw3 REQUIRED)
find_package(GLEW REQUIRED)
set(CMAKE_FIND_DEBUG_MODE true)
find_package(OpenGL REQUIRED)
set(CMAKE_EXPORT_COMPILE_COMMANDS ON)

set(CMAKE_RUNTIME_OUTPUT_DIRECTORY ${CMAKE_BINARY_DIR}/build)

set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED On)
set(CMAKE_CXX_EXTENSIONS Off)


set(IMGUI_DIR ${CMAKE_SOURCE_DIR}/external/imgui)

add_library(IMGUI STATIC)

target_sources(IMGUI PRIVATE ${IMGUI_DIR}/imgui_demo.cpp ${IMGUI_DIR}/imgui_draw.cpp ${IMGUI_DIR}/imgui_tables.cpp ${IMGUI_DIR}/imgui_widgets.cpp ${IMGUI_DIR}/imgui.cpp
	PRIVATE
	${IMGUI_DIR}/backends/imgui_impl_opengl3.cpp
	${IMGUI_DIR}/backends/imgui_impl_glfw.cpp
	)

target_include_directories( IMGUI
                            PUBLIC ${IMGUI_DIR}
                            PUBLIC ${IMGUI_DIR}/backends
                            PUBLIC ${SDL3_DIR}/include
                            )


include_directories( ${OpenCV_INCLUDE_DIRS} )

add_executable(PipeTest src/test.cpp src/some.cpp)

target_link_libraries( PipeTest ${OpenCV_LIBS} ${CURSES_LIBRARIES} IMGUI)

add_executable(ImGuiTest ${IMGUI_DIR}/examples/example_glfw_opengl3/main.cpp)
target_link_libraries(ImGuiTest OpenGL glfw IMGUI)


add_executable(BareGuiTest src/bare_gui.cpp)
target_link_libraries(BareGuiTest OpenGL glfw IMGUI GLEW)

target_include_directories(PipeTest PRIVATE ${CMAKE_SOURCE_DIR}/include)
