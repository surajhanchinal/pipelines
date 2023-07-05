#pragma once

#include "imconfig.h"
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include "node.h"
#include "opencv2/core/mat.hpp"
#include "opencv2/imgproc.hpp"
#include "types.h"
#include <GLFW/glfw3.h> // Will drag system OpenGL headers
#include <stdio.h>
#include <string>
using namespace std;

class ImGuiImageNode : public Node<type_list_t<TimedMat>, type_list_t<>> {

public:
  ImGuiImageNode(std::string _windowName, const cv::Size _imageSize) {
    windowName = _windowName;
    imageSize = _imageSize;

    // Window creation is in the constructor as it needs to be done in the main
    // thread. Also have other things such as texture creation and ImGui context
    // creation because why not.
    // GL 3.0 + GLSL 130

    window = glfwCreateWindow(imageSize.width, imageSize.height,
                              windowName.c_str(), nullptr, nullptr);

    glfwMakeContextCurrent(window);
    glfwSwapInterval(0); // Disable  vsync

    // That's all here. We need to create ImGui context in the thread to make
    // use of the threadlocal thingie
  }

  void initImGui() {

    glfwMakeContextCurrent(window);
    const char *glsl_version = "#version 130";
    // Setup Dear ImGui context
    IMGUI_CHECKVERSION();
    g_pcImGuiTLSContext = ImGui::CreateContext();
    auto io = ImGui::GetIO();
    io.ConfigFlags |=
        ImGuiConfigFlags_NavEnableKeyboard; // Enable Keyboard Controls
    io.ConfigFlags |=
        ImGuiConfigFlags_NavEnableGamepad; // Enable Gamepad Control
    // Setup Dear ImGui style
    ImGui::StyleColorsDark();
    // ImGui::StyleColorsLight();

    // Setup Platform/Renderer backends
    ImGui_ImplGlfw_InitForOpenGL(window, true);
    ImGui_ImplOpenGL3_Init(glsl_version);

    glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
    glGenTextures(1, &videotex); // Gen a new texture and store the handle
    glBindTexture(GL_TEXTURE_2D,
                  videotex); // Allocate GPU memory for handle (Texture ID)

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    // Set texture clamping method
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
#ifdef SSOPTIMIZED
    // glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_SWIZZLE_G, GL_RED);
    // glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_SWIZZLE_B, GL_RED);
#endif
  }

  void process() {

    ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);

    initImGui();

    std::cout << "init done" << std::endl;

    std::cout << " process:  " << g_pcImGuiTLSContext << std::endl;

    while (!glfwWindowShouldClose(window)) {
      ImGui::SetCurrentContext(g_pcImGuiTLSContext);
      glfwPollEvents();

      // Start the Dear ImGui frame
      ImGui_ImplOpenGL3_NewFrame();

      ImGui_ImplGlfw_NewFrame();
      ImGui::NewFrame();

      glBindTexture(GL_TEXTURE_2D,
                    videotex); // Allocate GPU memory for handle (Texture ID)

      // glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
      // glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

      // Set texture clamping method
      // glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
      // glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
      // auto frame = readData<0, TimedMat>();
      auto frameTimedMat = readData<0, TimedMat>();
      auto frame = frameTimedMat.mat;
      auto now = std::chrono::system_clock::now();

      auto delay = std::chrono::duration_cast<std::chrono::milliseconds>(
          now - frameTimedMat.timestamp);
#ifdef SSOPTIMIZED
      glTexImage2D(
          GL_TEXTURE_2D, // Type of texture
          0,             // Pyramid level (for mip-mapping) - 0 is the top level
          GL_LUMINANCE,  // Internal colour format to convert to
          frame.cols,    // Image width  i.e. 640 for Kinect in standard mode
          frame.rows,    // Image height i.e. 480 for Kinect in standard mode
          0,             // Border width in pixels (can either be 1 or 0)
          GL_LUMINANCE,  // Input image format (i.e. GL_RGB, GL_RGBA, GL_BGR
                         // etc.)
          GL_UNSIGNED_BYTE, // Image data type
          frame.ptr());     // The actual image data itself
#else
      glTexImage2D(
          GL_TEXTURE_2D, // Type of texture
          0,             // Pyramid level (for mip-mapping) - 0 is the top level
          GL_RGB,        // Internal colour format to convert to
          frame.cols,    // Image width  i.e. 640 for Kinect in standard mode
          frame.rows,    // Image height i.e. 480 for Kinect in standard mode
          0,             // Border width in pixels (can either be 1 or 0)
          GL_BGR, // Input image format (i.e. GL_RGB, GL_RGBA, GL_BGR etc.)
          GL_UNSIGNED_BYTE, // Image data type
          frame.ptr());     // The actual image data itself
#endif

      {

        // Show video cam0
        ImGui::Begin("cam0");
        ImGui::SetWindowSize(ImVec2(imageSize.width, imageSize.height + 80));
        // ImGui::Text("pointer = %p", videotex);A
        auto io = ImGui::GetIO();
        ImGui::SameLine();
        ImGui::Text("Application average %.3f ms/frame (%.1f FPS) %li",
                    1000.0f / io.Framerate, io.Framerate, delay.count());

        ImGui::Text("size = %d x %d", imageSize.width, imageSize.height);
        ImGui::Image((void *)(intptr_t)videotex,
                     ImVec2(imageSize.width, imageSize.height));
        render_conan_logo();
        ImGui::End();
      }

      // Rendering
      ImGui::Render();
      int display_w, display_h;
      glfwGetFramebufferSize(window, &display_w, &display_h);
      glViewport(0, 0, display_w, display_h);
      glClearColor(clear_color.x * clear_color.w, clear_color.y * clear_color.w,
                   clear_color.z * clear_color.w, clear_color.w);
      glClear(GL_COLOR_BUFFER_BIT);
      ImGui_ImplOpenGL3_RenderDrawData(ImGui::GetDrawData());

      glfwSwapBuffers(window);
    }
    // Cleanup
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext(g_pcImGuiTLSContext);

    glfwDestroyWindow(window);
    glfwTerminate();
  }

  void render_conan_logo() {
    ImDrawList *draw_list = ImGui::GetWindowDrawList();
    float sz = 300.0f;
    static ImVec4 col1 = ImVec4(68.0 / 255.0, 83.0 / 255.0, 89.0 / 255.0, 1.0f);
    static ImVec4 col2 = ImVec4(40.0 / 255.0, 60.0 / 255.0, 80.0 / 255.0, 1.0f);
    static ImVec4 col3 = ImVec4(50.0 / 255.0, 65.0 / 255.0, 82.0 / 255.0, 1.0f);
    static ImVec4 col4 = ImVec4(20.0 / 255.0, 40.0 / 255.0, 60.0 / 255.0, 1.0f);
    // const ImVec2 p = ImGui::GetCursorScreenPos();
    const ImVec2 p = ImGui::GetWindowPos();
    float x = p.x + 4.0f, y = p.y + 4.0f;
    draw_list->AddQuadFilled(
        ImVec2(x, y + 0.25 * sz), ImVec2(x + 0.5 * sz, y + 0.5 * sz),
        ImVec2(x + sz, y + 0.25 * sz), ImVec2(x + 0.5 * sz, y), ImColor(col1));
    draw_list->AddQuadFilled(ImVec2(x, y + 0.25 * sz),
                             ImVec2(x + 0.5 * sz, y + 0.5 * sz),
                             ImVec2(x + 0.5 * sz, y + 1.0 * sz),
                             ImVec2(x, y + 0.75 * sz), ImColor(col2));
    draw_list->AddQuadFilled(ImVec2(x + 0.5 * sz, y + 0.5 * sz),
                             ImVec2(x + sz, y + 0.25 * sz),
                             ImVec2(x + sz, y + 0.75 * sz),
                             ImVec2(x + 0.5 * sz, y + 1.0 * sz), ImColor(col3));
    draw_list->AddLine(ImVec2(x + 0.75 * sz, y + 0.375 * sz),
                       ImVec2(x + 0.75 * sz, y + 0.875 * sz), ImColor(col4));
    draw_list->AddBezierCubic(ImVec2(x + 0.72 * sz, y + 0.24 * sz),
                              ImVec2(x + 0.68 * sz, y + 0.15 * sz),
                              ImVec2(x + 0.48 * sz, y + 0.13 * sz),
                              ImVec2(x + 0.39 * sz, y + 0.17 * sz),
                              ImColor(col4), 10, 18);
    draw_list->AddBezierCubic(ImVec2(x + 0.39 * sz, y + 0.17 * sz),
                              ImVec2(x + 0.2 * sz, y + 0.25 * sz),
                              ImVec2(x + 0.3 * sz, y + 0.35 * sz),
                              ImVec2(x + 0.49 * sz, y + 0.38 * sz),
                              ImColor(col4), 10, 18);
  }

private:
  GLuint videotex;
  std::string windowName;
  GLFWwindow *window;
  cv::Size imageSize;
};
