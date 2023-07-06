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

class ImGuiImageNode
    : public Node<type_list_t<TimedMatWithCTree>, type_list_t<>> {

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
      auto frameTimedMat = readData<0, TimedMatWithCTree>();
      auto frame = frameTimedMat.mat;
      std::vector<std::vector<std::vector<cv::Point>>> *cgl =
          frameTimedMat.contourGroupList;
      auto now = std::chrono::system_clock::now();
      /*for (int i = 0; i < cgl->size(); i++) {
        cout << (*cgl)[i].size() << " ";
      }
      cout << endl;*/

      std::vector<std::vector<ImVec2>> polyLineGroupList;

      for (auto g : (*cgl)) {
        for (auto ctr : g) {
          std::vector<ImVec2> polyLine;
          for (auto pt : ctr) {
            polyLine.push_back(ImVec2(pt.x, pt.y));
          }
          polyLineGroupList.push_back(polyLine);
        }
      }

      delete cgl;

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
        auto io = ImGui::GetIO();
        ImGui::SameLine();
        ImGui::Text("Application average %.3f ms/frame (%.1f FPS) %li",
                    1000.0f / io.Framerate, io.Framerate, delay.count());

        ImGui::Text("size = %d x %d", imageSize.width, imageSize.height);
        auto cpos = ImGui::GetCursorScreenPos();
        ImGui::Image((void *)(intptr_t)videotex,
                     ImVec2(imageSize.width, imageSize.height));
        // Draw using offset from where image begins. We use the cursor position
        // before the image is rendered to get this offset
        render_contours(cpos, polyLineGroupList);
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

  void render_contours(ImVec2 &p, vector<vector<ImVec2>> &polyLines) {
    ImDrawList *draw_list = ImGui::GetWindowDrawList();
    // const ImVec2 p = ImGui::GetWindowPos();
    for (auto &pl : polyLines) {
      for (auto &pt : pl) {
        pt.x = pt.x + p.x;
        pt.y = pt.y + p.y;
      }
      draw_list->AddPolyline(
          pl.data(), pl.size(),
          ImColor(76.0f / 256.0, 175.0f / 256.0, 80.0f / 256.0, 1.0f),
          ImDrawFlags_Closed, 2);
    }
  }

private:
  GLuint videotex;
  std::string windowName;
  GLFWwindow *window;
  cv::Size imageSize;
};
