#pragma once

#include "imconfig.h"
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include "node.h"
#include "opencv2/core/mat.hpp"
#include "opencv2/imgproc.hpp"
#include <GLFW/glfw3.h> // Will drag system OpenGL headers
#include <stdio.h>
#include <string>

class ImGuiImageNode : public Node<type_list_t<cv::Mat>, type_list_t<>> {

public:
  ImGuiImageNode(std::string _windowName, GLFWwindow *_window) {
    windowName = _windowName;
    window = _window;
  }
  void init() {
    // GL 3.0 + GLSL 130
    const char *glsl_version = "#version 130";

    glfwMakeContextCurrent(window);
    glfwSwapInterval(0); // Enable vsync

    // Setup Dear ImGui context
    IMGUI_CHECKVERSION();
    g_pcImGuiTLSContext = ImGui::CreateContext();
    io = ImGui::GetIO();
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
  }

  void process() {

    std::cout << " process:  " << g_pcImGuiTLSContext << std::endl;
    ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);

    init();

    glfwMakeContextCurrent(window);
    while (!glfwWindowShouldClose(window)) {
      ImGui::SetCurrentContext(g_pcImGuiTLSContext);
      glfwPollEvents();

      // Start the Dear ImGui frame
      ImGui_ImplOpenGL3_NewFrame();

      ImGui_ImplGlfw_NewFrame();
      ImGui::NewFrame();

      // 2. Show a simple window that we create ourselves. We use a Begin/End
      // pair to create a named window.
      {
        static float f = 0.0f;
        static int counter = 0;

        ImGui::Begin("Hello, world!"); // Create a window called "Hello, world!"
                                       // and append into it.

        ImGui::Text("This is some useful text."); // Display some text (you can
                                                  // use a format strings too)

        ImGui::SliderFloat(
            "float", &f, 0.0f,
            1.0f); // Edit 1 float using a slider from 0.0f to 1.0f
        ImGui::ColorEdit3(
            "clear color",
            (float *)&clear_color); // Edit 3 floats representing a color

        if (ImGui::Button(
                "Button")) // Buttons return true when clicked (most widgets
                           // return true when edited/activated)
          counter++;
        ImGui::SameLine();
        ImGui::Text("counter = %d", counter);
        auto io = ImGui::GetIO();
        ImGui::Text("Application average %.3f ms/frame (%.1f FPS)",
                    1000.0f / io.Framerate, io.Framerate);
        ImGui::End();
      }

      glBindTexture(GL_TEXTURE_2D,
                    videotex); // Allocate GPU memory for handle (Texture ID)

      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

      // Set texture clamping method
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
      glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);

      auto frame = readData<0, cv::Mat>();

      cv::cvtColor(frame, frame, cv::COLOR_BGR2RGB);
      glTexImage2D(
          GL_TEXTURE_2D, // Type of texture
          0,             // Pyramid level (for mip-mapping) - 0 is the top level
          GL_RGB,        // Internal colour format to convert to
          frame.cols,    // Image width  i.e. 640 for Kinect in standard mode
          frame.rows,    // Image height i.e. 480 for Kinect in standard mode
          0,             // Border width in pixels (can either be 1 or 0)
          GL_RGB, // Input image format (i.e. GL_RGB, GL_RGBA, GL_BGR etc.)
          GL_UNSIGNED_BYTE, // Image data type
          frame.ptr());     // The actual image data itself

      {

        // Show video cam0
        ImGui::Begin("cam0");
        ImGui::Text("pointer = %p", videotex);
        ImGui::Text("size = %d x %d", 1920, 1080);
        ImGui::Image((void *)(intptr_t)videotex, ImVec2(1280, 720));
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

private:
  GLuint videotex;
  ImGuiContext *myCtx;
  std::string windowName;
  GLFWwindow *window;
  ImGuiIO io;
};
