#pragma once

#include "contour_tree.h"
#include "imconfig.h"
#include "imgui.h"
#include "imgui_impl_glfw.h"
#include "imgui_impl_opengl3.h"
#include "node.h"
#include "opencv2/core/mat.hpp"
#include "opencv2/imgproc.hpp"
#include "trajectory_store.h"
#include "types.h"
#include "utils.h"
#include <GLFW/glfw3.h> // Will drag system OpenGL headers
#include <chrono>
#include <stdio.h>
#include <string>
#include <utility>
using namespace std;

class MultiGuiNode : public Node<type_list_t<CameraPairData>, type_list_t<>> {
  std::vector<ImColor> colorPalette = {
      ImColor(83, 200, 33),  ImColor(255, 201, 40), ImColor(255, 148, 35),
      ImColor(255, 72, 162), ImColor(122, 71, 255), ImColor(42, 153, 235)};

public:
  MultiGuiNode(std::string _windowName, const cv::Size _imageSize,
               CameraParams cameraParams) {
    windowName = _windowName;
    imageSize = _imageSize;

    trajectoryStore = new TrajectoryStore(cameraParams);

    // Window creation is in the constructor as it needs to be done in the main
    // thread. Also have other things such as texture creation and ImGui context
    // creation because why not.
    // GL 3.0 + GLSL 130

    window = glfwCreateWindow(imageSize.width, imageSize.height,
                              windowName.c_str(), nullptr, nullptr);
    glfwMaximizeWindow(window);
    glfwMakeContextCurrent(window);
    glfwSwapInterval(0); // Disable  vsync

    // That's all here. We need to create ImGui context in the thread to make
    // use of the threadlocal thingie
  }

  void init() {

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

    createTexture(tex1);
    createTexture(tex2);

    ImGui_ImplOpenGL3_CreateDeviceObjects();
    initCalled = true;
  }

  void process() {

    ImVec4 clear_color = ImVec4(0.45f, 0.55f, 0.60f, 1.00f);
    if (!initCalled) {
      init();
    }

    std::cout << "init done" << std::endl;

    std::cout << " process:  " << g_pcImGuiTLSContext << std::endl;
    std::string w1 = "cam1";
    std::string w2 = "cam2";

    vector<vector<ImVec2>> actualTrajList1;
    vector<vector<ImVec2>> actualTrajList2;

    while (*running && !glfwWindowShouldClose(window)) {
      ImGui::SetCurrentContext(g_pcImGuiTLSContext);
      glfwPollEvents();

      // Start the Dear ImGui frame
      ImGui_ImplOpenGL3_NewFrame();

      ImGui_ImplGlfw_NewFrame();
      ImGui::NewFrame();

      auto cameraPairData = readData<0, CameraPairData>();

      auto frameTimedMat1 = cameraPairData.leftTMCT;
      auto frameTimedMat2 = cameraPairData.rightTMCT;
      auto trajectories = cameraPairData.trajectories;

      for (auto &traj : *trajectories) {
        trajectoryStore->addTrajectory(traj);
      }

      screenSize = ImGui::GetIO().DisplaySize;

      // auto windowHeight = screenSize.y / 2.0;
      // ratio = windowHeight / imageSize.height;
      auto windowWidth = screenSize.x / 2.5;
      ratio = windowWidth / imageSize.width;

      windowSize = ImVec2(ratio * imageSize.width, ratio * imageSize.height);

      trajectoryStore->setScreenSize(screenSize);
      trajectoryStore->setWindowSize(windowSize);
      trajectoryStore->setCurrentTIme(frameTimedMat1.timestamp);

      ImGui::PushStyleVar(ImGuiStyleVar_WindowPadding, ImVec2(0, 0));
      ImGui::SetNextWindowPos(ImVec2(0, 0));
      renderFrame(frameTimedMat1, tex1, w1);
      ImGui::SetNextWindowPos(ImVec2(0, windowSize.y));
      renderFrame(frameTimedMat2, tex2, w2);
      ImGui::SetNextWindowPos(ImVec2(windowSize.x, 0));
      renderMetricsWindow(frameTimedMat1.timestamp, frameTimedMat2.timestamp,*trajectories);

      ImGui::SetNextWindowPos(ImVec2(windowSize.x, windowSize.y));
      renderYZFrame(*trajectories);

      ImGui::SetNextWindowPos(ImVec2(windowSize.x * 2.0, 0));
      renderXZFrame(*trajectories);

      ImGui::PopStyleVar();

      delete trajectories;

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
    std::cout << "Closing the gui node" << std::endl;
    // Cleanup
    ImGui_ImplOpenGL3_Shutdown();
    ImGui_ImplGlfw_Shutdown();
    ImGui::DestroyContext(g_pcImGuiTLSContext);

    glfwDestroyWindow(window);
    glfwTerminate();
  }

  void render_contours2(ImVec2 &p, vector<SingleTrajectory> *trajectories) {
    ImDrawList *draw_list = ImGui::GetWindowDrawList();
    for (const auto &traj : *trajectories) {
      for (auto const &ctr : traj.tc) {
        vector<ImVec2> polyLine;
        for (auto const &pt : ctr.contour) {
          polyLine.push_back(ImVec2(p.x + pt.x * ratio, p.y + pt.y * ratio));
        }
        draw_list->AddPolyline(polyLine.data(), polyLine.size(),
                               colorPalette[traj.id % 6], ImDrawFlags_Closed,
                               2);
      }
    }
    /*for (auto &traj : *trajectories) {
      if (traj.tc.size() <= 1) {
        continue;
      }
      for (int i = 0; i < traj.tc.size() - 1; i++) {
        auto ctr1 = contourCenterPoint(traj.tc[i].contour);
        auto ctr2 = contourCenterPoint(traj.tc[i + 1].contour);

        draw_list->AddLine(ImVec2(p.x + ctr1.x*ratio, p.y + ctr1.y*ratio),
                           ImVec2(p.x + ctr2.x*ratio, p.y + ctr2.y*ratio),
                           colorPalette[traj.id % 6], 1);
      }
    }*/
  }

  void renderYZFrame(vector<CombinedTrajectory> &trajectories) {
    ImGui::Begin("yz", nullptr, ImGuiWindowFlags_NoDecoration);
    ImGui::SetWindowSize(windowSize);
    auto cpos = ImGui::GetCursorScreenPos();
    // Draw using offset from where image begins. We use the cursor
    // position before the image is rendered to get this offset
    // render_combined_contours(cpos, *trajectories);
    trajectoryStore->render_yz(cpos);
    ImGui::End();
  }

  void renderXZFrame(vector<CombinedTrajectory> &trajectories) {
    auto height = 2 * windowSize.y;
    auto width = screenSize.x - 2 * windowSize.x;

    ImGui::Begin("xz", nullptr, ImGuiWindowFlags_NoDecoration);
    ImGui::SetWindowSize(ImVec2(width, height));
    auto cpos = ImGui::GetCursorScreenPos();
    // Draw using offset from where image begins. We use the cursor
    // position before the image is rendered to get this offset
    trajectoryStore->render_xz(cpos);
    ImGui::End();
  }

  void render_combined_contours(ImVec2 &p,
                                vector<CombinedTrajectory> &trajectories) {
    ImDrawList *draw_list = ImGui::GetWindowDrawList();
    for (auto const &trajectory : trajectories) {
      for (auto const &atctr : trajectory.atc) {
        vector<ImVec2> leftPolyLine;
        vector<ImVec2> rightPolyLine;
        for (auto const &pt : atctr.lt.contour) {
          leftPolyLine.push_back(
              ImVec2(p.x + pt.x * ratio, p.y + pt.y * ratio));
        }
        draw_list->AddPolyline(leftPolyLine.data(), leftPolyLine.size(),
                               colorPalette[trajectory.lt.id % 6],
                               ImDrawFlags_Closed, 2);
        for (auto const &pt : atctr.rt.contour) {
          rightPolyLine.push_back(
              ImVec2(p.x + pt.x * ratio, p.y + pt.y * ratio));
        }

        draw_list->AddPolyline(rightPolyLine.data(), rightPolyLine.size(),
                               colorPalette[trajectory.rt.id % 6],
                               ImDrawFlags_Closed, 2);
      }
    }
  }

private:
  int k = 0;
  GLuint tex1, tex2;
  std::string windowName;
  GLFWwindow *window;
  cv::Size imageSize;
  bool initCalled = false;
  ImVec2 windowSize;
  float ratio;
  ImVec2 screenSize;
  TrajectoryStore *trajectoryStore;

  ImVec2 contourCenterPoint(std::vector<cv::Point> &contour) {
    cv::Moments M = cv::moments(contour);
    int X = M.m10 / M.m00;
    int Y = M.m01 / M.m00;
    return ImVec2(X, Y);
  }

  void createTexture(GLuint &textureID) {
    glTexEnvi(GL_TEXTURE_ENV, GL_TEXTURE_ENV_MODE, GL_REPLACE);
    glGenTextures(1, &textureID); // Gen a new texture and store the handle
    glBindTexture(GL_TEXTURE_2D,
                  textureID); // Allocate GPU memory for handle (Texture ID)

    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MIN_FILTER, GL_NEAREST);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_MAG_FILTER, GL_LINEAR);

    // Set texture clamping method
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_S, GL_CLAMP);
    glTexParameteri(GL_TEXTURE_2D, GL_TEXTURE_WRAP_T, GL_CLAMP);
  }

  void writeTexture(GLuint textureID, cv::Mat &frame) {
    glBindTexture(GL_TEXTURE_2D,
                  textureID); // Allocate GPU memory for handle (Texture ID)
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
  }

  void renderFrame(TimedMatWithCTree frameTimedMat, GLuint texID,
                   std::string &windowName) {
    auto frame = frameTimedMat.mat;

    writeTexture(texID, frame);
    {
      ImGui::Begin(windowName.c_str(), nullptr, ImGuiWindowFlags_NoDecoration);
      ImGui::SetWindowSize(windowSize);
      auto io = ImGui::GetIO();
      auto cpos = ImGui::GetCursorScreenPos();
      ImGui::Image((void *)(intptr_t)texID, windowSize);
      // Draw using offset from where image begins. We use the cursor
      // position before the image is rendered to get this offset
      render_contours2(cpos, frameTimedMat.contourGroupList);
      if (!windowName.compare("cam1")) {
        trajectoryStore->renderLeftPred(cpos);
      } else {
        trajectoryStore->renderRightPred(cpos);
      }
      delete frameTimedMat.contourGroupList;
      ImGui::End();
    }
  }

  void
  renderMetricsWindow(std::chrono::time_point<std::chrono::system_clock> f1Ts,
                      std::chrono::time_point<std::chrono::system_clock> f2Ts,
                      vector<CombinedTrajectory> &trajectories) {
    {

      auto now = std::chrono::system_clock::now();
      auto delay1 = scaledDelayInMs(now, f1Ts);
      auto delay2 = scaledDelayInMs(now, f2Ts);

      auto interDelay = scaledDelayInMs(f1Ts, f2Ts);

      ImGui::Begin("metrics", nullptr, ImGuiWindowFlags_NoDecoration);
      ImGui::SetWindowSize(windowSize);
      auto io = ImGui::GetIO();
      auto cpos = ImGui::GetCursorScreenPos();
      ImGui::Text("Framerate: (%.1f FPS)", io.Framerate);

      ImGui::BeginTable("Metrics", 3);
      ImGui::TableSetupColumn("Cam 1 delay");
      ImGui::TableSetupColumn("Cam 2 delay");
      ImGui::TableSetupColumn("Cross cam delay");
      ImGui::TableHeadersRow();
      ImGui::TableNextRow();
      ImGui::TableNextColumn();
      ImGui::Text("%.1f", delay1);
      ImGui::TableNextColumn();
      ImGui::Text("%.1f", delay2);
      ImGui::TableNextColumn();
      ImGui::Text("%.1f", interDelay);
      ImGui::EndTable();
      render_combined_contours(cpos, trajectories);

      trajectoryStore->renderMetrics();
      ImGui::End();
    }
  }
};
