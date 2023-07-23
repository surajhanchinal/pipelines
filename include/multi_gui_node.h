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
#include <chrono>
#include <stdio.h>
#include <string>
#include <utility>
using namespace std;

class MultiGuiNode
    : public Node<type_list_t<TimedMatWithCTree, TimedMatWithCTree>,
                  type_list_t<>> {
  std::vector<ImColor> colorPalette = {
      ImColor(83, 200, 33),  ImColor(255, 201, 40), ImColor(255, 148, 35),
      ImColor(255, 72, 162), ImColor(122, 71, 255), ImColor(42, 153, 235)};

public:
  MultiGuiNode(std::string _windowName, const cv::Size _imageSize) {
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

    while (!glfwWindowShouldClose(window)) {
      ImGui::SetCurrentContext(g_pcImGuiTLSContext);
      glfwPollEvents();

      // Start the Dear ImGui frame
      ImGui_ImplOpenGL3_NewFrame();

      ImGui_ImplGlfw_NewFrame();
      ImGui::NewFrame();

      auto frameTimedMat1 = readData<0, TimedMatWithCTree>();
      auto frameTimedMat2 = readData<1, TimedMatWithCTree>();

      vector<vector<ImVec2>> trajList1;
      vector<vector<ImVec2>> trajList2;
      vector<vector<vector<ImVec2>>> polyLineGroupList1;
      vector<vector<vector<ImVec2>>> polyLineGroupList2;
      renderFrame(frameTimedMat1, tex1, w1, polyLineGroupList1, trajList1);
      renderFrame(frameTimedMat2, tex2, w2, polyLineGroupList2, trajList2);
      renderMetricsWindow(frameTimedMat1.timestamp, frameTimedMat2.timestamp);

      /*vector<pair<double, pair<int, int>>> match_costs;
      for (int i = 0; i < trajList1.size(); i++) {
        if (trajList1[i].size() < 6) {
          continue;
        }
        auto &t1 = trajList1[i];

        double ocost = 0.5;
        int miniIdx = -1;

        vector<pair<int, vector<double>>> seqCosts;
        for (int j = 0; j < trajList2.size(); j++) {
          if (trajList2[j].size() < 6) {
            continue;
          }
          auto &t2 = trajList2[j];
          bool valid = true;
          bool sign = false;
          bool start = true;
          int prev = 300;
          double cost = 0;
          for (int z = 0; z < 6; z++) {
            auto &p1 = t1[t1.size() - 1 - z];
            auto &p2 = t2[t2.size() - 1 - z];

            auto xdiff = abs(p1.x - p2.x);
            if (start) {
              start = false;
              sign = (p1.x - p2.x) > 0;
            }
            if (xdiff > prev || ((p1.x - p2.x) > 0) != sign) {
              valid = false;
            } else {
              prev = xdiff;
            }
            cost += abs(p1.y - p2.y) / imageSize.height;
          }
          cost = cost / 4;
          if (cost < ocost) {
            ocost = cost;
            miniIdx = j;
          }
        }

        if (miniIdx != -1) {
          match_costs.push_back({ocost, {i, miniIdx}});
        }
      }
      double mini = 121312;
      int miniIdx = -1;

      for (int i = 0; i < match_costs.size(); i++) {
        if (match_costs[i].first < mini) {
          miniIdx = i;
          mini = match_costs[i].first;
        }
      }

      if (miniIdx != -1) {
        auto &mc = match_costs[miniIdx];
        vector<ImVec2> ntl1, ntl2;
        auto &t1 = trajList1[mc.second.first];
        auto &t2 = trajList2[mc.second.second];
        for (int i = 0; i < 4; i++) {
          ntl1.push_back(t1[t1.size() - 1 - i]);

          ntl2.push_back(t2[t2.size() - 1 - i]);
        }
        // actualTrajList1.push_back(ntl1);
        // actualTrajList2.push_back(ntl2);
        //  actualTrajList1 = {trajList1[mc.second.first]};
        //   actualTrajList2 = {trajList2[mc.second.second]};
        actualTrajList1.push_back(trajList1[mc.second.first]);

        actualTrajList2.push_back(trajList2[mc.second.second]);
      }

      {

        ImGui::Begin("actual");
        ImGui::SetWindowSize(ImVec2(imageSize.width, imageSize.height));
        auto io = ImGui::GetIO();
        auto cpos = ImGui::GetCursorScreenPos();
        // Draw using offset from where image begins. We use the cursor
        // position before the image is rendered to get this offset
        render_combined_contours(cpos, actualTrajList1, actualTrajList2);
        ImGui::End();
      }*/

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

  void render_contours(ImVec2 &p,
                       vector<vector<vector<ImVec2>>> &polyLineGroups,
                       vector<vector<ImVec2>> &trajList) {
    ImDrawList *draw_list = ImGui::GetWindowDrawList();
    // const ImVec2 p = ImGui::GetWindowPos();
    for (auto &polyLines : polyLineGroups) {
      for (auto &pl : polyLines) {
        for (auto &pt : pl) {
          pt.x = pt.x + p.x;
          pt.y = pt.y + p.y;
        }
        draw_list->AddPolyline(pl.data(), pl.size(), colorPalette[k % 6],
                               ImDrawFlags_Closed, 2);
      }
      k++;
      if (k > 1000) {
        k = 0;
      }
    }

    // draw lines instead of contours;
    for (auto &traj : trajList) {
      if (traj.size() > 1) {
        for (int i = 0; i < traj.size() - 1; i++) {
          draw_list->AddLine(ImVec2(p.x + traj[i].x, p.y + traj[i].y),
                             ImVec2(p.x + traj[i + 1].x, p.y + traj[i + 1].y),
                             colorPalette[k % 6], 1);
        }
      }
      k++;
      if (k > 1000) {
        k = 0;
      }
    }
  }

  void render_combined_contours(ImVec2 &p, vector<vector<ImVec2>> &trajList1,
                                vector<vector<ImVec2>> &trajList2) {
    ImDrawList *draw_list = ImGui::GetWindowDrawList();
    int k = 0;
    for (auto &traj : trajList1) {
      if (traj.size() > 1) {
        for (int i = 0; i < traj.size() - 1; i++) {
          draw_list->AddLine(ImVec2(p.x + traj[i].x, p.y + traj[i].y),
                             ImVec2(p.x + traj[i + 1].x, p.y + traj[i + 1].y),
                             colorPalette[k % 6], 2);
        }
      }
      k++;
    }
    k = 0;
    for (auto &traj : trajList2) {
      if (traj.size() > 1) {
        for (int i = 0; i < traj.size() - 1; i++) {
          draw_list->AddLine(ImVec2(p.x + traj[i].x, p.y + traj[i].y),
                             ImVec2(p.x + traj[i + 1].x, p.y + traj[i + 1].y),
                             colorPalette[k % 6], 2);
        }
      }
      k++;
    }
  }

private:
  int k = 0;
  GLuint tex1, tex2;
  std::string windowName;
  GLFWwindow *window;
  cv::Size imageSize;
  bool initCalled = false;

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
        GL_BGR,        // Input image format (i.e. GL_RGB, GL_RGBA, GL_BGR etc.)
        GL_UNSIGNED_BYTE, // Image data type
        frame.ptr());     // The actual image data itself
#endif
  }

  void extractAndCleanupTrajectories(
      std::vector<std::vector<std::vector<cv::Point>>> *cgl,
      vector<vector<vector<ImVec2>>> &polyLineGroupList,
      vector<vector<ImVec2>> &trajList) {
    for (auto g : (*cgl)) {
      std::vector<std::vector<ImVec2>> polyLines;
      for (auto ctr : g) {
        std::vector<ImVec2> contour;
        for (auto pt : ctr) {
          contour.push_back(ImVec2(pt.x, pt.y));
        }
        polyLines.push_back(contour);
      }
      polyLineGroupList.push_back(polyLines);
    }

    for (auto &g : (*cgl)) {
      vector<ImVec2> traj;
      for (auto &ctr : g) {
        traj.push_back(contourCenterPoint(ctr));
      }
      trajList.push_back(traj);
    }

    delete cgl;
  }

  void renderFrame(TimedMatWithCTree frameTimedMat, GLuint texID,
                   std::string &windowName,
                   vector<vector<vector<ImVec2>>> &polyLineGroupList,
                   vector<vector<ImVec2>> &trajList) {
    auto frame = frameTimedMat.mat;

    auto now = std::chrono::system_clock::now();
    auto delay = std::chrono::duration_cast<std::chrono::milliseconds>(
        now - frameTimedMat.timestamp);

    extractAndCleanupTrajectories(frameTimedMat.contourGroupList,
                                  polyLineGroupList, trajList);

    writeTexture(texID, frame);
    {

      ImGui::Begin(windowName.c_str());
      ImGui::SetWindowSize(ImVec2(imageSize.width, imageSize.height));
      auto io = ImGui::GetIO();
      auto cpos = ImGui::GetCursorScreenPos();
      ImGui::Image((void *)(intptr_t)texID,
                   ImVec2(imageSize.width, imageSize.height));
      // Draw using offset from where image begins. We use the cursor
      // position before the image is rendered to get this offset
      render_contours(cpos, polyLineGroupList, trajList);
      ImGui::End();
    }
  }

  void
  renderMetricsWindow(std::chrono::time_point<std::chrono::system_clock> f1Ts,
                      std::chrono::time_point<std::chrono::system_clock> f2Ts) {
    {

      auto now = std::chrono::system_clock::now();
      auto delay1 =
          std::chrono::duration_cast<std::chrono::milliseconds>(now - f1Ts);
      auto delay2 =
          std::chrono::duration_cast<std::chrono::milliseconds>(now - f2Ts);

      auto interDelay =
          std::chrono::duration_cast<std::chrono::microseconds>(f1Ts - f2Ts);

      ImGui::Begin("metrics", nullptr, ImGuiWindowFlags_AlwaysAutoResize);
      auto io = ImGui::GetIO();
      ImGui::Text("Framerate: (%.1f FPS)", io.Framerate);

      ImGui::BeginTable("Metrics", 3);
      ImGui::TableSetupColumn("Cam 1 delay");
      ImGui::TableSetupColumn("Cam 2 delay");
      ImGui::TableSetupColumn("Cross cam delay");
      ImGui::TableHeadersRow();
      ImGui::TableNextRow();
      ImGui::TableNextColumn();
      ImGui::Text("%li", delay1.count());
      ImGui::TableNextColumn();
      ImGui::Text("%li", delay2.count());
      ImGui::TableNextColumn();
      ImGui::Text("%li", interDelay.count());
      ImGui::EndTable();
      ImGui::End();
    }
  }
};