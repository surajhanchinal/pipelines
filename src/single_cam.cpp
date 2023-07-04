#include "camel_buffer.h"
#include "file_reader.h"
#include "frame_display.h"
#include "frame_processor.h"
#include "imgui.h"
#include "imgui_image_node.h"
#include "node.h"
#include "orchestrator.h"
#include <GL/gl.h>
#include <GL/glu.h>
#include <GLFW/glfw3.h>
#include <iostream>
#include <opencv2/core/utility.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <time.h>

// thread_local ImGuiContext *g_pcImGuiTLSContext{nullptr};
static void glfw_error_callback(int error, const char *description) {
  fprintf(stderr, "GLFW Error %d: %s\n", error, description);
}

using namespace cv;
using namespace std;

int main() {
  const int height = 720;
  const int width = 1280;
  const cv::Size captureSize(width, height);

  // auto frameReader1 = new FrameReader(0, "camera", captureSize);
  auto fileReader1 = new FileReader("../national_project_720.mp4", captureSize);

  auto frameProcessor1 = new FrameProcessor(captureSize, 0);

  glfwSetErrorCallback(glfw_error_callback);
  glfwInit();

  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);

  auto imguiImageNode = new ImGuiImageNode("w1", captureSize);
  // auto imguiImageNode2 = new ImGuiImageNode("w2", captureSize);

  auto o1 = Orchestrator();

  o1.registerNode(fileReader1);
  o1.registerNode(frameProcessor1);
  o1.registerNode(imguiImageNode, true);
  // o1.registerNode(imguiImageNode2);

  fileReader1->attachPort<0, 0>(frameProcessor1);
  frameProcessor1->attachPort<0, 0>(imguiImageNode);
  // frameProcessor1->attachPort<1, 0>(imguiImageNode2);

  auto isValid = o1.start();
  return 0;
}
