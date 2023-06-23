#include "camel_buffer.h"
#include "frame_display.h"
#include "frame_processor.h"
#include "frame_reader.h"
#include "imgui.h"
#include "imgui_image_node.h"
#include "node.h"
#include "orchestrator.h"
#include "window_node.h"
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

  auto frameReader1 = new FrameReader(0, "camera");

  auto frameProcessor1 = new FrameProcessor(720, 1280, 3);

  glfwSetErrorCallback(glfw_error_callback);
  glfwInit();

  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);

  auto window1 = glfwCreateWindow(1280, 720, "w1", nullptr, nullptr);

  auto imguiImageNode = new ImGuiImageNode("w1", window1);

  auto o1 = Orchestrator();

  o1.registerNode(frameReader1);
  o1.registerNode(frameProcessor1);
  o1.registerNode(imguiImageNode, true);

  frameReader1->attachPort<0, 0>(frameProcessor1);
  frameProcessor1->attachPort<0, 0>(imguiImageNode);

  auto isValid = o1.start();
  return 0;
}
