#include "capture_signaler.h"
#include "file_reader.h"
#include "frame_processor.h"
#include "frame_syncer.h"
#include "imgui.h"
#include "multi_gui_node.h"
#include "node.h"
#include "orchestrator.h"
#include "types.h"
#include "utils.h"
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
 
  auto frameReader1 = new FileReader("../red_ball_left_5.mp4", captureSize);

  auto frameReader2 = new FileReader("../red_ball_right_5.mp4", captureSize);

  auto frameProcessor1 = new FrameProcessor(captureSize, 0);

  auto frameProcessor2 = new FrameProcessor(captureSize, 0);

  CameraParams cameraParams;
  loadCameraParams(cameraParams);
  auto frameSyncer = new FrameSyncer(cameraParams);
  auto captureSignaler = new CaptureSignaler();
  glfwSetErrorCallback(glfw_error_callback);
  glfwInit();

  glfwWindowHint(GLFW_CONTEXT_VERSION_MAJOR, 3);
  glfwWindowHint(GLFW_CONTEXT_VERSION_MINOR, 0);

  auto multiGuiNode = new MultiGuiNode("w1", captureSize,cameraParams);

  auto o1 = Orchestrator();
  o1.registerNode(captureSignaler);
  o1.registerNode(frameReader1);
  o1.registerNode(frameReader2);
  o1.registerNode(frameProcessor1);
  o1.registerNode(frameProcessor2);
  o1.registerNode(multiGuiNode, true);
  o1.registerNode(frameSyncer);

  captureSignaler->attachPort<0, 0>(frameReader1);
  captureSignaler->attachPort<1, 0>(frameReader2);

  frameReader1->attachPort<0, 0>(frameProcessor1);
  frameProcessor1->attachPort<0, 0>(frameSyncer);
  frameReader2->attachPort<0, 0>(frameProcessor2);
  frameProcessor2->attachPort<0, 1>(frameSyncer);

  frameSyncer->attachPort<0, 0>(multiGuiNode);

  cout << "size:" << sizeof(cv::Mat) << endl;

  auto isValid = o1.start();
  cout << "Is graph valid: " << isValid << endl;

  return 0;
}
