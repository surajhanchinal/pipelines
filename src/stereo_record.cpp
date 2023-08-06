#include "camel_buffer.h"
#include "capture_signaler.h"
#include "common_data.h"
#include "frame_processor.h"
#include "frame_reader.h"
#include "frame_recorder_utils.h"
#include "frame_syncer.h"
#include "imgui.h"
#include "multi_gui_node.h"
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
#include <signal.h>
#include <time.h>

using namespace cv;
using namespace std;

int main() {

  const int height = 720;
  const int width = 1280;
  const cv::Size captureSize(width, height);
  bool shouldClose = true;

  auto captureSignaler = new CaptureSignaler();

  auto frameReader1 = new FrameReader(4, "left", captureSize);

  auto frameReader2 = new FrameReader(2, "right", captureSize);

  auto frameSyncer = new LightFrameSyncer();

  auto leftWriter = new GstVideoWriter("green_ball_left_5", captureSize);

  auto rightWriter = new GstVideoWriter("green_ball_right_5", captureSize);

  auto o1 = Orchestrator();

  o1.registerNode(captureSignaler, true);
  o1.registerNode(frameReader1);
  o1.registerNode(frameReader2);
  o1.registerNode(frameSyncer);
  o1.registerNode(leftWriter);
  o1.registerNode(rightWriter);

  captureSignaler->attachPort<0, 0>(frameReader1);
  captureSignaler->attachPort<1, 0>(frameReader2);

  frameReader1->attachPort<0, 0>(frameSyncer);

  frameReader2->attachPort<0, 1>(frameSyncer);

  frameSyncer->attachPort<0, 0>(leftWriter);

  frameSyncer->attachPort<1, 0>(rightWriter);

  o1.start();

  cout << "here" << endl;

  return 0;
}
