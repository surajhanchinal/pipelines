#include "camel_buffer.h"
#include "frame_display.h"
#include "frame_processor.h"
#include "frame_reader.h"
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

using namespace cv;
using namespace std;

int main() {

  auto frameReader1 = new FrameReader(0, "camera");

  auto frameReader2 = new FrameReader(2, "camera");

  auto frameDisplay = new FrameDisplay("fr1");
  auto fD2 = new FrameDisplay("fr2");
  auto frameProcessor1 = new FrameProcessor(1080, 1920, 3);
  auto frameProcessor2 = new FrameProcessor(1080, 1920, 3);

  auto o1 = Orchestrator();

  o1.registerNode(frameReader1);
  o1.registerNode(frameReader2);
  o1.registerNode(frameDisplay, true);
  o1.registerNode(frameProcessor1);
  o1.registerNode(fD2);
  // o1.registerNode(frameProcessor2);

  // frameReader1->attachPort<0, 0>(frameProcessor1);

  frameReader2->attachPort<0, 0>(frameDisplay);

  frameReader1->attachPort<0, 0>(frameProcessor1);
  frameProcessor1->attachPort<0, 0>(fD2);
  // frameProcessor1->attachPort<0, 0>(frameDisplay);
  // frameProcessor1->attachPort<1, 1>(frameDisplay);

  // frameReader2->attachPort<0, 0>(frameProcessor2);
  // frameProcessor2->attachPort<0, 2>(frameDisplay);
  // frameProcessor2->attachPort<1, 3>(frameDisplay);
  cout << "size:" << sizeof(cv::Mat) << endl;

  auto isValid = o1.start();
  cout << "Is graph valid: " << isValid << endl;

  return 0;
}
