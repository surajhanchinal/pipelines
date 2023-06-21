#include "camel_buffer.h"
#include "frame_display.h"
#include "frame_processor.h"
#include "frame_reader.h"
#include "node.h"
#include "orchestrator.h"
#include <iostream>
#include <opencv2/core/utility.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
#include <time.h>
using namespace cv;
using namespace std;

int main() {

  auto frameReader = new FrameReader(0, "camera");

  auto frameDisplay = new FrameDisplay();
  auto frameProcessor = new FrameProcessor(1080, 1920, 3);

  auto o1 = Orchestrator();

  o1.registerNode(frameReader);
  o1.registerNode(frameDisplay, true);
  o1.registerNode(frameProcessor);

  frameReader->attachPort<0, 0>(frameProcessor);
  frameProcessor->attachPort<0, 0>(frameDisplay);
  frameProcessor->attachPort<1, 1>(frameDisplay);
  cout << "size:" << sizeof(cv::Mat) << endl;

  auto isValid = o1.start();
  cout << "Is graph valid: " << isValid << endl;

  return 0;
}
