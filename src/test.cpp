#include "camel_buffer.h"
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

  auto node1 = new ExampleFirstNode();

  auto node2 = new ExampleSecondNode();

  auto o1 = Orchestrator();

  o1.registerNode(node1);
  o1.registerNode(node2);

  node1->attachPort<0, 0>(node2);
  node1->attachPort<1, 1>(node2);
  node1->attachPort<2, 2>(node2);

  // auto isValid = o1.start();

  VideoCapture cap(0, CAP_V4L2);
  cap.set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));

  cap.set(CAP_PROP_FRAME_HEIGHT, 1080);
  cap.set(CAP_PROP_FRAME_WIDTH, 1920);

  cap.set(CAP_PROP_AUTO_EXPOSURE, 1);
  cap.set(CAP_PROP_EXPOSURE, 400);

  namedWindow("frame1", WINDOW_OPENGL);
  resizeWindow("frame1", 1920, 1080);

  Mat frame;
  int fps_count = 0;
  int64 start = cv::getTickCount();
  while (1) {
    cap >> frame;
    if (frame.empty()) {
      break;
    }
    imshow("frame1", frame);
    char c = (char)waitKey(1);
    if (c == 27)
      break;
    if (fps_count == 60) {
      int64 end = getTickCount();
      double te = (end - start) / getTickFrequency();
      double fps = 60 / te;
      std::cout << "FPS : " << fps << std::endl;
      start = end;
      fps_count = 0;
    }
    fps_count++;
  }

  cap.release();
  destroyAllWindows();

  // cout << "Is graph valid: " << isValid << endl;

  return 0;
}
