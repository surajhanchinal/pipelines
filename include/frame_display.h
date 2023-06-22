#pragma once
#include "fps_counter.h"
#include "node.h"
#include <iostream>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/utility.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
using namespace std;
using input_type_fd = type_list_t<cv::Mat>;
using output_type_fd = type_list_t<>;

class FrameDisplay : public Node<input_type_fd, output_type_fd> {

public:
  FrameDisplay(string _windowName) { this->windowName = _windowName; }

  void process() {
    cout << "hey: " << windowName << endl;
    cv::namedWindow(windowName, cv::WINDOW_OPENGL);
    cv::resizeWindow(windowName, 1920, 1080);
    cout << "hey after: " << windowName << endl;

    // cv::namedWindow("frame2", cv::WINDOW_OPENGL);
    // cv::resizeWindow("frame2", 1920, 1080);
    auto fc = FpsCounter(120);
    cv::Mat frame1;
    cv::Mat frame2;
    while (1) {
      fc.loop();
      frame1 = readData<0, cv::Mat>();
      // frame2 = readData<1, cv::Mat>();
      // frame2 = readData<1, cv::Mat>();
      // frame4 = readData<3, cv::Mat>();

      if (frame1.empty()) {
        break;
      }
      // cout << frame1.size() << " " << frame2.size() << endl;
      imshow(windowName, frame1);
      // imshow("frame2", frame2);
      char c = (char)cv::waitKey(1);
      if (c == 27)
        break;
    }
    fc.reset();
  }

  string windowName;

  ~FrameDisplay() { cv::destroyAllWindows(); }
};
