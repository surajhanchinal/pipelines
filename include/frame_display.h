#pragma once
#include "fps_counter.h"
#include "node.h"
#include <iostream>
#include <opencv2/core/utility.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/videoio.hpp>
using namespace std;
using input_type_fd = type_list_t<cv::Mat>;
using output_type_fd = type_list_t<>;

class FrameDisplay : public Node<input_type_fd, output_type_fd> {

  void process() {
    cv::namedWindow("frame1", cv::WINDOW_OPENGL);
    cv::resizeWindow("frame1", 1920, 1080);
    auto fc = FpsCounter(120);
    cv::Mat frame;
    while (1) {
      fc.loop();
      frame = readData<0, cv::Mat>();
      if (frame.empty()) {
        break;
      }
      imshow("frame1", frame);
      char c = (char)cv::waitKey(1);
      if (c == 27)
        break;
    }
    fc.reset();
  }

  ~FrameDisplay() { cv::destroyAllWindows(); }
};
