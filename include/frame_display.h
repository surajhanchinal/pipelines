#pragma once
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

    cv::Mat frame;
    int fps_count = 0;
    int64 start = cv::getTickCount();
    while (1) {
      frame = readData<0, cv::Mat>();
      if (frame.empty()) {
        break;
      }
      imshow("frame1", frame);
      char c = (char)cv::waitKey(1);
      if (c == 27)
        break;
      if (fps_count == 60) {
        int64 end = cv::getTickCount();
        double te = (end - start) / cv::getTickFrequency();
        double fps = 60 / te;
        std::cout << "FPS : " << fps << std::endl;
        start = end;
        fps_count = 0;
      }
      fps_count++;
    }
  }

  ~FrameDisplay() { cv::destroyAllWindows(); }
};
