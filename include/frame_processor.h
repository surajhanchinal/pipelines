#pragma once
#include "atomicops.h"
#include "fps_counter.h"
#include "frame_buffer.h"
#include "node.h"
#include <iostream>
#include <opencv2/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc.hpp>

namespace frame_processor {
using input_type = type_list_t<cv::Mat>;
using output_type = type_list_t<cv::Mat>;
}; // namespace frame_processor

class FrameProcessor
    : public Node<frame_processor::input_type, frame_processor::output_type> {
public:
  FrameProcessor(int _height, int _width, int _gap) {
    height = _height;
    width = _width;
    gap = _gap;
    fb = new FrameBuffer(height, width, gap);
    cv::Mat circleImage = cv::Mat(height, width, CV_8UC1);
    circle(circleImage, cv::Point(width / 2, height / 2), 100, cv::Scalar(255),
           -1, cv::LINE_AA);
    vector<vector<cv::Point>> contours;
    findContours(circleImage, contours, cv::RETR_EXTERNAL,
                 cv::CHAIN_APPROX_SIMPLE);
    circleContour = contours[0];
  }

  void process() {
    cv::Mat prev, curr, next;
    cv::Mat gprev, gcurr, gnext;
    cv::Mat diff1, diff2;
    cv::Mat diff_and;
    FpsCounter fc(60);
    while (true) {
      // fc.loop();
      auto inputFrame = readData<0, cv::Mat>();
      fb->insertFrame(inputFrame);
      fb->getFrames(prev, curr, next);

      cv::cvtColor(prev, gprev, cv::COLOR_BGR2GRAY);
      cv::cvtColor(curr, gcurr, cv::COLOR_BGR2GRAY);
      cv::cvtColor(prev, gnext, cv::COLOR_BGR2GRAY);
      cv::absdiff(gprev, gcurr, diff1);
      cv::absdiff(gcurr, gnext, diff2);

      threshold(diff1, diff1, 20, 255, cv::THRESH_BINARY);
      threshold(diff2, diff2, 20, 255, cv::THRESH_BINARY);

      bitwise_and(diff1, diff2, diff_and);

      vector<vector<cv::Point>> contours;
      vector<vector<cv::Point>> filtered_contours;
      findContours(diff_and, contours, cv::RETR_EXTERNAL,
                   cv::CHAIN_APPROX_SIMPLE);

      for (auto x : contours) {
        if (cv::contourArea(x) > 100 and
            cv::matchShapes(circleContour, x, 1, 0.0) < 0.05) {
          filtered_contours.push_back(x);
        }
      }
      curr.copyTo(inputFrame);
      for (int i = 0; i < filtered_contours.size(); i++) {
        cv::drawContours(inputFrame, filtered_contours, i, cv::Scalar(255), 10,
                         cv::LINE_AA);
      }

      writeData<0>(inputFrame);
      // writeData<1>(diff_and);
    }
  }

private:
  vector<cv::Point> circleContour;
  FrameBuffer *fb;
  int height;
  int width;
  int gap;
};
