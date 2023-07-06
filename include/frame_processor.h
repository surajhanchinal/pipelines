#pragma once
#include "contour_tree.h"
#include "fps_counter.h"
#include "frame_buffer.h"
#include "node.h"
#include "types.h"
#include <iostream>
#include <memory>
#include <opencv2/core.hpp>
#include <opencv2/core/mat.hpp>
#include <opencv2/core/types.hpp>
#include <opencv2/imgproc.hpp>
namespace frame_processor {
using input_type = type_list_t<TimedMat>;
using output_type = type_list_t<TimedMatWithCTree>;
}; // namespace frame_processor

class FrameProcessor
    : public Node<frame_processor::input_type, frame_processor::output_type> {
public:
  FrameProcessor(const cv::Size _imageSize, int _gap) {
    imageSize = _imageSize;
    gap = _gap;
    fb = new FrameBuffer(imageSize.height, imageSize.width, gap);
    ctree = new ContourTree();
  }

  string type2str(int type) {
    string r;

    uchar depth = type & CV_MAT_DEPTH_MASK;
    uchar chans = 1 + (type >> CV_CN_SHIFT);

    switch (depth) {
    case CV_8U:
      r = "8U";
      break;
    case CV_8S:
      r = "8S";
      break;
    case CV_16U:
      r = "16U";
      break;
    case CV_16S:
      r = "16S";
      break;
    case CV_32S:
      r = "32S";
      break;
    case CV_32F:
      r = "32F";
      break;
    case CV_64F:
      r = "64F";
      break;
    default:
      r = "User";
      break;
    }

    r += "C";
    r += (chans + '0');

    return r;
  }

  void process() {
    cv::Mat prev, curr, next;
    cv::Mat gprev, gcurr, gnext;
    cv::Mat diff1, diff2;
    cv::Mat diff_and;
    cv::Mat inputGray;
    FpsCounter fc(60);
    while (true) {
      // fc.loop();
      // auto inputFrame = readData<0, cv::Mat>();
      auto inputTimedMat = readData<0, TimedMat>();
      auto inputFrame = inputTimedMat.mat;

#ifdef SSOPTIMIZED
      cv::cvtColor(inputFrame, inputGray, cv::COLOR_BGR2GRAY);
      fb->insertFrame(inputGray);
      fb->getFrames(gprev, gcurr, gnext);
#else
      fb->insertFrame(inputFrame);
      fb->getFrames(prev, curr, next);

      cv::cvtColor(prev, gprev, cv::COLOR_BGR2GRAY);
      cv::cvtColor(curr, gcurr, cv::COLOR_BGR2GRAY);
      cv::cvtColor(next, gnext, cv::COLOR_BGR2GRAY);

#endif

      cv::GaussianBlur(gprev, gprev, cv::Size(5, 5), 0);

      cv::GaussianBlur(gcurr, gcurr, cv::Size(5, 5), 0);

      cv::GaussianBlur(gnext, gnext, cv::Size(5, 5), 0);

      cv::absdiff(gprev, gcurr, diff1);
      cv::absdiff(gcurr, gnext, diff2);

      cv::GaussianBlur(diff1, diff1, cv::Size(11, 11), 0);

      cv::GaussianBlur(diff2, diff2, cv::Size(11, 11), 0);

      threshold(diff1, diff1, 10, 255, cv::THRESH_BINARY);
      threshold(diff2, diff2, 10, 255, cv::THRESH_BINARY);

      cv::bitwise_and(diff1, diff2, diff_and);

      vector<vector<cv::Point>> contours;
      vector<vector<cv::Point>> filtered_contours;
      findContours(diff_and, contours, cv::RETR_EXTERNAL,
                   cv::CHAIN_APPROX_SIMPLE);

      for (auto x : contours) {
        auto bb = cv::boundingRect(x);
        float extent = cv::contourArea(x) / (bb.width * bb.height + 1);
        float ap = (float)bb.width / ((float)bb.height + 1);

        if (cv::contourArea(x) > 30 && extent >= 0.5 && ap >= 0.8 and
            ap <= 1.2) {
          filtered_contours.push_back(x);
        }
      }
      auto currTime = timeSinceEpochMillisec();
#ifdef SSOPTIMIZED
      inputFrame = gcurr.clone();
#else
      curr.copyTo(inputFrame);
#endif
      ctree->addContours2(filtered_contours, currTime, inputFrame);
      vector<vector<vector<cv::Point>>> cgl;
      ctree->getContourGroupList(cgl);
      auto newVec = new vector(cgl);
#ifdef SSOPTIMIZED
      TimedMatWithCTree newTimedMat = {.mat = inputFrame,
                                       .timestamp = inputTimedMat.timestamp,
                                       .contourGroupList = newVec};
      writeData<0>(newTimedMat);
#else
      TimedMatWithCTree newTimedMat = {.mat = inputFrame,
                                       .timestamp = inputTimedMat.timestamp,
                                       .contourGroupList = newVec};
      writeData<0>(newTimedMat);
#endif
    }
  }

private:
  vector<cv::Point> circleContour;
  FrameBuffer *fb;
  cv::Size imageSize;
  int gap;
  ContourTree *ctree;
  uint64_t timeSinceEpochMillisec() {
    using namespace std::chrono;
    return duration_cast<milliseconds>(system_clock::now().time_since_epoch())
        .count();
  }
};
