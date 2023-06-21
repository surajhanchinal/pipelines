#pragma once
#include <deque>
#include <iostream>
#include <opencv2/opencv.hpp>
using namespace std;

class FrameBuffer {
public:
  int gap;
  int height;
  int width;
  deque<cv::Mat> prevFrames, nextFrames;

  FrameBuffer(int height, int width, int gap) {
    this->gap = gap;
    this->height = height;
    this->width = width;
    for (int i = 0; i < gap + 2; i++) {
      prevFrames.push_back(cv::Mat(height, width, CV_8UC3));
    }
    for (int i = 0; i < gap + 1; i++) {
      nextFrames.push_back(cv::Mat(height, width, CV_8UC3));
    }
  }

  void getFrames(cv::Mat &prev, cv::Mat &curr, cv::Mat &next) {
    prev = prevFrames.front();
    curr = prevFrames.back();
    next = nextFrames.back();
  }

  void insertFrame(cv::Mat &frame) {
    cv::Mat fromNext = nextFrames.front();
    cv::Mat fromPrev = prevFrames.front();
    nextFrames.pop_front();
    prevFrames.pop_front();

    // reuse the allocated fromPrev
    frame.copyTo(fromPrev);
    nextFrames.push_back(fromPrev);

    prevFrames.push_back(fromNext);
  }
};
