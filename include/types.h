#pragma once

#include "opencv2/core/mat.hpp"
#include <chrono>
#include <opencv2/core/types.hpp>
#include <vector>

using namespace std;

struct TimedContour {
  std::vector<cv::Point> contour;
  std::chrono::time_point<std::chrono::system_clock> timestamp;
};

struct AlignedTimedContours {
  TimedContour leftContour;
  TimedContour rightContour;
};

struct TimedMat {
  cv::Mat mat;
  std::chrono::time_point<std::chrono::system_clock> timestamp;
};

struct TimedMatWithCTree {
  cv::Mat mat;
  std::chrono::time_point<std::chrono::system_clock> timestamp;
  std::vector<std::vector<TimedContour>> *contourGroupList;
};

struct CameraPairData {
  TimedMatWithCTree leftTMCT;
  TimedMatWithCTree rightTMCT;
  vector<vector<AlignedTimedContours>> *trajectories;
};
