#pragma once

#include "imgui.h"
#include "opencv2/core/mat.hpp"
#include <chrono>
#include <opencv2/core/types.hpp>
#include <vector>

using namespace std;

struct TimedContour {
  std::vector<cv::Point> contour;
  std::chrono::time_point<std::chrono::system_clock> timestamp;
};

struct SingleTrajectory {
  vector<TimedContour> tc;
  int id;
};

struct AlignedTimedContour {
  TimedContour lt;
  TimedContour rt;
  ImVec2 leftCenter;
  ImVec2 rightCenter;
  double y_avg;
  std::chrono::time_point<std::chrono::system_clock> t_avg;
};

struct TimedMat {
  cv::Mat mat;
  std::chrono::time_point<std::chrono::system_clock> timestamp;
};

struct TimedMatWithCTree {
  cv::Mat mat;
  std::chrono::time_point<std::chrono::system_clock> timestamp;
  std::vector<SingleTrajectory> *contourGroupList;
};

struct CombinedTrajectory {
  SingleTrajectory lt;
  SingleTrajectory rt;
  vector<AlignedTimedContour> atc;
};

struct CameraPairData {
  TimedMatWithCTree leftTMCT;
  TimedMatWithCTree rightTMCT;
  vector<CombinedTrajectory> *trajectories;
};
