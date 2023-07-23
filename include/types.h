#pragma once

#include "contour_tree.h"
#include "opencv2/core/mat.hpp"
#include <chrono>
#include <opencv2/core/types.hpp>
#include <vector>
struct TimedMat {
  cv::Mat mat;
  std::chrono::time_point<std::chrono::system_clock> timestamp;
};

struct TimedMatWithCTree {
  cv::Mat mat;
  std::chrono::time_point<std::chrono::system_clock> timestamp;
  std::vector<std::vector<TimedContour>> *contourGroupList;
};
