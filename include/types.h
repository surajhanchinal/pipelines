#pragma once

#include "opencv2/core/mat.hpp"
#include <chrono>
struct TimedMat {
  cv::Mat mat;
  std::chrono::time_point<std::chrono::system_clock> timestamp;
};
