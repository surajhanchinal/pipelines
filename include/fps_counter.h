#pragma once

#include <iostream>
#include <opencv2/core/utility.hpp>
class FpsCounter {
public:
  FpsCounter(int _cycleCount) { cycleCount = _cycleCount; }

  void reset() { iCounter = -1; }

  void loop() {
    if (iCounter == cycleCount) {
      int64 end = cv::getTickCount();

      double te = (end - start) / cv::getTickFrequency();
      double fps = cycleCount / te;
      std::cout << "FPS : " << fps << std::endl;
      start = end;
      iCounter = 0;
    }
    if (iCounter == -1) {
      start = cv::getTickCount();
      iCounter = 0;
    }
    iCounter++;
  }

private:
  int cycleCount;
  int iCounter = 0;
  int64 start;
};
