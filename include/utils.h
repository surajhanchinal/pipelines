#pragma once
#include "config_store.h"
#include "types.h"
#include <chrono>
#include <opencv2/core/persistence.hpp>
// Want to keep the apparent time the pipeline as close to realtime as possible
// even if we slow down the whole pipeline.
double
scaledDelayInMs(const std::chrono::time_point<std::chrono::system_clock> &tp1,
                const std::chrono::time_point<std::chrono::system_clock> &tp2) {
  return std::chrono::duration_cast<std::chrono::milliseconds>(tp1 - tp2)
             .count() /
         ConfigStore::timeScale;
}

double
scaledDelayInMicro(std::chrono::time_point<std::chrono::system_clock> &tp1,
                   std::chrono::time_point<std::chrono::system_clock> &tp2) {
  return std::chrono::duration_cast<std::chrono::microseconds>(tp1 - tp2)
             .count() /
         ConfigStore::timeScale;
}

void loadCameraParams(CameraParams &cameraParams) {
  cv::FileStorage fs("../scripts/stereoParams.xml", cv::FileStorage::READ);
  fs["K1"] >> cameraParams.K1;
  fs["K2"] >> cameraParams.K2;
  fs["D1"] >> cameraParams.D1;
  fs["D2"] >> cameraParams.D2;
  fs["R1"] >> cameraParams.R1;
  fs["R2"] >> cameraParams.R2;
  fs["P1"] >> cameraParams.P1;
  fs["P2"] >> cameraParams.P2;
  fs["T"] >> cameraParams.T1;
  fs["T"] >> cameraParams.T2;
}
