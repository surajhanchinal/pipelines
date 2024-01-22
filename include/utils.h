#pragma once
#include "config_store.h"
#include "types.h"
#include <chrono>
#include <eigen3/Eigen/Dense>
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

Eigen::Affine3f rotate(double ax, double ay, double az) {
    Eigen::Affine3f rx =
        Eigen::Affine3f(Eigen::AngleAxisf(ax, Eigen::Vector3f(1, 0, 0)));
    Eigen::Affine3f ry =
        Eigen::Affine3f(Eigen::AngleAxisf(ay, Eigen::Vector3f(0, 1, 0)));
    Eigen::Affine3f rz =
        Eigen::Affine3f(Eigen::AngleAxisf(az, Eigen::Vector3f(0, 0, 1)));
    return rz * ry * rx;
  }

  Eigen::Affine3f translate(double x,double y,double z){
    Eigen::Affine3f t(Eigen::Translation3f(Eigen::Vector3f(x,y,z)));
    return t;
  }

void loadCameraParams(CameraParams &cameraParams) {
  cv::FileStorage fs("../scripts/stereoParams.xml", cv::FileStorage::READ);
  fs["K1"] >> cameraParams.K1;
  fs["K2"] >> cameraParams.K2;
  fs["D1"] >> cameraParams.D1;
  fs["D2"] >> cameraParams.D2;
  fs["R1"] >> cameraParams.R1;
  fs["R2"] >> cameraParams.R2;
  fs["R"] >> cameraParams.R;
  fs["P1"] >> cameraParams.P1;
  fs["P2"] >> cameraParams.P2;
  fs["T"] >> cameraParams.T1;
  fs["T"] >> cameraParams.T2;
}
