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

void loadCameraParams(StereoCameraParams &stereoCameraParams) {
  cv::FileStorage fs("../scripts/stereoParams.xml", cv::FileStorage::READ);
  cv::FileStorage cameraAssignment("../scripts/camera_assignment.xml", cv::FileStorage::READ);

  fs["K1"] >> stereoCameraParams.C1.K;
  fs["D1"] >> stereoCameraParams.C1.D;
  fs["R1"] >> stereoCameraParams.C1.R;
  fs["P1"] >> stereoCameraParams.C1.P;
  fs["T"] >> stereoCameraParams.C1.T;
  stereoCameraParams.C1.isLeft = true;
  cameraAssignment["left_camera_path"] >> stereoCameraParams.C1.cameraNumber;

  fs["K2"] >> stereoCameraParams.C2.K;
  fs["D2"] >> stereoCameraParams.C2.D;
  fs["R2"] >> stereoCameraParams.C2.R;
  fs["P2"] >> stereoCameraParams.C2.P;
  fs["T"] >> stereoCameraParams.C2.T;
  stereoCameraParams.C2.isLeft = false;
  cameraAssignment["right_camera_path"] >> stereoCameraParams.C2.cameraNumber;
}

void loadFileParams(std::pair<string,string> &filePaths){
  cv::FileStorage fileAssignment("../scripts/file_assignment.xml", cv::FileStorage::READ);
  fileAssignment["left_file_path"] >> filePaths.first;
  fileAssignment["right_file_path"] >> filePaths.second;
}
