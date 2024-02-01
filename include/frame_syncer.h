#pragma once

#include "contour_tree.h"
#include "fps_counter.h"
#include "node.h"
#include "types.h"
#include "utils.h"
#include <chrono>
#include <eigen3/Eigen/Dense>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/persistence.hpp>
#include <thread>
namespace frame_syncer {
using input_type = type_list_t<TimedMatWithCTree, TimedMatWithCTree>;
using output_type = type_list_t<CameraPairData>;
}; // namespace frame_syncer

using namespace std;

class FrameSyncer
    : public Node<frame_syncer::input_type, frame_syncer::output_type> {
public:
  FrameSyncer(StereoCameraParams stereoCameraParams) : stereoCameraParams(stereoCameraParams) {
    rotMatrix = rotate(ConfigStore::x_angle_offset*(M_PI/180.0),ConfigStore::y_angle_offset*(M_PI/180.0),ConfigStore::z_angle_offset*(M_PI/180.0)).inverse();
  }
  void process() {
    FpsCounter fc(240, "FS");
    // std::this_thread::sleep_for(std::chrono::milliseconds(3000));
    while (*running) {
      auto dt1 = readData<0, TimedMatWithCTree>();
      auto dt2 = readData<1, TimedMatWithCTree>();
      fc.loop();
      // std::this_thread::sleep_for(std::chrono::microseconds(16500));
      auto delay = scaledDelayInMs(dt1.timestamp, dt2.timestamp);

      if (delay > 4) {
        std::cout << "delay happen, what do: " << delay << std::endl;
      }

      vector<CombinedTrajectory> alignedTrajectories;
      alignGroups(dt1.contourGroupList, dt2.contourGroupList,
                  alignedTrajectories);

      auto vecPtr = new vector(alignedTrajectories);

      CameraPairData dt = {
          .leftTMCT = dt1, .rightTMCT = dt2, .trajectories = vecPtr};
      writeData<0>(dt);
    }
    std::cout << "Closing heavy frame syncer" << std::endl;
  }

  void alignGroups(vector<SingleTrajectory> *leftGroups,
                   vector<SingleTrajectory> *rightGroups,
                   vector<CombinedTrajectory> &alignedTrajectories) {
    auto wLeftGroups = *leftGroups;
    auto wRightGroups = *rightGroups;
    for (int i = 0; i < wLeftGroups.size(); i++) {
      if (wLeftGroups[i].tc.size() < 4) {
        continue;
      }
      for (int j = 0; j < wRightGroups.size(); j++) {
        if (wRightGroups[j].tc.size() < 4) {
          continue;
        }
        SingleTrajectory &leftGroup = wLeftGroups[i];
        SingleTrajectory &rightGroup = wRightGroups[j];

        // Align a pair of trajectories.
        alignTrajectory(leftGroup, rightGroup, alignedTrajectories);
      }
    }
  }

  void alignTrajectory(SingleTrajectory &leftTrajectory,
                       SingleTrajectory &rightTrajectory,
                       vector<CombinedTrajectory> &alignedTrajectories) {
    int leftIdx = leftTrajectory.tc.size() - 1;
    int rightIdx = rightTrajectory.tc.size() - 1;
    vector<pair<int, int>> alignedIndices;
    while (leftIdx >= 0 && rightIdx >= 0) {
      TimedContour &left = leftTrajectory.tc[leftIdx];
      TimedContour &right = rightTrajectory.tc[rightIdx];
      bool alignedInTime = areAlignedInTime(left, right);
      // bool alignedInTime = false;
      bool alignedInY = areAlignedInY(left, right);
      auto offset_in_ms = scaledDelayInMs(left.timestamp, right.timestamp);

      if (!alignedInTime || !alignedInY) {
        // The latest contours are not aligned, we will not consider this
        // trajectory further;

        if (alignedInTime) {
          // They belong to the same frame but not together. Sad! Ignore both
          leftIdx--;
          rightIdx--;
        } else {
          auto offset_in_ms = scaledDelayInMs(left.timestamp, right.timestamp);
          // offset more than zero means that left is on a newer node than
          // right. Make left go back because we can't make right go forward.
          // Because reverse iteration and vice versa.
          if (offset_in_ms > 0) {
            leftIdx--;
          } else {
            rightIdx--;
          }
        }
      } else if (alignedInTime && alignedInY) {
        // They are aligned in both time and space. True love
        alignedIndices.push_back({leftIdx, rightIdx});
        leftIdx--;
        rightIdx--;
      }
    }

    if (alignedIndices.size() >= 4) {
      vector<AlignedTimedContour> alignedTrajectory;
      for (int i = alignedIndices.size() - 1; i >= 0; i--) {
        int li = alignedIndices[i].first;
        int ri = alignedIndices[i].second;

        auto &ltc = leftTrajectory.tc[li];
        auto &rtc = rightTrajectory.tc[ri];
        ImVec2 leftctr, rightctr;
        alignedCenter(ltc, rtc, leftctr, rightctr);
        auto diff = std::chrono::duration_cast<std::chrono::milliseconds>(
            ltc.timestamp - rtc.timestamp);
        auto diff_2 = diff / 2;
        auto t_avg = rtc.timestamp + diff_2;

        double x, y, z;
        get3dCoords(leftctr, rightctr, x, y, z);

        AlignedTimedContour alignedTimedContour = {
            .lt = ltc,
            .rt = rtc,
            .leftCenter = leftctr,
            .rightCenter = rightctr,
            .x = x,
            .y = y,
            .z = z,
            .t_avg = ltc.timestamp,
        };
        alignedTrajectory.push_back(alignedTimedContour);
        // cout << x << " " << height << " " << depth << endl;
      }
      alignedTrajectories.push_back({
          .lt = leftTrajectory,
          .rt = rightTrajectory,
          .atc = alignedTrajectory,
      });
    }
  }

  void get3dCoords(ImVec2 &leftctr, ImVec2 &rightctr, double &x, double &y,
                   double &z) {

    double fx = stereoCameraParams.C1.P.at<double>(0, 0);
    double fy = stereoCameraParams.C1.P.at<double>(1, 1);
    double cx = stereoCameraParams.C1.P.at<double>(0, 2);
    double cy = stereoCameraParams.C1.P.at<double>(1, 2);
    double xln = leftctr.x;
    double xrn = rightctr.x;
    double baseline =
        stereoCameraParams.C1.T.at<double>(0, 0) / 1000.0; // input is in mm
    auto disparity = xrn - xln;
    z = (fx * baseline) / disparity;
    y = ((leftctr.y - cy) * z) / fy;
    x = (((leftctr.x + rightctr.x - 2 * cx) / 2.0) * z) / fx;
    //Align to ground using transform
    Eigen::Vector4f vec;
    vec << x ,y,z,1;
    auto vec2 = rotMatrix * vec;
    x = vec2(0);
    y = vec2(1);
    z = vec2(2);
  }

  bool areAlignedInTime(TimedContour &left, TimedContour &right) {
    auto offset_in_ms = abs(scaledDelayInMs(left.timestamp, right.timestamp));
    return offset_in_ms <= ConfigStore::maxFrameOffset;
  }

  bool areAlignedInY(TimedContour &left, TimedContour &right) {
    ImVec2 leftctr, rightctr;
    alignedCenter(left, right, leftctr, rightctr);

    return abs(leftctr.y - rightctr.y) < 20;
  }

  void alignedCenter(TimedContour &left, TimedContour &right, ImVec2 &leftctr,
                     ImVec2 &rightctr) {
    auto leftCenter = contourCenterPoint(left.contour);
    auto rightCenter = contourCenterPoint(right.contour);
    vector<cv::Point2f> lefts = {leftCenter};
    vector<cv::Point2f> unLC;
    vector<cv::Point2f> rights = {rightCenter};
    vector<cv::Point2f> unRC;
    cv::undistortPoints(lefts, unLC, stereoCameraParams.C1.K, stereoCameraParams.C1.D,
                        stereoCameraParams.C1.R, stereoCameraParams.C1.P);
    cv::undistortPoints(rights, unRC, stereoCameraParams.C2.K, stereoCameraParams.C2.D,
                        stereoCameraParams.C2.R, stereoCameraParams.C2.P);
    leftctr.x = unLC[0].x;
    leftctr.y = unLC[0].y;
    rightctr.x = unRC[0].x;
    rightctr.y = unRC[0].y;
  }

  cv::Point contourCenterPoint(std::vector<cv::Point> &contour) {

    cv::Moments M = cv::moments(contour);
    int X = M.m10 / M.m00;
    int Y = M.m01 / M.m00;
    return cv::Point(X, Y);
  }

  ImVec2 contourCenterVec2(vector<cv::Point> &contour) {
    cv::Moments M = cv::moments(contour);
    int X = M.m10 / M.m00;
    int Y = M.m01 / M.m00;
    return ImVec2(X, Y);
  }

  Eigen::Affine3f rotMatrix;
  StereoCameraParams stereoCameraParams;
};
