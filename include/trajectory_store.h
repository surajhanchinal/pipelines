#pragma once
#include "config_store.h"
#include "types.h"
#include "utils.h"
#include <algorithm>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <map>
#include <opencv2/calib3d.hpp>
#include <opencv2/core/types.hpp>
#include <vector>

using namespace std;

struct EstimatedParams {
  double vx, vy, vz;
  double x0, y0, z0;
  double g;
  double error;
};

struct Trajectory {
  vector<AlignedTimedContour> alignedContours;
  map<int, EstimatedParams> estimatedParams;
  bool showLine;
  bool showTrajectory;
  bool showPrediction;
};

// Responsibilities:
// 1. Have a map of all trajectories, extended trajectories are updated, new
// trajectories are added.
// 2. Also stores each length > 4's gravity estimate, error assuming gravity,
// vx,vy,vz,x0,y0,z0. I want to understand what the best length to use is.
// 3. Given a trajectory, and current time, predict the position at the current
// time. Provide coordinates for all 4 screens
// 4. Helper functions given window size to determine screen coordinates.
// 5. It needs to input camera params
// 6. Might be good to have it estimate all the Lin Alg stuff, but need to move
// and think about frame time.
class TrajectoryStore {
private:
  long long cantorHash(int l, int r) {
    long long l_l = l;
    long long l_r = r;

    return (((l_l + l_r) * (l_l + l_r + 1)) / (long long)2) + l_r;
  }

  std::vector<ImColor> colorPalette = {
      ImColor(83, 200, 33),  ImColor(255, 201, 40), ImColor(255, 148, 35),
      ImColor(255, 72, 162), ImColor(122, 71, 255), ImColor(42, 153, 235)};

  double groundHeight = 1.7;
  ImVec2 windowSize, screenSize;
  std::chrono::time_point<std::chrono::system_clock> currentTime;
  map<long long, Trajectory> trajectories;

  CameraParams cameraParams;

  void addEstimatedParams(Trajectory &traj) {
    double x0, y0, z0, vx, vy, vz, g, error;
    solveAssumingGravity(traj.alignedContours, x0, y0, z0, vx, vy, vz, error);
    solveForGravity(traj.alignedContours, g);
    traj.estimatedParams[traj.alignedContours.size()] = {
        .vx = vx,
        .vy = vy,
        .vz = vz,
        .x0 = x0,
        .y0 = y0,
        .z0 = z0,
        .g = g,
        .error = 0,
    };
  }

  void solveAssumingGravity(vector<AlignedTimedContour> &atc, double &x0,
                            double &y0, double &z0, double &vx, double &vy,
                            double &vz, double &error) {
    auto obvSize = atc.size();
    Eigen::MatrixXf A(obvSize, 2);
    Eigen::MatrixXf b(obvSize, 1);
    auto t0 = atc[0].t_avg;
    for (int i = 0; i < atc.size(); i++) {
      auto &cm = atc[i];
      auto time_elapsed = scaledDelayInMicro(cm.t_avg, t0) / (1000.0 * 1000.0);
      A(i, 0) = 1;
      A(i, 1) = time_elapsed;
    }
    // Solve for X params
    for (int i = 0; i < atc.size(); i++) {
      auto &cm = atc[i];
      b(i) = cm.x;
    }
    Eigen::MatrixXf solnX = (A.transpose() * A).ldlt().solve(A.transpose() * b);
    x0 = solnX(0);
    vx = solnX(1);

    // Solve for X params
    for (int i = 0; i < atc.size(); i++) {
      auto &cm = atc[i];
      b(i) = cm.y - 9.8 * A(i, 1) * A(i, 1) / 2.0;
    }
    Eigen::MatrixXf solnY = (A.transpose() * A).ldlt().solve(A.transpose() * b);
    y0 = solnY(0);
    vy = solnY(1);

    // Solve for Z params
    for (int i = 0; i < atc.size(); i++) {
      auto &cm = atc[i];
      b(i) = cm.z;
    }
    Eigen::MatrixXf solnZ = (A.transpose() * A).ldlt().solve(A.transpose() * b);
    z0 = solnZ(0);
    vz = solnZ(1);
  }

  void solveForGravity(vector<AlignedTimedContour> &atc, double &g) {
    auto obvSize = atc.size();
    Eigen::MatrixXf A(obvSize, 3);
    Eigen::MatrixXf b(obvSize, 1);
    auto t0 = atc[0].t_avg;
    for (int i = 0; i < atc.size(); i++) {
      auto &cm = atc[i];
      auto time_elapsed = scaledDelayInMs(cm.t_avg, t0) / (1000.0 * 1000.0);
      A(i, 0) = 1;
      A(i, 1) = time_elapsed;
      A(i, 2) = (time_elapsed * time_elapsed) / 2;
      b(i) = cm.y;
    }
    Eigen::MatrixXf soln = (A.transpose() * A).ldlt().solve(A.transpose() * b);
    g = soln(2);
  }

  void getPredictedPointAtTime(const EstimatedParams &params,
                               double elapsedTimeInSeconds, double &x,
                               double &y, double &z) {
    x = params.x0 + elapsedTimeInSeconds * params.vx;
    z = params.z0 + elapsedTimeInSeconds * params.vz;

    double a = 9.8 / 2.0;
    double b = params.vy;
    double c = params.y0 - groundHeight;
    double timeToContact = (-b + sqrt(b * b - 4 * a * c)) / (2 * a);
    double elasticity = 0.8;
    double velocityAtContact = elasticity * (params.vy + (9.8 * timeToContact));

    if (elapsedTimeInSeconds < timeToContact) {
      y = params.y0 + elapsedTimeInSeconds * params.vy +
          4.9 * elapsedTimeInSeconds * elapsedTimeInSeconds;
    } else {
      double timeFromContact = elapsedTimeInSeconds - timeToContact;
      y = groundHeight - (velocityAtContact * timeFromContact) +
          (4.9 * timeFromContact * timeFromContact);
    }
  }

  int trainingPointCount(int alignedSize) {
    // Minimum size is 4 and this is set in the frame_syncer. so min estimated
    // param uses 4 aligned contours;
    return min(6, alignedSize);
  }

public:
  // TrajectoryStore(Eigen::MatrixXf K1, Eigen::MatrixXf K2, Eigen::MatrixXf D1,
  //                 Eigen::MatrixXf D2, Eigen::MatrixXf R1, Eigen::MatrixXf R2,
  //                 Eigen::MatrixXf P1, Eigen::MatrixXf P2)
  //     : K1(K1), K2(K2), D1(D1), D2(D2), R1(R1), R2(R2), P1(P1), P2(P2) {}
  TrajectoryStore(CameraParams cameraParams) : cameraParams(cameraParams) {}
  void setGroundHeight(double gh) { groundHeight = gh; }
  void setWindowSize(ImVec2 sz) { windowSize = sz; }
  void setScreenSize(ImVec2 sz) { screenSize = sz; }
  void setCurrentTIme(std::chrono::time_point<std::chrono::system_clock> curr) {
    currentTime = curr;
  }

  void addTrajectory(CombinedTrajectory &ct) {
    long long hash = cantorHash(ct.lt.id, ct.lt.id);
    if (trajectories.find(hash) != trajectories.end()) {
      bool newPoints =
          trajectories[hash].alignedContours.size() != ct.atc.size();
      trajectories[hash].alignedContours = ct.atc;
      if (newPoints) {
        // estimate parameters and add them to map
        addEstimatedParams(trajectories[hash]);
      }

    } else {
      // add the new trajectory
      Trajectory tt = {
          .alignedContours = ct.atc,
          .estimatedParams = map<int, EstimatedParams>(),
          .showLine = true,
          .showTrajectory = true,
          .showPrediction = true,
      };
      trajectories[hash] = std::move(tt);

      addEstimatedParams(trajectories[hash]);
    }
  }

  void getScreenYZ(ImVec2 &p, double y, double z, double &sx, double &sy) {
    sx = (z / 15) * windowSize.x + p.x;
    sy = ((y + 0.5) / (0.5 + groundHeight + 0.1)) * windowSize.y + p.y;
  }

  void render_yz(ImVec2 &p) {
    ImDrawList *draw_list = ImGui::GetWindowDrawList();

    double xp1, xp2, yp1, yp2;
    getScreenYZ(p, groundHeight, 0, xp1, yp1);

    getScreenYZ(p, groundHeight, 30, xp2, yp2);

    draw_list->AddLine(ImVec2(xp1, yp1), ImVec2(xp2, yp2), colorPalette[3], 2);

    for (auto const &[_, traj] : trajectories) {
      int trainingCount = trainingPointCount(traj.alignedContours.size());
      for (int ii = 0; ii < traj.alignedContours.size(); ii++) {
        const auto &pt = traj.alignedContours[ii];
        double x, y;
        getScreenYZ(p, pt.y, pt.z, x, y);
        draw_list->AddCircle(ImVec2(x, y), 10,
                             (ii + 1) <= trainingCount ? colorPalette[0]
                                                       : colorPalette[1]);
      }

      double timeElapsed =
          scaledDelayInMs(currentTime, traj.alignedContours[0].t_avg) / 1000.0;
      double px, py, pz;
      getPredictedPointAtTime(traj.estimatedParams.at(trainingCount),
                              timeElapsed, px, py, pz);
      double ssx, ssy;
      getScreenYZ(p, py, pz, ssx, ssy);

      draw_list->AddCircle(ImVec2(ssx, ssy), 30, colorPalette[2]);

      for (int i = -50; i < 50; i++) {
        double sx1, sy1;
        double newx1, newy1, newz1;
        getPredictedPointAtTime(traj.estimatedParams.at(trainingCount),
                                i * ConfigStore::frameTimeInSeconds, newx1,
                                newy1, newz1);
        getScreenYZ(p, newy1, newz1, sx1, sy1);

        double sx2, sy2;
        double newx2, newy2, newz2;
        getPredictedPointAtTime(traj.estimatedParams.at(trainingCount),
                                (i + 1) * ConfigStore::frameTimeInSeconds,
                                newx2, newy2, newz2);
        getScreenYZ(p, newy2, newz2, sx2, sy2);

        draw_list->AddLine(ImVec2(sx2, sy2), ImVec2(sx1, sy1), colorPalette[4],
                           2);
      }
    }
  }

  void getScreenXZ(ImVec2 &p, double x, double z, double &sx, double &sy) {

    auto height = 2 * windowSize.y;
    auto width = screenSize.x - 2 * windowSize.x;
    sy = height - (z / 15) * height + p.y;
    sx = (x / 8.0) * (width / 2.0) + width / 2.0 + p.x;
  }

  void render_xz(ImVec2 &p) {
    ImDrawList *draw_list = ImGui::GetWindowDrawList();

    // Render center line

    double xp1, xp2, yp1, yp2;
    getScreenXZ(p, 0, 0, xp1, yp1);

    getScreenXZ(p, 0, 30, xp2, yp2);

    draw_list->AddLine(ImVec2(xp1, yp1), ImVec2(xp2, yp2), colorPalette[3], 2);

    for (const auto &[_, traj] : trajectories) {
      int trainingCount = trainingPointCount(traj.alignedContours.size());
      for (int ii = 0; ii < traj.alignedContours.size(); ii++) {
        const auto &pt = traj.alignedContours[ii];
        double sx, sy;
        getScreenXZ(p, pt.x, pt.z, sx, sy);
        draw_list->AddCircle(ImVec2(sx, sy), 10,
                             (ii + 1) <= trainingCount ? colorPalette[0]
                                                       : colorPalette[1]);
      }

      // Draw where current point would be

      double timeElapsed =
          scaledDelayInMs(currentTime, traj.alignedContours[0].t_avg) / 1000.0;
      double px, py, pz;
      getPredictedPointAtTime(traj.estimatedParams.at(trainingCount),
                              timeElapsed, px, py, pz);
      double ssx, ssy;
      getScreenXZ(p, px, pz, ssx, ssy);

      draw_list->AddCircle(ImVec2(ssx, ssy), 30, colorPalette[2]);

      // Draw trajectory line
      for (int i = -50; i < 50; i++) {
        double sx1, sy1;
        double newx1, newy1, newz1;
        getPredictedPointAtTime(traj.estimatedParams.at(trainingCount),
                                i * ConfigStore::frameTimeInSeconds, newx1,
                                newy1, newz1);
        getScreenXZ(p, newx1, newz1, sx1, sy1);

        double sx2, sy2;
        double newx2, newy2, newz2;
        getPredictedPointAtTime(traj.estimatedParams.at(trainingCount),
                                (i + 1) * ConfigStore::frameTimeInSeconds,
                                newx2, newy2, newz2);
        getScreenXZ(p, newx2, newz2, sx2, sy2);

        draw_list->AddLine(ImVec2(sx2, sy2), ImVec2(sx1, sy1), colorPalette[4],
                           2);
      }
    }
  }

  void renderLeftPred(ImVec2 &p) {

    ImDrawList *draw_list = ImGui::GetWindowDrawList();
    for (const auto &[_, traj] : trajectories) {
      int trainingCount = trainingPointCount(traj.alignedContours.size());
      double timeElapsed =
          scaledDelayInMs(currentTime, traj.alignedContours[0].t_avg) / 1000.0;
      double px, py, pz;
      getPredictedPointAtTime(traj.estimatedParams.at(trainingCount),
                              timeElapsed, px, py, pz);

      double fx = cameraParams.P1.at<double>(0, 0);
      double fy = cameraParams.P1.at<double>(1, 1);
      double cx = cameraParams.P1.at<double>(0, 2);
      double cy = cameraParams.P1.at<double>(1, 2);
      double baseline =
          cameraParams.T1.at<double>(0, 0) / 1000.0; // input is in mm

      double xl = ((px - (baseline / 2.0)) * (fx / pz)) + cx;
      double yl = (py * (fy / pz)) + cy;

      double frx = cameraParams.K1.at<double>(0, 0);
      double fry = cameraParams.K1.at<double>(1, 1);
      double crx = cameraParams.K1.at<double>(0, 2);
      double cry = cameraParams.K1.at<double>(1, 2);

      double xx = (xl - cx) / fx;
      double yy = (yl - cy) / fy;

      cv::Mat R1 = cameraParams.R1;

      cv::Mat newOld = R1.inv() * (cv::Mat_<double>(3, 1) << xx, yy, 1);

      double newx = newOld.at<double>(0, 0);
      double newy = newOld.at<double>(1, 0);

      newx = newx * frx + crx;
      newy = newy * fry + cry;

      double ratio = windowSize.x / 1280;

      draw_list->AddCircle(ImVec2(p.x + newx * ratio, p.y + newy * ratio), 30,
                           colorPalette[2], -1, 5);
    }
  }

  void renderRightPred(ImVec2 &p) {

    ImDrawList *draw_list = ImGui::GetWindowDrawList();
    for (const auto &[_, traj] : trajectories) {
      int trainingCount = trainingPointCount(traj.alignedContours.size());
      double timeElapsed =
          scaledDelayInMs(currentTime, traj.alignedContours[0].t_avg) / 1000.0;
      double px, py, pz;
      getPredictedPointAtTime(traj.estimatedParams.at(trainingCount),
                              timeElapsed, px, py, pz);

      double fx = cameraParams.P1.at<double>(0, 0);
      double fy = cameraParams.P1.at<double>(1, 1);
      double cx = cameraParams.P1.at<double>(0, 2);
      double cy = cameraParams.P1.at<double>(1, 2);
      double baseline =
          cameraParams.T1.at<double>(0, 0) / 1000.0; // input is in mm

      double xl = ((px + (baseline / 2.0)) * (fx / pz)) + cx;
      double yl = (py * (fy / pz)) + cy;

      double frx = cameraParams.K2.at<double>(0, 0);
      double fry = cameraParams.K2.at<double>(1, 1);
      double crx = cameraParams.K2.at<double>(0, 2);
      double cry = cameraParams.K2.at<double>(1, 2);

      double xx = (xl - cx) / fx;
      double yy = (yl - cy) / fy;

      cv::Mat R2 = cameraParams.R2;

      cv::Mat newOld = R2.inv() * (cv::Mat_<double>(3, 1) << xx, yy, 1);

      double newx = newOld.at<double>(0, 0);
      double newy = newOld.at<double>(1, 0);

      newx = newx * frx + crx;
      newy = newy * fry + cry;

      double ratio = windowSize.x / 1280;

      draw_list->AddCircle(ImVec2(p.x + newx * ratio, p.y + newy * ratio), 30,
                           colorPalette[2], -1, 5);
    }
  }
};
