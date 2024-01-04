#pragma once
#include "config_store.h"
#include "imgui.h"
#include "types.h"
#include "utils.h"
#include <algorithm>
#include <eigen3/Eigen/Dense>
#include <iostream>
#include <map>
#include "fk_matrices.h"
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

  ImVec2 windowSize, screenSize;
  std::chrono::time_point<std::chrono::system_clock> currentTime;

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
      auto time_elapsed = scaledDelayInMicro(cm.t_avg, t0) / (1000.0 * 1000.0);
      A(i, 0) = 1;
      A(i, 1) = time_elapsed;
      A(i, 2) = (time_elapsed * time_elapsed) / 2;
      b(i) = cm.y;
    }
    Eigen::MatrixXf soln = (A.transpose() * A).ldlt().solve(A.transpose() * b);
    g = soln(2);
  }

  

  int trainingPointCount(int alignedSize) {
    // Minimum size is 4 and this is set in the frame_syncer. so min estimated
    // param uses 4 aligned contours;
    return min(10, alignedSize);
  }

public:
  map<long long, Trajectory> trajectories;
  Matrix4f pos_transform;
  Matrix4f vel_transform;
  // TrajectoryStore(Eigen::MatrixXf K1, Eigen::MatrixXf K2, Eigen::MatrixXf D1,
  //                 Eigen::MatrixXf D2, Eigen::MatrixXf R1, Eigen::MatrixXf R2,
  //                 Eigen::MatrixXf P1, Eigen::MatrixXf P2)
  //     : K1(K1), K2(K2), D1(D1), D2(D2), R1(R1), R2(R2), P1(P1), P2(P2) {}
  TrajectoryStore(CameraParams cameraParams) : cameraParams(cameraParams) {
    calculatePositionTranformation();
    calculateVelocityTransformation();
  }
  void setWindowSize(ImVec2 sz) { windowSize = sz; }
  void setScreenSize(ImVec2 sz) { screenSize = sz; }
  void setCurrentTIme(std::chrono::time_point<std::chrono::system_clock> curr) {
    currentTime = curr;
  }

  static void getPredictedPointAtTime2(const EstimatedParams &params,
                               double elapsedTimeInSeconds, double &x,
                               double &y, double &z) {

    double groundHeight = 0;
    double a = -9.8 / 2.0;
    double b = params.vz;
    double c = params.z0 - groundHeight;
    double timeToContact = (-b + sqrt(b * b - 4 * a * c)) / (-2 * a);
    cout<<"time to contact 2: "<<timeToContact<<endl;
    double elasticity = ConfigStore::elasticity;
    double velocityAtContact = elasticity * (params.vz - (9.8 * timeToContact));

    if (elapsedTimeInSeconds < timeToContact) {
      x = params.x0 + elapsedTimeInSeconds * params.vx;
      y = params.y0 + elapsedTimeInSeconds * params.vy;
      z = params.z0 + elapsedTimeInSeconds * params.vz -
          4.9 * elapsedTimeInSeconds * elapsedTimeInSeconds;
    } else {
      double timeFromContact = elapsedTimeInSeconds - timeToContact;
      double factor = ConfigStore::slowDownFactor;
      x = (params.x0 + timeToContact * params.vx) +
          timeFromContact * factor * params.vx;
      y = (params.y0 + timeToContact * params.vy) +
          timeFromContact * factor * params.vy;
      z = groundHeight - (velocityAtContact * timeFromContact) -
          (4.9 * timeFromContact * timeFromContact);
    }
  }

  static void getPredictedPointAtTime(const EstimatedParams &params,
                               double elapsedTimeInSeconds, double &x,
                               double &y, double &z) {

    double groundHeight = ConfigStore::groundHeight;
    double a = 9.8 / 2.0;
    double b = params.vy;
    double c = params.y0 - groundHeight;
    double timeToContact = (-b + sqrt(b * b - 4 * a * c)) / (2 * a);
    double elasticity = ConfigStore::elasticity;
    double velocityAtContact = elasticity * (params.vy + (9.8 * timeToContact));
    //cout<<"time to contact: "<<timeToContact<<endl;

    if (elapsedTimeInSeconds < timeToContact) {
      x = params.x0 + elapsedTimeInSeconds * params.vx;
      z = params.z0 + elapsedTimeInSeconds * params.vz;
      y = params.y0 + elapsedTimeInSeconds * params.vy +
          4.9 * elapsedTimeInSeconds * elapsedTimeInSeconds;
    } else {
      double timeFromContact = elapsedTimeInSeconds - timeToContact;
      double factor = ConfigStore::slowDownFactor;
      x = (params.x0 + timeToContact * params.vx) +
          timeFromContact * factor * params.vx;
      z = (params.z0 + timeToContact * params.vz) +
          timeFromContact * factor * params.vz;
      y = groundHeight - (velocityAtContact * timeFromContact) +
          (4.9 * timeFromContact * timeFromContact);
    }
  }

  void addTrajectory(CombinedTrajectory &ct) {
    long long hash = cantorHash(ct.lt.id, ct.rt.id);
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
    sx = (z / 22) * windowSize.x + p.x;
    sy = ((y + 0.8) / (0.8 + ConfigStore::groundHeight + 0.1)) * windowSize.y + p.y;
  }

  void render_yz(ImVec2 &p) {
    ImDrawList *draw_list = ImGui::GetWindowDrawList();

    double xp1, xp2, yp1, yp2;
    getScreenYZ(p, ConfigStore::groundHeight, 0, xp1, yp1);

    getScreenYZ(p, ConfigStore::groundHeight, 22, xp2, yp2);

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

      for (int i = -50; i < 100; i++) {
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
    sy = height - (z / 22) * height + p.y;
    sx = (x / 8.0) * (width / 2.0) + width / 2.0 + p.x;
  }

  void render_xz(ImVec2 &p) {
    ImDrawList *draw_list = ImGui::GetWindowDrawList();

    // Render center line

    double xp1, xp2, yp1, yp2;
    getScreenXZ(p, 0, 1, xp1, yp1);

    getScreenXZ(p, 4.25, 22, xp2, yp2);

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
      for (int i = -50; i < 100; i++) {
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

  void project3dTo2d(cv::Mat &P1, cv::Mat &K1, cv::Mat &R1, cv::Mat &T1,
                     bool isLeft, ImVec2 &offset, double x, double y, double z,
                     double &sx, double &sy) {
    double fx = P1.at<double>(0, 0);
    double fy = P1.at<double>(1, 1);
    double cx = P1.at<double>(0, 2);
    double cy = P1.at<double>(1, 2);
    double baseline = T1.at<double>(0, 0) / 1000.0; // input is in mm

    double sgn = isLeft ? -1 : 1;

    double xl = ((x + sgn * (baseline / 2.0)) * (fx / z)) + cx;
    double yl = (y * (fy / z)) + cy;

    double frx = K1.at<double>(0, 0);
    double fry = K1.at<double>(1, 1);
    double crx = K1.at<double>(0, 2);
    double cry = K1.at<double>(1, 2);

    double xx = (xl - cx) / fx;
    double yy = (yl - cy) / fy;

    cv::Mat newOld = R1.inv() * (cv::Mat_<double>(3, 1) << xx, yy, 1);

    double newx = newOld.at<double>(0, 0);
    double newy = newOld.at<double>(1, 0);

    newx = newx * frx + crx;
    newy = newy * fry + cry;

    double ratio = windowSize.x / 1280;

    sx = offset.x + ratio * newx;
    sy = offset.y + ratio * newy;
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
      double sx, sy;
      project3dTo2d(cameraParams.P1, cameraParams.K1, cameraParams.R1,
                    cameraParams.T1, true, p, px, py, pz, sx, sy);

      draw_list->AddCircle(ImVec2(sx, sy), 10, colorPalette[5], -1, 2);
    }

    {
      Matrix4f ff = pos_transform.inverse();
      //std::cout<<ff(0,3)<< " "<<ff(1,3)<<" "<<ff(2,3)<<endl;
      double sx, sy;
      project3dTo2d(cameraParams.P1, cameraParams.K1, cameraParams.R1,
                    cameraParams.T1, true, p, ff(0,3),ff(1,3),ff(2,3), sx, sy);

      draw_list->AddCircle(ImVec2(sx, sy), 10, colorPalette[0], -1, 2);
    }
    // draw camera middle to stump line
    {
      double lx1, lx2, ly1, ly2;

      project3dTo2d(cameraParams.P1, cameraParams.K1, cameraParams.R1,
                    cameraParams.T1, true, p, 0, 1.7, 1, lx1, ly1);
      project3dTo2d(cameraParams.P1, cameraParams.K1, cameraParams.R1,
                    cameraParams.T1, true, p, 4.25, 1.7, 22, lx2, ly2);
      draw_list->AddLine(ImVec2(lx1, ly1), ImVec2(lx2, ly2), colorPalette[4],
                         2);
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

      double sx, sy;
      project3dTo2d(cameraParams.P2, cameraParams.K2, cameraParams.R2,
                    cameraParams.T2, false, p, px, py, pz, sx, sy);
      draw_list->AddCircle(ImVec2(sx, sy), 10, colorPalette[5], -1, 2);
    }
    {
      Matrix4f ff = pos_transform.inverse();
      double sx, sy;
      cout<< ff(0,3)<<" "<<ff(1,3)<<" "<<ff(2,3)<<" "<<endl;
      project3dTo2d(cameraParams.P2, cameraParams.K2, cameraParams.R2,
                    cameraParams.T2, false, p, ff(0,3),ff(1,3),ff(2,3), sx, sy);

      draw_list->AddCircle(ImVec2(sx, sy), 10, colorPalette[0], -1, 2);
    }

    {

      double lx1, lx2, ly1, ly2;

      project3dTo2d(cameraParams.P2, cameraParams.K2, cameraParams.R2,
                    cameraParams.T2, false, p, 0, 1.7, 1, lx1, ly1);
      project3dTo2d(cameraParams.P2, cameraParams.K2, cameraParams.R2,
                    cameraParams.T2, false, p, 4.25, 1.7, 22, lx2, ly2);

      draw_list->AddLine(ImVec2(lx1, ly1), ImVec2(lx2, ly2), colorPalette[4],
                         2);
    }
  }

  void renderMetrics() {
    for (auto const &[_, traj] : trajectories) {
      int paramIndex = trainingPointCount(traj.alignedContours.size());
      double gravity = traj.estimatedParams.at(paramIndex).g;
      double vx = traj.estimatedParams.at(paramIndex).vx;
      double vy = traj.estimatedParams.at(paramIndex).vy;
      double vz = traj.estimatedParams.at(paramIndex).vz;

      double speed = sqrt(vx * vx + vy * vy + vz * vz);

      double z = traj.alignedContours.at(paramIndex - 1).z;
      double remainingZ = 22 - z;
      double remainingTime = remainingZ / vz;

      // ImGui::Text("Gravity: %.2f m/s2",gravity);
      ImGui::Text("Speed: %.2f kmph", speed * 3.6);
      ImGui::Text("Predicted with %d points, at distance: %.2f", paramIndex, z);
      ImGui::Text("Remaining distance %.2f | Remaining time: %.2f", remainingZ,
                  remainingTime);
    }
  }


  void calculatePositionTranformation(){
    Eigen::Affine3f pos_tf  = translate(0,17,0);
    pos_tf  = pos_tf*rotate(0,0,M_PI);
    pos_tf  = pos_tf*translate(0,0,1.7);
    pos_tf = pos_tf*rotate(M_PI/2.0,0,0);
    pos_transform = pos_tf.matrix();
  }

  void calculateVelocityTransformation(){
    Eigen::Affine3f vel_tf = rotate(0,0,M_PI);
    vel_tf = vel_tf*rotate(M_PI/2.0,0,0);
    vel_transform = vel_tf.matrix();
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

};
