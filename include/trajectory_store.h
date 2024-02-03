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
#include "common_data.h"

using namespace std;

struct EstimatedParams
{
  double vx, vy, vz;
  double x0, y0, z0;
  double g;
  double error;
};

struct Trajectory
{
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
class TrajectoryStore
{
private:
  long long cantorHash(int l, int r)
  {
    long long l_l = l;
    long long l_r = r;

    return (((l_l + l_r) * (l_l + l_r + 1)) / (long long)2) + l_r;
  }

  std::vector<ImColor> colorPalette = { ImColor(83, 200, 33),  ImColor(255, 201, 40), ImColor(255, 148, 35),
                                        ImColor(255, 72, 162), ImColor(122, 71, 255), ImColor(42, 153, 235),ImColor(255,0,0) };

  ImVec2 windowSize, screenSize;
  std::chrono::time_point<std::chrono::system_clock> currentTime;

  StereoCameraParams stereoCameraParams;

  void addEstimatedParams(Trajectory& traj)
  {
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

  void solveAssumingGravity(vector<AlignedTimedContour>& atc, double& x0, double& y0, double& z0, double& vx,
                            double& vy, double& vz, double& error)
  {
    auto obvSize = atc.size();
    Eigen::MatrixXf A(obvSize, 2);
    Eigen::MatrixXf b(obvSize, 1);
    auto t0 = atc[0].t_avg;
    for (int i = 0; i < atc.size(); i++)
    {
      auto& cm = atc[i];
      auto time_elapsed = scaledDelayInMicro(cm.t_avg, t0) / (1000.0 * 1000.0);
      A(i, 0) = 1;
      A(i, 1) = time_elapsed;
    }
    // Solve for X params
    for (int i = 0; i < atc.size(); i++)
    {
      auto& cm = atc[i];
      b(i) = cm.x;
    }
    Eigen::MatrixXf solnX = (A.transpose() * A).ldlt().solve(A.transpose() * b);
    x0 = solnX(0);
    vx = solnX(1);

    // Solve for X params
    for (int i = 0; i < atc.size(); i++)
    {
      auto& cm = atc[i];
      b(i) = cm.y - 9.8 * A(i, 1) * A(i, 1) / 2.0;
    }
    Eigen::MatrixXf solnY = (A.transpose() * A).ldlt().solve(A.transpose() * b);
    y0 = solnY(0);
    vy = solnY(1);

    // Solve for Z params
    for (int i = 0; i < atc.size(); i++)
    {
      auto& cm = atc[i];
      b(i) = cm.z;
    }
    Eigen::MatrixXf solnZ = (A.transpose() * A).ldlt().solve(A.transpose() * b);
    z0 = solnZ(0);
    vz = solnZ(1);
  }

  void solveForGravity(vector<AlignedTimedContour>& atc, double& g)
  {
    auto obvSize = atc.size();
    Eigen::MatrixXf A(obvSize, 3);
    Eigen::MatrixXf b(obvSize, 1);
    auto t0 = atc[0].t_avg;
    for (int i = 0; i < atc.size(); i++)
    {
      auto& cm = atc[i];
      auto time_elapsed = scaledDelayInMicro(cm.t_avg, t0) / (1000.0 * 1000.0);
      A(i, 0) = 1;
      A(i, 1) = time_elapsed;
      A(i, 2) = (time_elapsed * time_elapsed) / 2;
      b(i) = cm.y;
    }
    Eigen::MatrixXf soln = (A.transpose() * A).ldlt().solve(A.transpose() * b);
    g = soln(2);
  }

  int trainingPointCount(int alignedSize)
  {
    // Minimum size is 4 and this is set in the frame_syncer. so min estimated
    // param uses 4 aligned contours;
    return min(10, alignedSize);
  }

  void project3dTo2d(CameraParams& cameraParams, ImVec2& offset, Eigen::Vector3f& oi, double& sx, double& sy)
  {
    auto oo = rotMatrix * oi;
    float x = oo(0);
    float y = oo(1);
    float z = oo(2);

    double baseline = cameraParams.T.at<double>(0, 0) / 1000.0;  // input is in mm

    // This might be wrong. If the camera is to the left. Then we need to add baseline/2. Because the left camera is to
    // the right of the origin This is correct. xo (center origin x coordinate) xo = xl + b/2 ( since xl is shifted in
    // the positive x axis. when xl = 0, xo = b/2. makes sense.)
    // Make sure baseline is positive. Everything comes with that. And baseline is positive when the left camera is the
    // left camera when seen from the front.
    double sgn = cameraParams.isLeft ? -1 : 1;

    double xx = (x + sgn * (baseline / 2.0)) / z;
    double yy = y / z;

    cv::Mat pt = cameraParams.R.inv() * (cv::Mat_<double>(3, 1) << xx, yy, 1);

    vector<cv::Point3d> inputPoints = { { pt.at<double>(0, 0), pt.at<double>(1, 0), 1 } };
    vector<cv::Point2d> outputPoints;

    auto rtvec = cv::Mat::zeros(3, 1, CV_32FC1);

    cv::projectPoints(inputPoints, rtvec, rtvec, cameraParams.K, cameraParams.D, outputPoints);

    double ratio = windowSize.x / 1280;

    sx = offset.x + ratio * outputPoints[0].x;
    sy = offset.y + ratio * outputPoints[0].y;
  }

  static void getPredictedPointAtTime(const EstimatedParams& params, double elapsedTimeInSeconds, double& x, double& y,
                                      double& z)
  {
    double groundHeight = ConfigStore::groundHeight;
    double a = 9.8 / 2.0;
    double b = params.vy;
    double c = params.y0 - groundHeight;
    double timeToContact = (-b + sqrt(b * b - 4 * a * c)) / (2 * a);
    double elasticity = ConfigStore::elasticity;
    double velocityAtContact = elasticity * (params.vy + (9.8 * timeToContact));
    // cout<<"time to contact: "<<timeToContact<<endl;

    if (elapsedTimeInSeconds < timeToContact)
    {
      x = params.x0 + elapsedTimeInSeconds * params.vx;
      z = params.z0 + elapsedTimeInSeconds * params.vz;
      y = params.y0 + elapsedTimeInSeconds * params.vy + 4.9 * elapsedTimeInSeconds * elapsedTimeInSeconds;
    }
    else
    {
      double timeFromContact = elapsedTimeInSeconds - timeToContact;
      double factor = ConfigStore::slowDownFactor;
      x = (params.x0 + timeToContact * params.vx) + timeFromContact * factor * params.vx;
      z = (params.z0 + timeToContact * params.vz) + timeFromContact * factor * params.vz;
      y = groundHeight - (velocityAtContact * timeFromContact) + (4.9 * timeFromContact * timeFromContact);
    }
  }

public:
  map<long long, Trajectory> trajectories;
  Matrix4f pos_transform;
  Matrix4f vel_transform;
  Eigen::Affine3f rotMatrix;

  TrajectoryStore(StereoCameraParams stereoCameraParams) : stereoCameraParams(stereoCameraParams)
  {
    calculatePositionTranformation();
    calculateVelocityTransformation();
    rotMatrix = rotate(ConfigStore::x_angle_offset * (M_PI / 180.0), ConfigStore::y_angle_offset * (M_PI / 180.0),
                       ConfigStore::z_angle_offset * (M_PI / 180.0));
  }
  void setWindowSize(ImVec2 sz)
  {
    windowSize = sz;
  }
  void setScreenSize(ImVec2 sz)
  {
    screenSize = sz;
  }
  void setCurrentTime(std::chrono::time_point<std::chrono::system_clock> curr)
  {
    currentTime = curr;
  }

  void addTrajectory(CombinedTrajectory& ct)
  {
    long long hash = cantorHash(ct.lt.id, ct.rt.id);
    if (trajectories.find(hash) != trajectories.end())
    {
      bool newPoints = trajectories[hash].alignedContours.size() != ct.atc.size();
      trajectories[hash].alignedContours = ct.atc;
      if (newPoints)
      {
        // estimate parameters and add them to map
        addEstimatedParams(trajectories[hash]);
      }
    }
    else
    {
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

  static void getPredictedPointAtTime2(const EstimatedParams& params, double elapsedTimeInSeconds, double& x, double& y,
                                       double& z)
  {
    double groundHeight = 0;
    double g = 9.8;
    double b = params.vz;
    double c = params.z0;
    double timeToContact = (b + sqrt(b * b + 2 * g * c)) / g;
    double elasticity = ConfigStore::elasticity;
    double velocityAtContact = elasticity * (params.vz - (9.8 * timeToContact));

    if (elapsedTimeInSeconds < timeToContact)
    {
      x = params.x0 + elapsedTimeInSeconds * params.vx;
      y = params.y0 + elapsedTimeInSeconds * params.vy;
      z = params.z0 + elapsedTimeInSeconds * params.vz - 4.9 * elapsedTimeInSeconds * elapsedTimeInSeconds;
    }
    else
    {
      double timeFromContact = elapsedTimeInSeconds - timeToContact;
      double factor = ConfigStore::slowDownFactor;
      x = (params.x0 + timeToContact * params.vx) + timeFromContact * factor * params.vx;
      y = (params.y0 + timeToContact * params.vy) + timeFromContact * factor * params.vy;
      z = -(velocityAtContact * timeFromContact) - (4.9 * timeFromContact * timeFromContact);
    }
  }

  void getScreenYZ(ImVec2& p, double y, double z, double& sx, double& sy)
  {
    sx = (z - ConfigStore::Z_MIN)*(windowSize.x/(ConfigStore::Z_MAX - ConfigStore::Z_MIN)) + p.x;
    sy = (y - ConfigStore::Y_MIN)*(windowSize.y)/(ConfigStore::Y_MAX - ConfigStore::Y_MIN) +  p.y;
  }

  void render_yz_line(ImVec2& p, double y1, double z1, double y2, double z2)
  {
    ImDrawList* draw_list = ImGui::GetWindowDrawList();
    double xp1, xp2, yp1, yp2;
    getScreenYZ(p, y1, z1, xp1, yp1);

    getScreenYZ(p, y2, z2, xp2, yp2);

    draw_list->AddLine(ImVec2(xp1, yp1), ImVec2(xp2, yp2), colorPalette[3], 2);
  }

  void render_yz(ImVec2& p)
  {
    ImDrawList* draw_list = ImGui::GetWindowDrawList();

    render_yz_line(p, 1.72, 0, 1.72, 20);

    for (auto const& [_, traj] : trajectories)
    {
      int trainingCount = trainingPointCount(traj.alignedContours.size());
      for (int ii = 0; ii < traj.alignedContours.size(); ii++)
      {
        const auto& pt = traj.alignedContours[ii];
        double x, y;
        getScreenYZ(p, pt.y, pt.z, x, y);
        draw_list->AddCircle(ImVec2(x, y), 10, (ii + 1) <= trainingCount ? colorPalette[0] : colorPalette[1]);
      }

      // Draw where the ball should be currently
      double timeElapsed = scaledDelayInMs(currentTime, traj.alignedContours[0].t_avg) / 1000.0;
      double px, py, pz;
      getPredictedPointAtTime(traj.estimatedParams.at(trainingCount), timeElapsed, px, py, pz);
      double ssx, ssy;
      getScreenYZ(p, py, pz, ssx, ssy);

      draw_list->AddCircle(ImVec2(ssx, ssy), 30, colorPalette[2]);

      // Draw the predicted trajectory
      for (int i = -50; i < 100; i++)
      {
        double newx1, newy1, newz1;
        getPredictedPointAtTime(traj.estimatedParams.at(trainingCount), i * ConfigStore::frameTimeInSeconds, newx1,
                                newy1, newz1);

        double newx2, newy2, newz2;
        getPredictedPointAtTime(traj.estimatedParams.at(trainingCount), (i + 1) * ConfigStore::frameTimeInSeconds,
                                newx2, newy2, newz2);

        render_yz_line(p, newy1, newz1, newy2, newz2);
      }
    }
  }

  void getScreenXZ(ImVec2& p, double x, double z, double& sx, double& sy)
  {
    auto height = 2 * windowSize.y;
    auto width = screenSize.x - 2 * windowSize.x;
    double fsy = (z - ConfigStore::Z_MIN)*(height/(ConfigStore::Z_MAX - ConfigStore::Z_MIN));
    sy = height - fsy + p.y;
    sx = (x - ConfigStore::X_MIN)*(width/(ConfigStore::X_MAX - ConfigStore::X_MIN)) + p.x;
  }

  void render_xz_line(ImVec2& p, double x1, double z1, double x2, double z2)
  {
    ImDrawList* draw_list = ImGui::GetWindowDrawList();
    double xp1, xp2, yp1, yp2;
    getScreenXZ(p, x1, z1, xp1, yp1);

    getScreenXZ(p, x2, z2, xp2, yp2);
    draw_list->AddLine(ImVec2(xp1, yp1), ImVec2(xp2, yp2), colorPalette[3], 2);
  }

  void render_xz(ImVec2& p)
  {
    ImDrawList* draw_list = ImGui::GetWindowDrawList();

    // Render center line
    render_xz_line(p, 0, 2, 0, 20);

    for (const auto& [_, traj] : trajectories)
    {
      int trainingCount = trainingPointCount(traj.alignedContours.size());
      for (int ii = 0; ii < traj.alignedContours.size(); ii++)
      {
        const auto& pt = traj.alignedContours[ii];
        double sx, sy;
        getScreenXZ(p, pt.x, pt.z, sx, sy);
        draw_list->AddCircle(ImVec2(sx, sy), 10, (ii + 1) <= trainingCount ? colorPalette[0] : colorPalette[1]);
      }

      // Draw where current point would be

      double timeElapsed = scaledDelayInMs(currentTime, traj.alignedContours[0].t_avg) / 1000.0;
      double px, py, pz;
      getPredictedPointAtTime(traj.estimatedParams.at(trainingCount), timeElapsed, px, py, pz);
      double ssx, ssy;
      getScreenXZ(p, px, pz, ssx, ssy);

      draw_list->AddCircle(ImVec2(ssx, ssy), 30, colorPalette[2]);

      // Draw trajectory line
      for (int i = -50; i < 100; i++)
      {
        double sx1, sy1;
        double newx1, newy1, newz1;
        getPredictedPointAtTime(traj.estimatedParams.at(trainingCount), i * ConfigStore::frameTimeInSeconds, newx1,
                                newy1, newz1);

        double sx2, sy2;
        double newx2, newy2, newz2;
        getPredictedPointAtTime(traj.estimatedParams.at(trainingCount), (i + 1) * ConfigStore::frameTimeInSeconds,
                                newx2, newy2, newz2);

        render_xz_line(p, newx1, newz1, newx2, newz2);
      }
    }

    if (common_data::solutionFound)
    {
      Eigen::Vector3f p1;
      Eigen::Vector3f p2;
      getBatFk(p1, p2);
      render_xz_line(p, p1(0), p1(2), p2(0), p2(2));
    }
  }

  void renderOnCameraView(ImVec2& offset, bool isLeft = true)
  {
    CameraParams& cameraParams = isLeft ? stereoCameraParams.C1 : stereoCameraParams.C2;

    ImDrawList* draw_list = ImGui::GetWindowDrawList();
    // Draw trajectories
    for (const auto& [_, traj] : trajectories)
    {
      int trainingCount = trainingPointCount(traj.alignedContours.size());
      double timeElapsed = scaledDelayInMs(currentTime, traj.alignedContours[0].t_avg) / 1000.0;
      double px, py, pz;
      getPredictedPointAtTime(traj.estimatedParams.at(trainingCount), timeElapsed, px, py, pz);
      Eigen::Vector3f p3d;
      p3d << px, py, pz;

      double sx, sy;
      project3dTo2d(cameraParams, offset, p3d, sx, sy);

      draw_list->AddCircle(ImVec2(sx, sy), 5, colorPalette[6], -1, 2);
      draw_list->AddCircle(ImVec2(sx, sy), 1, colorPalette[6], -1, 1);
    
    }

    Eigen::Vector3f p1(0, 1.73, 2);
    Eigen::Vector3f p2(0, 1.73, 16.9);

    renderLine(p1, p2, cameraParams, offset);

    if (common_data::solutionFound)
    {
      Eigen::Vector3f p1;
      Eigen::Vector3f p2;
      getBatFk(p1, p2);
      renderLine(p1, p2, cameraParams, offset);
    }
  }

  void renderLine(Eigen::Vector3f& p1, Eigen::Vector3f& p2, CameraParams& cameraParams, ImVec2& offset)
  {
    double lx1, lx2, ly1, ly2;
    ImDrawList* draw_list = ImGui::GetWindowDrawList();
    project3dTo2d(cameraParams, offset, p1, lx1, ly1);
    project3dTo2d(cameraParams, offset, p2, lx2, ly2);

    draw_list->AddLine(ImVec2(lx1, ly1), ImVec2(lx2, ly2), colorPalette[4], 2);
  }

  void getBatFk(Eigen::Vector3f& p1, Eigen::Vector3f& p2)
  {
    auto pti = pos_transform.inverse();
    auto comb = pti * common_data::bat_fk;
    auto pt1 = Eigen::Vector4f(0, 0, 0, 1);
    auto pt2 = Eigen::Vector4f(0, 0, 3, 1);

    p1 = (comb * pt1).head(3);
    p2 = (comb * pt2).head(3);
  }

  void renderMetrics()
  {
    for (auto const& [_, traj] : trajectories)
    {
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
      ImGui::Text("Remaining distance %.2f | Remaining time: %.2f", remainingZ, remainingTime);
    }
  }

  void calculatePositionTranformation()
  {
    Eigen::Affine3f pos_tf = translate(0.5, 20.03, 0);
    pos_tf = pos_tf * rotate(0, 0, M_PI);
    pos_tf = pos_tf * translate(0, 0, 1.72);
    pos_tf = pos_tf * rotate(-M_PI / 2.0, 0, 0);
    pos_transform = pos_tf.matrix();
  }

  void calculateVelocityTransformation()
  {
    Eigen::Affine3f vel_tf = rotate(0, 0, M_PI);
    vel_tf = vel_tf * rotate(-M_PI / 2.0, 0, 0);
    vel_transform = vel_tf.matrix();
  }
};
