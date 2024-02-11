#pragma once
#include <cmath>

class ConfigStore {
public:
  // Frame processor params
  static constexpr double minContourArea = 30;
  static constexpr double minAspectRatio = 0.8;
  static constexpr double maxAspectRatio = 1.2;
  static constexpr double minExtent = 0.5;
  // Reduce this to 5 in case of bad detection at home.
  static constexpr int binaryThreshold = 10;

  // Contour tree params
  static const int contourValidSearchRadius = 200;
  static constexpr double contourMatchThreshold = 0.5;

  // Frame syncer params
  static constexpr int maxFrameOffset = 6;

  static constexpr double timeScale = 3;

  static constexpr double realFrameTime = 10500;

  //  Time between frames
  static constexpr int frameTime = realFrameTime * timeScale;

  static constexpr double frameTimeInSeconds =
      ((double)realFrameTime) / (1000.0 * 1000.0);

  // Time to live in milliseconds. Vary this based on your framerate. Usually
  // for nice-looking videos, use around 10 frames worth, but for actual usage
  // use, 3-4x frame time.
  static constexpr double contourGroupTimeToLive = (realFrameTime / 1000.0) * 7.0;
  // static constexpr int contourGroupTimeToLive = 160;

  static constexpr double groundHeight = 1.73;
  static constexpr double elasticity = 0.7;
  static constexpr double slowDownFactor = 0.75;
  static constexpr double j1_start = 0;
  static constexpr double j2_start = 0;
  static constexpr double j3_start = M_PI_2;
  static constexpr double j4_start = 0;
  static constexpr double j5_start = 0;
  // Angle in degrees 
  static constexpr double x_angle_offset = 4.02;
  static constexpr double y_angle_offset = -0.5;
  static constexpr double z_angle_offset = 0;

  // Range for lines xz and yz screens.
  static constexpr double Y_MAX = groundHeight + 0.1;
  static constexpr double Y_MIN = -0.8;
  static constexpr double X_MIN = -2;
  static constexpr double X_MAX = 2;
  static constexpr double Z_MIN = 0;
  static constexpr double Z_MAX = 18;

  // Robot position offset parameters. These are in
  // robot coordinate system
  static constexpr double X_OFFSET = 0.145;
  static constexpr double Y_OFFSET = 17.825; 
};
