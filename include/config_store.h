#pragma once

class ConfigStore {
public:
  // Frame processor params
  static constexpr double minContourArea = 30;
  static constexpr double minAspectRatio = 0.8;
  static constexpr double maxAspectRatio = 1.2;
  static constexpr double minExtent = 0.5;
  static constexpr int binaryThreshold = 10;

  // Contour tree params
  static const int contourValidSearchRadius = 100;
  static constexpr double contourMatchThreshold = 0.5;
  // Time to live in milliseconds. Vary this based on your framerate. Usually
  // for nice-looking videos, use around 10 frames worth, but for actual usage
  // use, 3-4x frame time.
  static constexpr int contourGroupTimeToLive = 160;

  // Frame syncer params
  static constexpr int maxFrameOffset = 6;
};
