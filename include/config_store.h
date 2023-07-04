#pragma once

class ConfigStore {
public:
  static const int contourValidSearchRadius = 60;
  static constexpr double contourMatchThreshold = 0.5;
  // Time to live in milliseconds. Vary this based on your framerate. Usually
  // for nice-looking videos, use around 10 frames worth, but for actual usage
  // use, 3-4x frame time.
  static constexpr int contourGroupTimeToLive = 400;
};
