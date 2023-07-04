#pragma once

#include "config_store.h"
#include "opencv2/core/types.hpp"
#include "opencv2/imgproc.hpp"

#include <algorithm>
#include <complex>
#include <cstdint>
#include <iostream>
#include <queue>
#include <thread>
#include <vector>
class ContourGroup {
public:
  std::vector<std::vector<cv::Point>> contours;
  uint64_t insertTime;
  cv::Scalar color;

  ContourGroup(std::vector<cv::Point> contour, uint64_t _time,
               cv::Scalar _color) {
    insertContour(contour, _time);
    color = _color;
  }

  void insertContour(std::vector<cv::Point> contour, uint64_t _time) {
    contours.push_back(contour);
    insertTime = _time;
  }

  uint64_t getInsertTime() { return insertTime; }

  std::vector<cv::Point> &getLatestContour() {
    return contours[contours.size() - 1];
  }
};

class ContourTree {
  std::vector<ContourGroup> groups;
  // time to live in milliseconds?
  uint64_t ttl = ConfigStore::contourGroupTimeToLive;
  int k = 0;
  std::vector<cv::Scalar> colorPalette = {
      cv::Scalar(83, 200, 33),  cv::Scalar(255, 201, 40),
      cv::Scalar(255, 148, 35), cv::Scalar(255, 72, 162),
      cv::Scalar(122, 71, 255), cv::Scalar(42, 153, 235)};

public:
  void addContours2(std::vector<std::vector<cv::Point>> inputContours,
                    uint64_t iTime, cv::Mat inputFrame) {
    // We want to give preference to longer contour chains with the latest
    // insert times. Because vibes.
    std::sort(groups.begin(), groups.end(),
              [](const ContourGroup &g1, const ContourGroup &g2) -> bool {
                if (g1.contours.size() != g2.contours.size()) {
                  return g1.contours.size() > g2.contours.size();
                } else {
                  return g1.insertTime > g2.insertTime;
                }
              });
    std::vector<bool> usedContours(inputContours.size(), 0);

    for (auto &g : groups) {
      auto srcCenter = contourCenterPoint(g.getLatestContour());
      std::priority_queue<std::pair<double, int>> pq;

      // find the 3 closest contours;
      for (int i = 0; i < inputContours.size(); i++) {
        if (usedContours[i] == 1) {
          continue;
        }

        auto newCenter = contourCenterPoint(inputContours[i]);

        auto dist = sqrt(pow(srcCenter.x - newCenter.x, 2) +
                         pow(srcCenter.y - newCenter.y, 2));
        if (dist <= ConfigStore::contourValidSearchRadius) {
          // Minus because we want to find closest contours, PQ by default
          // orders in highest to lowest.
          pq.push({-dist, i});
        }
      }
      // Upto 3 indices.
      std::vector<int> nearestContourIndices;
      while (!pq.empty() && nearestContourIndices.size() < 3) {
        nearestContourIndices.push_back(pq.top().second);
        pq.pop();
      }
      double score = ConfigStore::contourMatchThreshold;
      int idx = -1;
      for (auto c : nearestContourIndices) {
        int currScore = matchContour(g.getLatestContour(), inputContours[c]);
        if (currScore < score) {
          score = currScore;
          idx = c;
        }
      }
      if (idx != -1) {
        usedContours[idx] = 1;
        g.insertContour(inputContours[idx], iTime);
      }
    }
    // Create new groups out of unused contours. Maybe they find their partner
    // in the next frame. Hope is a good thing.
    for (int i = 0; i < inputContours.size(); i++) {
      if (usedContours[i] == 0) {
        groups.push_back(
            ContourGroup(inputContours[i], iTime, colorPalette[k % 6]));
        k++;
        if (k > 100) {
          k = 0;
        }
      }
    }

    // Purge groups who have been living for too long. Ask yourself, if they
    // haven't found their partner withing a certain time, do they even deserve
    // to exist?
    cleanupGroups(iTime);

    /*for (int i = 0; i < groups.size(); i++) {
      // draw each group with it's own color
      cv::drawContours(inputFrame, groups[i].contours, -1, groups[i].color, 5);

      // Draw lines between successive contours in a group, makes trajectory
      // look super cool
      if (groups[i].contours.size() > 1) {
        for (int m = 0; m < groups[i].contours.size() - 1; m++) {
          cv::arrowedLine(inputFrame, contourCenterPoint(groups[i].contours[m]),
                          contourCenterPoint(groups[i].contours[m + 1]),
                          groups[i].color, 3);
        }
      }
    }*/
  }

  void cleanupGroups(uint64_t currTime) {
    std::vector<ContourGroup> newGroups;
    for (int i = 0; i < groups.size(); i++) {
      if (currTime - groups[i].insertTime <= ttl) {
        newGroups.push_back(groups[i]);
      }
    }
    groups = newGroups;
  }

  double matchContour(std::vector<cv::Point> sourceCtr,
                      std::vector<cv::Point> newCtr) {
    return cv::matchShapes(sourceCtr, newCtr, 1, 0.0);
  }

  cv::Point contourCenterPoint(std::vector<cv::Point> &contour) {
    cv::Moments M = cv::moments(contour);
    int X = M.m10 / M.m00;
    int Y = M.m01 / M.m00;
    return cv::Point(X, Y);
  }
};
