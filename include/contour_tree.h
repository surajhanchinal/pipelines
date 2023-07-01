#pragma once

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
  uint64_t ttl = 200;
  // how far the new contour is from the source contour abs(srcCtr.x - newCtr.x)
  // < deltaX deltaY.
  int deltaX = 120;
  int deltaY = 120;
  double matchThreshold = 0.5;
  int k = 0;
  std::vector<cv::Scalar> colorPalette = {
      cv::Scalar(255, 0, 0),   cv::Scalar(0, 255, 0), cv::Scalar(0, 0, 255),
      cv::Scalar(0, 165, 255), cv::Scalar(0, 0, 0),   cv::Scalar(100, 0, 255)};

public:
  void addContours(std::vector<std::vector<cv::Point>> inputContours,
                   uint64_t iTime) {
    // We want to give preference to longer contour chains with the latest
    // insert times. Because vibes.
    /*std::sort(groups.begin(), groups.end(),
              [](const ContourGroup &g1, const ContourGroup &g2) -> bool {
                if (g1.contours.size() != g2.contours.size()) {
                  return g1.contours.size() > g2.contours.size();
                } else {
                  return g1.insertTime > g2.insertTime;
                }
              });*/
    std::vector<bool> usedGroups(groups.size(), 0);
    std::vector<ContourGroup> newGroups;
    for (auto contour : inputContours) {
      // std::vector<float> matchValues(groups.size(), 0);
      int minScore = 200;
      int minScoreGroup = -1;
      for (int i = 0; i < groups.size(); i++) {
        if (usedGroups[i] == 1) {
          break;
        }
        double score = matchContour(groups[i].getLatestContour(), contour);
        if (score < minScore) {
          minScoreGroup = i;
          minScore = score;
        }
      }
      if (minScore > matchThreshold) {
        minScoreGroup = -1;
        minScore = 200;
      }
      if (minScoreGroup != -1) {
        usedGroups[minScoreGroup] = 1;
        groups[minScoreGroup].insertContour(contour, iTime);
      } else {
        newGroups.push_back(ContourGroup(contour, iTime, colorPalette[k % 6]));
        k++;
        if (k > 100) {
          k = k % 6;
        }
      }
    }
    for (auto ng : newGroups) {
      groups.push_back(ng);
    }

    cleanupGroups(iTime);
  }

  void addContours2(std::vector<std::vector<cv::Point>> inputContours,
                    uint64_t iTime) {
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

    for (auto g : groups) {

      cv::Moments srcM = cv::moments(g.getLatestContour());
      int srcX = srcM.m10 / srcM.m00;
      int srcY = srcM.m01 / srcM.m00;
      std::priority_queue<std::pair<double, int>> pq;

      // find the 3 closest contours;
      for (int i = 0; i < inputContours.size(); i++) {
        if (usedContours[i] == 1) {
          break;
        }
        cv::Moments newM = cv::moments(inputContours[i]);
        int newX = newM.m10 / newM.m00;
        int newY = newM.m01 / newM.m00;

        auto dist = sqrt(pow(srcX - newX, 2) + pow(srcY - newY, 2));
      }
    }
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
    cv::Moments srcM = cv::moments(sourceCtr);
    int srcX = srcM.m10 / srcM.m00;
    int srcY = srcM.m01 / srcM.m00;

    cv::Moments newM = cv::moments(newCtr);
    int newX = newM.m10 / newM.m00;
    int newY = newM.m01 / newM.m00;
    if (!(abs(srcX - newX) <= deltaX and abs(srcY - newY) <= deltaY)) {
      return 10;
    }
    // std::cout << "coords: " << srcX << " " << newX << " , " << srcY << " "
    //           << newY << std::endl;
    float srcArea = cv::contourArea(sourceCtr);
    float newArea = cv::contourArea(newCtr);
    double areaRatio = srcArea / (newArea + 0.001);
    /*if (areaRatio < 0.8 and areaRatio > 1.2) {
      return 10;
    }*/
    auto srcBB = cv::boundingRect(sourceCtr);
    auto newBB = cv::boundingRect(newCtr);
    double score = cv::matchShapes(sourceCtr, newCtr, 1, 0.0);
    return score;
  }

  void drawTree(cv::Mat inputFrame) {
    std::cout << "f:  ";
    for (int i = 0; i < groups.size(); i++) {
      std::cout << groups[i].contours.size() << " ";
      cv::drawContours(inputFrame, groups[i].contours, -1, groups[i].color, 2,
                       cv::LINE_AA);
    }
    // std::chrono::milliseconds dura(80);
    // std::this_thread::sleep_for(dura);
    std::cout << std::endl;
  }
};
