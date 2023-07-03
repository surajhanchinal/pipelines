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
  uint64_t ttl = 500;
  // how far the new contour is from the source contour abs(srcCtr.x - newCtr.x)
  // < deltaX deltaY.
  int deltaX = 1000;
  int deltaY = 1000;
  double matchThreshold = 0.5;
  int k = 0;
  std::vector<cv::Scalar> colorPalette = {
      cv::Scalar(255, 0, 0),   cv::Scalar(0, 255, 0), cv::Scalar(0, 0, 255),
      cv::Scalar(0, 165, 255), cv::Scalar(0, 0, 0),   cv::Scalar(100, 0, 255)};

public:
  template <typename Func1>
  void addContours2(std::vector<std::vector<cv::Point>> inputContours,
                    uint64_t iTime, Func1 func1, cv::Mat inputFrame) {
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
      cv::Moments srcM = cv::moments(g.getLatestContour());
      int srcX = srcM.m10 / srcM.m00;
      int srcY = srcM.m01 / srcM.m00;
      std::priority_queue<std::pair<double, int>> pq;

      // find the 3 closest contours;
      for (int i = 0; i < inputContours.size(); i++) {
        if (usedContours[i] == 1) {
          continue;
        }
        cv::Moments newM = cv::moments(inputContours[i]);
        int newX = newM.m10 / newM.m00;
        int newY = newM.m01 / newM.m00;

        auto dist = sqrt(pow(srcX - newX, 2) + pow(srcY - newY, 2));
        if (dist <= 40) {
          pq.push({-dist, i});
        }
      }
      // Upto 3 indices.
      std::vector<int> nearestContourIndices;
      std::vector<std::vector<cv::Point>> nearestContours;
      while (!pq.empty() && nearestContourIndices.size() < 3) {
        nearestContourIndices.push_back(pq.top().second);
        nearestContours.push_back(inputContours[pq.top().second]);
        pq.pop();
      }
      double score = 0.5;
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

      // prettyDraw(g.getLatestContour(), nearestContours, func1, inputFrame);
    }
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
    cleanupGroups(iTime);
    for (int i = 0; i < groups.size(); i++) {
      cv::drawContours(inputFrame, groups[i].contours, -1, groups[i].color, 5);
      if (groups[i].contours.size() > 1) {
        for (int m = 0; m < groups[i].contours.size() - 1; m++) {
          auto p1 = contourCenter(groups[i].contours[m]);
          auto p2 = contourCenter(groups[i].contours[m + 1]);
          cv::arrowedLine(inputFrame, cv::Point(p1.first, p1.second),
                          cv::Point(p2.first, p2.second), colorPalette[0], 1);
        }
      }
    }
  }
  template <typename Func1>
  void prettyDraw(std::vector<cv::Point> &sourceContour,
                  std::vector<std::vector<cv::Point>> &nearestContours,
                  Func1 func, cv::Mat frame) {
    auto frame2 = frame.clone();
    cv::drawContours(frame2,
                     std::vector<std::vector<cv::Point>>(1, sourceContour), 0,
                     colorPalette[0], 5);
    auto srcCenter = contourCenter(sourceContour);
    cv::circle(frame2, cv::Point(srcCenter.first, srcCenter.second), 100,
               colorPalette[0], 5);
    func(frame2.clone());
    auto delay = std::chrono::milliseconds(125);
    std::this_thread::sleep_for(delay);

    for (auto c : nearestContours) {
      cv::drawContours(frame2, std::vector<std::vector<cv::Point>>(1, c), -1,
                       colorPalette[1], 5);
      auto center = contourCenter(c);
      cv::arrowedLine(frame2, cv::Point(srcCenter.first, srcCenter.second),
                      cv::Point(center.first, center.second), colorPalette[1],
                      5);
      func(frame2.clone());
      std::this_thread::sleep_for(delay);
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

  std::pair<int, int> contourCenter(std::vector<cv::Point> &contour) {
    cv::Moments M = cv::moments(contour);
    int X = M.m10 / M.m00;
    int Y = M.m01 / M.m00;
    return std::pair(X, Y);
  }
};
