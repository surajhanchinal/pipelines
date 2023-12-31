#pragma once

#include "fps_counter.h"
#include "node.h"
#include "types.h"
#include <chrono>
#include <iostream>
#include <opencv2/core/mat.hpp>
#include <opencv2/videoio.hpp>

// Is a generator node, no input

using input_type = type_list_t<bool>;
using output_type = type_list_t<TimedMat>;

class FrameReader : public Node<input_type, output_type> {
public:
  FrameReader(int _camera, std::string _cameraName, const cv::Size _imageSize) {
    // initialze camera object
    camera = _camera;
    cameraName = _cameraName;
    imageSize = _imageSize;
    cap = new cv::VideoCapture(camera, cv::CAP_V4L2);
    cap->set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));
    cap->set(cv::CAP_PROP_FPS, 60);

    cap->set(cv::CAP_PROP_FRAME_HEIGHT, imageSize.height);
    cap->set(cv::CAP_PROP_FRAME_WIDTH, imageSize.width);

    // cap->set(cv::CAP_PROP_AUTO_EXPOSURE, 1);
    // cap->set(cv::CAP_PROP_EXPOSURE, 400);
  }

  void process() {
    pthread_setname_np(pthread_self(), "FrameReader");
    FpsCounter fc(240);
    while (*running) {
      readData<0, bool>();
      // fc.loop();
      cap->grab();
      auto now = std::chrono::system_clock::now();
      cap->retrieve(_frame);
      // Should be very slow. Clone allocates new memory and writes to it. This
      // should be better done by implementing a memory pool of cv::Mats
      //  cv::Mat itself is a pointer to a chunk of memory where data is stored.
      //  That allocation is what we want to prevent, using memory pool.
      auto frameToSend = _frame.clone();
      TimedMat tm = {.mat = frameToSend, .timestamp = now};
      writeData<0>(tm);
    }
    cout << "ending framereader" << endl;
  }

  ~FrameReader() { cap->release(); }

private:
  cv::VideoCapture *cap;
  int camera;
  std::string cameraName;
  cv::Mat _frame;
  cv::Size imageSize;
};
