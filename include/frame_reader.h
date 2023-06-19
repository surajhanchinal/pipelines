#pragma once

#include "node.h"
#include <opencv2/core/mat.hpp>
#include <opencv2/videoio.hpp>

// Is a generator node, no input

using input_type = type_list_t<>;
using output_type = type_list_t<cv::Mat>;

class FrameReader : public Node<input_type, output_type> {
public:
  FrameReader(int _camera, std::string _cameraName) {
    // initialze camera object
    camera = _camera;
    cameraName = _cameraName;
    cap = new cv::VideoCapture(0, cv::CAP_V4L2);
    cap->set(cv::CAP_PROP_FOURCC, cv::VideoWriter::fourcc('M', 'J', 'P', 'G'));

    cap->set(cv::CAP_PROP_FRAME_HEIGHT, 1080);
    cap->set(cv::CAP_PROP_FRAME_WIDTH, 1920);

    cap->set(cv::CAP_PROP_AUTO_EXPOSURE, 1);
    cap->set(cv::CAP_PROP_EXPOSURE, 400);
  }

  void process() {
    while (1) {
      cap->read(_frame);
      // Should be very slow. Clone allocates new memory and writes to it. This
      // should be better done by implementing a memory pool of cv::Mats
      //  cv::Mat itself is a pointer to a chunk of memory where data is stored.
      //  That allocation is what we want to prevent, using memory pool.
      auto frameToSend = _frame.clone();
      writeData<0>(frameToSend);
      // cv::waitKey(1);
    }
  }

  ~FrameReader() { cap->release(); }

private:
  cv::VideoCapture *cap;
  int camera;
  std::string cameraName;
  cv::Mat _frame;
};
