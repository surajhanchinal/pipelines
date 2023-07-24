#pragma once

#include "fps_counter.h"
#include "node.h"
#include "types.h"
#include <chrono>
#include <opencv2/core/mat.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/videoio.hpp>
#include <thread>

// Is a generator node, no input

using input_type = type_list_t<bool>;
using output_type = type_list_t<TimedMat>;

class FileReader : public Node<input_type, output_type> {
public:
  FileReader(std::string filePath, const cv::Size _imageSize) {
    // initialze camera object
    imageSize = _imageSize;
    cap = new cv::VideoCapture(filePath);
  }

  void process() {
    FpsCounter fc(240);
    while (*running) {
      readData<0, bool>();
      fc.loop();
      cap->read(_frame);
      auto now = std::chrono::system_clock::now();
      if (_frame.empty()) {
        return;
      }
      // std::cout << "size: " << _frame.size() << std::endl;
      //  Should be very slow. Clone allocates new memory and writes to it. This
      //  should be better done by implementing a memory pool of cv::Mats
      //   cv::Mat itself is a pointer to a chunk of memory where data is
      //   stored. That allocation is what we want to prevent, using memory
      //   pool.
      auto frameToSend = _frame.clone();
      TimedMat outputTimedMat = {.mat = frameToSend, .timestamp = now};
      writeData<0>(outputTimedMat);
    }
    cout << "ending filereader" << endl;
  }

  ~FileReader() { cap->release(); }

private:
  cv::VideoCapture *cap;
  cv::Mat _frame;
  cv::Size imageSize;
};
