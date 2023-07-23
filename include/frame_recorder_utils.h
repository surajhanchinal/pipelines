#pragma once

#include "common_data.h"
#include "fps_counter.h"
#include "node.h"
#include "opencv2/videoio.hpp"
#include "types.h"
#include <iostream>
#include <sstream>

#include <signal.h>
using namespace std;
class LightFrameSyncer : public Node<type_list_t<TimedMat, TimedMat>,
                                     type_list_t<TimedMat, TimedMat>> {
public:
  void process() {
    while (true) {
      auto dt1 = readData<0, TimedMat>();
      auto dt2 = readData<1, TimedMat>();

      auto delay = std::chrono::duration_cast<std::chrono::milliseconds>(
          dt1.timestamp - dt2.timestamp);

      int iters = (abs(delay.count()) / 10) - 1;
      if (abs(delay.count()) > 4) {
        // std::cout << "delay happen, what do: " << delay.count() << std::endl;
      }

      writeData<0>(dt1);
      writeData<1>(dt2);
    }
  };
};

class GstVideoWriter : public Node<type_list_t<TimedMat>, type_list_t<>> {

public:
  GstVideoWriter(std::string _cameraName, const cv::Size _imageSize) {
    ostringstream line;
    line << "appsrc ! queue ! videoconvert ! nvh264enc ! h264parse ! mp4mux ! "
            "filesink "
            "location=/home/hyperion/";
    line << _cameraName;
    line << ".mp4";

    writer = new cv::VideoWriter(line.str(), cv::CAP_GSTREAMER, 0, 60,
                                 cv::Size(1280, 720), true);
  }
  void process() {

    FpsCounter fc(240, "writer");

    cout << "isOpen: " << writer->isOpened();
    while (common_data::running) {
      fc.loop();
      auto timedMat = readData<0, TimedMat>();
      writer->write(timedMat.mat);
    }
    cout << "Releasing" << endl;
    writer->release();
  }

  cv::VideoWriter *writer;
};
