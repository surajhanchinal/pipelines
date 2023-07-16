#pragma once

#include "fps_counter.h"
#include "node.h"
#include "types.h"
#include <chrono>
#include <thread>
namespace frame_syncer {
using input_type = type_list_t<TimedMatWithCTree, TimedMatWithCTree>;
using output_type = type_list_t<TimedMatWithCTree, TimedMatWithCTree>;
}; // namespace frame_syncer

class FrameSyncer
    : public Node<frame_syncer::input_type, frame_syncer::output_type> {
public:
  void process() {
    FpsCounter fc(240, "FS");
    while (true) {
      fc.loop();
      auto dt1 = readData<0, TimedMatWithCTree>();
      auto dt2 = readData<1, TimedMatWithCTree>();

      auto delay = std::chrono::duration_cast<std::chrono::milliseconds>(
          dt1.timestamp - dt2.timestamp);

      int iters = (abs(delay.count()) / 10) - 1;
      if (abs(delay.count()) > 4) {
        std::cout << "delay happen, what do: " << delay.count() << std::endl;
      }
      writeData<0>(dt1);
      writeData<1>(dt2);
    }
  }
};
