#pragma once

#include "config_store.h"
#include "node.h"
#include <chrono>
#include <thread>
class CaptureSignaler : public Node<type_list_t<>, type_list_t<bool, bool>> {
public:
  void process() {

    while (*running) {
      writeData<0>(true);
      writeData<1>(true);
      std::this_thread::sleep_for(std::chrono::microseconds(ConfigStore::frameTime));
    }
  }
};

class CaptureSignaler1 : public Node<type_list_t<>, type_list_t<bool>> {
public:
  void process() {
    while (*running) {
      writeData<0>(true);
      std::this_thread::sleep_for(std::chrono::microseconds(12000));
    }
    std::cout << "ending signaler1" << std::endl;
  }
};
