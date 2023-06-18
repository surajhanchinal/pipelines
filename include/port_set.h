#pragma once
#include "port.h"
#include <iostream>
#include <tuple>
#include <vector>

class PortSet {
public:
  std::vector<Port *> ports;
  int nodeIdx;
  PortSet(int n) {
    for (int i = 0; i < n; i++) {
      ports.push_back(new Port());
    }
  }

  void updateNodeIdx(int nodeIdx) {
    for (auto const &port : ports) {
      port->setNodeIdx(nodeIdx);
    }
  };

  void attachPort(int idx, Port *const dest) { ports[idx]->setOtherPort(dest); }
};
