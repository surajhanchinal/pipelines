#ifndef PORT_SET_H
#define PORT_SET_H

#include "port.h"
#include <iostream>
#include <tuple>

template <typename... Is> class PortSet {
public:
  std::tuple<Port<Is> *...> ports;
  int nodeIdx;
  PortSet<Is...>() { ports = std::make_tuple(new Port<Is>()...); }

  void updateNodeIdx(int nodeIdx) {
    std::apply([&](auto &&...args) { ((args->setNodeIdx(nodeIdx)), ...); },
               ports);
  };

  void createBuffers() {
    std::apply([&](auto &&...args) { ((args->createBuffer(nodeIdx)), ...); },
               ports);
  }

  template <typename T, typename... Us>
  void attachPort(int index, Port<T> *port) {
    Port<T> currPort = std::get<index>(ports);
    currPort.otherPort = port;
  }
};

#endif
