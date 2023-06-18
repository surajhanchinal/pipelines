#ifndef NODE_H
#define NODE_H

#include "buffer.h"
#include "port_set.h"
#include "utilities.h"
#include <iostream>
#include <tuple>
#include <type_traits>
#include <vector>

class Node {
public:
  std::string name;
  Node(int in_c,int out_c){
    inputs = new PortSet(in_c);
    outputs = new PortSet(out_c);
  }
  void process() {
    while (1) {
      
    }
  }
  PortSet *inputs;
  PortSet *outputs;
};

#endif
