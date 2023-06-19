#ifndef NODE_H
#define NODE_H

#include "buffer.h"
#include "port_set.h"
#include "utilities.h"
#include <iostream>
#include <tuple>
#include <type_traits>
#include <vector>

class NodeBase {
public:
  PortSet *inputs;
  PortSet *outputs;
  NodeBase(){};
  virtual ~NodeBase(){};
};

template <typename... Types> struct type_list_t {};

template <typename... Types> class Node : public NodeBase {};

template <typename... inputs_t, typename... outputs_t>
class Node<type_list_t<inputs_t...>, type_list_t<outputs_t...>>
    : public NodeBase {
public:
  std::string name;
  Node() {
    inputs = new PortSet(sizeof...(inputs_t));
    outputs = new PortSet(sizeof...(outputs_t));
  }
  void process() {
    while (1) {
    }
  }
};

class ExampleFirstNode
    : public Node<type_list_t<int, int, char>, type_list_t<char, int, char>> {};

class ExampleSecondNode
    : public Node<type_list_t<char, int, int>, type_list_t<int>> {};

#endif
