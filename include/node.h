#ifndef NODE_H
#define NODE_H

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

  template <int I> Port *getIPort() { return inputs->ports[I]; }

  template <int I> Port *getOPort() { return outputs->ports[I]; };

  template <typename T, int I> T readData() {
    Port *p = getIPort<I>();
    T t;
    p->buffer->try_dequeue(t);
    return t;
  }

  template <typename T, int I> void writeData(T t) {
    Port *p = getOPort<I>();
    p->buffer->try_enqueue(t);
    return;
  }
  template <int I, int J, typename T> void attachPort(T *otherNode) {
    getOPort<I>()->setOtherPort(otherNode->template getIPort<J>());
  }
};

class ExampleFirstNode
    : public Node<type_list_t<>, type_list_t<char, int, char>> {};

class ExampleSecondNode
    : public Node<type_list_t<char, int, char>, type_list_t<>> {};

#endif
