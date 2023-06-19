#ifndef NODE_H
#define NODE_H

#include "port_set.h"
#include "utilities.h"
#include <iostream>
#include <tuple>
#include <type_traits>
#include <utility>
#include <vector>

class NodeBase {
public:
  PortSet *inputs;
  PortSet *outputs;
  NodeBase(){};
  virtual ~NodeBase(){};
  virtual void process() = 0;
};

template <typename... Types> struct type_list_t {};

template <typename... Types> class Node : public NodeBase {};

template <typename... inputs_t, typename... outputs_t>
class Node<type_list_t<inputs_t...>, type_list_t<outputs_t...>>
    : public NodeBase {
public:
  using input_tuple_type = std::tuple<inputs_t...>;
  using output_tuple_type = std::tuple<outputs_t...>;
  Node() {
    inputs = new PortSet(sizeof...(inputs_t));
    outputs = new PortSet(sizeof...(outputs_t));
  }

  template <int I> Port *getIPort() { return inputs->ports[I]; }

  template <int I> Port *getOPort() { return outputs->ports[I]; };

  template <int I, typename T> T readData() {
    static_assert(std::is_same_v<T, std::tuple_element_t<I, input_tuple_type>>,
                  "The port type does not match the type being read. Please "
                  "use a compatible type. Refer to Node definition");
    Port *p = getIPort<I>();
    T t;
    p->buffer->try_dequeue(t);
    return t;
  }

  template <int I, typename T> void writeData(T t) {
    Port *p = getOPort<I>();
    p->buffer->try_enqueue(t);
    return;
  }
  template <int I, int J, typename T> void attachPort(T *otherNode) {
    getOPort<I>()->setOtherPort(otherNode->template getIPort<J>());
  }
};

class ExampleFirstNode
    : public Node<type_list_t<>, type_list_t<char, int, char>> {

  void process() {
    std::cout << "Hey N1" << std::endl;
    char first = 'h';
    int second = 10;
    char third = 'c';
    writeData<0>(first);
    writeData<1>(second);
    writeData<2>(third);
  }
};

class ExampleSecondNode
    : public Node<type_list_t<char, int, char>, type_list_t<>> {

  void process() {
    std::cout << "Hey N2" << std::endl;
    auto firstR = readData<0, char>();
    auto secondR = readData<1, int>();
    auto thirdR = readData<2, char>();

    std::cout << "This is the data I read:  " << firstR << "  " << secondR
              << "  " << thirdR << std::endl;
  }
};

#endif
