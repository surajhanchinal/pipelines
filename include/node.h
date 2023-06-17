#ifndef NODE_H
#define NODE_H

#include <iostream>
#include <tuple>
#include <type_traits>
#include "utilities.h"
// test struct wrapper. eventually a buffer
template <typename T> struct my_buffer {
  T a;
};

template <typename... Types> struct type_list_t {};

template <typename... Types> class Node {};
template <typename... inputs_t, typename... outputs_t>
class Node<type_list_t<inputs_t...>, type_list_t<outputs_t...>> {
private:
  using input_tuple_type = std::tuple<inputs_t...>;
  using output_tuple_type = std::tuple<outputs_t...>;

public:
  input_tuple_type i_tuple;
  output_tuple_type o_tuple;
  virtual void process(std::tuple<my_buffer<inputs_t>...>) = 0;
};

using frame_processor_input_t = type_list_t<int, int>;
using frame_processor_output_t = type_list_t<int, int>;
class FrameProcessorNode
    : Node<frame_processor_input_t, frame_processor_output_t> {
public:
   void process(std::tuple<my_buffer<int>, my_buffer<int>> tp)override {
    std::cout << "hey" << std::get<0>(tp).a + std::get<1>(tp).a << std::endl;
  }
};

#endif
