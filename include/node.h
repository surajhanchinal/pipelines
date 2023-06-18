#ifndef NODE_H
#define NODE_H

#include "buffer.h"
#include "port_set.h"
#include "utilities.h"
#include <iostream>
#include <tuple>
#include <type_traits>
#include <vector>

/*template <typename T> struct my_buffer {
  T a;
};*/

template <typename... Types> struct type_list_t {};

template <typename... Types> class Node {};

template <typename... inputs_t, typename... outputs_t>
class Node<type_list_t<inputs_t...>, type_list_t<outputs_t...>> {
private:
  using output_tuple_type = std::tuple<outputs_t...>;
  // template<typename... Ts>
  // void _attach(Node<Ts...> node);

  // template<typename... Ts,typename... Args>
  // void _attach(Node<Ts...> node,Args... args);

public:
  using input_tuple_type = std::tuple<inputs_t...>;
  input_tuple_type i_tuple;
  output_tuple_type o_tuple;
  // virtual void process(std::tuple<my_buffer<inputs_t>...>) = 0;
  template <typename... Args> void attach(Args... args) {
    static_assert(match_input_output<std::tuple<outputs_t...>, Args...>::value,
                  "This code is trash, Inputs of nodes being attach do not "
                  "match the output of the current node output");
  }
  int my_idx = -1;
  PortSet<inputs_t...> inputs;
  PortSet<outputs_t...> outputs;
};

class ExampleFirstNode : public Node<type_list_t<int, int, char>,
                                     type_list_t<char, int, char, int>> {};

class ExampleSecondNode
    : public Node<type_list_t<char, int>, type_list_t<int>> {};

#endif
