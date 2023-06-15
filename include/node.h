#ifndef NODE_H
#define NODE_H

#include <iostream>
#include <tuple>
#include <type_traits>

/*template<typename... Args>
class Node {
        using Tuple = std::tuple<Args...>;
        static constexpr auto Size  = sizeof...(Args);
        template <std::size_t N>
        using Nth = typename std::tuple_element<N, Tuple>::type;
        using First = Nth<0>;
        using Last = Nth<Size  - 1>;
};*/
/*template<typename... Ts>
class NodeSequence {
        private:
                int a = 5;
};

template<typename std::tuple<typename In...>::type,std::tuple<typename Out...>
OutT> class Node { template<typename... Ts,class I,class O> NodeSequence<Ts...>
attach(Node<I,O> node){ return NodeSequence<In,Out,I,O>();
}
NodeSequence attach(NodeSequence sequence);
};
*/
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
