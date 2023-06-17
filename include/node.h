#ifndef NODE_H
#define NODE_H

#include <iostream>
#include <tuple>
#include <type_traits>
#include "utilities.h"
template <typename T> struct my_buffer {
  T a;
};

template <typename... Types> struct type_list_t {};

template <typename... Types> class Node {};


// match_input_output Start

template<typename... unusedTypes>
struct match_input_output{};

template<typename InputTuple>
struct match_input_output<InputTuple>{
  public:
  static bool constexpr value = std::tuple_size<InputTuple>::value == 0;
};

template<typename InputTuple,typename CurrNode, typename... RestOfTheNodes>
struct match_input_output<InputTuple,CurrNode,RestOfTheNodes...>{};

template<typename InputTuple,typename... CurrNodeInputs,typename... CurrNodeOutputs,typename... RestOfTheNodes>
struct match_input_output<InputTuple,Node<type_list_t<CurrNodeInputs...>,type_list_t<CurrNodeOutputs...>>,RestOfTheNodes...>{ 

  using relevantOutputSeq = typename empty_seq_if_bigger_than_tuple<InputTuple,typename MakeSeq<0,sizeof...(CurrNodeInputs) - 1,0>::type>::type;

  using relevantNextCheckTupleSeq = typename empty_seq_if_bigger_than_tuple<InputTuple,typename MakeSeq<sizeof...(CurrNodeInputs),std::tuple_size<InputTuple>::value-1,sizeof...(CurrNodeInputs)>::type>::type;


  using relevantOutputType = typename filter_tuple<InputTuple,relevantOutputSeq>::type;
  using relevantNextCheckTuple = typename filter_tuple<InputTuple,relevantNextCheckTupleSeq>::type;

  public:
  static bool constexpr value = (sizeof...(CurrNodeInputs) <= std::tuple_size<InputTuple>::value) && std::is_same<relevantOutputType,std::tuple<CurrNodeInputs...>>::value && match_input_output<relevantNextCheckTuple,RestOfTheNodes...>::value;
};

// match_input_output End


template <typename... inputs_t, typename... outputs_t>
class Node<type_list_t<inputs_t...>, type_list_t<outputs_t...>> {
private:
  using input_tuple_type = std::tuple<inputs_t...>;
  using output_tuple_type = std::tuple<outputs_t...>;
  //template<typename... Ts>
  //void _attach(Node<Ts...> node);
  
  //template<typename... Ts,typename... Args>
  //void _attach(Node<Ts...> node,Args... args);

public:
  input_tuple_type i_tuple;
  output_tuple_type o_tuple;
 // virtual void process(std::tuple<my_buffer<inputs_t>...>) = 0;
  template<typename... Args>
  void attach(Args... args){
    static_assert(match_input_output<std::tuple<outputs_t...>,Args...>::value,"This code is trash, Inputs of nodes being attach do not match the output of the current node output");
  }
};

class ExampleFirstNode: public Node<type_list_t<int,int,char>,type_list_t<char,int,char,int>>{
  
};

class ExampleSecondNode: public Node<type_list_t<char,int>,type_list_t<int>>
{};

#endif
