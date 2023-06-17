#ifndef UTILITIES_H
#define UTILITIES_H

#include <iostream>
#include <tuple>
#include <type_traits>
#include <utility>

// New Sequence Start

template <int... Is> struct Seq {};

template <typename A, typename B> struct CombineSeq {};

template <int... Is, int... Js> struct CombineSeq<Seq<Is...>, Seq<Js...>> {
public:
  using type = Seq<Is..., Js...>;
};

template <int Start, int End, int Curr> struct MakeSeq {
private:
  static bool constexpr isValid =
      (Start <= End) && (Curr >= Start) && (Curr <= End);
  static int constexpr _start = isValid ? Start : -1;
  static int constexpr _end = isValid ? End : -1;
  using currSeq = typename std::conditional<isValid, Seq<Curr>, Seq<>>::type;

public:
  using type =
      typename CombineSeq<currSeq,
                          typename MakeSeq<_start, _end, Curr + 1>::type>::type;
};

template <int Curr> struct MakeSeq<-1, -1, Curr> {
public:
  using type = Seq<>;
};

// Better tuples start

template <typename... unusedTypes> struct FilterTuple {};

template <typename Tuple, int... Ints> struct FilterTuple<Tuple, Seq<Ints...>> {
private:
  template <typename... unusedTypes> struct IFilteredTuple {};

  template <typename ITuple, int... IInts>
  struct IFilteredTuple<ITuple, Seq<IInts...>> {
  public:
    using type =
        std::tuple<typename std::tuple_element<IInts, ITuple>::type...>;
  };
  static bool constexpr isValid =
      std::tuple_size<Tuple>::value >= sizeof...(Ints);
  using validatedSeqType =
      typename std::conditional<isValid, Seq<Ints...>, Seq<>>::type;

public:
  using type = typename IFilteredTuple<Tuple, validatedSeqType>::type;
};

template <typename Tuple, int N> struct get_first_n {
public:
  using type =
      typename FilterTuple<Tuple, typename MakeSeq<0, N - 1, 0>::type>::type;
};

template <typename Tuple, int N> struct get_after_first_n {
private:
  static int const tupleSize = std::tuple_size<Tuple>::value;

public:
  using type =
      typename FilterTuple<Tuple,
                           typename MakeSeq<N, tupleSize - 1, N>::type>::type;
};

// Better Tuples End

// match_input_output Start

template <typename... unusedTypes> struct match_input_output {};

template <typename InputTuple> struct match_input_output<InputTuple> {
public:
  static bool constexpr value = std::tuple_size<InputTuple>::value == 0;
};

template <typename InputTuple, typename CurrNode, typename... RestOfTheNodes>
struct match_input_output<InputTuple, CurrNode, RestOfTheNodes...> {

  static int constexpr currNodeInputSize =
      std::tuple_size<typename CurrNode::input_tuple_type>::value;
  static int constexpr currTupleSize = std::tuple_size<InputTuple>::value;

  using relevantOutputType =
      typename get_first_n<InputTuple, currNodeInputSize>::type;

  using relevantNextCheckType =
      typename get_after_first_n<InputTuple, currNodeInputSize>::type;

public:
  static bool constexpr value =
      (currNodeInputSize <= currTupleSize) &&
      std::is_same<relevantOutputType,
                   typename CurrNode::input_tuple_type>::value &&
      match_input_output<relevantNextCheckType, RestOfTheNodes...>::value;
};

// match_input_output End

#endif
