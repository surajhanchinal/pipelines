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

// Filter Tuple Start

template <typename... unusedTypes> struct filter_tuple {};

template <typename Tuple, int... Ints>
struct filter_tuple<Tuple, Seq<Ints...>> {
public:
  using type = std::tuple<typename std::tuple_element<Ints, Tuple>::type...>;
};

template <typename A, typename B> struct empty_seq_if_bigger_than_tuple {};

template <typename Tuple, int... Ints>
struct empty_seq_if_bigger_than_tuple<Tuple, Seq<Ints...>> {
  static bool constexpr validSeq =
      std::tuple_size<Tuple>::value >= sizeof...(Ints);

public:
  using type = typename std::conditional<validSeq, Seq<Ints...>, Seq<>>::type;
};

// Filter Tuple End

#endif
