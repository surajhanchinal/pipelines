#ifndef UTILITIES_H
#define UTILITIES_H

#include <iostream>
#include <tuple>
#include <type_traits>
#include <utility>

// Sequence Start

template<int... Is>
struct Seq{};

template<int Start,int End, int Curr,int... Is>
struct MakeSeq {
public:
static int constexpr newStart = (Curr > End) ? -1 : Start;
static int constexpr newEnd = (Curr > End) ? -1 : End;
using type = typename std::conditional<(Curr > End),Seq<Is...>,typename MakeSeq<newStart,newEnd,Curr+1,Is...,Curr>::type>::type;
};

template<int Start, int Curr, int... Is>
struct MakeSeq<Start, Curr, Curr, Is...>
{
public:
using type = Seq<Is...,Curr>;
};

template<int Curr,int... Is>
struct MakeSeq<-1,-1,Curr,Is...>{
public:
  using type = Seq<>;
};


// Sequence End

//Filter Tuple Start

template<typename... unusedTypes>
struct filter_tuple {};

template<typename Tuple,int... Ints>
struct filter_tuple<Tuple,Seq<Ints...>>{
public:
using type = std::tuple<typename std::tuple_element<Ints,Tuple>::type...>;
};

template<typename A,typename B>
struct empty_seq_if_bigger_than_tuple{};

template<typename Tuple,int... Ints>
struct empty_seq_if_bigger_than_tuple<Tuple,Seq<Ints...>>{
  static bool constexpr validSeq = std::tuple_size<Tuple>::value >= sizeof...(Ints);
  public:
    using type = typename std::conditional<validSeq,Seq<Ints...>,Seq<>>::type;
};

//Filter Tuple End


#endif
