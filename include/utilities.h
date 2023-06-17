#ifndef UTILITIES_H
#define UTILITIES_H

#include <iostream>
#include <tuple>
#include <type_traits>
#include <utility>

template<int... Is>
struct Seq{};

template<int Start,int End, int Curr,int... Is>
struct MakeSeq {
public:
using type = typename std::conditional<(Curr > End),Seq<Is...>,typename MakeSeq<Start,End,Curr+1,Is...,Curr>::type>::type;
};

template<int Start, int Curr, int... Is>
struct MakeSeq<Start, Curr, Curr, Is...>
{
public:
using type = Seq<Is...,Curr>;
};


template<typename... unusedTypes>
struct filter_tuple {};

template<typename Tuple,int... Ints>
struct filter_tuple<Tuple,Seq<Ints...>>{
public:
using type = std::tuple<typename std::tuple_element<Ints,Tuple>::type...>;
};
#endif
