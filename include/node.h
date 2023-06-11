#ifndef NODE_H
#define NODE_H

#include <iostream>
#include <type_traits>
#include <tuple>

template<typename... Args>
class Node {
	using Tuple = std::tuple<Args...>;
	static constexpr auto Size  = sizeof...(Args);
	template <std::size_t N>
	using Nth = typename std::tuple_element<N, Tuple>::type;
	using First = Nth<0>;
	using Last = Nth<Size  - 1>;
};

#endif
