#ifndef ORCHESTRATOR_H
#define ORCHESTRATOR_H
#include "node.h"
#include <iostream>
#include <tuple>
#include <type_traits>
#include <utility>


template <typename... Ts> class Orchestrator {
public:
  std::tuple<Ts...> nodes;
  template <class A, class B, class... Us>
  Orchestrator(Orchestrator<Us...> &&ns, Node<A, B> *t);
  Orchestrator() { nodes = std::tuple<>(); };
};

template <typename... Ts>
template <typename A, typename B, typename... Is>
Orchestrator<Ts...>::Orchestrator(Orchestrator<Is...> &&orchestrator,
                                  Node<A, B> *node) {
  nodes = std::tuple_cat(std::move(orchestrator.nodes),
                         std::tuple<Node<A, B> *>(node));
  node->my_idx = sizeof...(Is);
  node->inputs.updateNodeIdx(node->my_idx);
  node->outputs.createBuffers();
}

// template<typename... Ts>
// template<typename... inputs_t,typename... outputs_t,typename... Is>
// Orchestrator<Ts...>::Orchestrator<Node<type_list_t<inputs_t...>,type_list_t<outputs_t>>>(Orchestrator<Is...>
// &&orchestrator,Node<type_list_t<inputs_t...>,type_list_t<outputs_t...>>*
// node){}

template <class A,class B> Orchestrator(Orchestrator<> &&,Node<A,B> *fun) -> Orchestrator<Node<A,B>* >;

//template <class... Ts, class F>
//Orchestrator(Orchestrator<Ts...> &&, F *fun) -> Orchestrator<Ts..., F *>;

template <class... Ts, class A, class B>
Orchestrator(Orchestrator<Ts...> &&, Node<A, B> *fun)
    -> Orchestrator<Ts..., Node<A, B> *>;

// Chaining operator.
template <class... Ts, class A,class B>
auto operator|(Orchestrator<Ts...> &&pl, Node<A,B> *transform) {
  return Orchestrator(std::move(pl), transform);
}

#endif
