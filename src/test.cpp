#include "node.h"
#include "orchestrator.h"
#include "some.h"
#include "things.h"
#include "utilities.h"
#include <iostream>
using namespace std;

int main() {
  using node_input_type = type_list_t<int, int>;
  using node_output_type = type_list_t<int, int>;

  static_assert(std::is_same<typename MakeSeq<5, 10, 5>::type,
                             Seq<5, 6, 7, 8, 9, 10>>::value);
  auto node1 = new ExampleFirstNode();
  auto node2 = new ExampleSecondNode();
  auto node3 = new ExampleSecondNode();

  auto tuple = std::tuple(1, 2, 'c', node1);

  auto o1 = Orchestrator();

  // auto o2 = Orchestrator(o1, node1);

  // auto o3 = Orchestrator(Orchestrator(Orchestrator(),node2),node3);
  //  auto o8 = Orchestrator() | node1;
  auto o4 = Orchestrator() | node1 | node2 | node3;

  auto x = std::get<0>(std::get<2>(o4.nodes)->outputs.ports)->buffer->data;

  x = node3->my_idx;

  cout << "hey" << x << endl;

  sm::lbr::printSomething();
  cout << someString << endl;
  return 0;
}
