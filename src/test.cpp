#include "node.h"
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
auto myTuple =  node1->attach(node2, node3);

  cout << "Hello world!" << endl;
  
  sm::lbr::printSomething();
  cout << someString << endl;
  return 0;
}
