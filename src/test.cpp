#include "node.h"
#include "some.h"
#include "things.h"
#include <iostream>
#include "utilities.h"
using namespace std;

int main() {
  using node_input_type = type_list_t<int, int>;
  using node_output_type = type_list_t<int,int>;

  
  auto node1 =  ExampleFirstNode();
  auto node2 =  ExampleSecondNode();
  auto node3 =  ExampleSecondNode();
  node1.attach(node2,node3);

  cout << "Hello world!" << endl;
  sm::lbr::printSomething();
  cout << someString << endl;
  return 0;
}
