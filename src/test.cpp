#include "node.h"
#include "some.h"
#include "things.h"
#include <iostream>
using namespace std;

int main() {
  using node_input_type = type_list_t<int, int>;
  using node_output_type = type_list_t<int,int>;
  auto frameNode = new FrameProcessorNode();
  frameNode->process({
      {a : 100},
      {a : 20},
  });
  cout << "Hello world!" << endl;
  sm::lbr::printSomething();
  cout << someString << endl;
  return 0;
}
