#include "camel_buffer.h"
#include "node.h"
#include "orchestrator.h"
#include <iostream>
using namespace std;

int main() {

  auto node1 = new ExampleFirstNode();

  auto node2 = new ExampleSecondNode();

  auto o1 = Orchestrator();

  o1.registerNode(node1);
  o1.registerNode(node2);

  node1->attachPort<0, 0>(node2);
  node1->attachPort<1, 1>(node2);
  node1->attachPort<2, 2>(node2);

  auto isValid = o1.start();

  cout << "Is graph valid: " << isValid << endl;

  return 0;
}
