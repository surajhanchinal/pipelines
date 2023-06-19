#include "camel_buffer.h"
#include "node.h"
#include "orchestrator.h"
#include <iostream>
using namespace std;

int main() {

  auto node1 = new ExampleFirstNode();
  node1->name = "yuem";

  auto node2 = new ExampleSecondNode();
  node2->name = "bcd";

  auto o1 = Orchestrator();

  o1.registerNode(node1);
  o1.registerNode(node2);

  node1->outputs->ports[0]->setOtherPort(node2->inputs->ports[0]);

  node1->outputs->ports[1]->setOtherPort(node2->inputs->ports[1]);
  node1->outputs->ports[2]->setOtherPort(node2->inputs->ports[2]);

  auto isValid = o1.validateGraph();

  std::cout << "size of pointer: " << sizeof(node1) << " " << sizeof(NodeBase *)
            << std::endl;

  auto mybuffer =
      moodycamel::BlockingReaderWriterCircularBuffer(100, sizeof(NodeBase *));

  mybuffer.try_enqueue(node1);

  cout << "checking if I still have node: " << node1->name << endl;

  ExampleSecondNode *x;

  mybuffer.try_dequeue(x);

  cout << "hey  " << x->name << endl;

  return 0;
}
