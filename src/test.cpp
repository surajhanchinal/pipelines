#include "camel_buffer.h"
#include "node.h"
#include "orchestrator.h"
#include <iostream>
using namespace std;

int main() {

  auto node1 = new Node(0, 3);
  node1->name = "hey";

  auto node2 = new Node(3, 0);
  node2->name = "bcd";

  auto o1 = Orchestrator();

  o1.attachNode(node1);
  o1.attachNode(node2);

  node1->outputs->ports[0]->setOtherPort(node2->inputs->ports[0]);

  node1->outputs->ports[1]->setOtherPort(node2->inputs->ports[1]);
  node1->outputs->ports[2]->setOtherPort(node2->inputs->ports[2]);

  auto isValid = o1.validateGraph();


  std::cout<<"size of pointer: " << sizeof(node1) << " "<< sizeof(Node*)<<std::endl;

  auto mybuffer = moodycamel::BlockingReaderWriterCircularBuffer(100,sizeof(Node*));


  mybuffer.try_enqueue(node1);

  Node* x;

  mybuffer.try_dequeue(x);

  cout << "hey  " << x->name << endl;

  return 0;
}
