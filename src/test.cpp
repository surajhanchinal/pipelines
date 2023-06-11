#include "some.h"
#include "things.h"
#include "node.h"
#include <iostream>
using namespace std;

int main() {
  auto node = new Node<int,int>();
  cout << "Hello world!" << endl;
  sm::lbr::printSomething();
  cout << someString << endl;
  return 0;
}
