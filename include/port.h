#ifndef PORT_H
#define PORT_H
#include "buffer.h"

template <typename T> struct Port {
public:
  Buffer<T> *buffer;
  int ownIndex;
  int nodeIndex;
  Port<T> *otherPort;
  void setNodeIdx(int idx) { nodeIndex = idx; }
  void createBuffer(int idx) { buffer = new Buffer<T>(10); }
};

#endif
