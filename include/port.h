#pragma once

#include "camel_buffer.h"
#include <iostream>
struct Port {
  moodycamel::BlockingReaderWriterCircularBuffer *buffer;
  int nodeIndex = -1;
  Port *otherPort;
  bool isAttached = false;

  void setNodeIdx(int idx) { nodeIndex = idx; }
  void setBuffer(moodycamel::BlockingReaderWriterCircularBuffer *buf) {
    buffer = buf;
  }
  void _setOtherPort(Port *const other) { otherPort = other; }
  void setOtherPort(Port *const other) {
    if (nodeIndex == -1) {
      std::cout << "Register the node with the orchestrator first" << std::endl;
      return;
    }
    if (isAttached) {
      std::cout << nodeIndex << "  " << other->nodeIndex << std::endl;
      std::cout << "trying to attach a port that was already attached"
                << std::endl;
      return;
    }
    isAttached = true;
    other->isAttached = true;
    other->setBuffer(buffer);
    other->_setOtherPort(this);
    otherPort = other;
  }
};
