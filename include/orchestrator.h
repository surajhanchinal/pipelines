#pragma once

#include "buffer.h"
#include "camel_buffer.h"
#include "node.h"
#include <iostream>
#include <queue>
#include <thread>
#include <vector>

class Orchestrator {
private:
  std::vector<NodeBase *> nodes;
  std::vector<moodycamel::BlockingReaderWriterCircularBuffer *> bufs;

public:
  void registerNode(NodeBase *_node) {
    bool found = false;
    for (auto const &node : nodes) {
      if (node == _node) {
        std::cout << "Adding a node that is already present" << std::endl;
        found = true;
        break;
      }
    }
    if (found)
      return;
    nodes.push_back(_node);
    for (auto const &port : _node->outputs->ports) {
      auto buf = new moodycamel::BlockingReaderWriterCircularBuffer(100, 8);
      bufs.push_back(buf);
      port->setNodeIdx(nodes.size() - 1);
      port->setBuffer(buf);
    }
    for (auto const &port : _node->inputs->ports) {
      port->setNodeIdx(nodes.size() - 1);
    }
  }

  bool start() {
    if (!validateGraph()) {
      std::cout << "Computation graph is not valid" << std::endl;
      return false;
    }
    std::vector<std::thread> threads;
    for (auto const &node : nodes) {
      threads.push_back(std::thread(&NodeBase::process, node));
    }

    for (int i = 0; i < threads.size(); i++) {
      threads[i].join();
    }
    return true;
  }

  bool validateGraph() {
    std::vector<bool> visited(nodes.size(), 0);
    std::queue<NodeBase *> qq;
    for (int i = 0; i < nodes.size(); i++) {
      if (nodes[i]->inputs->ports.size() == 0) {
        qq.push(nodes[i]);
        visited[i] = true;
      }
    }

    if (qq.size() == 0) {
      std::cout << "You do not have any generate nodes, Please fix!"
                << std::endl;
      return false;
    }

    while (!qq.empty()) {
      auto node = qq.front();
      qq.pop();
      // validate input nodes
      for (auto const &port : node->inputs->ports) {
        if (!port->isAttached) {
          std::cout << "Not all input ports of a node are attached, Node IDX: "
                    << port->nodeIndex << std::endl;
          return false;
        }
      }
      for (auto const &port : node->outputs->ports) {
        // Port ain't attached
        if (!port->isAttached) {
          std::cout << "Not all output ports of a node are attached, Node IDX: "
                    << port->nodeIndex << std::endl;
          return false;
        } else {
          auto newNodeIdx = port->otherPort->nodeIndex;
          if (!visited[newNodeIdx]) {

            visited[newNodeIdx] = true;
            qq.push(nodes[newNodeIdx]);
          }
        }
      }
    }

    return true;
  }
};
