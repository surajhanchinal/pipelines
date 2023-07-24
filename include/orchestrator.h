#pragma once

#include "buffer.h"
#include "camel_buffer.h"
#include "common_data.h"
#include "node.h"
#include <atomic>
#include <chrono>
#include <iostream>
#include <queue>
#include <signal.h>
#include <thread>
#include <vector>

void sig_handle(int) {
  std::cout << "orchestrator sigint" << std::endl;
  common_data::running = false;
}

class Orchestrator {
private:
  std::vector<NodeBase *> nodes;
  std::vector<moodycamel::BlockingReaderWriterCircularBuffer *> bufs;
  int mainNodeIdx = -1;
  std::vector<std::atomic<bool>> thread_loop_signals;
  std::vector<int> bfs_order;

public:
  Orchestrator() { signal(SIGINT, sig_handle); }

  void registerNode(NodeBase *_node, bool runOnMain = false) {
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
      auto buf = new moodycamel::BlockingReaderWriterCircularBuffer(1000, 300);
      bufs.push_back(buf);
      port->setNodeIdx(nodes.size() - 1);
      port->setBuffer(buf);
    }
    for (auto const &port : _node->inputs->ports) {
      port->setNodeIdx(nodes.size() - 1);
    }
    if (runOnMain) {
      mainNodeIdx = nodes.size() - 1;
    }
  }

  bool start() {
    if (!validateGraph()) {
      std::cout << "Computation graph is not valid" << std::endl;
      return false;
    }

    if (mainNodeIdx != -1) {
      nodes[mainNodeIdx]->init();
    }
    thread_loop_signals = std::vector<std::atomic<bool>>(nodes.size());
    for (int i = 0; i < nodes.size(); i++) {
      auto &t = thread_loop_signals.at(i);
      t = true;
      nodes[i]->setThreadSignal(&thread_loop_signals[i]);
    }

    std::vector<std::thread> threads;
    for (int i = 0; i < nodes.size(); i++) {
      if (i == mainNodeIdx) {
        continue;
      }
      threads.push_back(std::thread(&NodeBase::process, nodes[i]));
    }

    threads.push_back(
        std::thread(&Orchestrator::close_threads_on_sigint, this));

    if (mainNodeIdx != -1) {
      nodes[mainNodeIdx]->process();
    }

    for (int i = 0; i < threads.size(); i++) {
      threads[i].join();
    }
    return true;
  }

  void close_threads_on_sigint() {
    std::cout << "starting signal thread" << std::endl;
    while (common_data::running) {
      std::this_thread::sleep_for(std::chrono::milliseconds(30));
    }
    std::cout << "closing threads" << std::endl;
    // Sigint called close the other threads in order;
    for (int i = bfs_order.size() - 1; i >= 0; i--) {
      thread_loop_signals[i] = false;
      std::this_thread::sleep_for(std::chrono::milliseconds(3));
    }
  }

  bool validateGraph() {
    std::vector<bool> visited(nodes.size(), 0);
    std::queue<std::pair<NodeBase *, int>> qq;
    for (int i = 0; i < nodes.size(); i++) {
      if (nodes[i]->inputs->ports.size() == 0) {
        qq.push({nodes[i], i});
        visited[i] = true;
      }
    }

    if (qq.size() == 0) {
      std::cout << "You do not have any generate nodes, Please fix!"
                << std::endl;
      return false;
    }

    while (!qq.empty()) {
      auto node = qq.front().first;
      bfs_order.push_back(qq.front().second);
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
            qq.push({nodes[newNodeIdx], newNodeIdx});
          }
        }
      }
    }

    return true;
  }
};
