#pragma once
#include <atomic>
#include "utils.h"
namespace common_data {
    std::atomic<bool> running = true;
    std::atomic<bool> solutionFound = false;
    Matrix4f bat_fk;
};
