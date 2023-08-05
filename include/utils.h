#pragma once
#include "config_store.h"
#include <chrono>
// Want to keep the apparent time the pipeline as close to realtime as possible
// even if we slow down the whole pipeline.
double
scaledDelayInMs(const std::chrono::time_point<std::chrono::system_clock> &tp1,
                const std::chrono::time_point<std::chrono::system_clock> &tp2) {
  return std::chrono::duration_cast<std::chrono::milliseconds>(tp1 - tp2)
             .count() /
         ConfigStore::timeScale;
}

double
scaledDelayInMicro(std::chrono::time_point<std::chrono::system_clock> &tp1,
                   std::chrono::time_point<std::chrono::system_clock> &tp2) {
  return std::chrono::duration_cast<std::chrono::microseconds>(tp1 - tp2)
             .count() /
         ConfigStore::timeScale;
}
