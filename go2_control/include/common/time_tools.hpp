/**
 * Time utility functions for Unitree API
 * EDTH Hackathon - Laelaps AI
 */
#pragma once

#include <chrono>
#include <ctime>

namespace unitree::common {

inline int64_t GetSystemUptimeInNanoseconds() {
  struct timespec ts {};
  clock_gettime(CLOCK_MONOTONIC, &ts);
  return static_cast<int64_t>(ts.tv_sec) * 1000000000 + ts.tv_nsec;
}

inline uint64_t GetCurrentTimeMilliseconds() {
  auto now = std::chrono::system_clock::now();
  auto duration = now.time_since_epoch();
  return std::chrono::duration_cast<std::chrono::milliseconds>(duration).count();
}

}  // namespace unitree::common

