#pragma once
#include <chrono>
#include <thread>

/**
 * Tries to keep the time between calls at the same rate.
 */
class LoopRate {
public:
  using clock = std::chrono::high_resolution_clock;
  LoopRate(double dt) : dt_(dt), start_(clock::now()) {
  }

  void sleep() {
    std::this_thread::sleep_until(start_ + hitCnt_ * dt_);
    ++hitCnt_;
  }


private:
  std::chrono::duration<double> dt_;
  unsigned long long hitCnt_ = 0;
  using time_t = decltype(clock::now());
  const time_t start_;
};
