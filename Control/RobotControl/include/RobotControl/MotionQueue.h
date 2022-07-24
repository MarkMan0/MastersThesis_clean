#pragma once

#include <deque>
#include <array>

#include "Navigation.h"
#include "Utility.h"

class MotionQueue {
public:
  enum class MotionType {
    TURN,
    XY,
    PAUSE,
  };
  struct Motion {
    MotionType type_;
    utils::Point p;
    double angle{ 0 };
    int ms{ 0 };
  };

  MotionQueue(Navigation& nav) : nav_{ nav } {};

  void push(const Motion& motion) {
    queue_.push_back(motion);
  }

  const std::deque<Motion>& get_queue() const {
    return queue_;
  }

  void tick();

  double get_dist_to_next_stop(utils::Point from) const;


private:
  std::deque<Motion> queue_;
  Navigation& nav_;
};
