#include "RobotControl/MotionQueue.h"

#include <thread>
#include <chrono>

void MotionQueue::tick() {
  if (!queue_.empty()) {
    const Motion m = queue_.front();
    queue_.pop_front();
    using MT = MotionType;
    switch (m.type_) {
      case MT::PAUSE: {
        std::this_thread::sleep_for(std::chrono::milliseconds(m.ms));
        break;
      }
      case MT::TURN: {
        nav_.turn_to(m.angle);
        break;
      }
      case MT::XY: {
        nav_.move_to(m.p);
        break;
      }
    }
    if (queue_.empty()) {
      nav_.stop();
    }
  }
}

double MotionQueue::get_dist_to_next_stop(utils::Point from) const {
  double accum = 0;


  if (queue_.empty()) {
    return 0;
  }

  auto iter = queue_.begin();

  for (auto iter = queue_.begin(); iter != queue_.end() && iter->type_ == MotionType::XY; ++iter) {
    accum += from.dist(iter->p);
    from = iter->p;
  }

  return accum;
}
