#include "RobotControl/Controller.h"
#include <algorithm>
#include "RobotControl/Utility.h"

double PIDController::tick(double e) {
  i_out_ += ki_ * e;
  i_out_ = std::clamp(i_out_, lower_, upper_);

  double derivative = kd_ * (e - last_e_) / sample_t_;
  last_e_ = e;

  return std::clamp(kp_ * e + i_out_ + derivative, lower_, upper_);
}

void PIDController::reset() {
  last_e_ = 0;
  i_out_ = 0;
}
