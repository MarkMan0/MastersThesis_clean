#pragma once

#include <limits>

class PIDController {
public:
  double kp_{ 0 }, ki_{ 0 }, kd_{ 0 };
  double upper_{ std::numeric_limits<double>::max() }, lower_{ -std::numeric_limits<double>::max() };
  double sample_t_{ 0.1 };

  double tick(double e);
  void reset();

private:
  double i_out_{ 0 };
  double last_e_{ 0 };
};
