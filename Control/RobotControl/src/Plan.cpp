#include "RobotControl/Plan.h"

#include "RobotControl/Logger.h"
#include "RobotControl/geometry.h"
#include "RobotControl/config.h"


void planning::PathPlan::apply_limits() {
  init_blocks();
  limit_speed();
  centrifugal_limit();
  acceleration_pass(blocks_.begin(), blocks_.end());
  acceleration_pass(blocks_.rbegin(), blocks_.rend());
  limit_min_speed();
  calculate_time();


#ifdef LOG_SPEED_LIMIT
  std::ofstream out(LOGS_DIR "/logs/vel.txt");
  for (const auto& b : blocks_) {
    out << b.elapsed_ << ", " << b.velocity_ << '\n';
  }

  std::ofstream out1(LOGS_DIR "/logs/cent.txt");
  std::ofstream out2(LOGS_DIR "/logs/spd_l.txt");
  std::ofstream out3(LOGS_DIR "/logs/spd_r.txt");
  std::ofstream out4(LOGS_DIR "/logs/accel_l.txt");
  std::ofstream out5(LOGS_DIR "/logs/accel_r.txt");
  std::ofstream out6(LOGS_DIR "/logs/accel.txt");

  for (int i = 1; i < blocks_.size(); ++i) {
    const auto [vl, vr] = wheel_speeds_radius(blocks_[i].velocity_, blocks_[i].radius_);
    const auto [vl_prev, vr_prev] = wheel_speeds_radius(blocks_[i - 1].velocity_, blocks_[i - 1].radius_);

    double d_spd_l = vl - vl_prev;
    double d_spd_r = vr - vr_prev;
    double dv = blocks_[i].velocity_ - blocks_[i - 1].velocity_;
    double dt = blocks_[i].elapsed_ - blocks_[i - 1].elapsed_;
    out1 << blocks_[i].elapsed_ << ", "
         << (config::robot_weight * utils::sqr(blocks_[i].velocity_) / std::abs(blocks_[i].radius_)) << '\n';
    out2 << blocks_[i].elapsed_ << ", " << vl << '\n';
    out3 << blocks_[i].elapsed_ << ", " << vr << '\n';
    out4 << blocks_[i].elapsed_ << ", " << d_spd_l / dt << '\n';
    out5 << blocks_[i].elapsed_ << ", " << d_spd_r / dt << '\n';
    out6 << blocks_[i].elapsed_ << ", " << dv / dt << '\n';
  }
#endif

  is_ready_ = true;
}

void planning::PathPlan::init_blocks() {
  blocks_.clear();
  auto points = path_->create_points();
  blocks_.reserve(points.size());
  for (const auto& p : points) {
    PlanBlock b;
    b.target_ = p;
    b.velocity_ = path_->get_speed();
    blocks_.push_back(b);
  }

  for (int i = 1; i < blocks_.size() - 1; ++i) {
    const auto [cent, rad] =
        geometry::circle_point_point_point(blocks_[i - 1].target_, blocks_[i].target_, blocks_[i + 1].target_);
    if (!isnan(rad) && !isinf(rad) && abs(rad) > 0.0001) {
      bool cw = geometry::is_arc_cw(cent, blocks_[i].target_, blocks_[i].target_.angle_to(blocks_[i + 1].target_));
      blocks_[i].radius_ = std::clamp(rad, 0.0, 100.0);
      if (cw) {
        blocks_[i].radius_ *= -1;
      }
    }
  }
}


void planning::PathPlan::limit_speed() {
  // limit max wheel speed
  for (auto& block : blocks_) {
    const auto [vl, vr] = wheel_speeds_radius(block.velocity_, std::abs(block.radius_));
    double fact = 1;
    fact = std::min(fact, config::max_velocity / vl);
    fact = std::min(fact, config::max_velocity / vr);
    block.velocity_ *= fact;
  }
}


void planning::PathPlan::centrifugal_limit() {
  constexpr double m = config::robot_weight, max_centrifugal = config::max_centrifugal;

  for (auto& block : blocks_) {
    if (block.radius_ == 0) {
      continue;
    }
    const double cent = m * utils::sqr(block.velocity_) / std::abs(block.radius_);
    if (cent > max_centrifugal) {
      const double fact = max_centrifugal / cent;
      block.velocity_ *= std::sqrt(fact);
    }
  }
}

template <class BlockIter>
void planning::PathPlan::acceleration_pass(BlockIter begin, BlockIter end) {
  if (end - begin < 3) {
    throw std::length_error("path is too short");
  }

  begin->velocity_ = 0;
  (end - 1)->velocity_ = 0;

  for (auto it = begin + 1; it != end; ++it) {
    double a_max = config::max_accel;
    if (it->radius_ != 0) {
      double r = std::abs(it->radius_);
      a_max = a_max / (1.0 / r * (r + config::wheel_dist / 2));
    }

    auto prev = it - 1;
    const double d = prev->target_.dist(it->target_);
    const double t = 1.0 / a_max * (-prev->velocity_ + std::sqrt(utils::sqr(prev->velocity_) + 2 * a_max * d));
    double v2 = prev->velocity_ + a_max * t;

    it->velocity_ = std::min(it->velocity_, v2);
  }
}


std::tuple<double, double> planning::PathPlan::wheel_speeds_radius(double v, double r) const {
  if (abs(r) < 0.01 || abs(r) > 100 || isnan(r) || isinf(r)) {
    return { v, v };
  }

  double spd_left = v / r * (r - config::wheel_dist / 2);
  double spd_right = v / r * (r + config::wheel_dist / 2);
  return { spd_left, spd_right };
}


std::tuple<double, double> planning::PathPlan::wheel_speeds_angular(double v, double omega) const {
  const double vl = v - 0.5 * omega * config::wheel_dist, vr = v + 0.5 * omega * config::wheel_dist;
  return { vl, vr };
}


std::tuple<double, double> planning::PathPlan::robot_speeds_from_wheels(double vl, double vr) const {
  double v = 0.5 * (vl + vr);
  double omega = (vr - vl) / config::wheel_dist;
  return { v, omega };
}

void planning::PathPlan::limit_min_speed() {
  for (size_t i = 0; i < blocks_.size(); ++i) {
    blocks_[i].velocity_ = std::max(config::min_velocity, blocks_[i].velocity_);
  }
}

void planning::PathPlan::calculate_time() {
  if (blocks_.size() < 3) {
    throw std::length_error("path is too short");
  }
  double elapsed = 0, dist_accum = 0;
  for (size_t i = 0; i < blocks_.size() - 1; ++i) {
    auto& curr = blocks_[i];
    const auto& next = blocks_[i + 1];
    const double d = curr.target_.dist(next.target_);
    curr.distance_travelled_ = dist_accum;
    curr.duration_ = d / curr.velocity_;
    curr.elapsed_ = elapsed;
    elapsed += curr.duration_;
    dist_accum += d;
  }
  blocks_.back().elapsed_ = elapsed;
}


utils::Point planning::PathPlan::get_closest(const utils::Point& p0) const {
  utils::Point ret = p0;
  double min_dist = std::numeric_limits<double>::max();

  for (const auto& p : blocks_) {
    const double d = p0.dist(p.target_);
    if (d < min_dist) {
      min_dist = d;
      ret = p.target_;
    }
  }

  return ret;
}

std::optional<utils::Point> planning::PathPlan::find_intersect(const utils::Point& p0, const utils::Point& p1) const {
  return path_->line_intersect(p0, p1);  // get the intersect
}

void planning::TurnPlan::apply_limit() {
  init_blocks();
  acceleration_pass(blocks_.begin(), blocks_.end());
  acceleration_pass(blocks_.rbegin(), blocks_.rend());
  limit_min_speed();
  calculate_time();
}


void planning::TurnPlan::init_blocks() {
  if (target_ > 0) {
    for (double d = 0; d < target_; d += utils::deg2rad(1)) {
      TurnBlock b;
      b.omega_ = omega_;
      b.target_ = d;
      blocks_.push_back(b);
    }
  } else {
    for (double d = 0; d > target_; d -= utils::deg2rad(1)) {
      TurnBlock b;
      b.omega_ = omega_;
      b.target_ = d;
      blocks_.push_back(b);
    }
  }
}


template <class BlockIter>
void planning::TurnPlan::acceleration_pass(BlockIter begin, BlockIter end) {
  if (end - begin < 3) {
    throw std::length_error("path is too short");
  }

  begin->omega_ = 0;
  (end - 1)->omega_ = 0;

  constexpr double a_max = config::max_angular_acc;  // rad s^(-2)
  for (auto it = begin + 1; it != end; ++it) {
    auto prev = it - 1;
    const double d = std::abs(it->target_ - prev->target_);
    const double t = 1.0 / a_max * (-prev->omega_ + std::sqrt(utils::sqr(prev->omega_) + 2 * a_max * d));
    const double v2 = prev->omega_ + a_max * t;
    it->omega_ = std::min(it->omega_, v2);
  }
}


void planning::TurnPlan::limit_min_speed() {
  for (size_t i = 0; i < blocks_.size() - 1; ++i) {
    blocks_[i].omega_ = std::max(config::min_angular_velocity, blocks_[i].omega_);
  }
}


void planning::TurnPlan::calculate_time() {
  if (blocks_.size() < 3) {
    throw std::length_error("path is too short");
  }
  double elapsed = 0, angle_accum_ = 0;
  for (size_t i = 0; i < blocks_.size() - 1; ++i) {
    auto& curr = blocks_[i];
    const auto& next = blocks_[i + 1];
    const double d = next.target_ - curr.target_;
    curr.angle_turned_ = angle_accum_;
    curr.duration_ = d / curr.omega_;
    curr.elapsed_ = elapsed;
    elapsed += curr.duration_;
    angle_accum_ += d;
  }
  blocks_.back().elapsed_ = elapsed;
}
