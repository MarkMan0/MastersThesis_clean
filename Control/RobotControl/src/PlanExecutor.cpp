#include "RobotControl/PlanExecutor.h"
#include <vector>
#include <stdexcept>
#include <fstream>
#include "RobotControl/Logger.h"
#include "RobotControl/SimpleMotion.h"
#include <chrono>
#include <iostream>
#include <stdexcept>
#include "RobotControl/geometry.h"
#include "RobotControl/LoopRate.h"
#include <numeric>

#include "fuzzy_system.h"

void planning::PlanExecutor::init_controllers() {
  fuzzy_engine_ = std::unique_ptr<fl::Engine>(fl::FisImporter().fromString(fuzzy_systems::line_control_fuzzy));
  dual_sensor_fuzzy_ = std::unique_ptr<fl::Engine>(fl::FisImporter().fromString(fuzzy_systems::fuzzy_dual_senzor));

  speed_controller_.kp_ = 0.05;
  // speed_controller_.ki_ = 0.002;

  tangent_controller_.kp_ = 15;


  path_dist_controller_.kp_ = 8;
  path_dist_controller_.ki_ = 0.1;
  path_dist_controller_.kd_ = 8;
  path_dist_controller_.upper_ = 5;
  path_dist_controller_.lower_ = -5;
}

void planning::PlanExecutor::path_loop_tick() {
  // cache current position
  robot_pos_ = motion_->get_pos();

  const double spd = speed_control();
  // const double omega = utils::combiner<1, 0>()(path_dist_cont(), spd * tangent_control());
  const double omega = utils::combiner<2, 3>()(fuzzy_dual_sensor_control(), spd * tangent_control());
  // static double last = 0;
  // const double omega = 0.8*fuzzy_dual_sensor_control();
  // last = last * 0.9 + omega*0.1;
#ifdef DEBUG_PATH_LOOP
  logging::logger.log("control.spd", spd);
  logging::logger.log("control.omega", omega);
#endif

  motion_->move_angular_velocity(spd, omega);
}

double planning::PlanExecutor::speed_control() {
  double d_pos = robot_pos_.p.dist(current_block_->target_);

  const double robot_heading = utils::normalize_angle(robot_pos_.theta);
  const double target = utils::normalize_angle(robot_pos_.p.angle_to(current_block_->target_));

  const double diff = utils::normalize_angle(target - robot_heading);

  if (abs(diff) > utils::PI / 2) {
    // ahead of path
    d_pos *= -1;
  }

  double u2 = speed_controller_.tick(d_pos);

#ifdef DEBUG_SPEED_CONTROL
  logging::logger.log("spd_cont.d_pos", d_pos);
  logging::logger.log("spd_cont.diff", diff);
#endif


  return current_block_->velocity_ + u2;
}

double planning::PlanExecutor::signed_distance_from_path() const {
  const auto closest = get_closest_point();
  // left or right side
  const auto p0 = robot_pos_.p;
  const auto p1 = p0 + utils::Point(std::cos(robot_pos_.theta), std::sin(robot_pos_.theta));

  const int multiplier = utils::sign((p1.x_ - p0.x_) * (closest.y_ - p0.y_) - (p1.y_ - p0.y_) * (closest.x_ - p0.x_));
  const double distance = closest.dist(robot_pos_.p);

#ifdef DEBUG_SIGNED_DIST_FROM_PATH
  logging::logger.log("signed_dist.multiplier", multiplier);
  logging::logger.log("signed_dist.distance", distance);
#endif

  return multiplier * distance;
}

double planning::PlanExecutor::path_dist_cont() {
  const double e = signed_distance_from_path();
  const double omega = path_dist_controller_.tick(e);

#ifdef DEBUG_PATH_DIST_CONTROL
  logging::logger.log("path_dist_cont.e", e);
  logging::logger.log("path_dist_cont.omega", omega);
#endif
  return omega;
}

double planning::PlanExecutor::line_follow_control() {
  get_line_follow_error();
  const auto [is, left, right] = get_line_follow_intersect(0.15, 0.3);
  std::optional<double> err = std::nullopt;
  if (is) {
    err = std::optional(2 * (is->dist(right) / left.dist(right) - 0.5));
  }
  // weighted average
  // if can't get radius from feedback, use only open loop radius
  // else calculate weighted average of both
  const double ratio = err.has_value() ? 0.2 : 1.0;
#ifdef DEBUG_LINE_FOLLOW
  logging::logger.log("ratio", 2 * sensor_3_hold_(err));
#endif
  return 1 * sensor_3_hold_();
  ;
}

double planning::PlanExecutor::tangent_control() {
  const double current = utils::normalize_angle(robot_pos_.theta);
  int look_ahead = 50;
  auto it_2 = current_block_;
  while (it_2 != plan_.get_blocks().end() - 1 && look_ahead > 0 && current_block_->target_.dist(it_2->target_) < 0.1) {
    ++it_2;
    --look_ahead;
  }

  const double target = utils::normalize_angle(current_block_->target_.angle_to(it_2->target_));
  const double diff = utils::normalize_angle(target - current);

#ifdef DEBUG_TANGENT_CONTROL
  logging::logger.log("tangent_cont.diff", diff);
#endif

  return tangent_controller_.tick(diff);
}

double planning::PlanExecutor::fuzzy_control() {
  static double last = 0;

  const double dist_from_path = -1 * utils::map_clamped(signed_distance_from_path(), 0.05, 1);

  const double line_follow_err = [this]() -> double {
    const auto [is, p_right, p_left] = this->get_line_follow_intersect(0.15, 0.3);
    if (is) {
      last = utils::map_clamped(is->dist(p_left), 0, p_left.dist(p_right), -1, 1);
    }
    // scale error
    return last;
  }();


  fuzzy_engine_->setInputValue("sensor_pos", line_follow_err);
  fuzzy_engine_->setInputValue("dist_from_path", dist_from_path);
  fuzzy_engine_->process();
  const double out = fuzzy_engine_->getOutputValue("omega");

  const double omega = utils::map_clamped(out, 1, 3);


#ifdef DEBUG_FUZZY_CONTROL
  logging::logger.log("dist_path", dist_from_path);
  logging::logger.log("line_err_path", line_follow_err);
  logging::logger.log("fuzzy_out", omega);
#endif
  return omega;
}

double planning::PlanExecutor::fuzzy_dual_sensor_control() {
  auto get_is_pos = [this](const double dist) -> std::optional<double> {
    const auto [is, left, right] = this->get_line_follow_intersect(dist, 0.15);
#ifdef DEBUG_FUZZY_DUAL_SENSOR
    logging::logger.log("sensor_p1_x_" + std::to_string(dist), left.x_);
    logging::logger.log("sensor_p1_y_" + std::to_string(dist), left.y_);
    logging::logger.log("sensor_p2_x_" + std::to_string(dist), right.x_);
    logging::logger.log("sensor_p2_y_" + std::to_string(dist), right.y_);

    logging::logger.log("sensor_is_x_" + std::to_string(dist), is.value_or(utils::Point(0, 0)).x_);
    logging::logger.log("sensor_is_y_" + std::to_string(dist), is.value_or(utils::Point(0, 0)).y_);
    logging::logger.log("sensor_is_" + std::to_string(dist), static_cast<int>(is.has_value()));
#endif

    if (!is.has_value()) {
      return std::nullopt;
    }

    return std::optional(2 * (is->dist(left) / left.dist(right) - 0.5));  // if intersect, return value [-1 .. 1]
  };

  sensor_1_hold_(get_is_pos(0.1));
  sensor_2_hold_(get_is_pos(0.2));

  dual_sensor_fuzzy_->setInputValue("sensor1", sensor_1_hold_());
  dual_sensor_fuzzy_->setInputValue("sensor2", sensor_2_hold_());
  dual_sensor_fuzzy_->process();
  const double out = dual_sensor_fuzzy_->getOutputValue("output");

  const double omega = 0 - utils::map_clamped(out, 1, 3);
#ifdef DEBUG_FUZZY_DUAL_SENSOR
  logging::logger.log("fuzzy_out", omega);
#endif

  return omega;
}

utils::Point planning::PlanExecutor::get_closest_point() const {
  const auto beg = plan_.get_blocks().begin();
  const auto end = plan_.get_blocks().end();

  constexpr ptrdiff_t band = 50;

  int offset_from = std::min(band, current_block_ - beg);
  int offset_to = std::min(band, end - current_block_ - 1);


  auto search_from = current_block_ - offset_from, search_until = current_block_ + offset_to;


  double min_dist = 99999999;
  auto closest = current_block_;
  for (; search_from < search_until; ++search_from) {
    const auto d = robot_pos_.p.dist(search_from->target_);
    if (d < min_dist) {
      min_dist = d;
      closest = search_from;
    }
    if (min_dist > d) {
      break;
    }
  }


  return closest->target_;
}

std::optional<double> planning::PlanExecutor::get_line_follow_error() const {
  // calculates two points on the line for the line-follower algorithm
  const auto [intersect, p1, p2] = get_line_follow_intersect(0.15, 0.3);

#ifdef DEBUG_LINE_FOLLOW
  logging::logger.log("intersect_p0_x", robot_pos_.p.x_);
  logging::logger.log("intersect_p0_y", robot_pos_.p.y_);
  logging::logger.log("intersect_p1_x", p1.x_);
  logging::logger.log("intersect_p1_y", p1.y_);
  logging::logger.log("intersect_p2_x", p2.x_);
  logging::logger.log("intersect_p2_y", p2.y_);
  logging::logger.log("is_intersect", intersect.has_value());


  if (intersect) {
    logging::logger.log("intersect_x", intersect->x_);
    logging::logger.log("intersect_y", intersect->y_);
    logging::logger.log("to_is", robot_pos_.p.angle_to(intersect.value()));
  } else {
    logging::logger.log("intersect_x", robot_pos_.p.x_);
    logging::logger.log("intersect_y", robot_pos_.p.y_);
  }
#endif

  if (intersect) {
    // if intersect exists, calculate radius from robot position to the intersect
    const auto& is = intersect.value();
    const auto [cent, r] =
        geometry::circe_point_point_angle(robot_pos_.p, is, utils::normalize_angle(robot_pos_.theta));
    return utils::sign(robot_pos_.p.angle_to(is) - utils::normalize_angle(robot_pos_.theta)) * r;
  } else {
    return std::nullopt;
  }
}

std::tuple<std::optional<utils::Point>, utils::Point, utils::Point>
planning::PlanExecutor::get_line_follow_intersect(double dist, double len) const {
  // calculates two points on the line for the line-follower algorithm
  const auto [p1, p2] = geometry::segment_distance_angle(robot_pos_.p, robot_pos_.theta, dist, len);
  const std::optional intersect = plan_.find_intersect(p1, p2);  // get the intersect
  return std::tuple(intersect, p1, p2);
}

void planning::PlanExecutor::execute_plan() {
  using namespace std::chrono_literals;

  const auto start_time = utils::now();

  current_block_ = plan_.get_blocks().begin();

  robot_pos_ = motion_->get_pos();

  metrics_init();
  auto dist_condition = [this]() {
    // std::cout << this->robot_pos_.p.dist(this->plan_.get_blocks().back().target_) << '\n';
    if ((this->plan_.get_blocks().end() - this->current_block_) > 100) {
      return true;
    }

    if (this->robot_pos_.p.dist(this->plan_.get_blocks().back().target_) < 0.03) {
      return false;
    }
    return true;
  };

  LoopRate rate(0.05);
  while (current_block_ != plan_.get_blocks().end()) {
    path_loop_tick();
    metrics_tick();
    rate.sleep();
#ifdef LOG_SET_VALUES
    logging::logger.log("path.v", current_block_->velocity_);
    logging::logger.log("path.omega",
                        SimpleMotion::radius_to_angular(current_block_->velocity_, current_block_->radius_));
    logging::logger.log("path.x", current_block_->target_.x_);
    logging::logger.log("path.y", current_block_->target_.y_);
#endif
    while (current_block_ != plan_.get_blocks().end() && utils::elapsed(start_time, current_block_->elapsed_)) {
      ++current_block_;
    }
  }
  motion_->stop();
  while (current_block_ != plan_.get_blocks().end()) {
#ifdef LOG_SET_VALUES
    logging::logger.log("path.v", current_block_->velocity_);
    logging::logger.log("path.omega",
                        SimpleMotion::radius_to_angular(current_block_->velocity_, current_block_->radius_));
    logging::logger.log("path.x", current_block_->target_.x_);
    logging::logger.log("path.y", current_block_->target_.y_);
#endif
    ++current_block_;
  }
  motion_->stop();
  std::this_thread::sleep_for(100ms);
  robot_pos_ = motion_->get_pos();
  metrics_end();
}


void planning::PlanExecutor::execute_turn_plan() {
  using namespace std::chrono_literals;
  using std::this_thread::sleep_for;

  auto current = turn_plan_.get_blocks().begin();
  auto start_time = utils::now();

  const auto start_pos = motion_->get_pos();
  LoopRate rate(0.05);

  while (current != turn_plan_.get_blocks().end() - 1) {
    motion_->turn(current->omega_);
    rate.sleep();

#ifdef LOG_SET_VALUES
    logging::logger.log("path.v", 0);
    logging::logger.log("path.omega", current->omega_);
    logging::logger.log("path.x", start_pos.p.x_);
    logging::logger.log("path.y", start_pos.p.y_);
#endif
    while (current != turn_plan_.get_blocks().end() && utils::elapsed(start_time, current->elapsed_)) {
      ++current;
    }
  }

  auto get_err = [&]() {
    double target = turn_plan_.get_blocks().back().target_;


    return abs(robot_pos_.theta - (start_pos.theta + target));
  };
  robot_pos_ = motion_->get_pos();
  const double err_start = get_err();
  double err_now = get_err();
  double spd = ((turn_plan_.get_blocks().end() - 2)->omega_);
  while (err_now >= err_start) {
    motion_->turn(spd);
    rate.sleep();
    robot_pos_ = motion_->get_pos();
    err_now = get_err();
    std::cout << "e: " << err_now << '\n';
    /* code */
  }
  motion_->stop();
}


void planning::PlanExecutor::metrics_init() {
  metrics_min_dist_.clear();
  metrics_dist_travelled_.clear();
  last_metrics_pos_ = robot_pos_.p;
}


void planning::PlanExecutor::metrics_tick() {
  double dist_from_path = robot_pos_.p.dist(current_block_->target_);
  double dist_travelled = robot_pos_.p.dist(last_metrics_pos_);
  last_metrics_pos_ = robot_pos_.p;

  metrics_min_dist_.push_back(dist_from_path);
  metrics_dist_travelled_.push_back(dist_travelled);
}


void planning::PlanExecutor::metrics_end() {
  const auto& target = plan_.get_blocks().back().target_;
  double err = robot_pos_.p.dist(target);

  double target_angle = (plan_.get_blocks().end() - 2)->target_.angle_to(target);

  double err_sum = std::accumulate(metrics_min_dist_.begin(), metrics_min_dist_.end(), 0.0);
  double dist = std::accumulate(metrics_dist_travelled_.begin(), metrics_dist_travelled_.end(), 0.0);

  std::stringstream ss;

  ss << "Iteration: " << metrics_cnt_ << '\n'
     << "\tTarget position was: { " << target.x_ << ", " << target.y_ << ", " << utils::rad2deg(target_angle) << "}\n"
     << "\tFinal  position was: { " << robot_pos_.p.x_ << ", " << robot_pos_.p.y_ << ", "
     << utils::rad2deg(utils::normalize_angle(robot_pos_.theta)) << "}\n"
     << "\tThe error is: " << err * 100 << "cm\n";
  ss << "\n\tThe sum of errors is: " << err_sum << "\n\tover distance: " << dist
     << "\n\tnormalized over path: " << err_sum / dist
     << "\n\twith average: " << err_sum / metrics_min_dist_.size() * 100 << "cm\n"
     << '\n';
  ss << "________________________________________________________________________________________________\n";


  std::cout << ss.str();
#ifdef LOG_METRICS
  std::ofstream(logging::log_path("metrics.summary_" + std::to_string(metrics_cnt_)),
                std::ios_base::app | std::ios_base::out)
      << ss.str();
  logging::log_range("metrics.path_dist_" + std::to_string(metrics_cnt_), metrics_min_dist_.begin(),
                     metrics_min_dist_.end());
  logging::log_range("metrics.dist_trav_" + std::to_string(metrics_cnt_), metrics_dist_travelled_.begin(),
                     metrics_dist_travelled_.end());
#endif
  ++metrics_cnt_;
}
