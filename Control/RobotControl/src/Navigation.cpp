#include "RobotControl/Navigation.h"
#include <thread>
#include <chrono>
#include <iostream>
#include <string>

#include "RobotControl/MotionQueue.h"
#include "RobotControl/Utility.h"

void Navigation::init_controllers() {
  turn_controller_.kp_ = 1.1;
  turn_controller_.ki_ = 0.1;
  turn_controller_.kd_ = 0.1;

  fwd_controller_.kp_ = 2.5;
  fwd_controller_.ki_ = 0.00;
  fwd_controller_.kd_ = 0.0;
  fwd_controller_.upper_ = 0.5;
  fwd_controller_.lower_ = -0.5;

  arc_controller_.kp_ = 2.51;
  arc_controller_.ki_ = 0.0;
  arc_controller_.kd_ = 1.3;
  arc_controller_.lower_ = -5;
  arc_controller_.upper_ = 5;
}


void Navigation::turn_to(double rad) {
  double e = rad - motion_.get_pos().theta;
  turn_controller_.reset();
  logging::logger.log("turn.w", rad);
  while (abs(e) > 0.3) {
    logging::logger.log("turn.e", e);
    double u = turn_controller_.tick(e);
    logging::logger.log("turn.u", u);
    motion_.turn(u);
    std::this_thread::sleep_for(std::chrono::duration<double>(0.1));
    e = rad - motion_.get_pos().theta;
  }

  motion_.stop();
}


void Navigation::move_to(const utils::Point& target) {
  logging::logger.log("target.x", target.x_);
  logging::logger.log("target.y", target.y_);


  // distance to target
  auto get_dist = [this, target]() { return target.dist(motion_.get_pos().p); };

  // angle to target
  auto get_theta_to_t = [this, target]() {
    const auto pos = motion_.get_pos();
    return std::atan2(target.y_ - pos.p.y_, target.x_ - pos.p.x_);
  };

  double target_angle = utils::closest_angle(motion_.get_pos().theta, get_theta_to_t());
  std::cout << "rob: " << motion_.get_pos().theta << " Point: " << get_theta_to_t() << "  Target: " << target_angle
            << "\n";

  if (abs(target_angle - motion_.get_pos().theta) > utils::PI / 2) {
    turn_to(target_angle);
  }
  fwd_controller_.reset();
  arc_controller_.reset();

  while (get_dist() > 0.1) {
    const auto pos = motion_.get_pos();

    const double target_angle = utils::closest_angle(pos.theta, get_theta_to_t());
    const double theta_e = target_angle - pos.theta;

    const double d = get_dist() + (mq_ ? mq_->get_dist_to_next_stop(target) : 0);

    double spd = fwd_controller_.tick(d);

    if (get_dist() < 0.4) {
      // close to intermediate target - slow down a bit
      // spd = std::min(spd, 0.3);
    }

    const double arc_out = arc_controller_.tick(theta_e);
    double radius = 1.0 / arc_out;


    if (abs(theta_e) > utils::PI / 2) {
      // reverse
      spd *= -1;
      radius = 0;
    }
    if (isnan(radius) || isinf(radius)) {
      radius = 0;
    }

    limit_spd_and_radius(spd, radius);

    motion_.move_arc(spd, radius);


    logging::logger.log("p2p.target_angle", target_angle);
    logging::logger.log("p2p.theta_e", theta_e);
    logging::logger.log("p2p.distance", d);
    logging::logger.log("p2p.speed", spd);
    logging::logger.log("p2p.arcout", arc_out);
    logging::logger.log("p2p.radius", radius);


    std::this_thread::sleep_for(std::chrono::milliseconds(100));
  }
}

void Navigation::stop() {
  motion_.stop();
}


void Navigation::limit_spd_and_radius(double& spd, double& radius) const {
  /*
   * 1: for small radii, limit speed
   * 2: for large radii, set to 0 so motion is straight
   */

  if (abs(radius) < 0.3) {
    spd = utils::sign(spd) * std::max(abs(spd), 0.2);
  } else if (abs(radius) > 200) {
    radius = 0;
  }
  if (abs(radius) < 0.15) {
    spd = utils::sign(spd) * std::max(abs(spd), 0.1);
  }
}
