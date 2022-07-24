#include "RobotControl/Position.h"
#include <stdexcept>
#include <cmath>
#include <iostream>
#include "RobotControl/config.h"
#include "RobotControl/Logger.h"

static constexpr double gyro_th = 0;

void Position::init() {
  enc_l_.init(servo_l_->get_encoder());
  enc_r_.init(servo_r_->get_encoder());

  pos_.p = utils::Point();
  pos_.theta = 0;
}


void Position::tick(double dt) {
  double dl = enc_l_.tick(servo_l_->get_encoder(), dt);
  double dr = enc_r_.tick(servo_r_->get_encoder(), dt);


  Position::odometery_inc_t inc{};

  if (gyro_ == nullptr) {
    inc = wheel_odometry(dl, dr, dt);
    pos_.omega = inc.dtheta / dt;
  } else {
    gyro_->tick();
    inc = gyro_odometry(dl, dr, dt);

    double g = gyro_->get_last_reading() / 131.0;
    if (std::abs(g) > gyro_th) {
      // reject small values
      pos_.gyro = utils::deg2rad(gyro_->get_last_reading() / 131.0);
    } else {
      pos_.gyro = 0;
    }
    pos_.omega = inc.dtheta / dt;
  }

  pos_.p.x_ += inc.dx;
  pos_.p.y_ += inc.dy;
  pos_.v = inc.v;
  pos_.theta += pos_.omega * dt;
}


double Position::gyro_out() const {
  return utils::deg2rad(gyro_->get_last_reading() / 131.0);
}


Position::odometery_inc_t Position::wheel_odometry(double dl, double dr, double dt) const {
  double d_theta = (dr - dl) / wheel_distance_;
  double dpos = (dl + dr) / 2;

  odometery_inc_t ret{};

  ret.v = dpos / dt;
  if (std::abs(d_theta) < utils::deg2rad(2)) {
    ret.dx = dpos * cos(pos_.theta);
    ret.dy = dpos * sin(pos_.theta);
    ret.dtheta = d_theta;
  } else {
    const double theta_prev = pos_.theta;
    ret.dtheta = d_theta;

    ret.dx = dpos / d_theta * (std::sin(theta_prev + d_theta) - std::sin(theta_prev));
    ret.dy = dpos / d_theta * (std::cos(theta_prev) - std::cos(theta_prev + d_theta));
  }

  return ret;
}

Position::odometery_inc_t Position::gyro_odometry(double dl, double dr, double dt) const {
  double d_theta_wheel = (dr - dl) / wheel_distance_;
  double dpos = (dl + dr) / 2;

  double d_theta_gyro = utils::deg2rad(gyro_->get_last_reading() / 131.0) * dt;

  double d_theta = 0;

  if (std::abs(d_theta_gyro) < gyro_th) {
    d_theta = d_theta_wheel;
  } else {
    constexpr double comp = 1;
    d_theta = comp * d_theta_wheel + (1 - comp) * d_theta_gyro;
  }

  odometery_inc_t ret{};

  ret.v = dpos / dt;
  if (std::abs(d_theta) < utils::deg2rad(2)) {
    ret.dx = dpos * cos(pos_.theta);
    ret.dy = dpos * sin(pos_.theta);
    ret.dtheta = d_theta;
  } else {
    const double theta_prev = pos_.theta;
    ret.dtheta = d_theta;

    ret.dx = dpos / d_theta * (std::sin(theta_prev + d_theta) - std::sin(theta_prev));
    ret.dy = dpos / d_theta * (std::cos(theta_prev) - std::cos(theta_prev + d_theta));
  }
  return ret;
}
