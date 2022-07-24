#include "RobotControl/SimpleMotion.h"

#include <cmath>
#include <iostream>

void SimpleMotion::init() {
  if (!servo_l_->init()) {
    throw std::runtime_error("Failed to init left servo");
  }
  if (!servo_r_->init()) {
    throw std::runtime_error("Failed to init right servo");
  }
  if (!servo_l_->set_enabled(true)) {
    throw std::runtime_error("Failed to enable left servo");
  }
  if (!servo_r_->set_enabled(true)) {
    throw std::runtime_error("Failed to enable right servo");
  }
  pos_.init();
}

void SimpleMotion::move_forward(double spd) {
  double motor_spd = utils::rad2deg(spd / wheel_radius_);
  servo_l_->set_speed(motor_spd);
  servo_r_->set_speed(motor_spd);
}

void SimpleMotion::tick(double dt) {
  if (!servo_l_->tick()) {
    ++err_cnt_[0];
  } else {
    err_cnt_[0] = 0;
  }

  if (!servo_r_->tick()) {
    ++err_cnt_[1];
  } else {
    err_cnt_[1] = 0;
  }

  pos_.tick(dt);
  log_all();

  constexpr int err_th = 10;
  using re = std::runtime_error;
  if (err_cnt_[0] > err_th) {
    throw re("Left servo too many tick() failures");
  }
  if (err_cnt_[1] > err_th) {
    throw re("Right servo too many tick() failures");
  }
  if (servo_l_->has_error()) {
    throw re("Left servo has error");
  }
  if (servo_r_->has_error()) {
    throw re("Right servo has error");
  }
}

void SimpleMotion::turn(double spd) {
  double lin_spd = spd * wheel_distance_ / 2;
  double rot_spd = utils::rad2deg(lin_spd / wheel_radius_);
  servo_l_->set_speed(-rot_spd);
  servo_r_->set_speed(rot_spd);
}

void SimpleMotion::move_arc(double spd, double r) {
  if (abs(r) < 0.1 || abs(r) > 10 || isnan(r) || isinf(r)) {
    move_forward(spd);
    return;
  }

  double lin_spd_left = spd / r * (r - wheel_distance_ / 2);
  double lin_spd_right = spd / r * (r + wheel_distance_ / 2);

  double rot_spd_left = utils::rad2deg(lin_spd_left / wheel_radius_);
  double rot_spd_right = utils::rad2deg(lin_spd_right / wheel_radius_);

  servo_l_->set_speed(rot_spd_left);
  servo_r_->set_speed(rot_spd_right);
}

void SimpleMotion::move_angle(double spd, double rads) {
  constexpr double sampleT = 0.1;

  const double vl = spd - 1 / (2 * sampleT) * wheel_distance_ * rads;
  const double vr = vl + 1 / sampleT * wheel_distance_ * rads;

  const double ang_l = vl / wheel_radius_, ang_r = vr / wheel_radius_;

  servo_l_->set_speed(utils::rad2deg(ang_l));
  servo_r_->set_speed(utils::rad2deg(ang_r));
}

void SimpleMotion::move_angular_velocity(double spd, double omega) {
  const double vl = spd - 0.5 * omega * wheel_distance_, vr = spd + 0.5 * omega * wheel_distance_;
  const double om_l = vl / wheel_radius_, om_r = vr / wheel_radius_;

  servo_l_->set_speed(utils::rad2deg(om_l));
  servo_r_->set_speed(utils::rad2deg(om_r));
}

void SimpleMotion::stop() {
  servo_l_->set_speed(0);
  servo_r_->set_speed(0);
}

void SimpleMotion::log_all() {
  using std::to_string;
  const auto p = get_pos();


#ifdef LOG_POSITION
  logging::logger.log("pos.x", p.p.x_);
  logging::logger.log("pos.y", p.p.y_);
  logging::logger.log("pos.theta", p.theta);
  logging::logger.log("pos.v", p.v);
  logging::logger.log("pos.omega", p.omega);
  logging::logger.log("pos.gyro", p.gyro);
  logging::logger.log("pos.Lset", utils::deg2rad(servo_l_->get_speed()));
  logging::logger.log("pos.Lget", pos_.get_enc_l().get_ang_speed());
  logging::logger.log("pos.Rset", utils::deg2rad(servo_r_->get_speed()));
  logging::logger.log("pos.Rget", pos_.get_enc_r().get_ang_speed());
#endif
}
