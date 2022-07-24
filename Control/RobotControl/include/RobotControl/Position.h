#pragma once

#include "Encoder.h"
#include "VirtualServo.h"
#include <memory>
#include <cstdio>
#include <string>
#include "Utility.h"
#include "gyro.h"

/**
 * Tracks the position of the robot in world coordinates.
 */
class Position {
public:
  /**
   * Robot world coordinates in meters and radian.
   */
  struct Pos {
    utils::Point p;
    double theta{ 0.0 };
    double v = 0;
    double omega = 0;
    double gyro = 0;

    std::string to_string() const {
      using std::to_string;
      std::string ret = "x: " + to_string(p.x_) + ", y: " + to_string(p.y_) + ", theta: " + to_string(theta);
      return ret;
    }
  };

  Position(std::shared_ptr<Servo> left, std::shared_ptr<Servo> right, double wheel_radius, double wheel_distance)
    : servo_l_(left)
    , servo_r_(right)
    , wheel_radius_{ wheel_radius }
    , wheel_distance_{ wheel_distance }
    , enc_l_{ wheel_radius_ }
    , enc_r_{ wheel_radius_ } {
  }
  void init();

  void tick(double dt);

  const Pos get_pos() const {
    return pos_;
  }

  const Encoder& get_enc_l() const {
    return enc_l_;
  }
  const Encoder& get_enc_r() const {
    return enc_r_;
  }

  void add_gyro(Gyro* ptr) {
    gyro_ = ptr;
  }

private:
  double gyro_out() const;
  struct odometery_inc_t {
    double dx, dy, dtheta, v;
  };
  odometery_inc_t wheel_odometry(double dl, double dr, double dt) const;
  odometery_inc_t gyro_odometry(double dl, double dr, double dt) const;
  Pos pos_;
  const double wheel_radius_, wheel_distance_;
  Encoder enc_l_, enc_r_;
  Gyro* gyro_{ nullptr };
  std::shared_ptr<const Servo> servo_l_, servo_r_;
};
