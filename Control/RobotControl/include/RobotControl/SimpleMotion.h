#pragma once

#include <memory>
#include "Servo.h"

#include "Position.h"
#include "Logger.h"
#include "config.h"
#include <vector>

/**
 * Implements open-loop control of robot.
 */
class SimpleMotion {
public:
  SimpleMotion(std::shared_ptr<Servo> left, std::shared_ptr<Servo> right)
    : servo_l_(left), servo_r_(right), pos_(left, right, wheel_radius_, wheel_distance_) {
  }

  void init();

  /**
   * Handles all things related to servos and position tracking.
   *
   * \details Calls tick on Servos, calls tick on position. Creates error messages if needed
   */
  void tick(double dt);


  const Position::Pos get_pos() const {
    return pos_.get_pos();
  }

  /**
   * Both motors with same speed.
   *
   * \param spd speed of robot in m/s
   */
  void move_forward(double spd);

  /**
   * Turn in place.
   *
   * \param spd Angular speed of turning in rad/s
   */
  void turn(double spd);

  /**
   * Movement along arc with given radius.
   *
   * \param spd linear speed of the robot in m/s
   * \param radius radius of movement in m
   */
  void move_arc(double spd, double radius);

  /**
   * Moves in such way, that in the next iteration, the robot angle increases by \param rads radians.
   *
   * \param spd linear speed of robot
   * \param rads angular speed
   */
  void move_angle(double spd, double rads);

  /**
   * @brief Moves the robot with linear speed @param spd and angular @param omega
   *
   * @param spd m/s
   * @param omega rads/s
   */
  void move_angular_velocity(double spd, double omega);

  /**
   * @brief Converts angular velocity to radius
   *
   * @param spd linear speed
   * @param omega angular velocity
   * @return double radius of movement
   */
  static double angular_to_radius(double spd, double omega) {
    if (omega == 0) {
      return 0;
    }
    return spd / omega;
  }

  /**
   * @brief Converts radius to angular velocity
   *
   * @param spd linear speed
   * @param radius radius of movement
   * @return double angular velocity of robot
   */
  static double radius_to_angular(double spd, double radius) {
    if (radius == 0) {
      return 0;
    }
    return spd / radius;
  }

  void stop();

  bool get_error() const {
    return error_flag_;
  }

private:
  bool error_flag_{ false };
  void log_all();
  int err_cnt_[2]{ 0 };
  const double wheel_radius_{ config::wheel_radius }, /*!< Wheel radius in meters */
      wheel_distance_{ config::wheel_dist };          /*!< Distance between wheels in meters */
  std::shared_ptr<Servo> servo_l_, servo_r_;

public:
  Position pos_;
};
