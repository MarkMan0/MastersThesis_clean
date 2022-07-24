#pragma once
#include "Utility.h"
#include "RobotControl/config.h"
/**
 * \brief Tracks the encoders of the servos
 *
 */
class Encoder {
public:
  Encoder(double radius) : wheel_radius_(radius){};

  /**
   * Inits the Encoder class.
   *
   * \param val the current position of the servo in degrees
   */
  void init(double val) {
    last_enc_ = val;
    position_ = 0.0;
  }

  /**
   * Return the linear movement done by the wheel. Handles over/underflow.
   *
   * \param measured the value of the encoder in degrees
   * \return the linear distance travelled since last tick()
   */
  double tick(double measured, double dt) {
    double diff = measured - last_enc_;
    last_enc_ = measured;

    if (diff < -config::encoder_of_threshold) {
      // overflow
      diff += 360;
    }
    if (diff > config::encoder_of_threshold) {
      // underflow
      diff -= 360;
    }
    diff = diff / 180.0 * utils::PI;
    auto dl = diff * wheel_radius_;
    speed_ = dl / dt;
    ang_speed_ = diff / dt;
    position_ += dl;
    return dl;
  }

  /**
   * \brief Linear speed in m/s.
   *
   */
  double get_speed() const {
    return speed_;
  }
  /**
   * Angular speed in rad/s.
   *
   */
  double get_ang_speed() const {
    return ang_speed_;
  }
  /**
   * Position in meters.
   *
   */
  double get_position() const {
    return position_;
  }

private:
  const double min_val_{ 0 };   /*!< Underflow value */
  const double max_val_{ 360 }; /*!< Overflow value */

  double last_enc_ = 0;
  double position_{ 0 }, /*!< The linear position of the servo in meters*/
      speed_{ 0 },       /*!< The linear speed of the servo in m/s */
      ang_speed_{ 0 };   /*!< The angular speed of the servo in rad/s */
  const double wheel_radius_ /*!< Wheel radius in meters*/;
};
