#pragma once

#include <cstdint>
#include <mutex>

/**
 * Drivebot servo control interface.
 */
class Servo {
public:
  Servo(int id) : id_(id){};

  [[nodiscard]] virtual bool init() = 0;
  [[nodiscard]] virtual bool set_enabled(bool) = 0;
  virtual void deinit() = 0;
  virtual bool has_error() const {
    return has_error_;
  };

  /**
   * Set the servo speed.
   *
   * \param spd Speed in deg/s
   */
  virtual void set_speed(double spd) {
    speed_ = spd;
  }
  /**
   * Get the encoder value.
   *
   * \return Encoder value in degrees
   */
  virtual double get_encoder() const {
    return encoder_;
  }
  /**
   * Return the last set speed.
   *
   * \return Last set speed in deg/s
   */
  virtual double get_speed() const {
    return speed_;
  }
  virtual bool tick() = 0;

  virtual ~Servo(){};

protected:
  using lock_t = std::lock_guard<std::mutex>;
  static std::mutex mtx;  // same mutex for all servos

  const int id_;
  bool has_error_{ false };
  double speed_{ 0 };
  double encoder_{ 0 };
};
