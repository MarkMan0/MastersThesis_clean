#pragma once
#include "Servo.h"
#include "SerialPortWrapper/SerialPortWrapper.h"

class VirtualServo : public Servo {
public:
  VirtualServo(int id, const std::shared_ptr<SerialPortWrapper>& port) : Servo(id), serial_(port) {
  }

  [[nodiscard]] bool init() override;
  [[nodiscard]] bool set_enabled(bool) override;
  [[nodiscard]] bool tick() override;

  void set_speed(double spd) override {
    speed_ = spd;
    speed_ = std::max(speed_, -720.0);
    speed_ = std::min(speed_, 720.0);
  }
  void deinit() override;

  ~VirtualServo() {
    deinit();
  }

private:
  [[nodiscard]] bool publish_speed();
  [[nodiscard]] bool query_encoder();
  const std::shared_ptr<SerialPortWrapper> serial_;
};
