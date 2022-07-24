#include "RobotControl/VirtualServo.h"

#include <iostream>
#include <chrono>

#include <cmath>


enum Commands {
  CMD_SET_SPEED = 0x01,
  CMD_GET_POSITION = 0x02,
};


bool VirtualServo::init() {
  std::cout << "SERVO INIT\n";
  return true;
}

bool VirtualServo::set_enabled(bool e) {
  if (e) {
    std::cout << "ENABLING\n";
  } else {
    std::cout << "DISABLING\n";
  }

  return true;
}


bool VirtualServo::tick() {
  lock_t lock(mtx);

  return publish_speed() && query_encoder();
}

bool VirtualServo::publish_speed() {
  double spd = speed_;

  uint8_t buff[4]{ 0 };
  buff[0] = id_;
  buff[1] = CMD_SET_SPEED;
  spd = std::min(spd, 720.0);
  spd = std::max(spd, -720.0);

  // rescale speed to int16_t values and put into msg[1] msg[2]
  *reinterpret_cast<int16_t*>(buff + 2) = static_cast<int16_t>(spd / 720 * std::numeric_limits<int16_t>::max());

  serial_->write(buff, 4);

  buff[0] = 0;
  using namespace std::chrono_literals;
  const auto start = std::chrono::steady_clock::now();

  while (buff[0] == 0) {
    // read byte-by-byte
    serial_->read(buff, 1);
    if (std::chrono::steady_clock::now() - start > 500ms) {
      return false;  // timeout
    }
  }
  return true;
}

bool VirtualServo::query_encoder() {
  uint8_t buff[4]{ 0 };
  buff[0] = id_;
  buff[1] = CMD_GET_POSITION;
  serial_->flush();
  serial_->write(buff, 4);
  buff[0] = 0;

  using namespace std::chrono_literals;
  const auto start = std::chrono::steady_clock::now();
  while (buff[0] != CMD_GET_POSITION) {
    // read byte-by-byte
    serial_->read(buff, 3);
    if (std::chrono::steady_clock::now() - start > 500ms) {
      return false;  // timeout
    }
  }

  int16_t val = *reinterpret_cast<int16_t*>(buff + 1);
  encoder_ = val / 32767.0 * 180;

  return true;
}

void VirtualServo::deinit() {
  // for virtual servo just stop, no need to turn off/brake
  set_speed(0);
  static_cast<void>(tick());
}
