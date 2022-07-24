#pragma once

#include "SerialPortWrapper/SerialPortWrapper.h"
#include <string>
#include <array>
#include <cstdint>
#include <algorithm>
#include <atomic>

class Gyro {
private:
  const std::string& port_no_;
  std::atomic_int last_reading_{};
  SerialPortWrapper port_;
  std::array<char, 30> buff_;

public:
  Gyro(const std::string& port) : port_no_{ port } {
  }

  void connect() {
    int i = 7;
    port_.begin(i, 115200);
  }

  int get_last_reading() const {
    return last_reading_;
  }

  void tick() {
    port_.flush();
    port_.put_char('x');
    bool finished = false;
    int cnt = 0;
    std::fill(buff_.begin(), buff_.end(), 0);

    while (!finished) {
      char c = port_.get_char();
      if (c == -1 || c == 'x') {
        continue;
      }
      buff_[cnt] = c;
      finished = c == '\n';
      ++cnt;
    }
    last_reading_ = std::stoi(std::string(buff_.data()));
  }
};
