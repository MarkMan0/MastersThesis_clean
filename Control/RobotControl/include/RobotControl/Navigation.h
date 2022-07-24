#pragma once

#include "Controller.h"
#include "SimpleMotion.h"
#include "Logger.h"
#include "Utility.h"

class MotionQueue;  // circular dependency

class Navigation {
public:
  Navigation(SimpleMotion& sm) : motion_(sm){};
  void init_controllers();
  void move_forward(double distance);
  void move_to(const utils::Point&);
  void turn_to(double radian);
  void stop();

  void insert_queue_dependency(const MotionQueue* ptr) {
    mq_ = ptr;
  }

private:
  SimpleMotion& motion_;
  PIDController fwd_controller_;
  PIDController turn_controller_;
  PIDController arc_controller_;

  const MotionQueue* mq_{ nullptr };

  void limit_spd_and_radius(double& spd, double& radius) const;
};
