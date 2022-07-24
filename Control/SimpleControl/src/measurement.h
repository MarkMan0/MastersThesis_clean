#pragma once

#include "RobotControl/Plan.h"
#include "RobotControl/PlanExecutor.h"
#include <thread>
#include <chrono>
#include <iostream>

namespace measure {

  using namespace std::chrono_literals;
  using std::this_thread::sleep_for;

  inline void measure_square(planning::PlanExecutor& planner) {
    planning::TurnPlan plan;
    plan.target_ = utils::deg2rad(90);
    plan.omega_ = 2;
    plan.apply_limit();

    planning::PathFactory fact;
    double angle_entry = 0;
    constexpr double size = 1.0;
    for (int i = 0; i < 1; ++i) {
      fact.from({ 0, 0 }).angle_entry(angle_entry).speed(1).to({ size, 0 });

      planner.set_path(fact.createLine());
      planner.execute_plan();
      planner.set_turn_plan(plan);
      planner.execute_turn_plan();
      angle_entry += 90;


      fact.from({ size, 0 }).angle_entry(utils::deg2rad(90)).to({ size, size });
      planner.set_path(fact.createLine());
      planner.execute_plan();
      planner.set_turn_plan(plan);
      planner.execute_turn_plan();
      angle_entry += 90;

      fact.from({ size, size }).angle_entry(utils::deg2rad(180)).to({ 0, size });
      planner.set_path(fact.createLine());
      planner.execute_plan();
      planner.set_turn_plan(plan);
      planner.execute_turn_plan();
      angle_entry += 90;

      fact.from({ 0, size }).angle_entry(utils::deg2rad(270)).to({ 0, 0 });
      planner.set_path(fact.createLine());
      planner.execute_plan();
      planner.set_turn_plan(plan);
      planner.execute_turn_plan();
      angle_entry += 90;
    }
  }


  inline void measure_bezier_1(planning::PlanExecutor& planner) {
    planning::PathFactory fact;
    fact.angle_entry(0).from({ 0, 0 }).speed(1);
    fact.angle_exit(0).to({ 2, 2 });

    planner.set_path(fact.createBezier());
    planner.execute_plan();
  }


  inline void measure_circle(planning::PlanExecutor& planner) {
    planning::JointPath jp;
    jp.set_start({ 0, 0 }, 0);
    jp.set_speed(0.1);

    planning::PathFactory fact;
    fact.from({ 0, 0 }).angle_entry(0).speed(1).to({ 0, 1 });
    jp.add(fact.createCircular());
    jp.add(fact.to({ 0, 0 }).createCircular());

    planner.set_path(jp);
    planner.execute_plan();
  }


  inline void measure_bezier_2(planning::PlanExecutor& planner) {
    planning::JointPath jp;
    jp.set_start({ 0, 0 }, 0);
    jp.set_speed(1);

    jp.add_waypoint({ 3, 1.5 }, utils::deg2rad(90));
    jp.add_waypoint({ -2, 2 }, utils::deg2rad(180));
    jp.add_waypoint({ 0, 0 }, 0);

    planner.set_path(jp);
    planner.execute_plan();
  }

  inline void measure_bezier_3(planning::PlanExecutor& planner) {
    planning::PathFactory fact;
    fact.from({ 0, 0 }).angle_entry(0).speed(1).to({ -3, 2 }).angle_exit(0).weight(1);
    planner.set_path(fact.createBezierLine());
    planner.execute_plan();
  }


};  // namespace measure
