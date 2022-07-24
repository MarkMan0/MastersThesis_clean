#pragma once

#define LOG_SPEED_LIMIT
// #define DEBUG_SPEED_CONTROL
// #define DEBUG_SIGNED_DIST_FROM_PATH
// #define DEBUG_ANGULAR_CONTROL
// #define DEBUG_LINE_FOLLOW
// #define DEBUG_TANGENT_CONTROL
// #define DEBUG_FUZZY_CONTROL
// #define DEBUG_FUZZY_DUAL_SENSOR
#define LOG_SET_VALUES
// #define LOG_METRICS
// #define LOG_GYRO
#define LOG_POSITION
#define DEBUG_PATH_LOOP

namespace config {

  inline constexpr double wheel_dist = 0.330503,  //!< distance between wheels
      wheel_radius = 0.08,                        //!< radius of wheels
      max_accel = 0.1,                            //!< maximum linear acceleration of the wheels
      max_centrifugal = 5,                        //!< maximum centrifugal force while turning
      encoder_of_threshold = 200,                 //!< encoder overflow threshold
      robot_weight = 1,                           //!< weight of the robot in kilograms
      min_velocity = 0.05,                        //!< minimum velocity for planning
      min_angular_velocity = 0.05,                //!< minimum angular velocity while turning
      max_angular_acc = 0.2;                      //!< maximum angular acceleration while turning

  inline constexpr double max_velocity = 100.0 / 360.0 * 2 * 3.1415 * wheel_radius;



}  // namespace config
