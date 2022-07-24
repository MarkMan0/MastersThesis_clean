#pragma once
#include "Utility.h"
#include <vector>
#include <optional>
#include <memory>
#include "RobotControl/Controller.h"
#include "RobotControl/Path.h"
#include "RobotControl/Position.h"
#include "RobotControl/Plan.h"

#include "fl/Headers.h"

class SimpleMotion;

namespace planning {

  class PlanExecutor {
  public:
    PlanExecutor() {
      init_controllers();
    };
    PlanExecutor(SimpleMotion* sm) : motion_{ sm } {
      init_controllers();
    };

    /**
     * @brief Set the current Path
     *
     * @param p
     */
    template <class T>
    void set_path(const T& path) {
      std::unique_ptr<planning::Path> p_path = std::make_unique<T>(path);
      plan_ = PathPlan(std::move(p_path));
      plan_.apply_limits();
    }

    void set_turn_plan(const TurnPlan& plan) {
      turn_plan_ = plan;
      turn_plan_.apply_limit();
    }

    /**
     * @brief Execute the current plan, first need to apply_limit()
     *
     */
    void execute_plan();


    void execute_turn_plan();

  private:
    /**
     * @brief Path following implementation
     *
     */
    void path_loop_tick();

    /**
     * @brief Linear speed control of robot
     *
     * @return double the linear speed in this tick
     */
    double speed_control();

    /**
     * @brief Angular speed control to follow the path
     *
     * @return double The angular velocity of the robot
     */
    double path_dist_cont();

    /**
     * @brief Combines virtual line follow sensor and radius control to follow the line
     *
     * @return double radius to follow the path
     */
    double line_follow_control();

    /**
     * @brief Control heading based on tangent
     *
     * @return double omega
     */
    double tangent_control();

    /**
     * @brief Fuzzy controller for orientation
     *
     * @return angular velocity to stay on path
     */
    double fuzzy_control();

    /**
     * @brief Fuzzy control using two sensor on the front side of the robot
     *
     * @return angular velocity
     */
    double fuzzy_dual_sensor_control();

    /**
     * @brief Returns the closest point on the path
     *
     * @return utils::Point closest point to robot position
     */
    utils::Point get_closest_point() const;

    /**
     * @brief Signed distance of robot from path
     *
     * @return positive if path is on left side, negative if on right
     */
    double signed_distance_from_path() const;

    /**
     * @brief Get the error from the line following algorithm
     *
     * @return std::optional<double> if intersect exists, return radius of circle from current pos to intersect
     */
    std::optional<double> get_line_follow_error() const;

    /**
     * @brief Get the intersect point of the line follower along with the two points defining the line follower segment
     *
     * @param dist distance of sensor from robot
     * @param len length of sensor
     * @return tuple: [ optional intersect, righy point of line follower, left point of line follower]
     */
    std::tuple<std::optional<utils::Point>, utils::Point, utils::Point> get_line_follow_intersect(double dist,
                                                                                                  double len) const;

    void init_controllers();

    void metrics_init();
    void metrics_tick();
    void metrics_end();

    std::vector<double> metrics_min_dist_, metrics_dist_travelled_;
    int metrics_cnt_ = 0;
    utils::Point last_metrics_pos_;

    PIDController path_dist_controller_, speed_controller_, tangent_controller_;
    PathPlan plan_;
    TurnPlan turn_plan_;
    std::vector<PlanBlock>::const_iterator current_block_;
    SimpleMotion* motion_ = nullptr;
    Position::Pos robot_pos_;  //!< Cached position in tick()
    std::unique_ptr<fl::Engine> fuzzy_engine_, dual_sensor_fuzzy_;
    utils::OptValueHold<double> sensor_1_hold_, sensor_2_hold_, sensor_3_hold_;
  };

}  // namespace planning
