/**
 * @file Plan.h
 *
 */

#pragma once

#include "RobotControl/Utility.h"
#include <optional>
#include <vector>
#include "RobotControl/Path.h"
#include <memory>
#include <tuple>
namespace planning {

  /**
   * @brief Point on the path, for internal use
   *
   */
  struct PlanBlock {
    utils::Point target_;
    double velocity_ = 0;
    double duration_ = 0;
    double elapsed_ = 0;
    double distance_travelled_ = 0;
    double radius_ = 0;
  };

  struct TurnBlock {
    double target_ = 0;
    double omega_ = 0;
    double elapsed_ = 0;
    double duration_ = 0;
    double angle_turned_ = 0;
  };

  class PathPlan {
  private:
    std::vector<PlanBlock> blocks_;
    bool is_ready_ = false;
    std::unique_ptr<Path> path_;

  public:
    void apply_limits();
    PathPlan() = default;
    PathPlan(std::unique_ptr<Path>&& p) : path_(std::move(p)){};
    const std::vector<PlanBlock>& get_blocks() const {
      return blocks_;
    }

    /**
     * @brief Return the closest point on the path to @p p0
     * @return utils::Point
     */
    utils::Point get_closest(const utils::Point& p0) const;
    /**
     * @brief Returns the intersect between Path and line defined by @p p0 and @p p1
     *
     * @param p0
     * @param p1
     * @return std::optional<utils::Point>
     */
    std::optional<utils::Point> find_intersect(const utils::Point& p0, const utils::Point& p1) const;


  private:
    /**
     * @brief Creates the blocks for the plan from Path
     *
     */
    void init_blocks();

    void limit_speed();
    /**
     * @brief Limits the centrifugal force on the robot, by limiting it's linear speed
     *
     */
    void centrifugal_limit();
    template <class BlockIter>
    /**
     * @brief Limit the acceleration in forward mode
     * @details Same method works for both limiting acceleration and deceleration.
     * But need to parse the Path in forward or reverse order, respectively. That is why we are using Iterators in this
     * method
     * @tparam Iter
     */
    void acceleration_pass(BlockIter, BlockIter);
    /**
     * @brief A minimum speed is required for each block for the planning to work.
     *
     */
    void limit_min_speed();
    /**
     * @brief Calculate the time each block will take
     *
     */
    void calculate_time();

    std::tuple<double, double> wheel_speeds_radius(double v, double r) const;
    std::tuple<double, double> wheel_speeds_angular(double v, double omega) const;
    std::tuple<double, double> robot_speeds_from_wheels(double vl, double vr) const;
  };


  class TurnPlan {
  private:
    std::vector<TurnBlock> blocks_;

  public:
    const std::vector<TurnBlock>& get_blocks() const {
      return blocks_;
    }

    void apply_limit();

    double target_ = 0;
    double omega_ = 1;

  private:
    void init_blocks();

    template <class BlockIter>
    void acceleration_pass(BlockIter beging, BlockIter end);
    void limit_min_speed();
    void calculate_time();
  };


}  // namespace planning
