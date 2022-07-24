#pragma once
/**
 * @file Path.h
 * @brief Path tools to use in planner
 *
 */

#include "RobotControl/Utility.h"
#include "RobotControl/geometry.h"
#include <array>
#include <vector>
#include <optional>
#include <memory>
namespace planning {


  class Path {
  public:
    virtual std::vector<utils::Point> create_points() const = 0;
    virtual std::optional<utils::Point> line_intersect(const utils::Point& p0, const utils::Point& p1) const = 0;

    Path() = default;
    Path(const utils::Point& from, const utils::Point& to, double angle_in, double angle_out, double speed)
      : from_{ from }, to_{ to }, angle_entry_{ angle_in }, angle_exit_{ angle_out }, speed_{ speed } {
    }

    virtual std::unique_ptr<Path> clone() const = 0;

    double get_speed() const {
      return speed_;
    }
    void set_speed(double spd) {
      speed_ = spd;
    }

    virtual ~Path() = default;


    const utils::Point from_, to_;
    const double angle_entry_{}, angle_exit_{};

  private:
    double speed_{};
  };


  /**
   * @brief Stores data about a path
   *
   */
  class BezierPath : public Path {
  public:
    BezierPath(const BezierPath&) = default;

    BezierPath(const std::array<utils::Point, 4> points, double angle_in, double angle_out, double speed)
      : Path(points[0], points.back(), angle_in, angle_out, speed), points_{ points } {
    }

    const std::array<utils::Point, 4>& get_points() const {
      return points_;
    }

    std::vector<utils::Point> create_points() const override {
      return Bezier::create_bezier(points_);
    }

    std::optional<utils::Point> line_intersect(const utils::Point& p0, const utils::Point& p1) const override {
      return Bezier::bezier_line_intersect(points_, p0, p1);
    }

    std::unique_ptr<Path> clone() const override {
      return std::make_unique<BezierPath>(*this);
    }

  private:
    const std::array<utils::Point, 4> points_;
  };


  class LinePath : public Path {
  public:
    LinePath(const LinePath&) = default;
    LinePath(const std::array<utils::Point, 2> points, double angle_in, double angle_out, double speed)
      : Path(points[0], points.back(), angle_in, angle_out, speed), points_{ points } {
    }
    std::vector<utils::Point> create_points() const override {
      std::vector<utils::Point> ret;
      for (double t = 0; t <= 1; t += 0.01) {
        ret.push_back((1 - t) * points_[0] + t * points_[1]);
      }
      return ret;
    }
    std::optional<utils::Point> line_intersect(const utils::Point& p0, const utils::Point& p1) const override {
      return geometry::segment_segment_intersect(points_[0], points_[1], p0, p1);
    }

    std::unique_ptr<Path> clone() const override {
      return std::make_unique<LinePath>(*this);
    }

    const std::array<utils::Point, 2> points_;
  };


  class CircularPath : public Path {
  public:
    CircularPath(const CircularPath&) = default;

    CircularPath(const std::array<utils::Point, 2> points, double angle_in, double angle_out, double speed,
                 const utils::Point& center, double radius)
      : Path(points[0], points.back(), angle_in, angle_out, speed)
      , points_{ points }
      , center_{ center }
      , radius_{ radius } {
    }

    std::vector<utils::Point> create_points() const override;

    std::optional<utils::Point> line_intersect(const utils::Point& p0, const utils::Point& p1) const override;

    std::unique_ptr<Path> clone() const override {
      return std::make_unique<CircularPath>(*this);
    }

    std::array<utils::Point, 2> points_;
    const utils::Point center_;
    const double radius_{};

    static CircularPath point_tangent_point(const utils::Point& from, const utils::Point& to, double angle_in,
                                            double speed);
  };

  /**
   * @brief Used for creating a valid Path object
   *
   */
  class PathFactory {
  public:
    /**
     * @brief Start of path
     *
     * @param p Start point
     * @return PathFactory&
     */
    PathFactory& from(const utils::Point& p) {
      from_ = p;
      return *this;
    }
    /**
     * @brief Target of path
     *
     * @param p target point
     * @return PathFactory&
     */
    PathFactory& to(const utils::Point& p) {
      to_ = p;
      return *this;
    }
    /**
     * @brief Entry angle, required, should be the current angle of the robot
     *
     * @param d radians
     * @return PathFactory&
     */
    PathFactory& angle_entry(double d) {
      angle_entry_ = d;
      return *this;
    }
    /**
     * @brief OPTIONAL, if not set, exit angle is entry.angle_to(exit)
     *
     * @param d radians
     * @return PathFactory&
     */
    PathFactory& angle_exit(double d) {
      angle_exit_ = d;
      return *this;
    }
    /**
     * @brief Nominal speed on path
     *
     * @param d m/s
     * @return PathFactory&
     */
    PathFactory& speed(double d) {
      speed_ = d;
      return *this;
    }
    /**
     * @brief The weight from which the two internal points are calculated
     *
     * @details The path is defined using a Bezier curve with control points P0...P3.
     * P0 is the entry, P3 is the exit
     * P1 is calculated: P0 + @p d * [cos(angle_entry), sin(angle_entry)]
     * P2 is calculated: P3 - @p d * [cos(angle_entry), sin(angle_entry)]
     *
     * @param d weight [meters]
     * @return PathFactory&
     */
    PathFactory& weight(double d) {
      weight_ = d;
      return *this;
    }

    /**
     * @brief Creates the path, if everythin is set correctly
     *
     * @return Path
     */
    BezierPath createBezier();

    LinePath createLine();

    CircularPath createCircular();

    class JointPath createBezierLine();

  private:
    std::optional<utils::Point> from_, to_;
    std::optional<double> angle_entry_, angle_exit_, speed_, weight_;

    PathFactory& next();
    const std::logic_error insufficient_data_err_{ "Insufficient data, can't create path" };
  };


  class JointPath : public Path {
  public:
    std::vector<utils::Point> create_points() const override;
    std::optional<utils::Point> line_intersect(const utils::Point& p0, const utils::Point& p1) const override;

    std::unique_ptr<Path> clone() const override {
      return std::make_unique<JointPath>(*this);
    }

    template <class T>
    void add(const T& p) {
      paths_.push_back(std::make_unique<T>(p));
    }

    void set_start(const utils::Point& p, double d);

    void add_waypoint(const utils::Point&);
    void add_waypoint(const utils::Point&, double);

    JointPath(const JointPath& other) : Path(other) {
      paths_.reserve(other.paths_.size());
      for (const auto& p : other.paths_) {
        paths_.push_back(std::move(p->clone()));
      }
    }
    JointPath() = default;
    ~JointPath() {
    }

  private:
    std::vector<std::unique_ptr<Path>> paths_;
    std::optional<utils::Point> start_;
    PathFactory factory_;
    double start_angle_{ 0 };
  };

}  // namespace planning
