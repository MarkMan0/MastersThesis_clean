/**
 * @file geometry.h
 * @brief Tools to work with cubic Bezier curves
 *
 */

#pragma once
#include <vector>
#include "Utility.h"
#include <cassert>
#include "Logger.h"
#include <optional>

#include <Eigen/Dense>



namespace geometry {

  /**
   * @brief Returns endpoints of segment perpendicular to @p angle and @p distance away from @p origin
   *
   * @param origin 2D point
   * @param angle direction of segment from origin
   * @param distance distance between segment and origin
   * @param segment_len length of segment
   * @return std::tuple<utils::Point, utils::Point> = [left, right] End points of the segment
   */
  inline std::tuple<utils::Point, utils::Point> segment_distance_angle(const utils::Point& origin, const double angle,
                                                                       const double distance,
                                                                       const double segment_len) {
    const double triangle_side_len = std::sqrt(utils::sqr(distance) + utils::sqr(segment_len / 2.0));
    const double triangle_angle = std::acos(distance / triangle_side_len);
    const double angle_1 = angle + triangle_angle, angle_2 = angle - triangle_angle;

    const auto p1 = origin + triangle_side_len * utils::Point(std::cos(angle_1), std::sin(angle_1));
    const auto p2 = origin + triangle_side_len * utils::Point(std::cos(angle_2), std::sin(angle_2));

    return std::make_tuple(p1, p2);
  }

  /**
   * @brief Calculates  radius of circle with points @p p0 and @p p1 where tangential line at @p p0 is at @p phi radians
   *
   * @param p0 Point where tangential line is at @p angle
   * @param p1 Second point
   * @param phi angle of tangential line in radians
   * @return double  radius
   */
  inline std::tuple<utils::Point, double> circe_point_point_angle(const utils::Point p0, const utils::Point p1,
                                                                  const double phi) {
    const double x0 = p0.x_, y0 = p0.y_, x1 = p1.x_, y1 = p1.y_, theta = phi + utils::PI / 2;

    double r = 0;
    utils::Point cent(0, 0);

    if (std::abs(p0.angle_to(p1) - phi) < utils::deg2rad(3)) {
      // line through p0 at angle radians passes through p1 -> circle cant be fit
      return { utils::Point(0, 0), 0 };
    }

    if (abs(abs(utils::normalize_angle(theta)) - utils::PI / 2) > 0.001) {
      const double m = std::tan(theta);
      const double b_line = y0 - m * x0;
      const double b = -2 * x0 + 2 * x1 - 2 * m * y0 + 2 * m * y1;
      const double c = utils::sqr(x0) - utils::sqr(x1) - utils::sqr(y1) - utils::sqr(y0) + 2 * y0 * y1 +
                       2 * m * x0 * y0 - 2 * m * x0 * y1;
      const double xc = -c / b;
      cent.x_ = xc;
      cent.y_ = m * xc + b_line;
      r = std::sqrt((1 + utils::sqr(m)) * utils::sqr(x0 - xc));
    } else {
      // tangent is 0, special case
      const double xc = x0;
      const double yc = (utils::sqr(y0) - utils::sqr(y1) - utils::sqr(x1 - x0)) / (2 * (y0 - y1));
      cent.x_ = xc;
      cent.y_ = yc;
      r = std::abs(y0 - yc);
    }
    return { cent, r };
  }

  inline std::tuple<utils::Point, double> circle_point_point_point(const utils::Point& p0, const utils::Point& p1,
                                                                   const utils::Point& p2) {
    auto slope = [](const utils::Point& a, const utils::Point& b) {
      if (a.x_ != b.x_) {
        return (b.y_ - a.y_) / (b.x_ - a.x_);
      } else {
        return 0.0;
      }
    };

    if (std::abs(slope(p0, p1) - slope(p1, p2)) < 0.001) {
      return { { 0, 0 }, 0 };
    }

    Eigen::Matrix3d D, Dg, Df, Dc;
    using utils::sqr;

    // clang-format off
    D <<
        2 * p0.x_, 2 * p0.y_, 1,
        2 * p1.x_, 2 * p1.y_, 1,
        2 * p2.x_, 2 * p2.y_, 1;
    Dg <<
        (-1*sqr(p0.x_) - sqr(p0.y_)), 2 * p0.y_, 1,
        (-1*sqr(p1.x_) - sqr(p1.y_)), 2 * p1.y_, 1,
        (-1*sqr(p2.x_) - sqr(p2.y_)), 2 * p2.y_, 1;
    Df <<
        2 * p0.x_, (-1*sqr(p0.x_) - sqr(p0.y_)), 1,
        2 * p1.x_, (-1*sqr(p1.x_) - sqr(p1.y_)), 1,
        2 * p2.x_, (-1*sqr(p2.x_) - sqr(p2.y_)), 1;
    Dc <<
        2 * p0.x_, 2 * p0.y_, (-1*sqr(p0.x_) - sqr(p0.y_)),
        2 * p1.x_, 2 * p1.y_, (-1*sqr(p1.x_) - sqr(p1.y_)),
        2 * p2.x_, 2 * p2.y_, (-1*sqr(p2.x_) - sqr(p2.y_));
    // clang-format on

    const double det_d = D.determinant();

    const double g = Dg.determinant() / det_d;
    const double f = Df.determinant() / det_d;
    const double c = Dc.determinant() / det_d;

    return { { -g, -f }, std::sqrt(sqr(g) + sqr(f) - c) };
  }

  inline bool is_arc_cw(const utils::Point& center, const utils::Point& p0, double tangent) {
    const auto p0_1 = p0 + utils::Point::from_angle(tangent);
    return utils::sign((p0_1.x_ - p0.x_) * (center.y_ - p0.y_) - (p0_1.y_ - p0.y_) * (center.x_ - p0.x_)) < 0;
  }

  inline constexpr std::optional<utils::Point> segment_segment_intersect(const utils::Point& p0, const utils::Point& p1,
                                                                         const utils::Point& p2,
                                                                         const utils::Point& p3) {
    const double t = ((p0.x_ - p2.x_) * (p2.y_ - p3.y_) - (p0.y_ - p2.y_) * (p2.x_ - p3.x_)) /
                     ((p0.x_ - p1.x_) * (p2.y_ - p3.y_) - (p0.y_ - p1.y_) * (p2.x_ - p3.x_));

    const double u = ((p0.x_ - p2.x_) * (p0.y_ - p1.y_) - (p0.y_ - p2.y_) * (p0.x_ - p1.x_)) /
                     ((p0.x_ - p1.x_) * (p2.y_ - p3.y_) - (p0.y_ - p1.y_) * (p2.x_ - p3.x_));

    if (t >= 0 && t <= 1 && u >= 0 && u <= 1) {
      const double x = p0.x_ + t * (p1.x_ - p0.x_);
      const double y = p0.y_ + t * (p1.y_ - p0.y_);
      return utils::Point(x, y);
    } else {
      return std::nullopt;
    }
  }


  inline std::tuple<std::optional<utils::Point>, std::optional<utils::Point>>
  circle_segment_intersect(const utils::Point& cent, const double r, const utils::Point& p0, const utils::Point& p1) {
    using utils::sqr;
    const auto u = p1 - p0;

    const double a = sqr(u.x_) + sqr(u.y_);
    const double b = 2 * p0.x_ * u.x_ - 2 * u.x_ * cent.x_ + 2 * p0.y_ * u.y_ - 2 * u.y_ * cent.y_;
    const double c =
        sqr(p0.x_) + sqr(cent.x_) - 2 * p0.x_ * cent.x_ + sqr(p0.y_) + sqr(cent.y_) - 2 * p0.y_ * cent.y_ - sqr(r);

    const auto sol = utils::solve_quadratic(a, b, c);

    bool s1_val = sol[0].imag() < 0.01 && sol[0].real() >= 0 && sol[0].real() <= 1;
    bool s2_val = sol[1].imag() < 0.01 && sol[1].real() >= 0 && sol[1].real() <= 1;

    if (s1_val && s2_val) {
      return { { p0 + sol[0].real() * u }, { p0 + sol[1].real() * u } };
    } else if (s1_val) {
      return { { p0 + sol[0].real() * u }, std::nullopt };
    } else if (s2_val) {
      return { { p0 + sol[1].real() * u }, std::nullopt };
    } else {
      return { std::nullopt, std::nullopt };
    }
  }


}  // namespace geometry

namespace Bezier {
  constexpr double step_sz = 0.001;
  constexpr size_t vec_sz = static_cast<size_t>(1.0 / step_sz) + 2;

  /**
   * @brief Evaluates bezier at
   *
   * @param p array of control points
   * @param t
   * @return utils::Point
   */
  inline utils::Point eval_bezier(const std::array<utils::Point, 4>& p, double t) {
    constexpr auto cub = [](double f) { return f * f * f; };
    return cub(1 - t) * p[0] + 3 * utils::sqr(1 - t) * t * p[1] + 3 * (1 - t) * utils::sqr(t) * p[2] + cub(t) * p[3];
  }

  /**
   * @brief Evaluate curvature of cubic Bezier at @p t
   *
   * @param p Bezier points
   * @param t
   * @return double
   */
  inline double eval_curvature(const std::array<utils::Point, 4>& p, double t) {
    auto deriv_1 = [&p](double t) -> utils::Point {
      return (-3 * t * t + 6 * t - 3) * p[0] + (9 * t * t - 12 * t + 3) * p[1] + (-9 * t * t + 6 * t) * p[2] +
             (3 * t * t) * p[3];
    };
    auto deriv_2 = [&p](double t) -> utils::Point {
      return (6 - 6 * t) * p[0] + (18 * t - 12) * p[1] + (-18 * t + 6) * p[2] + (6 * t) * p[3];
    };

    const auto d = deriv_1(t), dd = deriv_2(t);
    const double num = d.x_ * dd.y_ - dd.x_ * d.y_, den = std::pow(utils::sqr(d.x_) + utils::sqr(d.y_), 3.0 / 2.0);

    if (std::abs(num) < 0.001 || std::abs(den) < 0.001) {
      return 0;
    } else {
      return num / den;
    }
  }

  inline std::vector<utils::Point> create_bezier(const std::array<utils::Point, 4>& p) {
    std::vector<utils::Point> ret;
    ret.reserve(vec_sz);
    auto cub = [](double f) { return f * f * f; };
    for (double t = 0; t <= 1.0; t += step_sz) {
      ret.push_back(eval_bezier(p, t));
    }
    return ret;
  }

  inline std::vector<double> calcualte_radius(const std::array<utils::Point, 4>& p) {
    std::vector<double> ret;
    ret.reserve(vec_sz);

    for (double t = 0; t <= 1.0; t += step_sz) {
      const double kappa = eval_curvature(p, t);
      if (kappa != 0) {
        double r = 1.0 / kappa;
        if (std::abs(r) < 0.001) {
          r = 0;  // reject radii too low
        }
        ret.push_back(r);
      } else {
        ret.push_back(0);
      }
    }
    return ret;
  }


  inline std::optional<utils::Point> bezier_line_intersect(const std::array<utils::Point, 4>& points,
                                                           const utils::Point& p1, const utils::Point& p2) {
    const double x0 = points[0].x_, y0 = points[0].y_, x1 = points[1].x_, y1 = points[1].y_, x2 = points[2].x_,
                 y2 = points[2].y_, x3 = points[3].x_, y3 = points[3].y_;

    // y = mx + b for line
    const double m = std::tan(p1.angle_to(p2));
    const double b = p1.y_ - m * p1.x_;

    // cubic equation for bezier - line intersect in form (a1*t^3 + b1*t^2 + c1*t + d1 = 0)
    const double a1 = y0 - 3 * y1 + 3 * y2 - y3 - m * x0 + 3 * m * x1 - 3 * m * x2 + m * x3;
    const double b1 = -3 * y0 + 6 * y1 - 3 * y2 + 3 * m * x0 - 6 * m * x1 + 3 * m * x2;
    const double c1 = 3 * y0 - 3 * y1 - 3 * m * x0 + 3 * m * x1;
    const double d1 = -y0 + m * x0 + b;

    const std::array<double, 3> res = utils::solve_cubic_real(a1, b1, c1, d1);

    for (double t : res) {
      if (t >= 0 && t <= 1) {
        // t is on curve
        constexpr double eps = 0.05;
        utils::Point r = eval_bezier(points, t);
        if (std::abs(p1.dist(p2) - p1.dist(r) - p2.dist(r)) < eps) {
          // point r lies between p1 and p2, if the distance ||p1, r|| + ||p2, r|| == ||p1, p2||
          // or: ||p1, p2|| - ||p1, r|| - ||p2, r|| == 0

          return std::make_optional(r);
        }
      }
    }

    return std::nullopt;
  }

};  // namespace Bezier
