#include "RobotControl/Path.h"
#include "RobotControl/geometry.h"
#include <fstream>
#include <memory>


planning::PathFactory& planning::PathFactory::next() {
  from_ = to_;
  angle_entry_ = angle_exit_;

  to_ = std::nullopt;
  angle_exit_ = std::nullopt;

  return *this;
}


planning::BezierPath planning::PathFactory::createBezier() {
  if (!from_ || !to_ || !angle_entry_ || !speed_) {
    throw insufficient_data_err_;
  }

  if (!angle_exit_) {
    angle_exit_ = from_->angle_to(*to_);
  }

  if (!weight_) {
    weight_ = 3;
    weight_ = std::min(*weight_, 0.5 * from_->dist(*to_));
  }

  // TODO: document
  // create vectors of lentgth weight_, where the angle is angle_entry and angle_exit
  // from these vectors, calculate the two middle points on the curve, so the "direction" of the curve is known
  const utils::Point entry_vec = (*weight_) * utils::Point::from_angle(*angle_entry_);
  const utils::Point exit_vec = -1 * (*weight_) * utils::Point::from_angle(*angle_exit_);

  const utils::Point p2 = *from_ + entry_vec, p3 = *to_ + exit_vec;
  auto ret = BezierPath({ *from_, p2, p3, *to_ }, *angle_entry_, *angle_exit_, *speed_);
  next();
  return ret;
}

planning::LinePath planning::PathFactory::createLine() {
  if (!from_ || !to_ || !speed_) {
    throw insufficient_data_err_;
  }
  const double angle = from_->angle_to(*to_);
  angle_entry_ = angle;
  angle_exit_ = angle;
  auto ret = LinePath({ *from_, *to_ }, angle, angle, *speed_);
  next();
  return ret;
}

planning::CircularPath planning::PathFactory::createCircular() {
  if (!from_ || !to_ || !speed_ || !angle_entry_) {
    throw insufficient_data_err_;
  }

  CircularPath p = CircularPath::point_tangent_point(*from_, *to_, *angle_entry_, *speed_);
  angle_exit_ = p.angle_exit_;
  next();
  return p;
}


planning::JointPath planning::PathFactory::createBezierLine() {
  if (!from_ || !to_ || !angle_entry_ || !speed_) {
    throw insufficient_data_err_;
  }

  if (!angle_exit_) {
    angle_exit_ = from_->angle_to(*to_);
  }

  const double line_weight = 0.1;
  if (!weight_) {
    weight_ = 3;
    weight_ = std::min(*weight_, 0.5 * from_->dist(*to_));
    if (*weight_ > line_weight) {
      *weight_ -= line_weight;
    }
  }

  const utils::Point entry_vec = (line_weight)*utils::Point::from_angle(*angle_entry_);
  const utils::Point exit_vec = -1 * (line_weight)*utils::Point::from_angle(*angle_exit_);

  const utils::Point p2 = *from_ + entry_vec, p3 = *to_ + exit_vec;

  JointPath jp;
  jp.add(PathFactory().from(*from_).to(p2).speed(*speed_).createLine());
  jp.add(PathFactory()
             .from(p2)
             .to(p3)
             .angle_entry(*angle_entry_)
             .angle_exit(*angle_exit_)
             .weight(*weight_)
             .speed(*speed_)
             .createBezier());
  jp.add(PathFactory().from(p3).to(*to_).speed(*speed_).createLine());
  jp.set_speed(*speed_);

  next();
  return jp;
}

planning::CircularPath planning::CircularPath::point_tangent_point(const utils::Point& from, const utils::Point& to,
                                                                   double angle_in, double speed) {
  const auto [cent, r] = geometry::circe_point_point_angle(from, to, angle_in);
  const bool is_cw = geometry::is_arc_cw(cent, from, angle_in);
  const double angle_out = is_cw ? cent.angle_to(to) + utils::deg2rad(-90) : cent.angle_to(to) + utils::deg2rad(90);

  return CircularPath({ from, to }, angle_in, angle_out, speed, cent, r);
}


std::vector<utils::Point> planning::CircularPath::create_points() const {
  std::vector<utils::Point> ret;
  const bool is_cw = geometry::is_arc_cw(center_, points_[0], angle_entry_);

  const double increment = is_cw ? utils::deg2rad(-0.5) : utils::deg2rad(0.5);
  const double start = center_.angle_to(points_[0]), end = center_.angle_to(points_[1]);


  for (double theta = start; std::abs(utils::normalize_angle(theta) - end) > std::abs(increment); theta += increment) {
    ret.push_back(center_ + radius_ * utils::Point::from_angle(theta));
  }

  return ret;
}


std::optional<utils::Point> planning::CircularPath::line_intersect(const utils::Point& p0,
                                                                   const utils::Point& p1) const {
  const auto [s1, s2] = geometry::circle_segment_intersect(center_, radius_, p0, p1);

  if (s1 && s2) {
    if (s1->dist(p0) < s2->dist(p0)) {
      return s1;
    } else {
      return s2;
    }
  } else if (s1) {
    return s1;
  } else if (s2) {
    return s2;
  } else {
    return std::nullopt;
  }
}



std::vector<utils::Point> planning::JointPath::create_points() const {
  std::vector<utils::Point> ret;

  for (auto& p : paths_) {
    const auto pts = p->create_points();
    ret.insert(ret.end(), pts.begin(), pts.end());
  }

  return ret;
}


std::optional<utils::Point> planning::JointPath::line_intersect(const utils::Point& p0, const utils::Point& p1) const {
  for (auto it = paths_.rbegin(); it != paths_.rend(); ++it) {
    auto& p = *it;
    std::optional found = p->line_intersect(p0, p1);
    if (found) {
      return found;
    }
  }

  return std::nullopt;
}

void planning::JointPath::set_start(const utils::Point& p, double d) {
  start_ = p;
  start_angle_ = d;
}

void planning::JointPath::add_waypoint(const utils::Point& p, double d) {
  if (paths_.size() == 0) {
    if (!start_) {
      throw std::logic_error("Can't add waypoint without start or to empty path");
    }
    factory_.from(*start_).angle_entry(start_angle_).speed(1);
    auto path = factory_.to(p).angle_exit(d).createBezier();
    add(path);
  } else {
    auto path = factory_.to(p).angle_exit(d).createBezier();
    add(path);
  }
}

void planning::JointPath::add_waypoint(const utils::Point& p) {
  if (paths_.size() == 0) {
    if (!start_) {
      throw std::logic_error("Can't add waypoint without start or to empty path");
    }
    factory_.from(*start_).angle_entry(start_angle_).speed(1);
    auto path = factory_.to(p).createBezier();
    add(path);
  } else {
    auto path = factory_.to(p).createBezier();
    add(path);
  }
}
