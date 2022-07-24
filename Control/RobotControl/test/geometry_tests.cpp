#include <gtest/gtest.h>


#include "RobotControl/geometry.h"

TEST(GeometryTests, TestBezierLine) {
  std::array<utils::Point, 4> p{ { { 0, 0 }, { 1, 0 }, { 2, 0 }, { 3, 0 } } };

  auto pts = Bezier::create_bezier(p);
  constexpr double err = 0.005;
  for (const auto& point : pts) {
    EXPECT_NEAR(point.y_, 0, err);
  }
  EXPECT_NEAR(pts[0].x_, p[0].x_, err);
  EXPECT_NEAR(pts[0].y_, p[0].y_, err);
  EXPECT_NEAR(pts.rbegin()->x_, p[3].x_, err);
  EXPECT_NEAR(pts.rbegin()->y_, p[3].y_, err);
}


struct SegmentAngle {
  const utils::Point p0;
  const double angle, distance, len;

  const utils::Point p1, p2;

  std::tuple<utils::Point, utils::Point> eval() const {
    return geometry::segment_distance_angle(p0, angle, distance, len);
  }

  friend std::ostream& operator<<(std::ostream& os, const SegmentAngle& s) {
    os << "p0: (" << s.p0.x_ << ", " << s.p0.y_ << "), angle: " << s.angle << ", distance: " << s.distance
       << ", len: " << s.len << ", p1: (" << s.p1.x_ << ", " << s.p1.y_ << "), p2: " << s.p2.x_ << ", " << s.p2.y_
       << ")";

    return os;
  }
};

struct SegmentDistAngleTest : public testing::Test, public testing::WithParamInterface<SegmentAngle> {};

TEST_P(SegmentDistAngleTest, TestLen) {
  auto [p1_res, p2_res] = GetParam().eval();
  EXPECT_NEAR(p1_res.dist(p2_res), GetParam().len, 0.05);
}

TEST_P(SegmentDistAngleTest, TestDist) {
  auto [p1_res, p2_res] = GetParam().eval();
  const auto p_center = 0.5 * (p1_res + p2_res);
  EXPECT_NEAR(p_center.dist(GetParam().p0), GetParam().distance, 0.05);
}

TEST_P(SegmentDistAngleTest, TestAngle) {
  auto [p1_res, p2_res] = GetParam().eval();
  const double angle_res = utils::normalize_angle(p2_res.angle_to(p1_res));
  const double angle_exp = utils::normalize_angle(GetParam().angle + utils::PI / 2);
  EXPECT_NEAR(angle_res, angle_exp, 0.05);
}

TEST_P(SegmentDistAngleTest, TestEndPoints) {
  auto [p1_res, p2_res] = GetParam().eval();

  constexpr double err = 0.05;

  EXPECT_NEAR(p1_res.x_, GetParam().p1.x_, err);
  EXPECT_NEAR(p1_res.y_, GetParam().p1.y_, err);
  EXPECT_NEAR(p2_res.x_, GetParam().p2.x_, err);
  EXPECT_NEAR(p2_res.y_, GetParam().p2.y_, err);
}

static auto segmentDistAngleVals = testing::Values(
    SegmentAngle{ { 0, 0 }, 0, 1, 1, { 1, 0.5 }, { 1, -0.5 } },
    SegmentAngle{ { 1, 0.5 }, utils::PI / 2, 0.6, 0.3, { 0.85, 1.1 }, { 1.15, 1.1 } },
    SegmentAngle{ { 1.9355, 0.8336 }, utils::deg2rad(116.565051177), 1.6, 1.3, { 0.638, 1.974 }, { 1.801, 2.555 } },
    SegmentAngle{ { -1.3203, -0.2173 }, utils::deg2rad(-56.79279), 0.8, 0.2, { -0.798, -0.831 }, { -0.965, -0.941 } });


struct CirclePointPointAngle {
  const utils::Point p0, p1;
  const double angle;
  const utils::Point center;
  const double radius;

  std::tuple<utils::Point, double> eval() const {
    return geometry::circe_point_point_angle(p0, p1, angle);
  }
};


struct CirclePointPointAngleTest : public testing::Test, public testing::WithParamInterface<CirclePointPointAngle> {};

TEST_P(CirclePointPointAngleTest, TestRadius) {
  const auto [cent_ret, radius_ret] = GetParam().eval();
  const double radius_exp = GetParam().radius;
  const utils::Point cent_exp = GetParam().center;
  EXPECT_NEAR(radius_ret, radius_exp, 0.05);
  EXPECT_NEAR(cent_ret.dist(cent_exp), 0, 0.05);
}

static auto circlePointPointAngleVals = testing::Values(
    CirclePointPointAngle{ { 0, 0 }, { 0.2263, 0.0215 }, 0, { 0, 1.2 }, 1.2 },
    CirclePointPointAngle{ { 0.4, 0.2 }, { 0.13502, 0.878 }, utils::deg2rad(90), { -0.6, 0.2 }, 1 },
    CirclePointPointAngle{ { 0.219, 0.399 }, { 0.409, 0.606 }, utils::deg2rad(21.571), { 0.1, 0.7 }, 0.323266 },
    CirclePointPointAngle{ { 0.701, 0.272 }, { 0.4999, 0.3749 }, utils::deg2rad(139.3569), { 0.390, -0.089 }, 0.4775 });


INSTANTIATE_TEST_CASE_P(Default, SegmentDistAngleTest, segmentDistAngleVals);

INSTANTIATE_TEST_CASE_P(Default, CirclePointPointAngleTest, circlePointPointAngleVals);

struct CirclePointPointPoint {
  const utils::Point p0, p1, p2;
  const utils::Point center;
  const double radius;

  std::tuple<utils::Point, double> eval() const {
    return geometry::circle_point_point_point(p0, p1, p2);
  }
};

struct CirclePointPointPointTest : public testing::Test, public testing::WithParamInterface<CirclePointPointPoint> {};

TEST_P(CirclePointPointPointTest, TestCreate) {
  const auto [cent_ret, radius_ret] = GetParam().eval();
  const double radius_exp = GetParam().radius;
  const utils::Point cent_exp = GetParam().center;
  EXPECT_NEAR(radius_ret, radius_exp, 0.05);
  EXPECT_NEAR(cent_ret.dist(cent_exp), 0, 0.05);
}

static auto circlePointPointPointVals = testing::Values(
    CirclePointPointPoint{ { 0.6245, -0.0639 }, { 0.4574, 0.4315 }, { -0.0508, 0.4511 }, { 0.1887, 0.065 }, 0.4544 },
    CirclePointPointPoint{ { 0.4574, 0.4315 }, { 0.6245, -0.0639 }, { -0.0508, 0.4511 }, { 0.1887, 0.065 }, 0.4544 },
    CirclePointPointPoint{ { -0.0508, 0.4511 }, { 0.4574, 0.4315 }, { 0.6245, -0.0639 }, { 0.1887, 0.065 }, 0.4544 },
    CirclePointPointPoint{ { 1.4257, 1.8209 }, { 1.4245, 1.5506 }, { 1.3336, 1.3891 }, { 0.9921, 1.6877 }, 0.4536 },
    CirclePointPointPoint{ { 1.4245, 1.5506 }, { 1.3336, 1.3891 }, { 1.4257, 1.8209 }, { 0.9921, 1.6877 }, 0.4536 });


INSTANTIATE_TEST_CASE_P(Default, CirclePointPointPointTest, circlePointPointPointVals);


struct CircleSegmentIntersect {
  const utils::Point cent_;
  const double r_;
  const utils::Point p0_, p1_;
  using res_t = std::tuple<std::optional<utils::Point>, std::optional<utils::Point>>;
  res_t res;

  CircleSegmentIntersect(const utils::Point& c, double r, const utils::Point& p0, const utils::Point& p1,
                         std::optional<utils::Point> r0, std::optional<utils::Point> r1)
    : cent_(c), r_(r), p0_(p0), p1_(p1), res(r0, r1) {
  }

  res_t eval() const {
    return geometry::circle_segment_intersect(cent_, r_, p0_, p1_);
  }
};


struct CircleSegmentIntersectTest : public testing::Test, public testing::WithParamInterface<CircleSegmentIntersect> {};

TEST_P(CircleSegmentIntersectTest, TestCircleSegmentIntersect) {
  const auto [is0_exp, is1_exp] = GetParam().res;

  auto param = GetParam();

  const auto [is0, is1] = GetParam().eval();
  ASSERT_EQ(is0.has_value(), is0_exp.has_value());
  ASSERT_EQ(is1.has_value(), is1_exp.has_value());

  constexpr double delta = 0.05;

  if (is0.has_value() && is1.has_value()) {
    bool match = false;

    if (is0->dist(*is0_exp) < delta && is1->dist(*is1_exp) < delta) {
      match = true;
    } else if (is0->dist(*is1_exp) < delta && is1->dist(*is0_exp) < delta) {
      match = true;
    }
    EXPECT_TRUE(match);
  } else if (is1.has_value()) {
    EXPECT_NEAR(is0->dist(*is0_exp), 0, 0.05);
  }
}

static auto circleSegmentIntersectVals = testing::Values(
    CircleSegmentIntersect({ 0, 0 }, 0.5, { 0.8, -0.4 }, { -0.6, 0.4 }, utils::Point(0.4565, -0.203),
                           utils::Point(-0.407, 0.289)),
    CircleSegmentIntersect({ 0, 0 }, 0.5, { 0.2, 0.2 }, { 0.6, 0.4 }, utils::Point(0.4, 0.3), std::nullopt),
    CircleSegmentIntersect({ 0, 0 }, 0.5, { -0.4, 0.4 }, { 0.4, 0.4 }, utils::Point(-0.3, 0.4), utils::Point(0.3, 0.4)),
    CircleSegmentIntersect({ 1, 1 }, 0.2, { 0.4, 1 }, { 1.4, 0.8 }, utils::Point(0.818, 0.916),
                           utils::Point(1.135, 0.852)),
    CircleSegmentIntersect({ 1, 1 }, 0.2, { 1, 0.6 }, { 1, 1.4 }, utils::Point(1, 0.8), utils::Point(1, 1.2)));

INSTANTIATE_TEST_CASE_P(Default, CircleSegmentIntersectTest, circleSegmentIntersectVals);
