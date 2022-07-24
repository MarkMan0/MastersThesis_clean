#include <gtest/gtest.h>

#include <RobotControl/Utility.h>
#include <algorithm>


TEST(UtilityTest, Test_Signum) {
  using utils::sign;
  EXPECT_EQ(sign(3), 1);
  EXPECT_EQ(sign(-80), -1);
  EXPECT_EQ(sign(0), 0);
}


TEST(UtilityTest, Test_Normalize_angle_positive) {
  using utils::normalize_angle;

  constexpr double angle1 = utils::deg2rad(60);
  constexpr double mult = 2 * utils::PI;
  constexpr double prec = 0.0001;
  EXPECT_NEAR(normalize_angle(0), 0, prec);
  EXPECT_NEAR(normalize_angle(angle1), angle1, prec);
  EXPECT_NEAR(normalize_angle(angle1 + 5 * mult), angle1, prec);
  EXPECT_NEAR(normalize_angle(angle1 - 5 * mult), angle1, prec);
}

TEST(UtilityTest, Test_Normalize_angle_negative) {
  using utils::normalize_angle;

  constexpr double angle1 = utils::deg2rad(-40);
  constexpr double mult = 2 * utils::PI;
  constexpr double prec = 0.0001;
  EXPECT_NEAR(normalize_angle(0), 0, prec);
  EXPECT_NEAR(normalize_angle(angle1), angle1, prec);
  EXPECT_NEAR(normalize_angle(angle1 + 5 * mult), angle1, prec);
  EXPECT_NEAR(normalize_angle(angle1 - 5 * mult), angle1, prec);
}


struct CloseAngle {
  double angle_from;
  double angle_to;
  double expected_result;

  friend std::ostream& operator<<(std::ostream& os, const CloseAngle& ca) {
    os << "angle_from: " << ca.angle_from << " angle_to: " << ca.angle_to << " expected_result: " << ca.expected_result;
    return os;
  }
};
struct CloseAngleTest : testing::Test, testing::WithParamInterface<CloseAngle> {};

TEST_P(CloseAngleTest, TestClosestAngle) {
  using utils::deg2rad;
  const auto& ca = GetParam();
  double result = utils::closest_angle(deg2rad(ca.angle_from), deg2rad(ca.angle_to));

  EXPECT_NEAR(result, deg2rad(ca.expected_result), 0.0001);
}


INSTANTIATE_TEST_CASE_P(Default, CloseAngleTest,
                        testing::Values(CloseAngle{ 0, 0, 0 }, CloseAngle{ 10, 10, 10 },
                                        CloseAngle{ 40 + 360, 40, 40 + 360 },
                                        CloseAngle{ -60 + 4 * 360, -65, -65 + 4 * 360 },
                                        CloseAngle{ -70 - 5 * 360, -65, -65 - 5 * 360 },
                                        CloseAngle{ 175 + 2 * 360, -175, 185 + 2 * 360 }));


TEST(UtilityTest, TestPointDistance) {
  utils::Point p1, p2;
  EXPECT_EQ(p1.x_, 0);
  EXPECT_EQ(p1.y_, 0);
  p1.x_ = 1;

  p2.x_ = 2;
  p2.y_ = -1;

  EXPECT_DOUBLE_EQ(p1.dist(p2), std::sqrt(2));
  EXPECT_DOUBLE_EQ(p1.dist(p2), p2.dist(p1));
}


struct cubic_formula {
  std::array<double, 4> coef;
  std::array<std::complex<double>, 3> results;

  friend std::ostream& operator<<(std::ostream& os, const cubic_formula& ca) {
    os << "a: " << ca.coef[0] << ", b: " << ca.coef[1] << ", c: " << ca.coef[2] << ", d: " << ca.coef[3];
    os << ", x0: " << ca.results[0] << ", x1: " << ca.results[1] << ", x2: " << ca.results[2];
    return os;
  }
};

struct CubicFormulaTest : public testing::Test, public testing::WithParamInterface<cubic_formula> {};

TEST_P(CubicFormulaTest, TestFormula) {
  auto coef = GetParam().coef;
  auto sol = GetParam().results;
  auto ret = utils::solve_cubic(coef[0], coef[1], coef[2], coef[3]);


  auto a_in_b = [](const std::complex<double>& a, const std::array<std::complex<double>, 3>& b) {
    constexpr double delta = 0.01;
    for (const auto& comp : b) {
      if (std::abs(a.real() - comp.real()) < delta && std::abs(a.imag() - comp.imag()) < delta) {
        return true;
      }
    }
    return false;
  };

  for (int i = 0; i < 3; ++i) {
    EXPECT_TRUE(a_in_b(ret[i], sol));
  };
}

using cmpl = std::complex<double>;

INSTANTIATE_TEST_CASE_P(
    Default, CubicFormulaTest,
    testing::Values(cubic_formula{ { 2, 3, -11, -6 }, { cmpl{ 2, 0 }, cmpl{ -0.5, 0 }, cmpl{ -3, 0 } } },
                    cubic_formula{ { 1, 1, 1, 1 }, { cmpl{ -1, 0 }, cmpl{ 0, 1 }, cmpl{ 0, -1 } } },
                    cubic_formula{ { 45.78, -87.13, 0.25, 999.66 },
                                   { cmpl{ -2.28319, 0 }, cmpl{ 2.09321, 2.27647 }, cmpl{ 2.09321, -2.27647 } } }));


TEST(UtilityTest, TestMap) {
  using utils::map;
  // should have used TEST_P but too much boilerplate

  // upper
  EXPECT_DOUBLE_EQ(-4, map(5, -7, 5, -9, -4));
  // lower
  EXPECT_DOUBLE_EQ(-10.5, map(0.78, 0.78, 10.57, -10.5, 6.54));
  // middle
  EXPECT_DOUBLE_EQ(0.5, map(1, 0, 2, 0, 1));
}

TEST(UtilityTest, TestMapClamped) {
  using utils::map_clamped;
  // above
  EXPECT_DOUBLE_EQ(-4, map_clamped(8, -7, 5, -9, -4));
  // below
  EXPECT_DOUBLE_EQ(-10.5, map_clamped(-0.78, 0.78, 10.57, -10.5, 6.54));
  // normal
  EXPECT_DOUBLE_EQ(0.5, map_clamped(1, 0, 2, 0, 1));
}
