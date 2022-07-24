#include <gtest/gtest.h>
#include <vec2/vec2.h>



TEST(vec2Test, testCreate) {
  const vec2 v_zero(0, 0);
  const vec2 v_default;
  const vec2 v_non_zero(2, 3);
  const vec2 v_cpy(v_non_zero);
  vec2 v_assign;

  EXPECT_EQ(v_zero.x_, 0);
  EXPECT_EQ(v_zero.y_, 0);
  EXPECT_EQ(v_default, v_zero);

  EXPECT_EQ(v_non_zero.x_, 2);
  EXPECT_EQ(v_non_zero.y_, 3);

  EXPECT_EQ(v_cpy, v_non_zero);
  EXPECT_NE(v_cpy, v_zero);

  EXPECT_EQ(v_assign, v_default);
  v_assign = v_cpy;
  EXPECT_EQ(v_assign, v_cpy);
}


TEST(vec2Test, testDist) {
  const vec2 v0, v1(1, 1);
  EXPECT_EQ(v0.dist(v1), v1.dist(v0));
  EXPECT_NEAR(v0.dist(v1), std::sqrt(2), 0.0001);
}


TEST(vec2Test, testAngle) {
  const vec2 v0, v1(1, 0), v2(0, 1), v3(-1, 0), v4(0, -1);
  EXPECT_NEAR(v0.angle_to(v1), 0, 0.0001);
  EXPECT_NEAR(v0.angle_to(v2), M_PI / 2, 0.0001);
  EXPECT_NEAR(v0.angle_to(v3), M_PI, 0.0001);
  EXPECT_NEAR(v0.angle_to(v4), -M_PI / 2, 0.0001);
}

TEST(vec2Test, testUnaryMinus) {
  const vec2 v(2, 3);
  vec2 v2 = -v;
  EXPECT_EQ(v2.x_, -2);
  EXPECT_EQ(v2.y_, -3);
}

TEST(vec2Test, testArithmeticVec2Operators) {
  const vec2 incr(1, 1);
  const vec2 orig(0, 0);

  const vec2 r1 = orig + incr, r2 = orig - incr;
  EXPECT_EQ(r1, incr);
  EXPECT_EQ(r2, -incr);

  vec2 from(2, 2);
  vec2 other(2, 2);
  vec2 r3 = from * other, r4 = from / other;
  EXPECT_EQ(r3, vec2(4, 4));
  EXPECT_EQ(r4, vec2(1, 1));
}


TEST(vec2Test, testAssignmentOperators) {
  vec2 orig(1, 1), v1(2, 2), v2(1, 1), v3(4, 5);

  orig += v1;
  EXPECT_EQ(orig, vec2(3, 3));
  orig -= v2;
  EXPECT_EQ(orig, vec2(2, 2));
  orig *= v3;
  EXPECT_EQ(orig, vec2(8, 10));
  orig /= v3;
  EXPECT_EQ(orig, vec2(2, 2));
}


TEST(vec2Test, testMultiply) {
  vec2 orig(2, 4);

  EXPECT_EQ(2 * orig, orig * 2);
  EXPECT_EQ(2 * orig, vec2(4, 8));
  EXPECT_EQ(orig / 2.0, vec2(1, 2));
}
