#include <gtest/gtest.h>

#include <cmath>

#include <RobotControl/Encoder.h>
#include <cmath>
struct EncoderTest : testing::Test {
  Encoder* enc;
  EncoderTest() : enc(new Encoder(1.0)){};
  ~EncoderTest() {
    delete enc;
  }
};

static constexpr double inc_to_pos(double inc_deg) {
  return inc_deg / 180.0 * M_PI;
}
static constexpr double rad2deg(double rad) {
  return rad * 180.0 / M_PI;
}


TEST_F(EncoderTest, TestDefault) {
  EXPECT_EQ(enc->get_position(), 0);
  EXPECT_EQ(enc->get_speed(), 0);
  EXPECT_EQ(enc->get_ang_speed(), 0);
};

TEST_F(EncoderTest, TestInit) {
  enc->init(20);
  EXPECT_EQ(enc->get_position(), 0);
  EXPECT_EQ(enc->get_speed(), 0);
};

TEST_F(EncoderTest, TestRising) {
  enc->init(90);
  enc->tick(100, 0.1);
  EXPECT_DOUBLE_EQ(enc->get_position(), inc_to_pos(10));
}

TEST_F(EncoderTest, TestFalling) {
  enc->init(60);
  enc->tick(30, 0.1);
  EXPECT_DOUBLE_EQ(enc->get_position(), inc_to_pos(-30));
}

TEST_F(EncoderTest, TestOverflow) {
  enc->init(350);
  enc->tick(10, 0.1);
  EXPECT_DOUBLE_EQ(enc->get_position(), inc_to_pos(20));
}

TEST_F(EncoderTest, TestUnderflow) {
  enc->init(5);
  enc->tick(350, 0.1);
  EXPECT_DOUBLE_EQ(enc->get_position(), inc_to_pos(-15));
}


TEST_F(EncoderTest, TestSpeed) {
  enc->init(0);
  constexpr double increment = 0.2;
  enc->tick(rad2deg(increment), 0.1);

  EXPECT_DOUBLE_EQ(enc->get_position(), increment);
  EXPECT_DOUBLE_EQ(enc->get_speed(), 0.2 / 0.1);
}