#include <gtest/gtest.h>

#include "subsea_inspection/pid_controller.hpp"

using subsea_inspection::PidController;

/// @brief Verifies that proportional response matches Kp * error.
TEST(PidControllerTest, ProportionalResponse)
{
  PidController pid(1.5, 0.0, 0.0, 5.0, 1.0);
  double output = pid.compute(2.0, 0.1);
  EXPECT_DOUBLE_EQ(output, 3.0);
}

/// @brief Verifies that control output saturates at positive and negative max_output boundaries.
TEST(PidControllerTest, OutputSaturationClamping)
{
  PidController pid(2.0, 0.0, 0.0, 0.8, 1.0);

  double output = pid.compute(5.0, 0.1);
  EXPECT_DOUBLE_EQ(output, 0.8);

  output = pid.compute(-5.0, 0.1);
  EXPECT_DOUBLE_EQ(output, -0.8);
}

/// @brief Verifies that integral accumulation saturates at integral_limit (anti-windup).
TEST(PidControllerTest, AntiWindupClamping)
{
  PidController pid(0.0, 1.0, 0.0, 5.0, 0.5);

  for (int i = 0; i < 100; ++i) {
    pid.compute(1.0, 0.1);
  }

  double output = pid.compute(1.0, 0.1);
  EXPECT_DOUBLE_EQ(output, 0.5);
}

/// @brief Verifies that derivative action opposes rapid changes in error (braking effect).
TEST(PidControllerTest, DerivativeBrakingAction)
{
  PidController pid(0.0, 0.0, 0.5, 5.0, 1.0);
  pid.compute(2.0, 0.1);

  double output = pid.compute(1.0, 0.1);
  EXPECT_DOUBLE_EQ(output, -5.0);
}

/// @brief Verifies that non-positive dt produces 0.0 and prevents division by zero.
TEST(PidControllerTest, ZeroOrNegativeDtSafelyHandled)
{
  PidController pid(1.0, 1.0, 1.0, 5.0, 1.0);
  EXPECT_DOUBLE_EQ(pid.compute(2.0, 0.0), 0.0);
  EXPECT_DOUBLE_EQ(pid.compute(2.0, -0.1), 0.0);
}

/// @brief Verifies that reset() zeroes accumulated integral and clears derivative state.
TEST(PidControllerTest, ResetClearsState)
{
  PidController pid(0.0, 1.0, 0.0, 5.0, 5.0);
  pid.compute(1.0, 1.0);
  pid.reset();
  EXPECT_DOUBLE_EQ(pid.compute(0.0, 0.1), 0.0);
}

/// @brief Main entry point: runs unit tests.
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
