#include <gtest/gtest.h>

#include "subsea_inspection/common/manual_control_utils.hpp"
#include "subsea_inspection/pid_controller.hpp"
#include "subsea_inspection/station_keeping.hpp"

namespace
{

/// @brief Creates a unit-gain PID controller for station-keeping tests.
/// @param max_output Maximum absolute controller output.
/// @return A PID controller with unit proportional gain and no integral or derivative gain.
subsea_inspection::PidController make_unit_pid(double max_output = 1.0)
{
  return subsea_inspection::PidController(1.0, 0.0, 0.0, max_output, 1.0);
}
}  // namespace

TEST(StationKeepingTest, ProducesBodyFrameCorrectionsForWorldPositionErrors)
{
  auto surge_pid = make_unit_pid();
  auto sway_pid = make_unit_pid();
  auto heave_pid = make_unit_pid();
  auto yaw_pid = make_unit_pid();

  const auto command = subsea_inspection::compute_station_keeping_command(
    2.0, 1.0, -0.5, 0.0, 0.0, 0.1, surge_pid, sway_pid, heave_pid, yaw_pid);

  EXPECT_DOUBLE_EQ(command.surge_mps, 1.0);
  EXPECT_DOUBLE_EQ(command.sway_mps, 1.0);
  EXPECT_DOUBLE_EQ(command.heave_mps, -0.5);
  EXPECT_DOUBLE_EQ(command.yaw_rate_radps, 0.0);
}

TEST(StationKeepingTest, RotatesWorldErrorsIntoBodyFrame)
{
  auto surge_pid = make_unit_pid();
  auto sway_pid = make_unit_pid();
  auto heave_pid = make_unit_pid();
  auto yaw_pid = make_unit_pid(3.0);

  const auto command = subsea_inspection::compute_station_keeping_command(
    0.0, 1.0, 0.0, 0.0, 1.5707963267948966, 0.1, surge_pid, sway_pid, heave_pid, yaw_pid);

  EXPECT_NEAR(command.surge_mps, 1.0, 1e-9);
  EXPECT_NEAR(command.sway_mps, 0.0, 1e-9);
  EXPECT_NEAR(command.yaw_rate_radps, -1.5707963267948966, 1e-9);
}

TEST(StationKeepingTest, ConvertsRosLeftPositiveSwayToMavlinkRightPositiveAxis)
{
  EXPECT_DOUBLE_EQ(subsea_inspection::manual_control::ros_sway_to_mavlink_y(0.4), -0.4);
  EXPECT_DOUBLE_EQ(subsea_inspection::manual_control::ros_sway_to_mavlink_y(-0.4), 0.4);
  EXPECT_DOUBLE_EQ(subsea_inspection::manual_control::ros_sway_to_mavlink_y(0.0), 0.0);
}
