#include <gtest/gtest.h>

#include <chrono>

#include "subsea_inspection/common/input_watchdog.hpp"

TEST(InputWatchdogTest, ExpiresAtTimeoutBoundary)
{
  const auto last_input = std::chrono::steady_clock::time_point{};
  const std::chrono::duration<double> timeout(0.25);

  EXPECT_FALSE(subsea_inspection::input_watchdog::is_stale(
    last_input, last_input + std::chrono::milliseconds(249), timeout));
  EXPECT_TRUE(subsea_inspection::input_watchdog::is_stale(
    last_input, last_input + std::chrono::milliseconds(250), timeout));
}
