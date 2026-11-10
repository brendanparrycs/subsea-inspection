#include <gtest/gtest.h>

#include <limits>

#include "subsea_inspection/mission_route.hpp"

using subsea_inspection::has_passed_waypoint;

TEST(MissionRouteTest, DoesNotAdvanceBeforeOrInsideWaypointAcceptanceRadius)
{
  // Route approaches (1, 0) from (0, 0), then continues to (2, 0).
  EXPECT_FALSE(has_passed_waypoint(0.8, 0.0, 0.0, 0.0, true, 1.0, 0.0, 2.0, 0.0, 0.4, 1.0));
  EXPECT_FALSE(has_passed_waypoint(1.2, 0.0, 0.0, 0.0, true, 1.0, 0.0, 2.0, 0.0, 0.4, 1.0));
}

TEST(MissionRouteTest, AdvancesAfterPassingAcceptanceRadiusAlongRoute)
{
  EXPECT_TRUE(has_passed_waypoint(1.5, 0.0, 0.0, 0.0, true, 1.0, 0.0, 2.0, 0.0, 0.4, 1.0));
}

TEST(MissionRouteTest, DoesNotAdvanceWhenVehicleIsFarOffRoute)
{
  EXPECT_FALSE(has_passed_waypoint(1.5, 2.0, 0.0, 0.0, true, 1.0, 0.0, 2.0, 0.0, 0.4, 1.0));
}

TEST(MissionRouteTest, UsesIncomingDirectionAtSharpTurn)
{
  // The route arrives from (1, 0.2) and turns north at (0, 0).
  // A vehicle at (0.2, 0.04) is still before the waypoint on the arrival leg, even though
  // its projection onto the outgoing northbound segment is positive.
  EXPECT_FALSE(has_passed_waypoint(0.2, 0.04, 1.0, 0.2, true, 0.0, 0.0, 0.0, 1.0, 0.3, 1.0));

  // Once it continues west past the waypoint along the arrival leg, it is eligible to advance.
  EXPECT_TRUE(has_passed_waypoint(-0.4, -0.08, 1.0, 0.2, true, 0.0, 0.0, 0.0, 1.0, 0.3, 1.0));
}

TEST(MissionRouteTest, FirstWaypointUsesOutgoingDirection)
{
  EXPECT_FALSE(has_passed_waypoint(0.0, 0.8, 0.0, 0.0, false, 0.0, 1.0, 0.0, 2.0, 0.4, 1.0));
  EXPECT_TRUE(has_passed_waypoint(0.0, 1.5, 0.0, 0.0, false, 0.0, 1.0, 0.0, 2.0, 0.4, 1.0));
}

TEST(MissionRouteTest, RejectsInvalidGeometryAndLimits)
{
  const double nan = std::numeric_limits<double>::quiet_NaN();
  EXPECT_FALSE(has_passed_waypoint(nan, 0.0, 0.0, 0.0, true, 1.0, 0.0, 2.0, 0.0, 0.2, 1.0));
  EXPECT_FALSE(has_passed_waypoint(1.0, 0.0, 0.0, 0.0, true, 0.0, 0.0, 0.0, 0.0, 0.2, 1.0));
  EXPECT_FALSE(has_passed_waypoint(1.0, 0.0, 0.0, 0.0, true, 0.0, 0.0, 1.0, 0.0, 0.0, 1.0));
  EXPECT_FALSE(has_passed_waypoint(1.0, 0.0, 0.0, 0.0, true, 0.0, 0.0, 1.0, 0.0, 0.2, -1.0));
}
