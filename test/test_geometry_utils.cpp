#include <gtest/gtest.h>

#include <cmath>

#include "subsea_inspection/geometry_utils.hpp"

namespace geom = subsea_inspection::geometry;

/// @brief Verifies that wrap_angle normalizes angles outside [-PI, +PI].
TEST(GeometryUtilsTest, WrapAngle)
{
  constexpr double pi = M_PI;

  EXPECT_NEAR(geom::wrap_angle(0.0), 0.0, 1e-9);
  EXPECT_NEAR(geom::wrap_angle(pi), pi, 1e-9);
  EXPECT_NEAR(geom::wrap_angle(-pi), -pi, 1e-9);

  // 2*PI + 0.5 rad -> should wrap to 0.5 rad
  EXPECT_NEAR(geom::wrap_angle(2.0 * pi + 0.5), 0.5, 1e-9);

  // -2*PI - 0.5 rad -> should wrap to -0.5 rad
  EXPECT_NEAR(geom::wrap_angle(-2.0 * pi - 0.5), -0.5, 1e-9);

  // 3*PI -> should wrap to PI (or -PI)
  EXPECT_NEAR(std::abs(geom::wrap_angle(3.0 * pi)), pi, 1e-9);
}

/// @brief Verifies that angle_error finds the shortest rotational path across branch cut (+-180
/// deg).
TEST(GeometryUtilsTest, AngleErrorShortestPath)
{
  constexpr double deg2rad = M_PI / 180.0;

  // Normal case: from 10 deg to 30 deg -> +20 deg
  EXPECT_NEAR(geom::angle_error(30.0 * deg2rad, 10.0 * deg2rad), 20.0 * deg2rad, 1e-9);

  // Branch cut wrap: from +170 deg to -170 deg -> shortest path is +20 deg (turning right)
  EXPECT_NEAR(geom::angle_error(-170.0 * deg2rad, 170.0 * deg2rad), 20.0 * deg2rad, 1e-9);

  // Branch cut wrap: from -170 deg to +170 deg -> shortest path is -20 deg (turning left)
  EXPECT_NEAR(geom::angle_error(170.0 * deg2rad, -170.0 * deg2rad), -20.0 * deg2rad, 1e-9);
}

/// @brief Verifies 2D and 3D Euclidean distance computations.
TEST(GeometryUtilsTest, EuclideanDistances)
{
  // 3-4-5 right triangle
  EXPECT_NEAR(geom::distance_2d(3.0, 4.0), 5.0, 1e-9);

  // 1-2-2 vector in 3D -> sqrt(1 + 4 + 4) = 3.0
  EXPECT_NEAR(geom::distance_3d(1.0, 2.0, 2.0), 3.0, 1e-9);

  // Zero distance
  EXPECT_NEAR(geom::distance_3d(0.0, 0.0, 0.0), 0.0, 1e-9);
}

/// @brief Verifies desired_heading line-of-sight angle.
TEST(GeometryUtilsTest, DesiredHeading)
{
  constexpr double pi = M_PI;

  // Target due East: dx=1, dy=0 -> yaw = 0.0
  EXPECT_NEAR(geom::desired_heading(1.0, 0.0), 0.0, 1e-9);

  // Target due North: dx=0, dy=1 -> yaw = +PI/2
  EXPECT_NEAR(geom::desired_heading(0.0, 1.0), pi / 2.0, 1e-9);

  // Target due West: dx=-1, dy=0 -> yaw = PI
  EXPECT_NEAR(std::abs(geom::desired_heading(-1.0, 0.0)), pi, 1e-9);

  // Target due South: dx=0, dy=-1 -> yaw = -PI/2
  EXPECT_NEAR(geom::desired_heading(0.0, -1.0), -pi / 2.0, 1e-9);
}

/// @brief Verifies world frame error rotation into local body frame.
TEST(GeometryUtilsTest, WorldToBodyFrame)
{
  constexpr double pi = M_PI;
  double surge = 0.0;
  double sway = 0.0;

  // Case 1: Robot heading East (yaw = 0.0). Target dx=5.0, dy=2.0.
  // Body axes align with world axes: surge=5.0, sway=2.0.
  geom::world_to_body_frame(5.0, 2.0, 0.0, surge, sway);
  EXPECT_NEAR(surge, 5.0, 1e-9);
  EXPECT_NEAR(sway, 2.0, 1e-9);

  // Case 2: Robot heading North (yaw = PI/2). Target dx=5.0, dy=2.0.
  // In body frame: North is ahead (+surge=2.0), East is right (-sway=-5.0).
  geom::world_to_body_frame(5.0, 2.0, pi / 2.0, surge, sway);
  EXPECT_NEAR(surge, 2.0, 1e-9);
  EXPECT_NEAR(sway, -5.0, 1e-9);

  // Case 3: Robot heading South (yaw = -PI/2). Target dx=0.0, dy=-10.0 (directly South).
  // In body frame: Target is straight ahead -> surge=10.0, sway=0.0.
  geom::world_to_body_frame(0.0, -10.0, -pi / 2.0, surge, sway);
  EXPECT_NEAR(surge, 10.0, 1e-9);
  EXPECT_NEAR(sway, 0.0, 1e-9);
}

/// @brief Main entry point: runs unit tests.
int main(int argc, char **argv)
{
  testing::InitGoogleTest(&argc, argv);
  return RUN_ALL_TESTS();
}
