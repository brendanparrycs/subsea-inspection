#include <gtest/gtest.h>

#include <cmath>
#include <limits>
#include <vector>

#include "subsea_inspection/point_cloud_filter.hpp"

using subsea_inspection::filter_corridor;
using subsea_inspection::Point3D;
using subsea_inspection::RoiBoundingBox;

/// @brief Verifies that an empty point cloud produces no detections and infinite distance.
TEST(PointCloudFilterTest, EmptyCloudReturnsInfinity)
{
  const std::vector<Point3D> empty_cloud;
  const RoiBoundingBox roi;

  const auto result = filter_corridor(empty_cloud, roi);

  EXPECT_TRUE(result.corridor_points.empty());
  EXPECT_FALSE(result.obstacle_detected);
  EXPECT_TRUE(std::isinf(result.min_distance));
}

/// @brief Verifies that non-finite points (NaN, Inf) are filtered out.
TEST(PointCloudFilterTest, RejectsInvalidPoints)
{
  std::vector<Point3D> noisy_cloud = {{std::numeric_limits<double>::quiet_NaN(), 0.0, 0.0},
                                      {1.0, std::numeric_limits<double>::infinity(), 0.0},
                                      {-std::numeric_limits<double>::infinity(), 0.0, 0.0}};
  const RoiBoundingBox roi;

  const auto result = filter_corridor(noisy_cloud, roi);

  EXPECT_TRUE(result.corridor_points.empty());
  EXPECT_FALSE(result.obstacle_detected);
}

/// @brief Verifies that points outside the corridor boundaries are excluded.
TEST(PointCloudFilterTest, RejectsPointsOutsideRoi)
{
  std::vector<Point3D> outside_cloud = {
    {0.1, 0.0, 0.0},   // Chassis self-reflection (x < min_x)
    {6.0, 0.0, 0.0},   // Beyond horizon (x > max_x)
    {2.0, 1.5, 0.0},   // Outside port boundary (y > max_y)
    {2.0, -1.5, 0.0},  // Outside starboard boundary (y < min_y)
    {2.0, 0.0, 1.0},   // Above vehicle corridor (z > max_z)
    {2.0, 0.0, -1.0}   // Below vehicle corridor (z < min_z)
  };
  const RoiBoundingBox roi;

  const auto result = filter_corridor(outside_cloud, roi);

  EXPECT_TRUE(result.corridor_points.empty());
  EXPECT_FALSE(result.obstacle_detected);
  EXPECT_TRUE(std::isinf(result.min_distance));
}

/// @brief Verifies that corridor points are retained and the minimum distance is calculated
/// correctly.
TEST(PointCloudFilterTest, RetainsPointsInsideRoiAndFindsMinimumDistance)
{
  std::vector<Point3D> cloud = {
    {1.0, 0.0, 0.0},   // In corridor, dist = 1.0m
    {2.0, 0.2, -0.1},  // In corridor, dist ≈ 2.01m
    {0.05, 0.0, 0.0},  // Chassis reflection (rejected)
    {2.5, 0.1, 0.0}    // In corridor, dist ≈ 2.5m
  };
  const RoiBoundingBox roi;

  const auto result = filter_corridor(cloud, roi, 1.5);

  EXPECT_EQ(result.corridor_points.size(), 3u);
  EXPECT_TRUE(result.obstacle_detected);
  EXPECT_NEAR(result.min_distance, 1.0, 1e-6);
  EXPECT_NEAR(result.closest_point.x, 1.0, 1e-6);
}

/// @brief Verifies that points beyond the warning threshold do not flag an immediate obstacle.
TEST(PointCloudFilterTest, RespectsDetectionThreshold)
{
  std::vector<Point3D> distant_cloud = {
    {2.0, 0.0, 0.0}  // Inside corridor (2.0m), but beyond 1.5m threshold
  };
  const RoiBoundingBox roi;

  const auto result = filter_corridor(distant_cloud, roi, 1.5);

  EXPECT_EQ(result.corridor_points.size(), 1u);
  EXPECT_FALSE(result.obstacle_detected);
  EXPECT_NEAR(result.min_distance, 2.0, 1e-6);
}
