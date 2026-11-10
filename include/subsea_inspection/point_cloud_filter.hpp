#ifndef SUBSEA_INSPECTION_POINT_CLOUD_FILTER_HPP_
#define SUBSEA_INSPECTION_POINT_CLOUD_FILTER_HPP_

#include <cmath>
#include <limits>
#include <vector>

namespace subsea_inspection
{

/// Default obstacle distance threshold used when no runtime configuration is provided.
inline constexpr double kDefaultDetectionThresholdM = 1.5;

/// @brief Represents a single 3D point in Cartesian coordinates.
struct Point3D
{
  double x{0.0};
  double y{0.0};
  double z{0.0};

  Point3D() = default;
  Point3D(double px, double py, double pz)
  : x(px),
    y(py),
    z(pz)
  {}

  /// @brief Computes Euclidean distance to coordinate origin (0, 0, 0).
  /// @return 3D distance from origin.
  double distance_to_origin() const
  {
    return std::sqrt(x * x + y * y + z * z);
  }

  /// @brief Checks if point coordinates are finite (rejects NaN or Inf).
  /// @return True if x, y, and z are valid finite numbers.
  bool is_valid() const
  {
    return std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
  }
};

/// @brief Defines a 3D bounding box Region of Interest (ROI) in vehicle body frame.
struct RoiBoundingBox
{
  double min_x{0.35};   // Front of vehicle chassis (clears chassis self-reflections)
  double max_x{3.00};   // Forward detection horizon
  double min_y{-0.45};  // Starboard safety corridor boundary
  double max_y{0.45};   // Port safety corridor boundary
  double min_z{-0.20};  // Bottom safety corridor boundary
  double max_z{0.30};   // Top safety corridor boundary

  /// @brief Tests if a given 3D point lies strictly within the bounding box.
  /// @param point Point to evaluate.
  /// @return True if point is valid and inside bounding box.
  bool contains(const Point3D &point) const
  {
    if (!point.is_valid())
      return false;
    return point.x >= min_x && point.x <= max_x && point.y >= min_y && point.y <= max_y &&
           point.z >= min_z && point.z <= max_z;
  }
};

/// @brief Summary reult of safety corridor filtering.
struct FilterResult
{
  std::vector<Point3D> corridor_points{};
  double min_distance{std::numeric_limits<double>::infinity()};
  Point3D closest_point{};
  bool obstacle_detected{false};
};

/// @brief Filters an array of 3D points against a forward safety corridor.
/// @param input_points Array of 3D spatial points.
/// @param roi Bounding box defining the safety corridor.
/// @param detection_threshold_m Maximum distance (meters) to trigger an obstacle warning.
/// @return FilterResult containing retained corridor points, closest point, and alert flag.
inline FilterResult filter_corridor(const std::vector<Point3D> &input_points,
                                    const RoiBoundingBox &roi,
                                    double detection_threshold_m = kDefaultDetectionThresholdM)
{
  FilterResult result;

  for (const auto &point : input_points) {
    if (roi.contains(point)) {
      result.corridor_points.push_back(point);
      double distance = point.distance_to_origin();

      if (distance < result.min_distance) {
        result.min_distance = distance;
        result.closest_point = point;
      }
    }
  }

  result.obstacle_detected =
    !result.corridor_points.empty() && result.min_distance <= detection_threshold_m;
  return result;
}

}  // namespace subsea_inspection

#endif  // SUBSEA_INSPECTION_POINT_CLOUD_FILTER_HPP_
