#ifndef SUBSEA_INSPECTION__OBSTACLE_GEOMETRY_HPP_
#define SUBSEA_INSPECTION__OBSTACLE_GEOMETRY_HPP_

#include <cmath>

namespace subsea_inspection
{

/// @brief Obstacle distance components relative to a current-to-target route segment.
struct RouteRelativeObstacle
{
  /// @brief Distance from the current position along the route toward the target, in meters.
  double along_track_m{0.0};

  /// @brief Signed lateral distance from the route in meters; positive is left of travel.
  double cross_track_m{0.0};

  /// @brief Euclidean distance from the current position to the obstacle, in meters.
  double distance_m{0.0};
};

/// @brief Projects an obstacle point in the route frame into along- and cross-track distances.
/// @param current_x Current position X coordinate in the route frame, in meters.
/// @param current_y Current position Y coordinate in the route frame, in meters.
/// @param target_x Target position X coordinate in the route frame, in meters.
/// @param target_y Target position Y coordinate in the route frame, in meters.
/// @param obstacle_x Obstacle X coordinate in the route frame, in meters.
/// @param obstacle_y Obstacle Y coordinate in the route frame, in meters.
/// @param[out] projection Output structure receiving the projected distances.
/// @return True if the route direction and all output distances are valid; false otherwise.
inline bool project_obstacle_to_route(double current_x, double current_y, double target_x,
                                      double target_y, double obstacle_x, double obstacle_y,
                                      RouteRelativeObstacle &projection)
{
  if (!std::isfinite(current_x) || !std::isfinite(current_y) || !std::isfinite(target_x) ||
      !std::isfinite(target_y) || !std::isfinite(obstacle_x) || !std::isfinite(obstacle_y))
  {
    return false;
  }

  const double route_x = target_x - current_x;
  const double route_y = target_y - current_y;
  const double route_length = std::hypot(route_x, route_y);
  if (route_length <= 1e-3) {
    return false;
  }

  const double unit_x = route_x / route_length;
  const double unit_y = route_y / route_length;
  const double obstacle_relative_x = obstacle_x - current_x;
  const double obstacle_relative_y = obstacle_y - current_y;
  projection.along_track_m = obstacle_relative_x * unit_x + obstacle_relative_y * unit_y;
  projection.cross_track_m = unit_x * obstacle_relative_y - unit_y * obstacle_relative_x;
  projection.distance_m = std::hypot(obstacle_relative_x, obstacle_relative_y);
  return std::isfinite(projection.along_track_m) && std::isfinite(projection.cross_track_m) &&
         std::isfinite(projection.distance_m);
}

}  // namespace subsea_inspection

#endif  // SUBSEA_INSPECTION__OBSTACLE_GEOMETRY_HPP_
