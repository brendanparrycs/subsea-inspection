#ifndef SUBSEA_INSPECTION__MISSION_ROUTE_HPP_
#define SUBSEA_INSPECTION__MISSION_ROUTE_HPP_

#include <cmath>

namespace subsea_inspection
{

/// @brief Determines whether a vehicle has passed a waypoint along its route segment.
/// @param vehicle_x Current vehicle X position in the route frame.
/// @param vehicle_y Current vehicle Y position in the route frame.
/// @param previous_x Previous distinct waypoint X position.
/// @param previous_y Previous distinct waypoint Y position.
/// @param has_previous_waypoint Whether a previous distinct waypoint exists.
/// @param waypoint_x Active waypoint X position.
/// @param waypoint_y Active waypoint Y position.
/// @param next_x Next waypoint X position.
/// @param next_y Next waypoint Y position.
/// @param acceptance_radius_m Distance past the waypoint required before skipping it.
/// @param max_cross_track_m Maximum lateral distance from the route for a skip decision.
/// @return True if the vehicle is beyond the waypoint and within the route corridor.
inline bool has_passed_waypoint(double vehicle_x, double vehicle_y, double previous_x,
                                double previous_y, bool has_previous_waypoint, double waypoint_x,
                                double waypoint_y, double next_x, double next_y,
                                double acceptance_radius_m, double max_cross_track_m)
{
  if (!std::isfinite(vehicle_x) || !std::isfinite(vehicle_y) || !std::isfinite(waypoint_x) ||
      !std::isfinite(waypoint_y) || !std::isfinite(next_x) || !std::isfinite(next_y) ||
      (has_previous_waypoint && (!std::isfinite(previous_x) || !std::isfinite(previous_y))) ||
      !std::isfinite(acceptance_radius_m) || acceptance_radius_m <= 0.0 ||
      !std::isfinite(max_cross_track_m) || max_cross_track_m < 0.0)
  {
    return false;
  }

  const double route_dx = has_previous_waypoint ? waypoint_x - previous_x : next_x - waypoint_x;
  const double route_dy = has_previous_waypoint ? waypoint_y - previous_y : next_y - waypoint_y;
  const double route_length = std::hypot(route_dx, route_dy);
  if (route_length <= 0.0) {
    return false;
  }

  const double unit_x = route_dx / route_length;
  const double unit_y = route_dy / route_length;
  const double relative_x = vehicle_x - waypoint_x;
  const double relative_y = vehicle_y - waypoint_y;
  const double along_track = relative_x * unit_x + relative_y * unit_y;
  const double cross_track = std::abs(relative_x * unit_y - relative_y * unit_x);

  return along_track > acceptance_radius_m && cross_track <= max_cross_track_m;
}

}  // namespace subsea_inspection

#endif  // SUBSEA_INSPECTION__MISSION_ROUTE_HPP_
