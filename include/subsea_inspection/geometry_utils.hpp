#ifndef SUBSEA_INSPECTION__GEOMETRY_UTILS_HPP_
#define SUBSEA_INSPECTION__GEOMETRY_UTILS_HPP_

#include <cmath>

namespace subsea_inspection
{

namespace geometry
{

/// @brief Wraps any angle in radians to the shortest equivalent interval [-PI, +PI].
/// @param angle_rad Angle in radians.
/// @return Normalized angle in range [-PI, +PI].
inline double wrap_angle(double angle_rad)
{
  return std::atan2(std::sin(angle_rad), std::cos(angle_rad));
}

/// @brief Calculates the shortest angular difference (target - current) wrapped to [-PI, +PI].
/// @param target_rad Desired angle in radians.
/// @param current_rad Current angle in radians.
/// @return Shortest angular error in range [-PI, +PI].
inline double angle_error(double target_rad, double current_rad)
{
  return wrap_angle(target_rad - current_rad);
}

/// @brief Computes 2D Euclidean distance in the horizontal (X-Y) plane.
/// @param dx Distance along X axis.
/// @param dy Distance along Y axis.
/// @return 2D horizontal distance.
inline double distance_2d(double dx, double dy)
{
  return std::sqrt(dx * dx + dy * dy);
}

/// @brief Computes 3D Euclidean distance in space.
/// @param dx Distance along X axis.
/// @param dy Distance along Y axis.
/// @param dz Distance along Z axis.
/// @return 3D spatial distance.
inline double distance_3d(double dx, double dy, double dz)
{
  return std::sqrt(dx * dx + dy * dy + dz * dz);
}

/// @brief Computes the desired line-of-sight heading angle toward a target point.
/// @param dx World frame X displacement (target.x - current.x).
/// @param dy World frame Y displacement (target.y - current.y).
/// @return Desired yaw angle in radians in range [-PI, +PI].
inline double desired_heading(double dx, double dy)
{
  return std::atan2(dy, dx);
}

/// @brief Transforms a 2D world frame error vector into the robot's local body frame (surge and
/// sway).
/// @param dx_world World frame displacement along X axis.
/// @param dy_world World frame displacement along Y axis.
/// @param current_yaw Current vehicle yaw heading in radians.
/// @param[out] surge_error Forward error along vehicle's longitudinal axis.
/// @param[out] sway_error Lateral error along vehicle's transverse axis.
inline void world_to_body_frame(double dx_world, double dy_world, double current_yaw,
                                double &surge_error, double &sway_error)
{
  double cos_yaw = std::cos(current_yaw);
  double sin_yaw = std::sin(current_yaw);

  surge_error = dx_world * cos_yaw + dy_world * sin_yaw;
  sway_error = -dx_world * sin_yaw + dy_world * cos_yaw;
}

}  // namespace geometry

}  // namespace subsea_inspection

#endif  // SUBSEA_INSPECTION__GEOMETRY_UTILS_HPP_
