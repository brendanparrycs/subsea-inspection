#ifndef SUBSEA_INSPECTION__COMMON__CONFIG_VALIDATION_HPP_
#define SUBSEA_INSPECTION__COMMON__CONFIG_VALIDATION_HPP_

#include <cmath>
#include <string>

#include "subsea_inspection/point_cloud_filter.hpp"

namespace subsea_inspection::config_validation
{

/// @brief Checks whether a value is finite and greater than zero.
/// @param value Value to check.
/// @return True if the value is finite and positive.
inline bool is_finite_positive(double value)
{
  return std::isfinite(value) && value > 0.0;
}

/// @brief Checks whether a value is finite and at least zero.
/// @param value Value to check.
/// @return True if the value is finite and nonnegative.
inline bool is_finite_nonnegative(double value)
{
  return std::isfinite(value) && value >= 0.0;
}

/// @brief Checks that a region of interest has finite, increasing bounds on every axis.
/// @param roi Region of interest to check.
/// @return True if all bounds are finite and each minimum is below its maximum.
inline bool is_valid_roi(const RoiBoundingBox &roi)
{
  return std::isfinite(roi.min_x) && std::isfinite(roi.max_x) && roi.min_x < roi.max_x &&
         std::isfinite(roi.min_y) && std::isfinite(roi.max_y) && roi.min_y < roi.max_y &&
         std::isfinite(roi.min_z) && std::isfinite(roi.max_z) && roi.min_z < roi.max_z;
}

/// @brief Checks that a joystick deadzone is in the range [0, 1).
/// @param deadzone Deadzone fraction to check.
/// @return True if the deadzone is finite and valid.
inline bool is_valid_deadzone(double deadzone)
{
  return std::isfinite(deadzone) && deadzone >= 0.0 && deadzone < 1.0;
}

/// @brief Checks that all joystick axis indices are nonnegative.
/// @param surge Surge axis index.
/// @param sway Sway axis index.
/// @param heave Heave axis index.
/// @param yaw Yaw axis index.
/// @return True if every index is nonnegative.
inline bool are_valid_joystick_axes(int surge, int sway, int heave, int yaw)
{
  return surge >= 0 && sway >= 0 && heave >= 0 && yaw >= 0;
}

/// @brief Checks that all PID gains are finite.
/// @param kp Proportional gain.
/// @param ki Integral gain.
/// @param kd Derivative gain.
/// @return True if every gain is finite.
inline bool are_valid_pid_gains(double kp, double ki, double kd)
{
  return std::isfinite(kp) && std::isfinite(ki) && std::isfinite(kd);
}

/// @brief Checks that a heading tolerance is in the range (0, 180] degrees.
/// @param degrees Heading tolerance to check.
/// @return True if the tolerance is finite and within range.
inline bool is_valid_heading_tolerance(double degrees)
{
  return std::isfinite(degrees) && degrees > 0.0 && degrees <= 180.0;
}

/// @brief Checks that all waypoint coordinates are finite.
/// @param x Waypoint X coordinate.
/// @param y Waypoint Y coordinate.
/// @param z Waypoint Z coordinate.
/// @return True if each coordinate is finite.
inline bool are_valid_waypoint_coordinates(double x, double y, double z)
{
  return std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
}

/// @brief Checks whether an avoidance strategy is supported.
/// @param strategy Strategy name to check.
/// @return True for `fly_over` or `lateral_bypass`.
inline bool is_valid_avoidance_strategy(const std::string &strategy)
{
  return strategy == "fly_over" || strategy == "lateral_bypass";
}

/// @brief Checks inspection geometry and qualification limits.
/// @param standoff_height_m Positive vehicle standoff height in meters.
/// @param bin_resolution_m Positive coverage-bin resolution in meters.
/// @param min_standoff_m Minimum permitted standoff in meters.
/// @param max_standoff_m Maximum permitted standoff in meters.
/// @param max_cross_track_m Maximum permitted lateral deviation in meters.
/// @param max_heading_error_deg Maximum permitted heading error in degrees.
/// @return True if all values are finite and within their permitted ranges.
inline bool is_valid_inspection_configuration(double standoff_height_m, double bin_resolution_m,
                                              double min_standoff_m, double max_standoff_m,
                                              double max_cross_track_m,
                                              double max_heading_error_deg)
{
  return is_finite_positive(standoff_height_m) && is_finite_positive(bin_resolution_m) &&
         is_finite_nonnegative(min_standoff_m) && std::isfinite(max_standoff_m) &&
         max_standoff_m >= min_standoff_m && is_finite_nonnegative(max_cross_track_m) &&
         std::isfinite(max_heading_error_deg) && max_heading_error_deg >= 0.0 &&
         max_heading_error_deg <= 180.0;
}

/// @brief Checks waypoint coordinates, acceptance radius, and dwell duration.
/// @param x Waypoint X coordinate.
/// @param y Waypoint Y coordinate.
/// @param z Waypoint Z coordinate.
/// @param acceptance_radius_m Positive arrival radius in meters.
/// @param dwell_sec Nonnegative dwell duration in seconds.
/// @return True if the waypoint values are valid.
inline bool is_valid_mission_waypoint(double x, double y, double z, double acceptance_radius_m,
                                      double dwell_sec)
{
  return are_valid_waypoint_coordinates(x, y, z) && is_finite_positive(acceptance_radius_m) &&
         is_finite_nonnegative(dwell_sec);
}

}  // namespace subsea_inspection::config_validation

#endif  // SUBSEA_INSPECTION__COMMON__CONFIG_VALIDATION_HPP_
