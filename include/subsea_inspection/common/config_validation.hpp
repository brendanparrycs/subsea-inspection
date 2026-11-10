#ifndef SUBSEA_INSPECTION_COMMON_CONFIG_VALIDATION_HPP_
#define SUBSEA_INSPECTION_COMMON_CONFIG_VALIDATION_HPP_

#include <cmath>
#include <string>

#include "subsea_inspection/point_cloud_filter.hpp"

namespace subsea_inspection::config_validation
{

inline bool is_finite_positive(double value)
{
  return std::isfinite(value) && value > 0.0;
}

inline bool is_finite_nonnegative(double value)
{
  return std::isfinite(value) && value >= 0.0;
}

inline bool is_valid_roi(const RoiBoundingBox &roi)
{
  return std::isfinite(roi.min_x) && std::isfinite(roi.max_x) && roi.min_x < roi.max_x &&
         std::isfinite(roi.min_y) && std::isfinite(roi.max_y) && roi.min_y < roi.max_y &&
         std::isfinite(roi.min_z) && std::isfinite(roi.max_z) && roi.min_z < roi.max_z;
}

inline bool is_valid_deadzone(double deadzone)
{
  return std::isfinite(deadzone) && deadzone >= 0.0 && deadzone < 1.0;
}

inline bool are_valid_joystick_axes(int surge, int sway, int heave, int yaw)
{
  return surge >= 0 && sway >= 0 && heave >= 0 && yaw >= 0;
}

inline bool are_valid_pid_gains(double kp, double ki, double kd)
{
  return std::isfinite(kp) && std::isfinite(ki) && std::isfinite(kd);
}

inline bool is_valid_heading_tolerance(double degrees)
{
  return std::isfinite(degrees) && degrees > 0.0 && degrees <= 180.0;
}

inline bool are_valid_waypoint_coordinates(double x, double y, double z)
{
  return std::isfinite(x) && std::isfinite(y) && std::isfinite(z);
}

inline bool is_valid_avoidance_strategy(const std::string &strategy)
{
  return strategy == "fly_over" || strategy == "lateral_bypass";
}

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

inline bool is_valid_mission_waypoint(double x, double y, double z, double acceptance_radius_m,
                                      double dwell_sec)
{
  return are_valid_waypoint_coordinates(x, y, z) && is_finite_positive(acceptance_radius_m) &&
         is_finite_nonnegative(dwell_sec);
}

}  // namespace subsea_inspection::config_validation

#endif  // SUBSEA_INSPECTION_COMMON_CONFIG_VALIDATION_HPP_
