#ifndef SUBSEA_INSPECTION__STATION_KEEPING_HPP_
#define SUBSEA_INSPECTION__STATION_KEEPING_HPP_

#include "subsea_inspection/geometry_utils.hpp"
#include "subsea_inspection/pid_controller.hpp"

namespace subsea_inspection
{

/// @brief Body-frame velocity commands produced while holding a fixed pose.
struct StationKeepingCommand
{
  /// @brief Forward/backward velocity command in meters per second.
  double surge_mps{0.0};

  /// @brief Left/right velocity command in meters per second.
  double sway_mps{0.0};

  /// @brief Vertical velocity command in meters per second.
  double heave_mps{0.0};

  /// @brief Yaw rate command in radians per second.
  double yaw_rate_radps{0.0};
};

/// @brief Computes body-frame PID corrections for a fixed world-frame position and yaw.
/// @param position_error_world_m Target minus current position in world coordinates.
/// @param target_yaw_rad Desired vehicle yaw in radians.
/// @param current_yaw_rad Current vehicle yaw in radians.
/// @param dt_sec Time since the previous controller update in seconds.
/// @param surge_pid Forward/backward controller.
/// @param sway_pid Left/right controller using ROS body-frame convention (+Y left).
/// @param heave_pid Vertical controller.
/// @param yaw_pid Heading controller.
/// @return Velocity commands in ROS body-frame convention.
inline StationKeepingCommand compute_station_keeping_command(
  double error_x_world_m, double error_y_world_m, double error_z_world_m, double target_yaw_rad,
  double current_yaw_rad, double dt_sec, PidController &surge_pid, PidController &sway_pid,
  PidController &heave_pid, PidController &yaw_pid)
{
  double surge_error_m = 0.0;
  double sway_error_m = 0.0;
  geometry::world_to_body_frame(error_x_world_m, error_y_world_m, current_yaw_rad, surge_error_m,
                                sway_error_m);

  return {surge_pid.compute(surge_error_m, dt_sec), sway_pid.compute(sway_error_m, dt_sec),
          heave_pid.compute(error_z_world_m, dt_sec),
          yaw_pid.compute(geometry::angle_error(target_yaw_rad, current_yaw_rad), dt_sec)};
}

}  // namespace subsea_inspection

#endif  // SUBSEA_INSPECTION__STATION_KEEPING_HPP_
