#ifndef SUBSEA_INSPECTION__COMMON__MANUAL_CONTROL_UTILS_HPP_
#define SUBSEA_INSPECTION__COMMON__MANUAL_CONTROL_UTILS_HPP_

namespace subsea_inspection::manual_control
{

/// @brief Converts ROS body-frame sway (+Y left) to MAVLink manual-control Y (+Y right).
/// @param ros_sway_mps ROS body-frame lateral velocity in meters per second.
/// @return Sign-adjusted MAVLink lateral axis value, in the same scaled units as the input.
inline double ros_sway_to_mavlink_y(double ros_sway_mps)
{
  return -ros_sway_mps;
}

}  // namespace subsea_inspection::manual_control

#endif  // SUBSEA_INSPECTION__COMMON__MANUAL_CONTROL_UTILS_HPP_
