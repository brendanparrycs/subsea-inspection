#ifndef SUBSEA_INSPECTION__COMMON__TOPIC_NAMES_HPP_
#define SUBSEA_INSPECTION__COMMON__TOPIC_NAMES_HPP_

namespace subsea_inspection::topic_names
{

/// @brief Default velocity command topic.
inline constexpr char kDefaultCmdVel[] = "/cmd_vel";

/// @brief Default vehicle odometry topic.
inline constexpr char kDefaultOdometry[] = "/model/bluerov2_heavy/odometry";

/// @brief Default depth-camera point cloud topic.
inline constexpr char kDefaultPointCloud[] = "/model/bluerov2_heavy/depth_camera/points";

/// @brief Default closest-obstacle topic.
inline constexpr char kDefaultClosestObstacle[] = "/obstacle/closest_point";

/// @brief Default mission-completion topic.
inline constexpr char kDefaultMissionComplete[] = "/mission/complete";

}  // namespace subsea_inspection::topic_names

#endif  // SUBSEA_INSPECTION__COMMON__TOPIC_NAMES_HPP_
