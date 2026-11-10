#include <chrono>
#include <cmath>
#include <cstddef>
#include <functional>
#include <memory>
#include <string>

#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "subsea_inspection/common/constants.hpp"
#include "tf2/LinearMath/Matrix3x3.hpp"
#include "tf2/LinearMath/Quaternion.hpp"

namespace
{
constexpr std::size_t kOdometryQueueDepth = 10;
constexpr std::chrono::milliseconds kStateLogThrottlePeriod{1000};
}  // namespace

/// @brief ROS 2 node for monitoring BlueROV2 odometry (position, altitude, speed) from Gazebo via
/// ros_gz_bridge.
class RobotStateMonitor : public rclcpp::Node
{
public:
  /// @brief Constructs node, declares `odom_topic` parameter, configures reliable QoS, and
  /// subscribes to odometry.
  RobotStateMonitor()
  : Node("robot_state_monitor")
  {
    // Declare and get parameters
    this->declare_parameter<std::string>("odom_topic", "/model/bluerov2_heavy/odometry");
    std::string odom_topic = this->get_parameter("odom_topic").as_string();

    // Standard reliable QoS matching Gazebo bridge publisher
    rclcpp::QoS qos_profile(kOdometryQueueDepth);
    qos_profile.reliability(rclcpp::ReliabilityPolicy::Reliable);
    qos_profile.durability(rclcpp::DurabilityPolicy::Volatile);

    // Create subscription
    subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, qos_profile,
      std::bind(&RobotStateMonitor::odometry_callback, this, std::placeholders::_1));

    RCLCPP_INFO(this->get_logger(), "RobotStateMonitor initialized. Subscribed to %s.",
                odom_topic.c_str());
  }

private:
  /// @brief Extracts position, converts quaternion to Euler angles, and logs state at 1 Hz.
  /// @param msg Shared pointer to incoming nav_msg/msg/Odometry message.
  void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    // Position (meters)
    const auto &position = msg->pose.pose.position;

    // Linear velocity (m/s)
    const auto &linear_velocity = msg->twist.twist.linear;

    // Convert Quaternion to Euler angles
    tf2::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                      msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);
    tf2::Matrix3x3 m(q);
    double roll, pitch, yaw;
    m.getRPY(roll, pitch, yaw);

    // Convert radians to degrees for logging
    double roll_deg = roll * subsea_inspection::constants::kRadiansToDegrees;
    double pitch_deg = pitch * subsea_inspection::constants::kRadiansToDegrees;
    double yaw_deg = yaw * subsea_inspection::constants::kRadiansToDegrees;

    RCLCPP_INFO_THROTTLE(
      this->get_logger(), *this->get_clock(), kStateLogThrottlePeriod.count(),
      "[State] Pos: [X: %.2f, Y: %.2f, Z: %.2f] m | RPY: [%.1f, %.1f, %.1f] deg | Speed: %.2f m/s",
      position.x, position.y, position.z, roll_deg, pitch_deg, yaw_deg,
      std::sqrt(linear_velocity.x * linear_velocity.x + linear_velocity.y * linear_velocity.y +
                linear_velocity.z * linear_velocity.z));
  }

  /// @brief Subscription handle retained to keep the odometry listener active over the node's
  /// lifecycle.
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr subscription_;
};

/// @brief Main entry point: initializes rclcpp, spins the RobotStateMonitor node, and cleanly shuts
/// down.
int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<RobotStateMonitor>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
