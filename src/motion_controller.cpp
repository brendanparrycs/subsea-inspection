#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "mavros_msgs/msg/manual_control.hpp"
#include "mavros_msgs/msg/state.hpp"
#include "mavros_msgs/srv/command_bool.hpp"
#include "mavros_msgs/srv/set_mode.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{
constexpr double kDefaultLinearScale = 500.0;
constexpr double kDefaultAngularScale = 500.0;
constexpr double kDefaultCmdVelTimeoutSec = 0.5;
constexpr double kManualControlAxisLimit = 1000.0;
constexpr double kManualControlVerticalLimit = 500.0;
constexpr double kManualControlNeutralDepth = 500.0;
constexpr std::chrono::milliseconds kControlPeriod{50};
constexpr std::chrono::milliseconds kLogThrottlePeriod{2000};
constexpr std::size_t kQueueDepth = 10;
constexpr std::size_t kStateQueueDepth = 1;
constexpr auto kMavrosStateTopic = "/mavros/state";
constexpr auto kManualControlTopic = "/mavros/manual_control/send";
constexpr auto kSetModeService = "/mavros/set_mode";
constexpr auto kArmingService = "/mavros/cmd/arming";
constexpr auto kDefaultCmdVelTopic = "/cmd_vel";

void validate_motion_parameters(double linear_scale, double angular_scale,
                                double cmd_vel_timeout_sec)
{
  if (!std::isfinite(linear_scale) || linear_scale < 0.0 || !std::isfinite(angular_scale) ||
      angular_scale < 0.0)
  {
    throw std::invalid_argument("Motion controller scales must be finite and non-negative");
  }
  if (!std::isfinite(cmd_vel_timeout_sec) || cmd_vel_timeout_sec <= 0.0) {
    throw std::invalid_argument("Parameter 'cmd_vel_timeout' must be finite and > 0");
  }
}
}  // namespace

/// @brief ROS 2 node that controls BlueROV2 Heavy motion via MAVROS services and topics.
class MotionController : public rclcpp::Node
{
public:
  /// @brief Initializes node, sets up MAVROS subscribers, service clients, and control loop.
  MotionController()
  : Node("motion_controller"),
    last_cmd_vel_time_(0, 0, RCL_ROS_TIME)
  {
    // Declare and get parameters
    this->declare_parameter<double>("linear_scale", kDefaultLinearScale);
    this->declare_parameter<double>("angular_scale", kDefaultAngularScale);
    this->declare_parameter<double>("cmd_vel_timeout", kDefaultCmdVelTimeoutSec);
    this->declare_parameter<std::string>("cmd_vel_topic", kDefaultCmdVelTopic);

    linear_scale_ = this->get_parameter("linear_scale").as_double();
    angular_scale_ = this->get_parameter("angular_scale").as_double();
    cmd_vel_timeout_sec_ = this->get_parameter("cmd_vel_timeout").as_double();
    const auto cmd_vel_topic = this->get_parameter("cmd_vel_topic").as_string();

    validate_motion_parameters(linear_scale_, angular_scale_, cmd_vel_timeout_sec_);

    // QoS profile matching MAVROS state publisher
    rclcpp::QoS state_qos(kStateQueueDepth);
    state_qos.reliable();
    state_qos.transient_local();

    // Subscribe to MAVROS state
    state_subscription_ = this->create_subscription<mavros_msgs::msg::State>(
      kMavrosStateTopic, state_qos,
      std::bind(&MotionController::state_callback, this, std::placeholders::_1));

    // Subscribe to cmd_vel
    cmd_vel_subscription_ = this->create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_topic, kQueueDepth,
      std::bind(&MotionController::cmd_vel_callback, this, std::placeholders::_1));

    // Motion command publisher
    manual_control_publisher_ =
      this->create_publisher<mavros_msgs::msg::ManualControl>(kManualControlTopic, kQueueDepth);

    // Service clients for flight mode and motor arming
    set_mode_client_ = this->create_client<mavros_msgs::srv::SetMode>(kSetModeService);
    arming_client_ = this->create_client<mavros_msgs::srv::CommandBool>(kArmingService);

    timer_ =
      this->create_wall_timer(kControlPeriod, std::bind(&MotionController::control_loop, this));
    RCLCPP_INFO(this->get_logger(), "MotionController initialized. Awaiting MAVROS connection...");
  }

private:
  /// @brief Updates cached vehicle state (connection, armed status, flight mode).
  /// @param msg Shared pointer to incoming mavros_msgs/msg/State message.
  void state_callback(const mavros_msgs::msg::State::SharedPtr msg)
  {
    current_state_ = *msg;
  }

  /// @brief Updates target twist and watchdog timestamp.
  /// @param msg Shared pointer to incoming geometry_msgs/msg/Twist message.
  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    target_twist_ = *msg;
    last_cmd_vel_time_ = this->now();
  }

  /// @brief 50ms (20Hz) control loop: verifies state, sets ALT_HOLD, arms vehicle, and publishes
  /// motion commands.
  void control_loop()
  {
    // Wait for connection to ArduSub
    if (!current_state_.connected) {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), kLogThrottlePeriod.count(),
                           "Waiting for ArduSub connection...");
      return;
    }

    // Ensure vehicle is in ALT_HOLD mode
    if (current_state_.mode != "ALT_HOLD") {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), kLogThrottlePeriod.count(),
                           "Setting mode to ALT_HOLD...");
      set_mode("ALT_HOLD");
      return;
    }

    // Ensure vehicle thrusters are armed
    if (!current_state_.armed) {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), kLogThrottlePeriod.count(),
                           "Arming thrusters...");
      set_arm(true);
      return;
    }

    // Check if cmd_vel has timed out
    bool cmd_vel_active = (this->now() - last_cmd_vel_time_).seconds() < cmd_vel_timeout_sec_;

    float target_surge = cmd_vel_active ? static_cast<float>(target_twist_.linear.x) : 0.0f;
    float target_sway = cmd_vel_active ? static_cast<float>(target_twist_.linear.y) : 0.0f;
    float target_heave = cmd_vel_active ? static_cast<float>(target_twist_.linear.z) : 0.0f;
    float target_yaw = cmd_vel_active ? static_cast<float>(target_twist_.angular.z) : 0.0f;

    // Map to MAVROS range [-1000, 1000]; neutral depth is 500
    float x = static_cast<float>(
      std::clamp(target_surge * linear_scale_, -kManualControlAxisLimit, kManualControlAxisLimit));
    float y = static_cast<float>(
      std::clamp(target_sway * linear_scale_, -kManualControlAxisLimit, kManualControlAxisLimit));
    float z =
      static_cast<float>(kManualControlNeutralDepth + std::clamp(target_heave * linear_scale_,
                                                                 -kManualControlVerticalLimit,
                                                                 kManualControlVerticalLimit));
    float r = static_cast<float>(
      std::clamp(-target_yaw * angular_scale_, -kManualControlAxisLimit, kManualControlAxisLimit));

    send_control(x, y, z, r);
  }

  /// @brief Sends asynchronous request to change vehicle flight mode.
  /// @param mode String identifier of desired mode.
  void set_mode(const std::string &mode)
  {
    if (!set_mode_client_->service_is_ready())
      return;
    auto request = std::make_shared<mavros_msgs::srv::SetMode::Request>();
    request->custom_mode = mode;
    set_mode_client_->async_send_request(request);
  }

  /// @brief Sends asynchronous request to arm or disarm vehicle thrusters.
  /// @param arm True to arm, false to disarm.
  void set_arm(bool arm)
  {
    if (!arming_client_->service_is_ready())
      return;
    auto request = std::make_shared<mavros_msgs::srv::CommandBool::Request>();
    request->value = arm;
    arming_client_->async_send_request(request);
  }

  /// @brief Published normalized manual control command to MAVROS.
  /// @param x Surge: forward (+1000) to backward (-1000)
  /// @param y Sway: right (+1000) to left (-1000)
  /// @param z Throttle: neutral is 500 (depth hold), >500 ascends, <500 descends
  /// @param r Yaw: clockwise (+1000) to counter-clockwise (-1000)
  void send_control(float x, float y, float z, float r)
  {
    mavros_msgs::msg::ManualControl cmd;
    cmd.header.stamp = this->now();
    cmd.x = x;
    cmd.y = y;
    cmd.z = z;
    cmd.r = r;
    manual_control_publisher_->publish(cmd);
  }

  /// @brief Scaling factor converting linear velocity (m/s) to manual control throttle units.
  double linear_scale_;

  /// @brief Scaling factor converting angular velocity (rad/s) to manual control yaw units.
  double angular_scale_;

  /// @brief Watchdog timeout in seconds after which vehicle stops if no cmd_vel arrives.
  double cmd_vel_timeout_sec_;

  /// @brief Timestamp when the latest cmd_vel message was received.
  rclcpp::Time last_cmd_vel_time_;

  /// @brief Most recently received velocity command.
  geometry_msgs::msg::Twist target_twist_;

  /// @brief Cached vehicle state received from MAVROS.
  mavros_msgs::msg::State current_state_;

  /// @brief Subscription handle for MAVROS vehicle state.
  rclcpp::Subscription<mavros_msgs::msg::State>::SharedPtr state_subscription_;

  /// @brief Subscription handle for cmd_vel.
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_subscription_;

  /// @brief Publisher handle for manual motion commands.
  rclcpp::Publisher<mavros_msgs::msg::ManualControl>::SharedPtr manual_control_publisher_;

  /// @brief Service client for setting flight modes.
  rclcpp::Client<mavros_msgs::srv::SetMode>::SharedPtr set_mode_client_;

  /// @brief Service client for arming and disarming thrusters.
  rclcpp::Client<mavros_msgs::srv::CommandBool>::SharedPtr arming_client_;

  /// @brief 50ms (20Hz) control loop timer.
  rclcpp::TimerBase::SharedPtr timer_;
};

/// @brief Main entry point: initializes rclcpp, spins the MotionController node, and cleanly shuts
/// down.
int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<MotionController>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
