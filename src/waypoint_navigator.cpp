#include <algorithm>
#include <chrono>
#include <cmath>
#include <functional>
#include <limits>
#include <memory>
#include <stdexcept>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "subsea_inspection/action/navigate_to_waypoint.hpp"
#include "subsea_inspection/common/config_validation.hpp"
#include "subsea_inspection/common/constants.hpp"
#include "subsea_inspection/common/input_watchdog.hpp"
#include "subsea_inspection/common/topic_names.hpp"
#include "subsea_inspection/geometry_utils.hpp"
#include "subsea_inspection/pid_controller.hpp"
#include "subsea_inspection/station_keeping.hpp"
#include "tf2/LinearMath/Matrix3x3.hpp"
#include "tf2/LinearMath/Quaternion.hpp"

using namespace std::chrono_literals;
using subsea_inspection::PidController;
namespace geometry = subsea_inspection::geometry;

namespace
{
constexpr double kDefaultHeadingToleranceDeg = 20.0;

constexpr double kDefaultSurgeKp = 0.5;
constexpr double kDefaultSurgeKi = 0.02;
constexpr double kDefaultSurgeKd = 0.1;
constexpr double kDefaultSurgeMaxSpeedMps = 0.6;

constexpr double kDefaultSwayKp = 0.5;
constexpr double kDefaultSwayKi = 0.02;
constexpr double kDefaultSwayKd = 0.1;
constexpr double kDefaultSwayMaxSpeedMps = 0.6;

constexpr double kDefaultHeaveKp = 0.8;
constexpr double kDefaultHeaveKi = 0.01;
constexpr double kDefaultHeaveKd = 0.25;
constexpr double kDefaultHeaveMaxSpeedMps = 0.6;

constexpr double kDefaultYawKp = 1.6;
constexpr double kDefaultYawKi = 0.01;
constexpr double kDefaultYawKd = 0.25;
constexpr double kDefaultYawMaxSpeedMps = 0.6;
constexpr double kMinimumHeadingDistanceM = 0.1;
constexpr double kCloseRangeDistanceM = 0.6;
constexpr double kCloseRangeHeadingToleranceDeg = 20.0;
constexpr double kMaximumControlStepSec = 1.0;
constexpr double kPidIntegralLimit = 0.5;
constexpr double kDefaultOdometryTimeoutSec = 0.5;

/// @brief Validates PID gains and the corresponding maximum speed parameter.
/// @param kp Proportional gain.
/// @param ki Integral gain.
/// @param kd Derivative gain.
/// @param max_speed Maximum commanded speed.
/// @param axis_name Axis label used in validation errors.
void validate_pid_parameters(double kp, double ki, double kd, double max_speed,
                             const std::string &controller_name)
{
  if (!subsea_inspection::config_validation::are_valid_pid_gains(kp, ki, kd)) {
    throw std::invalid_argument(controller_name + " PID gains must be finite");
  }
  if (!subsea_inspection::config_validation::is_finite_nonnegative(max_speed)) {
    throw std::invalid_argument(controller_name + " maximum speed must be finite and non-negative");
  }
}
}  // namespace

using NavigateToWaypoint = subsea_inspection::action::NavigateToWaypoint;
using GoalHandleNavigate = rclcpp_action::ServerGoalHandle<NavigateToWaypoint>;

/// @brief Autonomous closed-loop waypoint navigation action server node for BlueROV2 Heavy.
class WaypointNavigator : public rclcpp::Node
{
public:
  /// @brief Initializes node, sets up parameters, action server, subscribers, publishers, and
  /// control loop.
  WaypointNavigator()
  : Node("waypoint_navigator"),
    target_x_(0.0),
    target_y_(0.0),
    target_z_(0.0),
    acceptance_radius_(0.0),
    has_odometry_(false),
    has_active_goal_(false),
    station_keeping_(false)
  {
    this->declare_parameter<std::string>("odom_topic",
                                         subsea_inspection::topic_names::kDefaultOdometry);
    this->declare_parameter<std::string>("cmd_vel_topic",
                                         subsea_inspection::topic_names::kDefaultCmdVel);
    this->declare_parameter<double>("odom_timeout", kDefaultOdometryTimeoutSec);
    odom_timeout_sec_ = this->get_parameter("odom_timeout").as_double();
    if (!subsea_inspection::config_validation::is_finite_positive(odom_timeout_sec_)) {
      throw std::invalid_argument("Parameter 'odom_timeout' must be finite and > 0");
    }

    // Control gating parameters
    this->declare_parameter<double>("heading_tolerance_deg", kDefaultHeadingToleranceDeg);

    // Surge PID parameters
    this->declare_parameter<double>("surge_kp", kDefaultSurgeKp);
    this->declare_parameter<double>("surge_ki", kDefaultSurgeKi);
    this->declare_parameter<double>("surge_kd", kDefaultSurgeKd);
    this->declare_parameter<double>("surge_max_speed", kDefaultSurgeMaxSpeedMps);

    // Sway PID parameters used by station keeping.
    this->declare_parameter<double>("sway_kp", kDefaultSwayKp);
    this->declare_parameter<double>("sway_ki", kDefaultSwayKi);
    this->declare_parameter<double>("sway_kd", kDefaultSwayKd);
    this->declare_parameter<double>("sway_max_speed", kDefaultSwayMaxSpeedMps);

    // Heave PID parameters
    this->declare_parameter<double>("heave_kp", kDefaultHeaveKp);
    this->declare_parameter<double>("heave_ki", kDefaultHeaveKi);
    this->declare_parameter<double>("heave_kd", kDefaultHeaveKd);
    this->declare_parameter<double>("heave_max_speed", kDefaultHeaveMaxSpeedMps);

    // Yaw PID parameters
    this->declare_parameter<double>("yaw_kp", kDefaultYawKp);
    this->declare_parameter<double>("yaw_ki", kDefaultYawKi);
    this->declare_parameter<double>("yaw_kd", kDefaultYawKd);
    this->declare_parameter<double>("yaw_max_speed", kDefaultYawMaxSpeedMps);

    load_parameters();
    const auto odom_topic = this->get_parameter("odom_topic").as_string();
    const auto cmd_vel_topic = this->get_parameter("cmd_vel_topic").as_string();

    // Subscribe to odometry
    rclcpp::QoS qos_profile(10);
    qos_profile.reliability(rclcpp::ReliabilityPolicy::Reliable);
    qos_profile.durability(rclcpp::DurabilityPolicy::Volatile);

    odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
      odom_topic, qos_profile,
      std::bind(&WaypointNavigator::odometry_callback, this, std::placeholders::_1));

    // Publisher for velocity commands
    cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic, 10);

    // Create action server for /navigate_to_waypoint
    action_server_ = rclcpp_action::create_server<NavigateToWaypoint>(
      this, "navigate_to_waypoint",
      std::bind(&WaypointNavigator::handle_goal, this, std::placeholders::_1,
                std::placeholders::_2),
      std::bind(&WaypointNavigator::handle_cancel, this, std::placeholders::_1),
      std::bind(&WaypointNavigator::handle_accepted, this, std::placeholders::_1));

    // 50ms (20Hz) control loop timer
    last_time_ = this->now();
    timer_ = this->create_wall_timer(50ms, std::bind(&WaypointNavigator::control_loop, this));

    RCLCPP_INFO(this->get_logger(), "WaypointNavigator action server initialized and ready.");
  }

private:
  /// @brief Loads parameters and initializes PID controllers.
  void load_parameters()
  {
    double heading_tolerance_deg = this->get_parameter("heading_tolerance_deg").as_double();
    if (!subsea_inspection::config_validation::is_valid_heading_tolerance(heading_tolerance_deg)) {
      throw std::invalid_argument("Parameter 'heading_tolerance_deg' must be in (0, 180]");
    }

    const double surge_kp = this->get_parameter("surge_kp").as_double();
    const double surge_ki = this->get_parameter("surge_ki").as_double();
    const double surge_kd = this->get_parameter("surge_kd").as_double();
    const double surge_max_speed = this->get_parameter("surge_max_speed").as_double();
    const double sway_kp = this->get_parameter("sway_kp").as_double();
    const double sway_ki = this->get_parameter("sway_ki").as_double();
    const double sway_kd = this->get_parameter("sway_kd").as_double();
    const double sway_max_speed = this->get_parameter("sway_max_speed").as_double();
    const double heave_kp = this->get_parameter("heave_kp").as_double();
    const double heave_ki = this->get_parameter("heave_ki").as_double();
    const double heave_kd = this->get_parameter("heave_kd").as_double();
    const double heave_max_speed = this->get_parameter("heave_max_speed").as_double();
    const double yaw_kp = this->get_parameter("yaw_kp").as_double();
    const double yaw_ki = this->get_parameter("yaw_ki").as_double();
    const double yaw_kd = this->get_parameter("yaw_kd").as_double();
    const double yaw_max_speed = this->get_parameter("yaw_max_speed").as_double();

    validate_pid_parameters(surge_kp, surge_ki, surge_kd, surge_max_speed, "Surge");
    validate_pid_parameters(sway_kp, sway_ki, sway_kd, sway_max_speed, "Sway");
    validate_pid_parameters(heave_kp, heave_ki, heave_kd, heave_max_speed, "Heave");
    validate_pid_parameters(yaw_kp, yaw_ki, yaw_kd, yaw_max_speed, "Yaw");

    heading_tolerance_rad_ =
      heading_tolerance_deg * subsea_inspection::constants::kDegreesToRadians;

    // Initialize PID controllers
    surge_pid_ = PidController(surge_kp, surge_ki, surge_kd, surge_max_speed, kPidIntegralLimit);
    sway_pid_ = PidController(sway_kp, sway_ki, sway_kd, sway_max_speed, kPidIntegralLimit);

    heave_pid_ = PidController(heave_kp, heave_ki, heave_kd, heave_max_speed, kPidIntegralLimit);

    yaw_pid_ = PidController(yaw_kp, yaw_ki, yaw_kd, yaw_max_speed, kPidIntegralLimit);
  }

  /// @brief Evaluates whether to accept or reject an incoming goal request.
  /// @param uuid Unique goal identifier.
  /// @param goal Shared pointer to requested navigation goal.
  /// @return Action goal response (ACCEPT_AND_EXECUTE or REJECT).
  rclcpp_action::GoalResponse handle_goal(const rclcpp_action::GoalUUID &uuid,
                                          std::shared_ptr<const NavigateToWaypoint::Goal> goal)
  {
    (void)uuid;
    RCLCPP_INFO(this->get_logger(),
                "Received goal request: [X: %.2f, Y: %.2f, Z: %.2f, Radius: %.2f]", goal->target_x,
                goal->target_y, goal->target_z, goal->acceptance_radius);

    if (!subsea_inspection::config_validation::are_valid_waypoint_coordinates(
          goal->target_x, goal->target_y, goal->target_z))
    {
      RCLCPP_WARN(this->get_logger(), "Rejected goal: target coordinates must be finite");
      return rclcpp_action::GoalResponse::REJECT;
    }

    if (!subsea_inspection::config_validation::is_finite_positive(goal->acceptance_radius)) {
      RCLCPP_WARN(this->get_logger(), "Rejected goal: acceptance_radius must be finite and > 0.0");
      return rclcpp_action::GoalResponse::REJECT;
    }

    return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
  }

  /// @brief Accepts requests to cancel the active goal (e.g. from mission planner or safety
  /// monitor).
  /// @param goal_handle Handle to goal requesting cancellation.
  /// @return CancelResponse indicating goal cancellation was accepted.
  rclcpp_action::CancelResponse handle_cancel(const std::shared_ptr<GoalHandleNavigate> goal_handle)
  {
    (void)goal_handle;
    RCLCPP_INFO(this->get_logger(), "Received request to cancel active navigation goal.");
    return rclcpp_action::CancelResponse::ACCEPT;
  }

  /// @brief Starts execution of an accepted goal, resets PID state, and configures targets.
  /// @param goal_handle Handle to the newly accepted goal.
  void handle_accepted(const std::shared_ptr<GoalHandleNavigate> goal_handle)
  {
    // Preempt previous goal if one was already active
    if (has_active_goal_ && active_goal_handle_ && active_goal_handle_->is_active()) {
      auto prev_result = std::make_shared<NavigateToWaypoint::Result>();
      prev_result->success = false;
      active_goal_handle_->abort(prev_result);
      RCLCPP_WARN(this->get_logger(), "Preempted previous navigation goal with new goal.");
    }

    active_goal_handle_ = goal_handle;
    last_odometry_time_ = std::chrono::steady_clock::now();
    const auto goal = goal_handle->get_goal();

    // Update target coordinates from action goal
    target_x_ = goal->target_x;
    target_y_ = goal->target_y;
    target_z_ = goal->target_z;
    acceptance_radius_ = goal->acceptance_radius;
    station_keeping_ = false;
    const double approach_dx = target_x_ - current_x_;
    const double approach_dy = target_y_ - current_y_;
    goal_approach_yaw_ =
      (geometry::distance_2d(approach_dx, approach_dy) > kMinimumHeadingDistanceM)
        ? geometry::desired_heading(approach_dx, approach_dy)
        : current_yaw_;

    // Reset PID integrators for a fresh step response
    surge_pid_.reset();
    sway_pid_.reset();
    heave_pid_.reset();
    yaw_pid_.reset();

    goal_start_time_ = this->now();
    has_active_goal_ = true;

    RCLCPP_INFO(this->get_logger(),
                "Pursuing waypoint: Target [X: %.2f, Y: %.2f, Z: %.2f, Radius: %.2f]", target_x_,
                target_y_, target_z_, acceptance_radius_);
  }

  /// @brief Extracts current 3D position and yaw heading from odometry message.
  /// @param msg Shared pointer to incoming nav_msgs/msg/Odometry message.
  void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg)
  {
    last_odometry_time_ = std::chrono::steady_clock::now();
    current_x_ = msg->pose.pose.position.x;
    current_y_ = msg->pose.pose.position.y;
    current_z_ = msg->pose.pose.position.z;

    tf2::Quaternion q(msg->pose.pose.orientation.x, msg->pose.pose.orientation.y,
                      msg->pose.pose.orientation.z, msg->pose.pose.orientation.w);

    tf2::Matrix3x3 m(q);
    double roll, pitch;
    m.getRPY(roll, pitch, current_yaw_);
    has_odometry_ = true;
  }

  /// @brief 20Hz control loop: calculates errors, computes PID outputs, and publishes velocity
  /// commands.
  void control_loop()
  {
    const bool odometry_stale = subsea_inspection::input_watchdog::is_stale(
      last_odometry_time_, std::chrono::steady_clock::now(),
      std::chrono::duration<double>(odom_timeout_sec_));
    if (has_odometry_ && odometry_stale && (has_active_goal_ || station_keeping_)) {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                            "Odometry exceeded %.2f s timeout; stopping navigation",
                            odom_timeout_sec_);
      geometry_msgs::msg::Twist stop_twist;
      cmd_vel_publisher_->publish(stop_twist);
      station_keeping_ = false;
      if (has_active_goal_ && active_goal_handle_ && active_goal_handle_->is_active()) {
        auto result = std::make_shared<NavigateToWaypoint::Result>();
        result->success = false;
        result->elapsed_time = (this->now() - goal_start_time_).seconds();
        result->final_distance = std::numeric_limits<double>::infinity();
        active_goal_handle_->abort(result);
      }
      has_active_goal_ = false;
      return;
    }

    if (!has_odometry_) {
      RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                           "Waiting for initial odometry...");
      return;
    }

    // Without a goal or a captured station-keeping target, remain neutral.
    if (!has_active_goal_ && !station_keeping_) {
      geometry_msgs::msg::Twist stop_twist;
      cmd_vel_publisher_->publish(stop_twist);
      return;
    }

    // Handle cancellation request from action client
    if (has_active_goal_ && active_goal_handle_->is_canceling()) {
      double dx = target_x_ - current_x_;
      double dy = target_y_ - current_y_;
      double dz = target_z_ - current_z_;
      double dist_3d = geometry::distance_3d(dx, dy, dz);

      auto result = std::make_shared<NavigateToWaypoint::Result>();
      result->success = false;
      result->elapsed_time = (this->now() - goal_start_time_).seconds();
      result->final_distance = dist_3d;
      active_goal_handle_->canceled(result);
      has_active_goal_ = false;

      geometry_msgs::msg::Twist stop_twist;
      cmd_vel_publisher_->publish(stop_twist);
      RCLCPP_INFO(this->get_logger(), "Navigation goal canceled by client.");
      return;
    }

    rclcpp::Time current_time = this->now();
    double dt = (current_time - last_time_).seconds();
    last_time_ = current_time;
    if (dt <= 0.0 || dt > kMaximumControlStepSec)
      return;

    // Calculate world frame displacements
    double dx = target_x_ - current_x_;
    double dy = target_y_ - current_y_;
    double dz = target_z_ - current_z_;

    double dist_3d = geometry::distance_3d(dx, dy, dz);
    double dist_2d = geometry::distance_2d(dx, dy);

    // Check waypoint arrival
    if (has_active_goal_ && dist_3d < acceptance_radius_) {
      auto result = std::make_shared<NavigateToWaypoint::Result>();
      result->success = true;
      result->elapsed_time = (this->now() - goal_start_time_).seconds();
      result->final_distance = dist_3d;

      // A zero velocity command only removes commanded thrust; it does not resist horizontal
      // current. Capture the achieved pose as a hold target and keep running the same closed-loop
      // controller until another action goal replaces it.
      target_x_ = current_x_;
      target_y_ = current_y_;
      target_z_ = current_z_;
      station_yaw_ = goal_approach_yaw_;
      station_keeping_ = true;
      surge_pid_.reset();
      sway_pid_.reset();
      heave_pid_.reset();
      yaw_pid_.reset();

      active_goal_handle_->succeed(result);
      has_active_goal_ = false;

      RCLCPP_INFO(this->get_logger(),
                  ">> Waypoint Reached! Station-keeping at [X: %.2f, Y: %.2f, Z: %.2f] after "
                  "%.2fs (error: %.2fm)",
                  current_x_, current_y_, current_z_, result->elapsed_time, dist_3d);
      return;
    }

    double cmd_surge = 0.0;
    double cmd_sway = 0.0;
    double cmd_heave = heave_pid_.compute(dz, dt);
    double cmd_yaw = 0.0;
    double yaw_error = 0.0;

    if (station_keeping_ && !has_active_goal_) {
      // Hold the captured yaw and correct position in the vehicle's body frame. Using sway
      // avoids rotating the inspection camera away from the pipeline just to fight lateral current.
      const auto command = subsea_inspection::compute_station_keeping_command(
        dx, dy, dz, station_yaw_, current_yaw_, dt, surge_pid_, sway_pid_, heave_pid_, yaw_pid_);
      cmd_surge = command.surge_mps;
      cmd_sway = command.sway_mps;
      cmd_heave = command.heave_mps;
      cmd_yaw = command.yaw_rate_radps;
      yaw_error = geometry::angle_error(station_yaw_, current_yaw_);
    } else {
      // Waypoint transit uses line-of-sight heading and turn-before-drive guidance.
      const double desired_yaw =
        (dist_2d > kMinimumHeadingDistanceM) ? geometry::desired_heading(dx, dy) : current_yaw_;
      yaw_error = geometry::angle_error(desired_yaw, current_yaw_);
      cmd_yaw = yaw_pid_.compute(yaw_error, dt);

      const double effective_tolerance =
        (dist_2d < kCloseRangeDistanceM)
          ? (kCloseRangeHeadingToleranceDeg * subsea_inspection::constants::kDegreesToRadians)
          : heading_tolerance_rad_;
      if (std::abs(yaw_error) < effective_tolerance) {
        const double cos_err = std::max(0.0, std::cos(yaw_error));
        const double alignment_factor = cos_err * cos_err;
        cmd_surge = surge_pid_.compute(dist_2d, dt) * alignment_factor;
      }
    }

    // Publish velocity command
    geometry_msgs::msg::Twist cmd_vel;
    cmd_vel.linear.x = cmd_surge;
    cmd_vel.linear.y = cmd_sway;
    cmd_vel.linear.z = cmd_heave;
    cmd_vel.angular.z = cmd_yaw;
    cmd_vel_publisher_->publish(cmd_vel);

    // Publish feedback only while an action goal is active. In station-keeping mode the same
    // controller resists current around the captured pose, but there is no active action client.
    if (has_active_goal_) {
      auto feedback = std::make_shared<NavigateToWaypoint::Feedback>();
      feedback->distance_remaining = dist_3d;
      feedback->heading_error_deg = yaw_error * subsea_inspection::constants::kRadiansToDegrees;
      active_goal_handle_->publish_feedback(feedback);
    }

    RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                         "[Nav Action] Dist: %.2f m (2D: %.2f, Z: %.2f) | YawErr: %.1f deg | Cmd: "
                         "[Surge: %.2f, Heave: %.2f, Yaw: %.2f]",
                         dist_3d, dist_2d, dz,
                         yaw_error * subsea_inspection::constants::kRadiansToDegrees, cmd_surge,
                         cmd_heave, cmd_yaw);
  }

  /// @brief Target X coordinate in world frame (meters).
  double target_x_;

  /// @brief Target Y coordinate in world frame (meters).
  double target_y_;

  /// @brief Target Z coordinate in world frame (meters).
  double target_z_;

  /// @brief Radius around target considered arrived (meters).
  double acceptance_radius_;

  /// @brief Maximum angular error to allow forward movement (radians).
  double heading_tolerance_rad_;

  /// @brief Current X position from odometry.
  double current_x_;

  /// @brief Current Y position from odometry.
  double current_y_;

  /// @brief Current Z position from odometry.
  double current_z_;

  /// @brief Current yaw orientation from odometry (radians).
  double current_yaw_;

  /// @brief Flag indicating odometry data has been received.
  bool has_odometry_;

  /// @brief Maximum wall-clock age of odometry before navigation stops.
  double odom_timeout_sec_{kDefaultOdometryTimeoutSec};

  /// @brief Wall-clock time at which the latest odometry message arrived.
  std::chrono::steady_clock::time_point last_odometry_time_{std::chrono::steady_clock::now()};

  /// @brief Flag indicating an action goal is actively being pursued.
  bool has_active_goal_;

  /// @brief Whether the controller should hold the most recently reached pose between goals.
  bool station_keeping_;

  /// @brief Yaw captured when station keeping begins, in radians.
  double station_yaw_{0.0};

  /// @brief Horizontal approach heading for the most recently accepted waypoint.
  double goal_approach_yaw_{0.0};

  /// @brief Timestamp when current goal began execution.
  rclcpp::Time goal_start_time_;

  /// @brief Timestamp of previous control cycle.
  rclcpp::Time last_time_;

  /// @brief PID controller for forward surge velocity.
  PidController surge_pid_;

  /// @brief PID controller for lateral sway during station keeping.
  PidController sway_pid_;

  /// @brief PID controller for vertical heave velocity.
  PidController heave_pid_;

  /// @brief PID controller for angular yaw velocity.
  PidController yaw_pid_;

  /// @brief Subscription handle for vehicle odometry.
  rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;

  /// @brief Publisher handle for velocity commands on /cmd_vel.
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;

  /// @brief Action server handle for /navigate_to_waypoint.
  rclcpp_action::Server<NavigateToWaypoint>::SharedPtr action_server_;

  /// @brief Currently active goal handle.
  std::shared_ptr<GoalHandleNavigate> active_goal_handle_;

  /// @brief 50ms (20Hz) periodic loop timer.
  rclcpp::TimerBase::SharedPtr timer_;
};

/// @brief Main entry point: initializes rclcpp, spins the WaypointNavigator node, and cleanly shuts
/// down.
int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<WaypointNavigator>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
