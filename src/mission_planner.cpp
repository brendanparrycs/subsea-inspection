#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <exception>
#include <filesystem>
#include <functional>
#include <memory>
#include <stdexcept>
#include <string>
#include <vector>

#include "geometry_msgs/msg/point_stamped.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "std_msgs/msg/bool.hpp"
#include "subsea_inspection/action/navigate_to_waypoint.hpp"
#include "subsea_inspection/common/config_validation.hpp"
#include "subsea_inspection/common/topic_names.hpp"
#include "subsea_inspection/mission_route.hpp"
#include "subsea_inspection/obstacle_geometry.hpp"
#include "tf2/exceptions.hpp"
#include "tf2_geometry_msgs/tf2_geometry_msgs.hpp"
#include "tf2_ros/buffer.hpp"
#include "tf2_ros/transform_listener.hpp"
#include "yaml-cpp/yaml.h"

using namespace std::chrono_literals;
using NavigateToWaypoint = subsea_inspection::action::NavigateToWaypoint;
using GoalHandleNavigate = rclcpp_action::ClientGoalHandle<NavigateToWaypoint>;

namespace {
constexpr double kDefaultMissionAcceptanceRadiusM = 0.5;
constexpr double kDefaultInitialDepthM = -2.0;
constexpr double kDefaultAvoidanceHeaveClearanceM = 0.8;
constexpr double kDefaultAvoidanceLateralOffsetM = 0.8;
constexpr double kDefaultAvoidanceCooldownSec = 3.0;
constexpr double kDetourAcceptanceRadiusM = 0.6;
constexpr double kMinimumObstacleForwardDistanceM = 0.8;
constexpr double kDetourForwardClearanceM = 1.2;
constexpr double kSurfaceDepthLimitM = -0.5;
constexpr double kMinimumSegmentLengthM = 1e-3;
constexpr std::chrono::milliseconds kSequencerPeriod{100};
}  // namespace

/// @brief Represents a single 3D inspection waypoint along the pipeline route.
struct Waypoint {
    std::string name;
    double x{0.0};
    double y{0.0};
    double z{0.0};
    double acceptance_radius{kDefaultMissionAcceptanceRadiusM};
    double dwell_sec{0.0};
};

/// @brief Possible states of the mission sequencer state machine.
enum class MissionState {
    IDLE,
    SENDING_GOAL,
    NAVIGATING,
    DWELLING,
    AVOIDING,
    MISSION_COMPLETE,
    ABORTED
};

/// @brief High-level mission planner and sequencer with reactive obstacle avoidance for autonomous
/// pipeline inspection.
class MissionPlanner : public rclcpp::Node {
  public:
    /// @brief Initializes node, declares parameters, loads YAML mission, and sets up subscribers
    /// and action client.
    MissionPlanner()
    : Node("mission_planner"),
      state_(MissionState::IDLE),
      default_acceptance_radius_(kDefaultMissionAcceptanceRadiusM),
      current_waypoint_index_(0),
      total_waypoints_completed_(0),
      obstacle_avoidance_events_(0),
      current_x_(0.0),
      current_y_(0.0),
      current_z_(kDefaultInitialDepthM),
      is_detour_active_(false),
      has_avoided_once_(false) {
        tf_buffer_ = std::make_unique<tf2_ros::Buffer>(this->get_clock());
        tf_listener_ = std::make_shared<tf2_ros::TransformListener>(*tf_buffer_, this, false);

        // Declare mission and avoidance parameters
        this->declare_parameter<std::string>("mission_file", "");
        this->declare_parameter<std::string>("odom_topic",
                                             subsea_inspection::topic_names::kDefaultOdometry);
        this->declare_parameter<std::string>(
            "obstacle_topic", subsea_inspection::topic_names::kDefaultClosestObstacle);
        this->declare_parameter<std::string>(
            "mission_complete_topic", subsea_inspection::topic_names::kDefaultMissionComplete);
        this->declare_parameter<std::string>("avoidance_strategy", "fly_over");
        this->declare_parameter<double>("avoidance_heave_clearance",
                                        kDefaultAvoidanceHeaveClearanceM);
        this->declare_parameter<double>("avoidance_lateral_offset",
                                        kDefaultAvoidanceLateralOffsetM);
        this->declare_parameter<double>("avoidance_cooldown_sec", kDefaultAvoidanceCooldownSec);

        mission_file_ = this->get_parameter("mission_file").as_string();
        odom_topic_ = this->get_parameter("odom_topic").as_string();
        obstacle_topic_ = this->get_parameter("obstacle_topic").as_string();
        const auto mission_complete_topic =
            this->get_parameter("mission_complete_topic").as_string();
        avoidance_strategy_ = this->get_parameter("avoidance_strategy").as_string();
        avoidance_heave_clearance_ = this->get_parameter("avoidance_heave_clearance").as_double();
        avoidance_lateral_offset_ = this->get_parameter("avoidance_lateral_offset").as_double();
        avoidance_cooldown_sec_ = this->get_parameter("avoidance_cooldown_sec").as_double();

        if (!subsea_inspection::config_validation::is_valid_avoidance_strategy(
                avoidance_strategy_)) {
            throw std::invalid_argument(
                "Parameter 'avoidance_strategy' must be 'fly_over' or 'lateral_bypass'");
        }
        if (!subsea_inspection::config_validation::is_finite_nonnegative(avoidance_cooldown_sec_)) {
            throw std::invalid_argument(
                "Parameter 'avoidance_cooldown_sec' must be finite and >= 0");
        }
        if (avoidance_strategy_ == "fly_over" &&
            !subsea_inspection::config_validation::is_finite_positive(avoidance_heave_clearance_)) {
            throw std::invalid_argument(
                "Parameter 'avoidance_heave_clearance' must be finite and > 0 for fly_over");
        }
        if (avoidance_strategy_ == "lateral_bypass" &&
            !subsea_inspection::config_validation::is_finite_positive(avoidance_lateral_offset_)) {
            throw std::invalid_argument(
                "Parameter 'avoidance_lateral_offset' must be finite and > 0 for lateral_bypass");
        }

        if (mission_file_.empty()) {
            RCLCPP_ERROR(this->get_logger(), "Parameter 'mission_file' is empty! Cannot proceed.");
            state_ = MissionState::ABORTED;
            return;
        }

        // Load and parse mission YAML
        if (!load_mission_file(mission_file_)) {
            RCLCPP_ERROR(this->get_logger(), "Failed to load mission file: %s",
                         mission_file_.c_str());
            state_ = MissionState::ABORTED;
            return;
        }

        // Create action client for /navigate_to_waypoint
        action_client_ =
            rclcpp_action::create_client<NavigateToWaypoint>(this, "navigate_to_waypoint");

        // Odometry subscription to track vehicle's live world coordinates
        rclcpp::QoS odom_qos(10);
        odom_qos.reliability(rclcpp::ReliabilityPolicy::Reliable);
        odom_qos.durability(rclcpp::DurabilityPolicy::Volatile);
        odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            odom_topic_, odom_qos,
            std::bind(&MissionPlanner::odometry_callback, this, std::placeholders::_1));

        // Obstacle detector subscription for safety corridor intrusion alerts
        obstacle_subscription_ = this->create_subscription<geometry_msgs::msg::PointStamped>(
            obstacle_topic_, 10,
            std::bind(&MissionPlanner::obstacle_callback, this, std::placeholders::_1));

        // Publisher for mission completion notification
        mission_complete_publisher_ =
            this->create_publisher<std_msgs::msg::Bool>(mission_complete_topic, 10);

        // 100ms (10Hz) sequencer loop timer
        timer_ = this->create_wall_timer(kSequencerPeriod,
                                         std::bind(&MissionPlanner::sequencer_loop, this));

        mission_start_time_ = this->now();

        RCLCPP_INFO(this->get_logger(),
                    "MissionPlanner initialized. Strategy: '%s' (Heave clearance: %.2fm, Lateral "
                    "offset: %.2fm). Waiting for action server...",
                    avoidance_strategy_.c_str(), avoidance_heave_clearance_,
                    avoidance_lateral_offset_);
    }

  private:
    /// @brief Odometry callback keeping track of ROV live position in world frame.
    /// @param msg Incoming odometry message from Gazebo/localization.
    void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        odometry_frame_id_ = msg->header.frame_id;
        current_x_ = msg->pose.pose.position.x;
        current_y_ = msg->pose.pose.position.y;
        current_z_ = msg->pose.pose.position.z;
    }

    /// @brief Obstacle callback triggering reactive detour avoidance when an obstacle enters the
    /// corridor.
    /// @param msg Incoming closest obstacle point stamped in the sensor frame.
    void obstacle_callback(const geometry_msgs::msg::PointStamped::SharedPtr msg) {
        // Only react if actively navigating toward an inspection waypoint
        if (state_ != MissionState::NAVIGATING) {
            return;
        }

        // Avoid re-triggering during cooldown period
        if (has_avoided_once_) {
            double elapsed_since_avoidance = (this->now() - last_avoidance_time_).seconds();
            if (elapsed_since_avoidance < avoidance_cooldown_sec_) {
                return;
            }
        }

        if (!current_goal_handle_) {
            return;
        }

        if (odometry_frame_id_.empty()) {
            RCLCPP_ERROR(this->get_logger(),
                         "Cannot plan obstacle detour before the odometry frame is known");
            state_ = MissionState::ABORTED;
            action_client_->async_cancel_goal(current_goal_handle_);
            return;
        }

        geometry_msgs::msg::PointStamped obstacle_in_odometry_frame;
        try {
            obstacle_in_odometry_frame = tf_buffer_->transform(*msg, odometry_frame_id_);
        } catch (const tf2::TransformException &exception) {
            RCLCPP_ERROR(this->get_logger(), "Cannot transform obstacle point into '%s': %s",
                         odometry_frame_id_.c_str(), exception.what());
            state_ = MissionState::ABORTED;
            action_client_->async_cancel_goal(current_goal_handle_);
            return;
        }

        const auto &obstacle = obstacle_in_odometry_frame.point;
        const double obs_dist =
            std::hypot(obstacle.x - current_x_, obstacle.y - current_y_, obstacle.z - current_z_);
        RCLCPP_WARN(this->get_logger(),
                    "\n"
                    "========================================================================\n"
                    " [OBSTACLE DETECTED] Nearest point: %.2fm ahead [X: %.2f, Y: %.2f, Z: %.2f]m\n"
                    " Triggering reactive '%s' avoidance maneuver!\n"
                    "========================================================================",
                    obs_dist, obstacle.x, obstacle.y, obstacle.z, avoidance_strategy_.c_str());

        // Plan dynamic detour waypoint
        active_detour_waypoint_ = compute_detour_waypoint(obstacle);
        obstacle_avoidance_events_++;
        has_avoided_once_ = true;
        last_avoidance_time_ = this->now();

        // Switch to AVOIDING state and cancel current navigation goal
        state_ = MissionState::AVOIDING;
        is_detour_active_ = false;
        action_client_->async_cancel_goal(current_goal_handle_);
    }

    /// @brief Computes a 3D detour waypoint based on chosen avoidance strategy (Fly-Over vs Lateral
    /// Bypass).
    /// @param obstacle_point Detected obstacle coordinates in the odometry/world frame.
    /// @return 3D detour waypoint in world coordinates.
    Waypoint compute_detour_waypoint(const geometry_msgs::msg::Point &obstacle_point) {
        const auto &target_wp = waypoints_[current_waypoint_index_];
        Waypoint detour;
        detour.name = target_wp.name + "_detour";
        detour.acceptance_radius = kDetourAcceptanceRadiusM;
        detour.dwell_sec = 0.0;

        const double route_x = target_wp.x - current_x_;
        const double route_y = target_wp.y - current_y_;
        const double route_length = std::hypot(route_x, route_y);
        const double ux = (route_length > kMinimumSegmentLengthM) ? route_x / route_length : 1.0;
        const double uy = (route_length > kMinimumSegmentLengthM) ? route_y / route_length : 0.0;

        subsea_inspection::RouteRelativeObstacle route_projection;
        if (!subsea_inspection::project_obstacle_to_route(current_x_, current_y_, target_wp.x,
                                                          target_wp.y, obstacle_point.x,
                                                          obstacle_point.y, route_projection)) {
            RCLCPP_WARN(
                this->get_logger(),
                "Obstacle detour route geometry is degenerate; using minimum forward advance");
            route_projection = {kMinimumObstacleForwardDistanceM, 0.0, 0.0};
        }

        // Advance along the route past the obstacle with a clearance margin.
        const double advance_dist =
            std::max(kMinimumObstacleForwardDistanceM, route_projection.along_track_m) +
            kDetourForwardClearanceM;

        if (avoidance_strategy_ == "lateral_bypass") {
            // Lateral bypass: move perpendicularly to the line of transit
            // Positive cross-track is left of route travel, so bypass to the opposite side.
            const double side_sign = (route_projection.cross_track_m < 0.0) ? 1.0 : -1.0;
            detour.x =
                current_x_ + (ux * advance_dist) + (-uy * avoidance_lateral_offset_ * side_sign);
            detour.y =
                current_y_ + (uy * advance_dist) + (ux * avoidance_lateral_offset_ * side_sign);
            detour.z = current_z_;
        } else {
            // Industry standard: Vertical Fly-Over (+heave clearance)
            // Ascend to safely clear over obstacle; clamp to -0.5m so ROV does not breach surface
            detour.x = current_x_ + (ux * advance_dist);
            detour.y = current_y_ + (uy * advance_dist);
            detour.z = std::min(kSurfaceDepthLimitM, current_z_ + avoidance_heave_clearance_);
        }

        RCLCPP_INFO(
            this->get_logger(),
            "[Detour Planned] Current ROV: [%.2f, %.2f, %.2f] -> Detour: [%.2f, %.2f, %.2f]",
            current_x_, current_y_, current_z_, detour.x, detour.y, detour.z);

        return detour;
    }

    /// @brief Parses YAML mission definition into waypoint structs.
    /// @param filepath Absolute or relative path to mission YAML file.
    /// @return True if parsing succeeded and waypoints exist; false otherwise.
    bool load_mission_file(const std::string &filepath) {
        if (!std::filesystem::exists(filepath)) {
            RCLCPP_ERROR(this->get_logger(), "Mission file does not exist: %s", filepath.c_str());
            return false;
        }

        try {
            YAML::Node root = YAML::LoadFile(filepath);
            if (!root["mission"]) {
                RCLCPP_ERROR(this->get_logger(), "YAML root missing 'mission' key.");
                return false;
            }

            YAML::Node mission_node = root["mission"];
            mission_name_ = mission_node["name"].as<std::string>("unnamed_mission");
            default_acceptance_radius_ = mission_node["default_acceptance_radius"].as<double>(
                kDefaultMissionAcceptanceRadiusM);
            if (!subsea_inspection::config_validation::is_finite_positive(
                    default_acceptance_radius_)) {
                RCLCPP_ERROR(this->get_logger(),
                             "Mission default_acceptance_radius must be finite and > 0.");
                return false;
            }

            YAML::Node waypoints_node = mission_node["waypoints"];
            if (!waypoints_node || !waypoints_node.IsSequence()) {
                RCLCPP_ERROR(this->get_logger(), "'waypoints' key is missing or not a sequence.");
                return false;
            }

            waypoints_.clear();
            for (std::size_t i = 0; i < waypoints_node.size(); ++i) {
                YAML::Node wp = waypoints_node[i];
                Waypoint waypoint;
                waypoint.name = wp["name"].as<std::string>("wp_" + std::to_string(i + 1));
                waypoint.x = wp["x"].as<double>(0.0);
                waypoint.y = wp["y"].as<double>(0.0);
                waypoint.z = wp["z"].as<double>(kDefaultInitialDepthM);
                waypoint.acceptance_radius =
                    wp["acceptance_radius"].as<double>(default_acceptance_radius_);
                waypoint.dwell_sec = wp["dwell_sec"].as<double>(0.0);

                if (!subsea_inspection::config_validation::is_valid_mission_waypoint(
                        waypoint.x, waypoint.y, waypoint.z, waypoint.acceptance_radius,
                        waypoint.dwell_sec)) {
                    RCLCPP_ERROR(
                        this->get_logger(),
                        "Waypoint '%s' must have finite coordinates, positive acceptance radius, "
                        "and finite non-negative dwell_sec.",
                        waypoint.name.c_str());
                    return false;
                }

                waypoints_.push_back(waypoint);
            }

            RCLCPP_INFO(this->get_logger(), "Loaded mission '%s' with %zu waypoints from %s.",
                        mission_name_.c_str(), waypoints_.size(), filepath.c_str());

            return !waypoints_.empty();
        } catch (const YAML::Exception &e) {
            RCLCPP_ERROR(this->get_logger(), "YAML parsing error: %s", e.what());
            return false;
        }
    }

    /// @brief 100ms (10Hz) state machine loop managing mission execution lifecycle.
    void sequencer_loop() {
        switch (state_) {
        case MissionState::IDLE: {
            // Wait for action server
            if (!action_client_->wait_for_action_server(0s)) {
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000,
                                     "Waiting for /navigate_to_waypoint action server...");
                return;
            }

            RCLCPP_INFO(this->get_logger(), "Action server detected. Starting mission execution!");
            mission_start_time_ = this->now();
            state_ = MissionState::SENDING_GOAL;
            break;
        }

        case MissionState::SENDING_GOAL: {
            if (current_waypoint_index_ >= waypoints_.size()) {
                state_ = MissionState::MISSION_COMPLETE;
                return;
            }

            send_current_waypoint_goal();
            state_ = MissionState::NAVIGATING;
            break;
        }

        case MissionState::NAVIGATING: {
            // Monitored via action feedback and obstacle alerts
            break;
        }

        case MissionState::AVOIDING: {
            // Active during detour transit
            break;
        }

        case MissionState::DWELLING: {
            double elapsed_dwell = (this->now() - dwell_start_time_).seconds();
            const auto &wp = waypoints_[current_waypoint_index_];

            if (elapsed_dwell >= wp.dwell_sec) {
                RCLCPP_INFO(this->get_logger(),
                            "Dwell complete at '%s'. Advancing to next waypoint.", wp.name.c_str());

                current_waypoint_index_++;
                state_ = MissionState::SENDING_GOAL;
            } else {
                RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                                     "[Dwell] Paused at '%s' (%.1f / %.1f s)", wp.name.c_str(),
                                     elapsed_dwell, wp.dwell_sec);
            }

            break;
        }

        case MissionState::MISSION_COMPLETE: {
            print_mission_report();

            auto complete_msg = std_msgs::msg::Bool();
            complete_msg.data = true;
            mission_complete_publisher_->publish(complete_msg);

            timer_->cancel();
            break;
        }

        case MissionState::ABORTED: {
            RCLCPP_ERROR(this->get_logger(), "Mission aborted due to error.");
            timer_->cancel();
            break;
        }
        }
    }

    /// @brief Sends the active waypoint goal to the action server.
    void send_current_waypoint_goal() {
        const auto &wp = waypoints_[current_waypoint_index_];

        auto goal_msg = NavigateToWaypoint::Goal();
        goal_msg.target_x = wp.x;
        goal_msg.target_y = wp.y;
        goal_msg.target_z = wp.z;
        goal_msg.acceptance_radius = wp.acceptance_radius;

        RCLCPP_INFO(
            this->get_logger(),
            "\n>>> [Waypoint %zu/%zu] Dispatching '%s' -> Target: [X: %.2f, Y: %.2f, Z: %.2f, "
            "Radius: %.2f]",
            current_waypoint_index_ + 1, waypoints_.size(), wp.name.c_str(), wp.x, wp.y, wp.z,
            wp.acceptance_radius);

        auto send_goal_options = rclcpp_action::Client<NavigateToWaypoint>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            std::bind(&MissionPlanner::goal_response_callback, this, std::placeholders::_1);
        send_goal_options.feedback_callback = std::bind(
            &MissionPlanner::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
        send_goal_options.result_callback =
            std::bind(&MissionPlanner::result_callback, this, std::placeholders::_1);
        action_client_->async_send_goal(goal_msg, send_goal_options);
    }

    /// @brief Sends the dynamically planned obstacle detour waypoint to the action server.
    void send_detour_goal() {
        auto goal_msg = NavigateToWaypoint::Goal();
        goal_msg.target_x = active_detour_waypoint_.x;
        goal_msg.target_y = active_detour_waypoint_.y;
        goal_msg.target_z = active_detour_waypoint_.z;
        goal_msg.acceptance_radius = active_detour_waypoint_.acceptance_radius;

        RCLCPP_INFO(
            this->get_logger(),
            "\n>>> [Detour Maneuver] Dispatching '%s' -> Target: [X: %.2f, Y: %.2f, Z: %.2f, "
            "Radius: %.2f]",
            active_detour_waypoint_.name.c_str(), active_detour_waypoint_.x,
            active_detour_waypoint_.y, active_detour_waypoint_.z,
            active_detour_waypoint_.acceptance_radius);

        is_detour_active_ = true;

        auto send_goal_options = rclcpp_action::Client<NavigateToWaypoint>::SendGoalOptions();
        send_goal_options.goal_response_callback =
            std::bind(&MissionPlanner::goal_response_callback, this, std::placeholders::_1);
        send_goal_options.feedback_callback = std::bind(
            &MissionPlanner::feedback_callback, this, std::placeholders::_1, std::placeholders::_2);
        send_goal_options.result_callback =
            std::bind(&MissionPlanner::result_callback, this, std::placeholders::_1);
        action_client_->async_send_goal(goal_msg, send_goal_options);
    }

    /// @brief Callback when action server accepts or rejects goal.
    /// @param goal_handle Shared pointer to client goal handle.
    void goal_response_callback(const GoalHandleNavigate::SharedPtr &goal_handle) {
        if (!goal_handle) {
            RCLCPP_ERROR(this->get_logger(), "Goal was rejected by action server!");
            state_ = MissionState::ABORTED;
        } else {
            current_goal_handle_ = goal_handle;
            if (state_ == MissionState::AVOIDING) {
                RCLCPP_INFO(this->get_logger(),
                            "Detour goal accepted by waypoint navigator. Avoiding obstacle...");
            } else {
                RCLCPP_INFO(this->get_logger(),
                            "Goal accepted by waypoint navigator. In transit...");
            }
        }
    }

    /// @brief Periodic feedback callback during transit.
    /// @param feedback Shared pointer to incoming NavigateToWaypoint feedback.
    void feedback_callback(GoalHandleNavigate::SharedPtr,
                           const std::shared_ptr<const NavigateToWaypoint::Feedback> feedback) {
        RCLCPP_INFO_THROTTLE(
            this->get_logger(), *this->get_clock(), 1500,
            "[%s] Distance remaining: %.2f m | Heading error: %.1f deg",
            (state_ == MissionState::AVOIDING ? "Detour Transit" : "Inspection Transit"),
            feedback->distance_remaining, feedback->heading_error_deg);
    }

    /// @brief Terminal result callback when waypoint is reached, canceled, or fails.
    /// @param result Wrapped action result from action server.
    void result_callback(const GoalHandleNavigate::WrappedResult &result) {
        switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED: {
            if (state_ == MissionState::AVOIDING && is_detour_active_) {
                is_detour_active_ = false;

                // If detour carried vehicle past current waypoint, advance to next downstream
                // waypoint
                while (current_waypoint_index_ + 1 < waypoints_.size()) {
                    const auto &curr_wp = waypoints_[current_waypoint_index_];
                    const auto &next_wp = waypoints_[current_waypoint_index_ + 1];

                    double seg_dx = next_wp.x - curr_wp.x;
                    double seg_dy = next_wp.y - curr_wp.y;
                    double seg_len = std::hypot(seg_dx, seg_dy);
                    if (seg_len < kMinimumSegmentLengthM) {
                        current_waypoint_index_++;
                        continue;
                    }

                    // Find the previous distinct waypoint so route resumption uses the arrival
                    // direction. The first waypoint has no arrival leg and uses the outgoing
                    // segment instead.
                    double previous_x = curr_wp.x;
                    double previous_y = curr_wp.y;
                    bool has_previous_waypoint = false;
                    for (std::size_t previous_index = current_waypoint_index_; previous_index > 0;
                         --previous_index) {
                        const auto &previous_wp = waypoints_[previous_index - 1];
                        const double incoming_dx = curr_wp.x - previous_wp.x;
                        const double incoming_dy = curr_wp.y - previous_wp.y;
                        if (std::hypot(incoming_dx, incoming_dy) >= kMinimumSegmentLengthM) {
                            previous_x = previous_wp.x;
                            previous_y = previous_wp.y;
                            has_previous_waypoint = true;
                            break;
                        }
                    }

                    double max_cross_track_m = curr_wp.acceptance_radius + kDetourAcceptanceRadiusM;
                    if (avoidance_strategy_ == "lateral_bypass") {
                        max_cross_track_m += avoidance_lateral_offset_;
                    }

                    if (subsea_inspection::has_passed_waypoint(
                            current_x_, current_y_, previous_x, previous_y, has_previous_waypoint,
                            curr_wp.x, curr_wp.y, next_wp.x, next_wp.y, curr_wp.acceptance_radius,
                            max_cross_track_m)) {
                        RCLCPP_INFO(this->get_logger(),
                                    "[Route Resumption] Detour bypassed '%s'. Advancing to next "
                                    "waypoint '%s'.",
                                    curr_wp.name.c_str(), next_wp.name.c_str());
                        total_waypoints_completed_++;
                        current_waypoint_index_++;
                    } else {
                        break;
                    }
                }

                RCLCPP_INFO(
                    this->get_logger(),
                    "=== Detour '%s' Reached! Obstacle safely bypassed. Resuming route to '%s' ===",
                    active_detour_waypoint_.name.c_str(),
                    waypoints_[current_waypoint_index_].name.c_str());

                send_current_waypoint_goal();
                state_ = MissionState::NAVIGATING;
                break;
            }

            total_waypoints_completed_++;
            const auto &wp = waypoints_[current_waypoint_index_];

            RCLCPP_INFO(this->get_logger(),
                        "=== Reached '%s' in %.2f s (Final error: %.2f m) ===", wp.name.c_str(),
                        result.result->elapsed_time, result.result->final_distance);

            if (wp.dwell_sec > 0.0) {
                dwell_start_time_ = this->now();
                state_ = MissionState::DWELLING;
            } else {
                current_waypoint_index_++;
                state_ = MissionState::SENDING_GOAL;
            }

            break;
        }

        case rclcpp_action::ResultCode::CANCELED: {
            if (state_ == MissionState::AVOIDING && !is_detour_active_) {
                RCLCPP_INFO(
                    this->get_logger(),
                    "[Obstacle Avoidance] Original goal canceled. Dispatching detour goal now...");
                send_detour_goal();
            } else {
                RCLCPP_WARN(this->get_logger(), "Waypoint goal was canceled!");
                state_ = MissionState::ABORTED;
            }
            break;
        }

        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(this->get_logger(), "Waypoint goal was aborted by server!");
            state_ = MissionState::ABORTED;
            break;

        default:
            RCLCPP_ERROR(this->get_logger(), "Unknown action result code.");
            state_ = MissionState::ABORTED;
            break;
        }
    }

    /// @brief Prints formatted mission performance summary report.
    void print_mission_report() {
        double total_duration = (this->now() - mission_start_time_).seconds();
        RCLCPP_INFO(this->get_logger(), "==================================================");
        RCLCPP_INFO(this->get_logger(), "          SUBSEA MISSION COMPLETE REPORT          ");
        RCLCPP_INFO(this->get_logger(), "==================================================");
        RCLCPP_INFO(this->get_logger(), " Mission Name:              %s", mission_name_.c_str());
        RCLCPP_INFO(this->get_logger(), " Status:                    SUCCESS");
        RCLCPP_INFO(this->get_logger(), " Waypoints Visited:         %zu / %zu",
                    total_waypoints_completed_, waypoints_.size());
        RCLCPP_INFO(this->get_logger(), " Obstacle Avoidance Events: %zu",
                    obstacle_avoidance_events_);
        RCLCPP_INFO(this->get_logger(), " Avoidance Strategy Used:   %s",
                    avoidance_strategy_.c_str());
        RCLCPP_INFO(this->get_logger(), " Total Mission Time:        %.2f seconds", total_duration);
        RCLCPP_INFO(this->get_logger(), "==================================================");
    }

    /// @brief Path to the YAML mission configuration file.
    std::string mission_file_;

    /// @brief ROS 2 topic name for odometry feedback.
    std::string odom_topic_;

    /// @brief Coordinate frame declared by the incoming odometry messages.
    std::string odometry_frame_id_;

    /// @brief ROS 2 topic name for closest obstacle point alerts.
    std::string obstacle_topic_;

    /// @brief Active avoidance strategy: "fly_over" (default) or "lateral_bypass".
    std::string avoidance_strategy_;

    /// @brief Vertical heave clearance in meters for fly-over avoidance.
    double avoidance_heave_clearance_;

    /// @brief Lateral offset in meters for lateral bypass avoidance.
    double avoidance_lateral_offset_;

    /// @brief Minimum seconds between successive avoidance triggers to prevent oscillation.
    double avoidance_cooldown_sec_;

    /// @brief Current state of the sequencer state machine.
    MissionState state_;

    /// @brief Default acceptance radius for waypoints if not specified per waypoint.
    double default_acceptance_radius_;

    /// @brief Index of the currently active waypoint in the waypoints sequence.
    std::size_t current_waypoint_index_;

    /// @brief Counter tracking the total number of successfully completed waypoints.
    std::size_t total_waypoints_completed_;

    /// @brief Counter tracking the total number of dynamic obstacle avoidance events triggered.
    std::size_t obstacle_avoidance_events_;

    /// @brief Name identifier of the mission loaded from YAML.
    std::string mission_name_;

    /// @brief Ordered list of waypoints loaded from the mission file.
    std::vector<Waypoint> waypoints_;

    /// @brief Current live X position of the ROV in world coordinates.
    double current_x_;

    /// @brief Current live Y position of the ROV in world coordinates.
    double current_y_;

    /// @brief Current live Z position of the ROV in world coordinates.
    double current_z_;

    /// @brief Flag indicating if the currently active goal is an obstacle detour.
    bool is_detour_active_;

    /// @brief Flag indicating if at least one obstacle avoidance event has occurred.
    bool has_avoided_once_;

    /// @brief Dynamically computed detour waypoint for the active obstacle event.
    Waypoint active_detour_waypoint_;

    /// @brief Timestamp when mission execution began.
    rclcpp::Time mission_start_time_;

    /// @brief Timestamp when current dwell period began.
    rclcpp::Time dwell_start_time_;

    /// @brief Timestamp when last avoidance event occurred.
    rclcpp::Time last_avoidance_time_;

    /// @brief Active goal handle for sending and canceling waypoint navigation goals.
    GoalHandleNavigate::SharedPtr current_goal_handle_;

    /// @brief Action client handle for sending waypoint goals.
    rclcpp_action::Client<NavigateToWaypoint>::SharedPtr action_client_;

    /// @brief Odometry subscriber for live vehicle positioning.
    rclcpp::Subscription<nav_msgs::msg::Odometry>::SharedPtr odom_subscription_;

    /// @brief Obstacle subscriber for closest point detections in safety corridor.
    rclcpp::Subscription<geometry_msgs::msg::PointStamped>::SharedPtr obstacle_subscription_;

    /// @brief Publisher notifying downstream subsystems of mission completion.
    rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr mission_complete_publisher_;

    /// @brief 100ms (10Hz) periodic loop timer for the sequencer state machine.
    rclcpp::TimerBase::SharedPtr timer_;

    /// @brief TF buffer/listener used to convert obstacle points into the odometry frame.
    std::unique_ptr<tf2_ros::Buffer> tf_buffer_;

    std::shared_ptr<tf2_ros::TransformListener> tf_listener_;
};

/// @brief Main entry point: initializes rclcpp, spins the MissionPlanner node, and cleanly shuts
/// down.
int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<MissionPlanner>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
