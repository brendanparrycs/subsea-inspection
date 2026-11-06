#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>

#include "geometry_msgs/msg/twist.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "tf2/LinearMath/Matrix3x3.hpp"
#include "tf2/LinearMath/Quaternion.hpp"

#include "subsea_inspection/action/navigate_to_waypoint.hpp"
#include "subsea_inspection/geometry_utils.hpp"
#include "subsea_inspection/pid_controller.hpp"

using namespace std::chrono_literals;
using subsea_inspection::PidController;
namespace geometry = subsea_inspection::geometry;

using NavigateToWaypoint = subsea_inspection::action::NavigateToWaypoint;
using GoalHandleNavigate = rclcpp_action::ServerGoalHandle<NavigateToWaypoint>;

/// @brief Autonomous closed-loop waypoint navigation action server node for BlueROV2 Heavy.
class WaypointNavigator: public rclcpp::Node {
public:
    /// @brief Initializes node, sets up parameters, action server, subscribers, publishers, and control loop.
    WaypointNavigator(): 
        Node("waypoint_navigator"),
        target_x_(0.0),
        target_y_(0.0),
        target_z_(0.0),
        acceptance_radius_(1.0),
        has_odometry_(false),
        has_active_goal_(false)
    {
        // Default target coordinates / tolerances
        this->declare_parameter<double>("target_x", 5.0);
        this->declare_parameter<double>("target_y", 0.0);
        this->declare_parameter<double>("target_z", -2.0);
        this->declare_parameter<double>("acceptance_radius", 1.0);

        // Control gating parameters
        this->declare_parameter<double>("heading_tolerance_deg", 45.0);

        // Surge PID parameters
        this->declare_parameter<double>("surge_kp", 0.5);
        this->declare_parameter<double>("surge_ki", 0.02);
        this->declare_parameter<double>("surge_kd", 0.1);
        this->declare_parameter<double>("surge_max_speed", 0.6);

        // Heave PID parameters
        this->declare_parameter<double>("heave_kp", 0.6);
        this->declare_parameter<double>("heave_ki", 0.15);
        this->declare_parameter<double>("heave_kd", 0.1);
        this->declare_parameter<double>("heave_max_speed", 0.6);

        // Yaw PID parameters
        this->declare_parameter<double>("yaw_kp", 1.0);
        this->declare_parameter<double>("yaw_ki", 0.01);
        this->declare_parameter<double>("yaw_kd", 0.2);
        this->declare_parameter<double>("yaw_max_speed", 0.6);

        load_parameters();

        // Subscribe to odometry
        rclcpp::QoS qos_profile(10);
        qos_profile.reliability(rclcpp::ReliabilityPolicy::Reliable);
        qos_profile.durability(rclcpp::DurabilityPolicy::Volatile);

        odom_subscription_ = this->create_subscription<nav_msgs::msg::Odometry>(
            "/model/bluerov2_heavy/odometry",
            qos_profile,
            std::bind(&WaypointNavigator::odometry_callback, this, std::placeholders::_1)
        );

        // Publisher for velocity commands
        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // Create action server for /navigate_to_waypoint
        action_server_ = rclcpp_action::create_server<NavigateToWaypoint>(
            this,
            "navigate_to_waypoint",
            std::bind(&WaypointNavigator::handle_goal, this, std::placeholders::_1, std::placeholders::_2),
            std::bind(&WaypointNavigator::handle_cancel, this, std::placeholders::_1),
            std::bind(&WaypointNavigator::handle_accepted, this, std::placeholders::_1)
        );

        // 50ms (20Hz) control loop timer
        last_time_ = this->now();
        timer_ = this->create_wall_timer(50ms, std::bind(&WaypointNavigator::control_loop, this));

        RCLCPP_INFO(this->get_logger(), "WaypointNavigator action server initialized and ready.");
    }

private:
    /// @brief Loads parameters and initializes PID controllers.
    void load_parameters() {
        target_x_ = this->get_parameter("target_x").as_double();
        target_y_ = this->get_parameter("target_y").as_double();
        target_z_ = this->get_parameter("target_z").as_double();
        acceptance_radius_ = this->get_parameter("acceptance_radius").as_double();

        double heading_tolerance_deg = this->get_parameter("heading_tolerance_deg").as_double();
        heading_tolerance_rad_ = heading_tolerance_deg * (M_PI / 180.0);

        // Initialize PID controllers
        surge_pid_ = PidController(
            this->get_parameter("surge_kp").as_double(),
            this->get_parameter("surge_ki").as_double(),
            this->get_parameter("surge_kd").as_double(),
            this->get_parameter("surge_max_speed").as_double(),
            0.5
        );

        heave_pid_ = PidController(
            this->get_parameter("heave_kp").as_double(),
            this->get_parameter("heave_ki").as_double(),
            this->get_parameter("heave_kd").as_double(),
            this->get_parameter("heave_max_speed").as_double(),
            0.5
        );

        yaw_pid_ = PidController(
            this->get_parameter("yaw_kp").as_double(),
            this->get_parameter("yaw_ki").as_double(),
            this->get_parameter("yaw_kd").as_double(),
            this->get_parameter("yaw_max_speed").as_double(),
            0.5
        );
    }

    /// @brief Evaluates whether to accept or reject an incoming goal request.
    /// @param uuid Unique goal identifier.
    /// @param goal Shared pointer to requested navigation goal.
    /// @return Action goal response (ACCEPT_AND_EXECUTE or REJECT).
    rclcpp_action::GoalResponse handle_goal(
        const rclcpp_action::GoalUUID &uuid,
        std::shared_ptr<const NavigateToWaypoint::Goal> goal)
    {
        (void)uuid;
        RCLCPP_INFO(
            this->get_logger(),
            "Received goal request: [X: %.2f, Y: %.2f, Z: %.2f, Radius: %.2f]",
            goal->target_x,
            goal->target_y,
            goal->target_z,
            goal->acceptance_radius
        );

        if (goal->acceptance_radius <= 0.0) {
            RCLCPP_WARN(this->get_logger(), "Rejected goal: acceptance_radius must be > 0.0");
            return rclcpp_action::GoalResponse::REJECT;
        }

        return rclcpp_action::GoalResponse::ACCEPT_AND_EXECUTE;
    }

    /// @brief Accepts requests to cancel the active goal (e.g. from mission planner or safety monitor).
    /// @param goal_handle Handle to goal requesting cancellation.
    /// @return CancelResponse indicating goal cancellation was accepted.
    rclcpp_action::CancelResponse handle_cancel(
        const std::shared_ptr<GoalHandleNavigate> goal_handle)
    {
        (void)goal_handle;
        RCLCPP_INFO(this->get_logger(), "Received request to cancel active navigation goal.");
        return rclcpp_action::CancelResponse::ACCEPT;
    }

    /// @brief Starts execution of an accepted goal, resets PID state, and configures targets.
    /// @param goal_handle Handle to the newly accepted goal.
    void handle_accepted(const std::shared_ptr<GoalHandleNavigate> goal_handle) {
        // Preempt previous goal if one was already active
        if (has_active_goal_ && active_goal_handle_ && active_goal_handle_->is_active()) {
            auto prev_result = std::make_shared<NavigateToWaypoint::Result>();
            prev_result->success = false;
            active_goal_handle_->abort(prev_result);
            RCLCPP_WARN(this->get_logger(), "Preempted previous navigation goal with new goal.");
        }

        active_goal_handle_ = goal_handle;
        const auto goal = goal_handle->get_goal();

        // Update target coordinates from action goal
        target_x_ = goal->target_x;
        target_y_ = goal->target_y;
        target_z_ = goal->target_z;
        acceptance_radius_ = goal->acceptance_radius;

        // Reset PID integrators for a fresh step response
        surge_pid_.reset();
        heave_pid_.reset();
        yaw_pid_.reset();

        goal_start_time_ = this->now();
        has_active_goal_ = true;

        RCLCPP_INFO(
            this->get_logger(),
            "Pursuing waypoint: Target [X: %.2f, Y: %.2f, Z: %.2f, Radius: %.2f]",
            target_x_,
            target_y_,
            target_z_,
            acceptance_radius_
        );
    }

    /// @brief Extracts current 3D position and yaw heading from odometry message.
    /// @param msg Shared pointer to incoming nav_msgs/msg/Odometry message.
    void odometry_callback(const nav_msgs::msg::Odometry::SharedPtr msg) {
        current_x_ = msg->pose.pose.position.x;
        current_y_ = msg->pose.pose.position.y;
        current_z_ = msg->pose.pose.position.z;

        tf2::Quaternion q(
            msg->pose.pose.orientation.x,
            msg->pose.pose.orientation.y,
            msg->pose.pose.orientation.z,
            msg->pose.pose.orientation.w
        );

        tf2::Matrix3x3 m(q);
        double roll, pitch;
        m.getRPY(roll, pitch, current_yaw_);
        has_odometry_ = true;
    }

    /// @brief 20Hz control loop: calculates errors, computes PID outputs, and publishes velocity commands.
    void control_loop() {
        if (!has_odometry_) {
            RCLCPP_INFO_THROTTLE(this->get_logger(), *this->get_clock(), 2000, "Waiting for initial odometry...");
            return;
        }

        // When no goal is active, hold position by zeroing velocities
        if (!has_active_goal_) {
            geometry_msgs::msg::Twist stop_twist;
            cmd_vel_publisher_->publish(stop_twist);
            return;
        }

        // Handle cancellation request from action client
        if (active_goal_handle_->is_canceling()) {
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
        if (dt <= 0.0 || dt > 1.0) return;

        // Calculate world frame displacements
        double dx = target_x_ - current_x_;
        double dy = target_y_ - current_y_;
        double dz = target_z_ - current_z_;

        double dist_3d = geometry::distance_3d(dx, dy, dz);
        double dist_2d = geometry::distance_2d(dx, dy);

        // Check waypoint arrival
        if (dist_3d < acceptance_radius_) {
            geometry_msgs::msg::Twist stop_twist;
            cmd_vel_publisher_->publish(stop_twist);

            auto result = std::make_shared<NavigateToWaypoint::Result>();
            result->success = true;
            result->elapsed_time = (this->now() - goal_start_time_).seconds();
            result->final_distance = dist_3d;
            active_goal_handle_->succeed(result);
            has_active_goal_ = false;

            RCLCPP_INFO(
                this->get_logger(),
                ">> Waypoint Reached! Holding station at [X: %.2f, Y: %.2f, Z: %.2f] in %.2fs (error: %.2fm)",
                current_x_,
                current_y_,
                current_z_,
                result->elapsed_time,
                dist_3d
            );
            return;
        }

        // 1. Yaw Heading Guidance (Line-of-Sight)
        double desired_yaw = (dist_2d > 0.1) ? geometry::desired_heading(dx, dy) : current_yaw_;
        double yaw_error = geometry::angle_error(desired_yaw, current_yaw_);
        double cmd_yaw = yaw_pid_.compute(yaw_error, dt);

        // 2. Vertical Heave Guidance
        double cmd_heave = heave_pid_.compute(dz, dt);

        // 3. Forward Surge Guidance (with Turn-Before-Drive gating)
        // When close to waypoint (<1.5m), tighten tolerance to 25 deg to prevent orbiting
        double effective_tolerance = (dist_2d < 1.5) ? (25.0 * M_PI / 180.0) : heading_tolerance_rad_;
        double cmd_surge = 0.0;
        if (std::abs(yaw_error) < effective_tolerance) {
            // Smoothly scale speed as heading error drops to zero
            double alignment_factor = std::max(0.0, std::cos(yaw_error));
            cmd_surge = surge_pid_.compute(dist_2d, dt) * alignment_factor;
        }

        // Publish velocity command
        geometry_msgs::msg::Twist cmd_vel;
        cmd_vel.linear.x = cmd_surge;
        cmd_vel.linear.y = 0.0;
        cmd_vel.linear.z = cmd_heave;
        cmd_vel.angular.z = cmd_yaw;
        cmd_vel_publisher_->publish(cmd_vel);

        // Publish Action Feedback
        auto feedback = std::make_shared<NavigateToWaypoint::Feedback>();
        feedback->distance_remaining = dist_3d;
        feedback->heading_error_deg = yaw_error * (180.0 / M_PI);
        active_goal_handle_->publish_feedback(feedback);

        RCLCPP_INFO_THROTTLE(
            this->get_logger(),
            *this->get_clock(),
            1000,
            "[Nav Action] Dist: %.2f m (2D: %.2f, Z: %.2f) | YawErr: %.1f deg | Cmd: [Surge: %.2f, Heave: %.2f, Yaw: %.2f]",
            dist_3d, dist_2d, dz,
            yaw_error * (180.0 / M_PI),
            cmd_surge, cmd_heave, cmd_yaw
        );
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

    /// @brief Flag indicating an action goal is actively being pursued.
    bool has_active_goal_;

    /// @brief Timestamp when current goal began execution.
    rclcpp::Time goal_start_time_;

    /// @brief Timestamp of previous control cycle.
    rclcpp::Time last_time_;

    /// @brief PID controller for forward surge velocity.
    PidController surge_pid_;

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

/// @brief Main entry point: initializes rclcpp, spins the WaypointNavigator node, and cleanly shuts down.
int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<WaypointNavigator>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
