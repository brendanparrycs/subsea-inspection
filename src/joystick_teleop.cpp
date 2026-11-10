#include <algorithm>
#include <chrono>
#include <cmath>
#include <memory>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joy.hpp"

using namespace std::chrono_literals;

/// @brief Translates gamepad sensor_msgs/msg/Joy axis and button states into geometry_msgs/msg/Twist for BlueROV2 Heavy.
class JoystickTeleop: public rclcpp::Node {
public:
    /// @brief Initializes node, declares parameters, creates subscribers, publishers, and control loop timer.
    JoystickTeleop(): Node("joystick_teleop") {
        // Declare axis indices
        this->declare_parameter<int>("axis_surge", 1);
        this->declare_parameter<int>("axis_sway", 0);
        this->declare_parameter<int>("axis_heave", 4);
        this->declare_parameter<int>("axis_yaw", 3);

        // Declare axis inversion flags
        this->declare_parameter<bool>("invert_surge", false);
        this->declare_parameter<bool>("invert_sway", true);
        this->declare_parameter<bool>("invert_heave", false);
        this->declare_parameter<bool>("invert_yaw", true);

        // Declare deadzone and velocity limits
        this->declare_parameter<double>("deadzone", 0.10);
        this->declare_parameter<double>("max_linear_speed", 0.8);
        this->declare_parameter<double>("max_heave_speed", 0.5);
        this->declare_parameter<double>("max_angular_speed", 0.8);
        this->declare_parameter<double>("joy_timeout", 0.5);

        // Retrieve parameter values
        axis_surge_ = this->get_parameter("axis_surge").as_int();
        axis_sway_ = this->get_parameter("axis_sway").as_int();
        axis_heave_ = this->get_parameter("axis_heave").as_int();
        axis_yaw_ = this->get_parameter("axis_yaw").as_int();

        invert_surge_ = this->get_parameter("invert_surge").as_bool();
        invert_sway_ = this->get_parameter("invert_sway").as_bool();
        invert_heave_ = this->get_parameter("invert_heave").as_bool();
        invert_yaw_ = this->get_parameter("invert_yaw").as_bool();

        deadzone_ = this->get_parameter("deadzone").as_double();
        max_linear_speed_ = this->get_parameter("max_linear_speed").as_double();
        max_heave_speed_ = this->get_parameter("max_heave_speed").as_double();
        max_angular_speed_ = this->get_parameter("max_angular_speed").as_double();
        joy_timeout_sec_ = this->get_parameter("joy_timeout").as_double();

        // Subscribe to /joy topic from joy_node
        joy_subscription_ = this->create_subscription<sensor_msgs::msg::Joy>(
            "/joy",
            10,
            std::bind(&JoystickTeleop::joy_callback, this, std::placeholders::_1)
        );

        // Publisher for velocity commands
        cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("/cmd_vel", 10);

        // 100ms (10Hz) timer loop to publish cmd_vel to satisfy motion_controller watchdog
        timer_ = this->create_wall_timer(100ms, std::bind(&JoystickTeleop::publish_twist, this));

        RCLCPP_INFO(this->get_logger(), "JoystickTeleop initialized. Awaiting /joy messages...");
    }

private:
    /// @brief Filters stick jitter below deadzone threshold.
    /// @param value Raw axis value in range [-1.0, 1.0].
    /// @return Deadzone-filtered axis value.
    double apply_deadzone(double value) const {
        return (std::abs(value) < deadzone_) ? 0.0 : value;
    }

    /// @brief Safely extracts an axis value from Joy message with deadzone filtering and optional sign inversion.
    /// @param msg Shared pointer to incoming sensor_msgs/msg/Joy message.
    /// @param axis_index Axis array index to read.
    /// @param invert True to invert the axis sign.
    /// @return Filtered, normalized axis value in range [-1.0, 1.0].
    double get_axis_value(const sensor_msgs::msg::Joy::SharedPtr &msg, int axis_index, bool invert) const {
        if (axis_index < 0 || static_cast<size_t>(axis_index) >= msg->axes.size()) {
            return 0.0;
        }

        double val = apply_deadzone(msg->axes[axis_index]);
        return invert ? -val : val;
    }

    /// @brief Processes incoming Joy messages and maps controller axes to target velocities.
    /// @param msg Shared pointer to incoming sensor_msgs/msg/Joy message.
    void joy_callback(const sensor_msgs::msg::Joy::SharedPtr msg) {
        current_twist_.linear.x = get_axis_value(msg, axis_surge_, invert_surge_) * max_linear_speed_;
        current_twist_.linear.y = get_axis_value(msg, axis_sway_, invert_sway_) * max_linear_speed_;
        current_twist_.linear.z = get_axis_value(msg, axis_heave_, invert_heave_) * max_heave_speed_;
        current_twist_.angular.z = get_axis_value(msg, axis_yaw_, invert_yaw_) * max_angular_speed_;
        last_joy_message_time_ = std::chrono::steady_clock::now();
        has_received_joy_message_ = true;
    }

    /// @brief 10Hz periodic timer callback that publishes current velocity commands on /cmd_vel.
    void publish_twist() {
        const auto elapsed = std::chrono::steady_clock::now() - last_joy_message_time_;
        if (!has_received_joy_message_ || elapsed >= std::chrono::duration<double>(joy_timeout_sec_)) {
            cmd_vel_publisher_->publish(geometry_msgs::msg::Twist());
            return;
        }

        cmd_vel_publisher_->publish(current_twist_);
    }

    /// @brief Axis index for forward/backward surge motion.
    int axis_surge_;

    /// @brief Axis index for lateral sway motion.
    int axis_sway_;

    /// @brief Axis index for vertical heave motion.
    int axis_heave_;

    /// @brief Axis index for rotational yaw motion.
    int axis_yaw_;

    /// @brief Sign inversion flag for surge axis.
    bool invert_surge_;

    /// @brief Sign inversion flag for sway axis.
    bool invert_sway_;

    /// @brief Sign inversion flag for heave axis.
    bool invert_heave_;

    /// @brief Sign inversion flag for yaw axis.
    bool invert_yaw_;

    /// @brief Minimum magnitude required on an axis to register input.
    double deadzone_;

    /// @brief Maximum forward and lateral linear velocity in m/s.
    double max_linear_speed_;

    /// @brief Maximum vertical ascent/descent velocity in m/s.
    double max_heave_speed_;

    /// @brief Maximum yaw turn rate in rad/s.
    double max_angular_speed_;

    /// @brief Maximum age in seconds allowed for joystick input before commands are stopped.
    double joy_timeout_sec_;

    /// @brief Wall-clock time when the most recent joystick message was received.
    std::chrono::steady_clock::time_point last_joy_message_time_{};

    /// @brief Whether at least one joystick message has been received.
    bool has_received_joy_message_{false};

    /// @brief Current velocity command published continuously to satisfy safety watchdog.
    geometry_msgs::msg::Twist current_twist_;

    /// @brief Subscription handle for gamepad input on /joy.
    rclcpp::Subscription<sensor_msgs::msg::Joy>::SharedPtr joy_subscription_;

    /// @brief Publisher handle for velocity commands on /cmd_vel.
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;

    /// @brief 100ms (10Hz) periodic loop timer.
    rclcpp::TimerBase::SharedPtr timer_;
};

/// @brief Main entry point: initializes rclcpp, spins the JoystickTeleop node, and cleanly shuts down.
int main(int argc, char *argv[]) {
    rclcpp::init(argc, argv);

    auto node = std::make_shared<JoystickTeleop>();
    rclcpp::spin(node);
    rclcpp::shutdown();

    return 0;
}
