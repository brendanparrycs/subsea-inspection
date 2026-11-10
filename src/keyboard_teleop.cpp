#include <termios.h>
#include <unistd.h>

#include <chrono>
#include <functional>
#include <iostream>
#include <memory>
#include <stdexcept>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "subsea_inspection/common/config_validation.hpp"
#include "subsea_inspection/common/input_watchdog.hpp"
#include "subsea_inspection/common/topic_names.hpp"

namespace
{
constexpr double kDefaultLinearSpeedMps = 0.5;
constexpr double kDefaultHeaveSpeedMps = 0.3;
constexpr double kDefaultAngularSpeedRadps = 0.5;
constexpr double kDefaultKeyTimeoutSec = 0.75;
constexpr std::chrono::milliseconds kPollPeriod{100};
}  // namespace

/// @brief Interactive terminal keyboard teleoperation node for BlueROV2 Heavy.
class KeyboardTeleop : public rclcpp::Node
{
public:
  /// @brief Initializes node, sets terminal to raw non-blocking mode, declares parameters, and
  /// starts control loop.
  KeyboardTeleop()
  : Node("keyboard_teleop"),
    linear_speed_(kDefaultLinearSpeedMps),
    heave_speed_(kDefaultHeaveSpeedMps),
    angular_speed_(kDefaultAngularSpeedRadps),
    key_timeout_sec_(kDefaultKeyTimeoutSec)
  {
    // Declare and get parameters
    this->declare_parameter<double>("linear_speed", kDefaultLinearSpeedMps);
    this->declare_parameter<double>("heave_speed", kDefaultHeaveSpeedMps);
    this->declare_parameter<double>("angular_speed", kDefaultAngularSpeedRadps);
    this->declare_parameter<double>("key_timeout", kDefaultKeyTimeoutSec);
    this->declare_parameter<std::string>("cmd_vel_topic",
                                         subsea_inspection::topic_names::kDefaultCmdVel);

    linear_speed_ = this->get_parameter("linear_speed").as_double();
    heave_speed_ = this->get_parameter("heave_speed").as_double();
    angular_speed_ = this->get_parameter("angular_speed").as_double();
    key_timeout_sec_ = this->get_parameter("key_timeout").as_double();
    const auto cmd_vel_topic = this->get_parameter("cmd_vel_topic").as_string();

    if (!subsea_inspection::config_validation::is_finite_nonnegative(linear_speed_) ||
        !subsea_inspection::config_validation::is_finite_nonnegative(heave_speed_) ||
        !subsea_inspection::config_validation::is_finite_nonnegative(angular_speed_))
    {
      throw std::invalid_argument("Keyboard teleoperation speeds must be finite and non-negative");
    }
    if (!subsea_inspection::config_validation::is_finite_positive(key_timeout_sec_)) {
      throw std::invalid_argument("Parameter 'key_timeout' must be finite and > 0");
    }

    // Publisher for velocity commands
    cmd_vel_publisher_ = this->create_publisher<geometry_msgs::msg::Twist>(cmd_vel_topic, 10);

    // Put terminal in raw non-blocking mode
    tcgetattr(STDIN_FILENO, &orig_termios_);
    struct termios raw = orig_termios_;
    raw.c_lflag &= ~(ECHO | ICANON);  // Disable echo and canonical mode
    raw.c_cc[VMIN] = 0;               // Non-blocking read
    raw.c_cc[VTIME] = 0;
    tcsetattr(STDIN_FILENO, TCSANOW, &raw);

    print_instructions();

    // 100ms (10Hz) loop to poll keystrokes and publish cmd_vel to satisfy motion_controller
    // watchdog
    timer_ = this->create_wall_timer(kPollPeriod, std::bind(&KeyboardTeleop::spin_once, this));
    RCLCPP_INFO(this->get_logger(), "KeyboardTeleop initialized. Ready for keyboard input.");
  }

  /// @brief Destructor: restores original terminal configuration upon shutdown.
  ~KeyboardTeleop() override
  {
    tcsetattr(STDIN_FILENO, TCSANOW, &orig_termios_);
  }

private:
  /// @brief Displays control keybindings in terminal output.
  void print_instructions()
  {
    std::cout << "\n============================================\n"
              << "  BlueROV2 Heavy Keyboard Teleoperation\n"
              << "============================================\n"
              << "  Surge (X) : [w] Forward    | [s] Backward\n"
              << "  Sway  (Y) : [a] Strafe Left | [d] Strafe Right\n"
              << "  Heave (Z) : [i] Ascend      | [k] Descend\n"
              << "  Yaw   (R) : [j] Turn Left   | [l] Turn Right\n"
              << "  Stop      : [Space] or [x] Full Stop\n"
              << "  Quit      : [q]\n"
              << "============================================\n\r" << std::flush;
  }

  /// @brief 10Hz timer loop: reads keystrokes without blocking and publishes active velocity
  /// command.
  void spin_once()
  {
    char c = '\0';
    if (read(STDIN_FILENO, &c, 1) > 0)
      handle_key(c);

    const auto now = std::chrono::steady_clock::now();
    if (has_active_key_ && subsea_inspection::input_watchdog::is_stale(
                             last_key_time_, now, std::chrono::duration<double>(key_timeout_sec_)))
    {
      current_twist_ = geometry_msgs::msg::Twist();
      has_active_key_ = false;
    }
    cmd_vel_publisher_->publish(current_twist_);
  }

  /// @brief Refreshes the key watchdog so movement stops after key-repeat events cease.
  void mark_motion_key()
  {
    last_key_time_ = std::chrono::steady_clock::now();
    has_active_key_ = true;
  }

  /// @brief Updates active twist velocity based on character pressed.
  /// @param c ASCII character received from standard input.
  void handle_key(char c)
  {
    switch (c) {
    // Surge (forward/backward)
    case 'w':
      mark_motion_key();
      current_twist_.linear.x = linear_speed_;
      break;
    case 's':
      mark_motion_key();
      current_twist_.linear.x = -linear_speed_;
      break;

    // Sway (left/right)
    case 'a':
      mark_motion_key();
      current_twist_.linear.y = linear_speed_;
      break;
    case 'd':
      mark_motion_key();
      current_twist_.linear.y = -linear_speed_;
      break;

    // Heave (ascend/descend)
    case 'i':
      mark_motion_key();
      current_twist_.linear.z = heave_speed_;
      break;
    case 'k':
      mark_motion_key();
      current_twist_.linear.z = -heave_speed_;
      break;

    // Yaw (counter-clockwise/clockwise)
    case 'j':
      mark_motion_key();
      current_twist_.angular.z = angular_speed_;
      break;
    case 'l':
      mark_motion_key();
      current_twist_.angular.z = -angular_speed_;
      break;

    // Full stop
    case ' ':
    case 'x':
      current_twist_ = geometry_msgs::msg::Twist();
      has_active_key_ = false;
      std::cout << ">> Stopped\r\n" << std::flush;
      break;

    // Quit
    case 'q':
      std::cout << ">> Exiting teleop...\r\n" << std::flush;
      current_twist_ = geometry_msgs::msg::Twist();
      cmd_vel_publisher_->publish(current_twist_);
      rclcpp::shutdown();
      break;

    default:
      break;
    }
  }

  /// @brief Target forward and lateral linear velocity in m/s.
  double linear_speed_;

  /// @brief Target vertical ascent/descent velocity in m/s.
  double heave_speed_;

  /// @brief Target yaw turn rate in rad/s.
  double angular_speed_;

  /// @brief Seconds without a repeated movement key before the command returns to zero.
  double key_timeout_sec_;

  /// @brief Wall-clock time at which the last movement key was received.
  std::chrono::steady_clock::time_point last_key_time_{};

  /// @brief Whether a movement key command is currently active.
  bool has_active_key_{false};

  /// @brief Saved terminal configuration to restore on shutdown.
  struct termios orig_termios_;

  /// @brief Current velocity command published continuously to satisfy safety watchdog.
  geometry_msgs::msg::Twist current_twist_;

  /// @brief Publisher handle for velocity commands on /cmd_vel.
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_publisher_;

  /// @brief 100ms (10Hz) periodic loop timer.
  rclcpp::TimerBase::SharedPtr timer_;
};

/// @brief Main entry point: initializes rclcpp, spins the KeyboardTeleop node, and cleanly shuts
/// down.
int main(int argc, char *argv[])
{
  rclcpp::init(argc, argv);

  auto node = std::make_shared<KeyboardTeleop>();
  rclcpp::spin(node);
  rclcpp::shutdown();

  return 0;
}
