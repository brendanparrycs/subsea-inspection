#include <termios.h>
#include <unistd.h>

#include <chrono>
#include <iostream>
#include <memory>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"

namespace
{
constexpr double kDefaultLinearSpeedMps = 0.5;
constexpr double kDefaultHeaveSpeedMps = 0.3;
constexpr double kDefaultAngularSpeedRadps = 0.5;
constexpr std::chrono::milliseconds kPollPeriod{100};
constexpr auto kDefaultCmdVelTopic = "/cmd_vel";
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
    angular_speed_(kDefaultAngularSpeedRadps)
  {
    // Declare and get parameters
    this->declare_parameter<double>("linear_speed", kDefaultLinearSpeedMps);
    this->declare_parameter<double>("heave_speed", kDefaultHeaveSpeedMps);
    this->declare_parameter<double>("angular_speed", kDefaultAngularSpeedRadps);
    this->declare_parameter<std::string>("cmd_vel_topic", kDefaultCmdVelTopic);

    linear_speed_ = this->get_parameter("linear_speed").as_double();
    heave_speed_ = this->get_parameter("heave_speed").as_double();
    angular_speed_ = this->get_parameter("angular_speed").as_double();
    const auto cmd_vel_topic = this->get_parameter("cmd_vel_topic").as_string();

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
    cmd_vel_publisher_->publish(current_twist_);
  }

  /// @brief Updates active twist velocity based on character pressed.
  /// @param c ASCII character received from standard input.
  void handle_key(char c)
  {
    switch (c) {
    // Surge (forward/backward)
    case 'w':
      current_twist_.linear.x = linear_speed_;
      break;
    case 's':
      current_twist_.linear.x = -linear_speed_;
      break;

    // Sway (left/right)
    case 'a':
      current_twist_.linear.y = -linear_speed_;
      break;
    case 'd':
      current_twist_.linear.y = linear_speed_;
      break;

    // Heave (ascend/descend)
    case 'i':
      current_twist_.linear.z = heave_speed_;
      break;
    case 'k':
      current_twist_.linear.z = -heave_speed_;
      break;

    // Yaw (counter-clockwise/clockwise)
    case 'j':
      current_twist_.angular.z = -angular_speed_;
      break;
    case 'l':
      current_twist_.angular.z = angular_speed_;
      break;

    // Full stop
    case ' ':
    case 'x':
      current_twist_ = geometry_msgs::msg::Twist();
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
