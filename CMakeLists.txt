cmake_minimum_required(VERSION 3.28)
project(subsea_inspection)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(yaml-cpp REQUIRED)

# Generate custom ROS 2 action interfaces
rosidl_generate_interfaces(${PROJECT_NAME}
    "action/NavigateToWaypoint.action"
)

# Include project headers
include_directories(include)

# Build the robot_state_monitor executable
add_executable(robot_state_monitor src/robot_state_monitor.cpp)
target_link_libraries(robot_state_monitor
    rclcpp::rclcpp
    nav_msgs::nav_msgs
    geometry_msgs::geometry_msgs
    tf2::tf2
    tf2_geometry_msgs::tf2_geometry_msgs
)

# Build the motion_controller executable
add_executable(motion_controller src/motion_controller.cpp)
target_link_libraries(motion_controller
    rclcpp::rclcpp
    mavros_msgs::mavros_msgs
    geometry_msgs::geometry_msgs
)

# Build the keyboard_teleop executable
add_executable(keyboard_teleop src/keyboard_teleop.cpp)
target_link_libraries(keyboard_teleop
    rclcpp::rclcpp
    geometry_msgs::geometry_msgs
)

# Build the joystick_teleop executable
add_executable(joystick_teleop src/joystick_teleop.cpp)
target_link_libraries(joystick_teleop
    rclcpp::rclcpp
    geometry_msgs::geometry_msgs
    sensor_msgs::sensor_msgs
)

# Build the waypoint_navigator executable
add_executable(waypoint_navigator src/waypoint_navigator.cpp)
target_link_libraries(waypoint_navigator
    rclcpp::rclcpp
    rclcpp_action::rclcpp_action
    geometry_msgs::geometry_msgs
    nav_msgs::nav_msgs
    tf2::tf2
    tf2_geometry_msgs::tf2_geometry_msgs
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)
target_link_libraries(waypoint_navigator "${cpp_typesupport_target}")

# Build the mission_planner executable
add_executable(mission_planner src/mission_planner.cpp)
target_link_libraries(mission_planner
    rclcpp::rclcpp
    rclcpp_action::rclcpp_action
    yaml-cpp
    "${cpp_typesupport_target}"
)

# Install executables to lib/subsea_inspection
install(TARGETS
    robot_state_monitor
    motion_controller
    keyboard_teleop
    joystick_teleop
    waypoint_navigator
    mission_planner
    DESTINATION lib/${PROJECT_NAME}
)

# Install headers
install(DIRECTORY include/
    DESTINATION include
)

# Install launch, config, and models directories
install(DIRECTORY 
    launch
    config
    models
    DESTINATION share/${PROJECT_NAME}
    OPTIONAL
)

# Automated testing
if(BUILD_TESTING)
    find_package(ament_lint_auto REQUIRED)
    find_package(ament_cmake_gtest REQUIRED)

    # Unit test for PidController
    ament_add_gtest(test_pid_controller test/test_pid_controller.cpp)
    target_include_directories(test_pid_controller PRIVATE include)

    # Unit test for geometry utilities
    ament_add_gtest(test_geometry_utils test/test_geometry_utils.cpp)
    target_include_directories(test_geometry_utils PRIVATE include)
endif()

ament_package()