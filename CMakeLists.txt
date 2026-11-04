cmake_minimum_required(VERSION 3.28)
project(subsea_inspection)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find dependencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(sensor_msgs REQUIRED)

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

# Install executables to lib/subsea_inspection
install(TARGETS
    robot_state_monitor
    motion_controller
    keyboard_teleop
    joystick_teleop
    DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY 
    launch
    config
    DESTINATION share/${PROJECT_NAME}
    OPTIONAL
)

ament_package()