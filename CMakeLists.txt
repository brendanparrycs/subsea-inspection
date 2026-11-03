cmake_minimum_required(VERSION 3.28)
project(subsea_inspection)

if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
    add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# Find depenencies
find_package(ament_cmake REQUIRED)
find_package(rclcpp REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Build the robot_state_monitor executable
add_executable(robot_state_monitor src/robot_state_monitor.cpp)
target_link_libraries(robot_state_monitor
    rclcpp::rclcpp
    nav_msgs::nav_msgs
    geometry_msgs::geometry_msgs
    tf2::tf2
    tf2_geometry_msgs::tf2_geometry_msgs
)

# Install executable to lib/subsea_inspection
install(TARGETS
    robot_state_monitor
    DESTINATION lib/${PROJECT_NAME}
)

INSTALL(DIRECTORY 
    launch
    config
    DESTINATION share/${PROJECT_NAME}
    OPTIONAL
)

ament_package()