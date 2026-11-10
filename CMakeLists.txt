cmake_minimum_required(VERSION 3.28)
project(subsea_inspection LANGUAGES C CXX)

# The generated ROS action includes C support targets; the nodes themselves use C++17.
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

find_package(ament_cmake REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(mavros_msgs REQUIRED)
find_package(nav_msgs REQUIRED)
find_package(rclcpp REQUIRED)
find_package(rclcpp_action REQUIRED)
find_package(rosidl_default_generators REQUIRED)
find_package(sensor_msgs REQUIRED)
find_package(std_msgs REQUIRED)
find_package(std_srvs REQUIRED)
find_package(tf2 REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(yaml-cpp REQUIRED)

# Generate the project's action interface and expose its C++ typesupport to the two clients.
rosidl_generate_interfaces(${PROJECT_NAME}
  "action/NavigateToWaypoint.action"
)
rosidl_get_typesupport_target(cpp_typesupport_target ${PROJECT_NAME} rosidl_typesupport_cpp)

function(configure_subsea_target target)
  # Keep project headers and compiler warnings local to each executable or test target.
  target_include_directories(${target} PRIVATE "${PROJECT_SOURCE_DIR}/include")

  if(CMAKE_CXX_COMPILER_ID MATCHES "GNU|Clang")
    target_compile_options(${target} PRIVATE -Wall -Wextra -Wpedantic)
  endif()
endfunction()

# Build each ROS node as its own executable with only its direct dependencies.
add_executable(robot_state_monitor src/robot_state_monitor.cpp)
configure_subsea_target(robot_state_monitor)
target_link_libraries(robot_state_monitor PRIVATE rclcpp::rclcpp nav_msgs::nav_msgs tf2::tf2)

add_executable(motion_controller src/motion_controller.cpp)
configure_subsea_target(motion_controller)
target_link_libraries(motion_controller PRIVATE
  rclcpp::rclcpp mavros_msgs::mavros_msgs geometry_msgs::geometry_msgs
)

add_executable(keyboard_teleop src/keyboard_teleop.cpp)
configure_subsea_target(keyboard_teleop)
target_link_libraries(keyboard_teleop PRIVATE rclcpp::rclcpp geometry_msgs::geometry_msgs)

add_executable(joystick_teleop src/joystick_teleop.cpp)
configure_subsea_target(joystick_teleop)
target_link_libraries(joystick_teleop PRIVATE
  rclcpp::rclcpp geometry_msgs::geometry_msgs sensor_msgs::sensor_msgs
)

add_executable(waypoint_navigator src/waypoint_navigator.cpp)
configure_subsea_target(waypoint_navigator)
target_link_libraries(waypoint_navigator PRIVATE
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  geometry_msgs::geometry_msgs
  nav_msgs::nav_msgs
  tf2::tf2
  "${cpp_typesupport_target}"
)

add_executable(mission_planner src/mission_planner.cpp)
configure_subsea_target(mission_planner)
target_link_libraries(mission_planner PRIVATE
  rclcpp::rclcpp
  rclcpp_action::rclcpp_action
  nav_msgs::nav_msgs
  geometry_msgs::geometry_msgs
  std_msgs::std_msgs
  std_srvs::std_srvs
  yaml-cpp
  "${cpp_typesupport_target}"
)

add_executable(obstacle_detector src/obstacle_detector.cpp)
configure_subsea_target(obstacle_detector)
target_link_libraries(obstacle_detector PRIVATE
  rclcpp::rclcpp
  sensor_msgs::sensor_msgs
  geometry_msgs::geometry_msgs
  std_msgs::std_msgs
  visualization_msgs::visualization_msgs
)

add_executable(inspection_tracker src/inspection_tracker.cpp)
configure_subsea_target(inspection_tracker)
target_link_libraries(inspection_tracker PRIVATE
  rclcpp::rclcpp
  nav_msgs::nav_msgs
  geometry_msgs::geometry_msgs
  visualization_msgs::visualization_msgs
  std_msgs::std_msgs
  std_srvs::std_srvs
  tf2::tf2
  yaml-cpp
)

set(SUBSEA_INSPECTION_EXECUTABLES
  robot_state_monitor
  motion_controller
  keyboard_teleop
  joystick_teleop
  waypoint_navigator
  mission_planner
  obstacle_detector
  inspection_tracker
)

# Install executables, public headers, launch/configuration assets, and the scenario generator.
install(TARGETS ${SUBSEA_INSPECTION_EXECUTABLES}
  RUNTIME DESTINATION lib/${PROJECT_NAME}
)

install(DIRECTORY include/
  DESTINATION include
)
ament_export_include_directories(include)

install(DIRECTORY launch config models
  DESTINATION share/${PROJECT_NAME}
)

install(FILES LICENSE
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS scripts/generate_pipeline_scenario.py
  DESTINATION lib/${PROJECT_NAME}
)

if(BUILD_TESTING)
  find_package(ament_cmake_gtest REQUIRED)
  find_package(ament_cmake_lint_cmake REQUIRED)

  # Share target setup so tests compile with the same language standard and warnings as the nodes.
  function(add_subsea_gtest target source)
    ament_add_gtest(${target} ${source})
    configure_subsea_target(${target})
  endfunction()

  add_subsea_gtest(test_pid_controller test/test_pid_controller.cpp)
  add_subsea_gtest(test_geometry_utils test/test_geometry_utils.cpp)
  add_subsea_gtest(test_point_cloud_filter test/test_point_cloud_filter.cpp)
  add_subsea_gtest(test_pipeline_tracker test/test_pipeline_tracker.cpp)
  add_subsea_gtest(test_config_validation test/test_config_validation.cpp)

  # Run ROS linters on build metadata only; source-wide linting will be enabled after the code pass.
  ament_lint_cmake(CMakeLists.txt)
endif()

ament_package()
