"""Launch the shared motion controller and vehicle state monitor."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="subsea_inspection",
            executable="motion_controller",
            name="motion_controller",
            output="screen",
            parameters=[{"use_sim_time": True}],
        ),
        Node(
            package="subsea_inspection",
            executable="robot_state_monitor",
            name="robot_state_monitor",
            output="screen",
            parameters=[{"use_sim_time": True}],
        ),
    ])
