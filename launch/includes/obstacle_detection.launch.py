"""Launch the point-cloud obstacle detector."""

from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        Node(
            package="subsea_inspection",
            executable="obstacle_detector",
            name="obstacle_detector",
            output="screen",
            parameters=[{"use_sim_time": True}],
        )
    ])
