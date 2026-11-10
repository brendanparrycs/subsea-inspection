"""Launch the shared motion controller and vehicle state monitor."""

from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription


def generate_launch_description():
    common_topics_file = PathJoinSubstitution(
        [
            FindPackageShare('subsea_inspection'),
            'config',
            'common_topics.yaml',
        ]
    )
    motion_controller_config_file = PathJoinSubstitution(
        [
            FindPackageShare('subsea_inspection'),
            'config',
            'motion_controller.yaml',
        ]
    )

    return LaunchDescription(
        [
            Node(
                package='subsea_inspection',
                executable='motion_controller',
                name='motion_controller',
                output='screen',
                parameters=[
                    common_topics_file,
                    motion_controller_config_file,
                    {'use_sim_time': True},
                ],
            ),
            Node(
                package='subsea_inspection',
                executable='robot_state_monitor',
                name='robot_state_monitor',
                output='screen',
                parameters=[common_topics_file, {'use_sim_time': True}],
            ),
        ]
    )
