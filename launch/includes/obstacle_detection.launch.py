"""Launch the point-cloud obstacle detector."""

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
    perception_config_file = PathJoinSubstitution(
        [
            FindPackageShare('subsea_inspection'),
            'config',
            'perception.yaml',
        ]
    )

    return LaunchDescription(
        [
            Node(
                package='subsea_inspection',
                executable='obstacle_detector',
                name='obstacle_detector',
                output='screen',
                parameters=[common_topics_file, perception_config_file, {'use_sim_time': True}],
            )
        ]
    )
