"""
Launch file for autonomous waypoint navigation of BlueROV2 Heavy in POSIM Gazebo.
Launches the simulation, motion controller, robot state monitor, and waypoint navigator.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generates the launch description for autonomous waypoint navigation testing."""

    # Launch arguments
    start_sim_arg = DeclareLaunchArgument(
        'start_sim',
        default_value='true',
        description='Whether to start the POSIM Gazebo simulation and ArduSub',
        choices=['true', 'false'],
    )

    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Whether to open Gazebo GUI',
        choices=['true', 'false'],
    )

    target_x_arg = DeclareLaunchArgument(
        'target_x', default_value='5.0', description='Target X coordinate in world frame (meters)'
    )

    target_y_arg = DeclareLaunchArgument(
        'target_y', default_value='0.0', description='Target Y coordinate in world frame (meters)'
    )

    target_z_arg = DeclareLaunchArgument(
        'target_z', default_value='-2.0', description='Target Z coordinate in world frame (meters)'
    )

    acceptance_radius_arg = DeclareLaunchArgument(
        'acceptance_radius',
        default_value='1.0',
        description='Arrival acceptance radius around target (meters)',
    )

    start_sim_choice = LaunchConfiguration('start_sim')
    gui_choice = LaunchConfiguration('gui')
    target_x_choice = LaunchConfiguration('target_x')
    target_y_choice = LaunchConfiguration('target_y')
    target_z_choice = LaunchConfiguration('target_z')
    acceptance_radius_choice = LaunchConfiguration('acceptance_radius')
    common_topics_file = PathJoinSubstitution(
        [
            FindPackageShare('subsea_inspection'),
            'config',
            'common_topics.yaml',
        ]
    )
    navigation_config_file = PathJoinSubstitution(
        [
            FindPackageShare('subsea_inspection'),
            'config',
            'navigation.yaml',
        ]
    )

    # Shared POSIM demo simulation and core vehicle nodes.
    posim_simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare('subsea_inspection'),
                    'launch',
                    'includes',
                    'posim_demo_simulation.launch.py',
                ]
            )
        ),
        launch_arguments={'start_sim': start_sim_choice, 'gui': gui_choice}.items(),
    )

    autonomy_core = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare('subsea_inspection'),
                    'launch',
                    'includes',
                    'autonomy_core.launch.py',
                ]
            )
        )
    )

    # Autonomous waypoint navigation node
    waypoint_navigator_node = Node(
        package='subsea_inspection',
        executable='waypoint_navigator',
        name='waypoint_navigator',
        output='screen',
        parameters=[
            common_topics_file,
            navigation_config_file,
            {
                'use_sim_time': True,
                'target_x': target_x_choice,
                'target_y': target_y_choice,
                'target_z': target_z_choice,
                'acceptance_radius': acceptance_radius_choice,
            },
        ],
    )

    return LaunchDescription(
        [
            start_sim_arg,
            gui_arg,
            target_x_arg,
            target_y_arg,
            target_z_arg,
            acceptance_radius_arg,
            posim_simulation,
            autonomy_core,
            waypoint_navigator_node,
        ]
    )
