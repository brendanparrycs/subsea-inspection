"""Launch manual BlueROV2 Heavy teleoperation in POSIM Gazebo."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """Generate the launch description for manual ROV simulation testing."""
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

    start_sim_choice = LaunchConfiguration('start_sim')
    gui_choice = LaunchConfiguration('gui')
    common_topics_file = PathJoinSubstitution(
        [
            FindPackageShare('subsea_inspection'),
            'config',
            'common_topics.yaml',
        ]
    )
    teleop_config_file = PathJoinSubstitution(
        [
            FindPackageShare('subsea_inspection'),
            'config',
            'teleop.yaml',
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

    # Launch keyboard teleoperation in a dedicated terminal so raw keystrokes are captured cleanly.
    keyboard_teleop_node = Node(
        package='subsea_inspection',
        executable='keyboard_teleop',
        name='keyboard_teleop',
        output='screen',
        prefix='gnome-terminal -- ',
        parameters=[common_topics_file, teleop_config_file],
    )

    return LaunchDescription(
        [
            start_sim_arg,
            gui_arg,
            posim_simulation,
            autonomy_core,
            keyboard_teleop_node,
        ]
    )
