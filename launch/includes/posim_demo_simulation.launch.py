"""Launch the POSIM demo BlueROV2 Heavy simulation."""

from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription


def generate_launch_description():
    start_sim_arg = DeclareLaunchArgument(
        'start_sim',
        default_value='true',
        description='Whether to start the POSIM simulation and ArduSub',
        choices=['true', 'false'],
    )
    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Whether to open the Gazebo GUI',
        choices=['true', 'false'],
    )

    simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare('posim_demos'),
                    'launch',
                    'posim_robot.launch.py',
                ]
            )
        ),
        launch_arguments={
            'namespace': 'bluerov2_heavy',
            'world_name': 'posim_ocean_waves',
            'paused': 'false',
            'gui': LaunchConfiguration('gui'),
            'headless': 'false',
            'use_teleop': 'false',
            'use_web_joystick': 'false',
            'open_qgc': 'false',
            'open_virtual_joystick': 'false',
        }.items(),
        condition=IfCondition(LaunchConfiguration('start_sim')),
    )

    return LaunchDescription([start_sim_arg, gui_arg, simulation])
