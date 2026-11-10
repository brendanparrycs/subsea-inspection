"""Launch the simulated depth point-cloud verification scenario."""

from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare

from launch import LaunchDescription


def _include_component(filename, launch_arguments=None):
    source = PythonLaunchDescriptionSource(
        PathJoinSubstitution(
            [
                FindPackageShare('subsea_inspection'),
                'launch',
                'includes',
                filename,
            ]
        )
    )
    return IncludeLaunchDescription(source, launch_arguments=launch_arguments)


def generate_launch_description():
    start_sim_arg = DeclareLaunchArgument(
        'start_sim',
        default_value='true',
        description='Whether to start Gazebo and ArduSub',
        choices=['true', 'false'],
    )
    gui_arg = DeclareLaunchArgument(
        'gui',
        default_value='true',
        description='Whether to open the Gazebo GUI',
        choices=['true', 'false'],
    )

    sensor_simulation = _include_component(
        'sensor_simulation.launch.py',
        {
            'start_sim': LaunchConfiguration('start_sim'),
            'gui': LaunchConfiguration('gui'),
        }.items(),
    )

    return LaunchDescription(
        [
            start_sim_arg,
            gui_arg,
            sensor_simulation,
            _include_component('depth_sensor.launch.py'),
            _include_component('autonomy_core.launch.py'),
            _include_component('obstacle_detection.launch.py'),
        ]
    )
