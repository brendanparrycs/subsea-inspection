"""Launch Gazebo, the sensor-equipped vehicle, and its POSIM robot config."""

from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    LogInfo,
    RegisterEventHandler,
)
from launch.conditions import IfCondition
from launch.event_handlers import OnProcessExit
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

VEHICLE_SPAWN_X_M = 0.0
VEHICLE_SPAWN_Y_M = 0.0
VEHICLE_SPAWN_Z_M = -2.0
VEHICLE_SPAWN_ROLL_RAD = 0.0
VEHICLE_SPAWN_PITCH_RAD = 0.0
VEHICLE_SPAWN_YAW_RAD = 0.0


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

    start_sim = LaunchConfiguration('start_sim')
    gui = LaunchConfiguration('gui')
    gz_args = PythonExpression(
        [
            "'",
            PathJoinSubstitution(
                [
                    FindPackageShare('posim_worlds'),
                    'worlds',
                    'posim_ocean_waves.world',
                ]
            ),
            "' + (' -r' if '",
            gui,
            "' == 'true' else ' -r -s')",
        ]
    )

    gazebo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare('ros_gz_sim'),
                    'launch',
                    'gz_sim.launch.py',
                ]
            )
        ),
        launch_arguments={'gz_args': gz_args}.items(),
        condition=IfCondition(start_sim),
    )

    vehicle_model = PathJoinSubstitution(
        [
            FindPackageShare('subsea_inspection'),
            'models',
            'bluerov2_heavy',
            'model.sdf',
        ]
    )
    vehicle_spawner = Node(
        package='ros_gz_sim',
        executable='create',
        name='gz_spawner',
        arguments=[
            '-name',
            'bluerov2_heavy',
            '-file',
            vehicle_model,
            '-x',
            str(VEHICLE_SPAWN_X_M),
            '-y',
            str(VEHICLE_SPAWN_Y_M),
            '-z',
            str(VEHICLE_SPAWN_Z_M),
            '-R',
            str(VEHICLE_SPAWN_ROLL_RAD),
            '-P',
            str(VEHICLE_SPAWN_PITCH_RAD),
            '-Y',
            str(VEHICLE_SPAWN_YAW_RAD),
        ],
        output='screen',
        parameters=[{'use_sim_time': True}],
        condition=IfCondition(start_sim),
    )

    robot_config = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [
                    FindPackageShare('posim_robot_models'),
                    'config',
                    'bluerov2_heavy',
                    'robot_config.py',
                ]
            )
        ),
        launch_arguments={
            'namespace': 'bluerov2_heavy',
            'world_name': 'oceans_waves',
            'use_ardusub': 'true',
            'use_teleop': 'false',
            'use_web_joystick': 'false',
            'open_qgc': 'false',
            'open_virtual_joystick': 'false',
            'zoom_camera': 'false',
        }.items(),
        condition=IfCondition(start_sim),
    )

    configure_after_spawn = RegisterEventHandler(
        OnProcessExit(
            target_action=vehicle_spawner,
            on_exit=[
                LogInfo(msg='BlueROV2 Heavy model spawned, launching ArduSub SITL & MAVROS'),
                robot_config,
            ],
        ),
        condition=IfCondition(start_sim),
    )

    return LaunchDescription(
        [
            start_sim_arg,
            gui_arg,
            gazebo,
            vehicle_spawner,
            configure_after_spawn,
        ]
    )
