"""Launch the autonomous pipeline inspection scenario."""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

TEST_OBSTACLE_SIZE_M = 0.5


def _include_component(filename, launch_arguments=None):
    """Include a reusable launch description from this package."""
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
    mission_file_arg = DeclareLaunchArgument(
        'mission_file',
        default_value=PathJoinSubstitution(
            [
                FindPackageShare('subsea_inspection'),
                'config',
                'pipeline_mission.yaml',
            ]
        ),
        description='Path to the YAML pipeline inspection mission',
    )
    avoidance_strategy_arg = DeclareLaunchArgument(
        'avoidance_strategy',
        default_value='fly_over',
        description='Obstacle avoidance strategy: [fly_over, lateral_bypass]',
        choices=['fly_over', 'lateral_bypass'],
    )
    avoidance_heave_clearance_arg = DeclareLaunchArgument(
        'avoidance_heave_clearance',
        default_value='0.8',
        description='Vertical clearance for fly-over avoidance (m)',
    )
    spawn_pipeline_arg = DeclareLaunchArgument(
        'spawn_pipeline',
        default_value='true',
        description='Whether to spawn the pipeline model',
        choices=['true', 'false'],
    )
    spawn_obstacle_arg = DeclareLaunchArgument(
        'spawn_obstacle',
        default_value='false',
        description='Whether to spawn a test obstacle',
        choices=['true', 'false'],
    )
    obstacle_x_arg = DeclareLaunchArgument(
        'obstacle_x',
        default_value='2.0',
        description='Test obstacle world X coordinate (m)',
    )
    obstacle_y_arg = DeclareLaunchArgument(
        'obstacle_y',
        default_value='0.0',
        description='Test obstacle world Y coordinate (m)',
    )
    obstacle_z_arg = DeclareLaunchArgument(
        'obstacle_z',
        default_value='-2.0',
        description='Test obstacle world Z coordinate (m)',
    )

    start_sim = LaunchConfiguration('start_sim')
    gui = LaunchConfiguration('gui')
    mission_file = LaunchConfiguration('mission_file')
    common_topics_file = PathJoinSubstitution(
        [
            FindPackageShare('subsea_inspection'),
            'config',
            'common_topics.yaml',
        ]
    )
    inspection_config_file = PathJoinSubstitution(
        [
            FindPackageShare('subsea_inspection'),
            'config',
            'inspection.yaml',
        ]
    )
    navigation_config_file = PathJoinSubstitution(
        [
            FindPackageShare('subsea_inspection'),
            'config',
            'navigation.yaml',
        ]
    )
    mission_planner_config_file = PathJoinSubstitution(
        [
            FindPackageShare('subsea_inspection'),
            'config',
            'mission_planner.yaml',
        ]
    )
    avoidance_strategy = LaunchConfiguration('avoidance_strategy')
    avoidance_heave_clearance = LaunchConfiguration('avoidance_heave_clearance')
    spawn_pipeline = LaunchConfiguration('spawn_pipeline')
    spawn_obstacle = LaunchConfiguration('spawn_obstacle')
    obstacle_x = LaunchConfiguration('obstacle_x')
    obstacle_y = LaunchConfiguration('obstacle_y')
    obstacle_z = LaunchConfiguration('obstacle_z')

    sensor_simulation = _include_component(
        'sensor_simulation.launch.py',
        {'start_sim': start_sim, 'gui': gui}.items(),
    )
    depth_sensor = _include_component('depth_sensor.launch.py')
    autonomy_core = _include_component('autonomy_core.launch.py')
    obstacle_detection = _include_component('obstacle_detection.launch.py')

    pipeline_model = PathJoinSubstitution(
        [
            FindPackageShare('subsea_inspection'),
            'models',
            'pipeline',
            'model.sdf',
        ]
    )
    pipeline_spawner = Node(
        package='ros_gz_sim',
        executable='create',
        name='pipeline_spawner',
        arguments=[
            '-world',
            'oceans_waves',
            '-name',
            'subsea_pipeline',
            '-file',
            pipeline_model,
        ],
        output='screen',
        condition=IfCondition(spawn_pipeline),
    )

    waypoint_navigator = Node(
        package='subsea_inspection',
        executable='waypoint_navigator',
        name='waypoint_navigator',
        output='screen',
        parameters=[
            common_topics_file,
            navigation_config_file,
            {'use_sim_time': True},
        ],
    )
    mission_planner = Node(
        package='subsea_inspection',
        executable='mission_planner',
        name='mission_planner',
        output='screen',
        parameters=[
            common_topics_file,
            mission_planner_config_file,
            {
                'use_sim_time': True,
                'mission_file': mission_file,
                'avoidance_strategy': avoidance_strategy,
                'avoidance_heave_clearance': avoidance_heave_clearance,
            },
        ],
    )

    obstacle_sdf = PythonExpression(
        [
            '\'<sdf version="1.8"><model name="inspection_obstacle"><static>true</static><link name="link"><pose>\' + ',
            "str('",
            obstacle_x,
            "') + ' ' + str('",
            obstacle_y,
            "') + ' ' + str('",
            obstacle_z,
            "') + ' 0 0 0</pose>' + ",
            f'\'<collision name="c"><geometry><box><size>{TEST_OBSTACLE_SIZE_M} {TEST_OBSTACLE_SIZE_M} {TEST_OBSTACLE_SIZE_M}</size></box></geometry></collision>\' + ',
            f'\'<visual name="v"><geometry><box><size>{TEST_OBSTACLE_SIZE_M} {TEST_OBSTACLE_SIZE_M} {TEST_OBSTACLE_SIZE_M}</size></box></geometry>\' + ',
            "'<material><ambient>1 0 0 1</ambient><diffuse>1 0 0 1</diffuse></material></visual></link></model></sdf>'",
        ]
    )
    obstacle_spawner = Node(
        package='ros_gz_sim',
        executable='create',
        name='obstacle_spawner',
        arguments=[
            '-world',
            'oceans_waves',
            '-name',
            'inspection_obstacle',
            '-string',
            obstacle_sdf,
        ],
        output='screen',
        condition=IfCondition(spawn_obstacle),
    )

    inspection_tracker = Node(
        package='subsea_inspection',
        executable='inspection_tracker',
        name='inspection_tracker',
        output='screen',
        parameters=[
            common_topics_file,
            inspection_config_file,
            {
                'use_sim_time': True,
                'mission_file': mission_file,
            },
        ],
    )

    return LaunchDescription(
        [
            start_sim_arg,
            gui_arg,
            mission_file_arg,
            avoidance_strategy_arg,
            avoidance_heave_clearance_arg,
            spawn_pipeline_arg,
            spawn_obstacle_arg,
            obstacle_x_arg,
            obstacle_y_arg,
            obstacle_z_arg,
            sensor_simulation,
            pipeline_spawner,
            depth_sensor,
            autonomy_core,
            obstacle_detection,
            waypoint_navigator,
            mission_planner,
            obstacle_spawner,
            inspection_tracker,
        ]
    )
