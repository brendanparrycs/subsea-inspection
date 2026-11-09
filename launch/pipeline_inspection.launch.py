"""
Launch file for autonomous pipeline inspection mission of BlueROV2 Heavy in POSIM Gazebo.
Launches simulation, sensor bridge, motion controller, obstacle detector, waypoint navigator,
and mission planner with reactive obstacle avoidance.
"""

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


def generate_launch_description():
    """Generates the launch description for autonomous pipeline inspection."""

    # Launch arguments
    start_sim_arg = DeclareLaunchArgument(
        "start_sim",
        default_value="true",
        description="Whether to start the POSIM Gazebo simulation and ArduSub",
        choices=["true", "false"]
    )

    gui_arg = DeclareLaunchArgument(
        "gui",
        default_value="true",
        description="Whether to open Gazebo GUI",
        choices=["true", "false"]
    )

    mission_file_arg = DeclareLaunchArgument(
        "mission_file",
        default_value=PathJoinSubstitution([
            FindPackageShare("subsea_inspection"),
            "config",
            "pipeline_mission.yaml"
        ]),
        description="Path to YAML file defining inspection route waypoints"
    )

    avoidance_strategy_arg = DeclareLaunchArgument(
        "avoidance_strategy",
        default_value="fly_over",
        description="Obstacle avoidance strategy: [fly_over, lateral_bypass]",
        choices=["fly_over", "lateral_bypass"]
    )

    avoidance_heave_clearance_arg = DeclareLaunchArgument(
        "avoidance_heave_clearance",
        default_value="0.8",
        description="Vertical heave clearance in meters for fly-over avoidance"
    )

    spawn_obstacle_arg = DeclareLaunchArgument(
        "spawn_obstacle",
        default_value="false",
        description="Whether to spawn a test obstacle on the pipeline path",
        choices=["true", "false"]
    )

    obstacle_x_arg = DeclareLaunchArgument(
        "obstacle_x",
        default_value="2.0",
        description="World X coordinate for test obstacle (meters)"
    )

    obstacle_y_arg = DeclareLaunchArgument(
        "obstacle_y",
        default_value="0.0",
        description="World Y coordinate for test obstacle (meters)"
    )

    obstacle_z_arg = DeclareLaunchArgument(
        "obstacle_z",
        default_value="-2.0",
        description="World Z coordinate for test obstacle (meters)"
    )

    start_sim_choice = LaunchConfiguration("start_sim")
    gui_choice = LaunchConfiguration("gui")
    mission_file_choice = LaunchConfiguration("mission_file")
    avoidance_strategy_choice = LaunchConfiguration("avoidance_strategy")
    avoidance_heave_clearance_choice = LaunchConfiguration("avoidance_heave_clearance")
    spawn_obstacle_choice = LaunchConfiguration("spawn_obstacle")
    obstacle_x_choice = LaunchConfiguration("obstacle_x")
    obstacle_y_choice = LaunchConfiguration("obstacle_y")
    obstacle_z_choice = LaunchConfiguration("obstacle_z")

    # Determine Gazebo arguments based on GUI selection
    # "-r" runs simulation unpaused; "-s" runs server-only (headless) when gui is false
    gz_args = PythonExpression([
        "'", PathJoinSubstitution([
            FindPackageShare("posim_worlds"),
            "worlds",
            "posim_ocean_waves.world"
        ]), "' + (' -r' if '", gui_choice, "' == 'true' else ' -r -s')"
    ])

    # 1. Launch Gazebo Sim with posim_ocean_waves world
    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("ros_gz_sim"),
                "launch",
                "gz_sim.launch.py"
            ])
        ),
        launch_arguments={
            "gz_args": gz_args
        }.items(),
        condition=IfCondition(start_sim_choice)
    )

    # 2. Path to custom BlueROV2 Heavy model with forward 3D depth camera
    model_sdf_path = PathJoinSubstitution([
        FindPackageShare("subsea_inspection"),
        "models",
        "bluerov2_heavy",
        "model.sdf"
    ])

    # 3. Spawn BlueROV2 Heavy in Gazebo at initial inspection depth (z = -2.0m)
    gz_spawner = Node(
        package="ros_gz_sim",
        executable="create",
        name="gz_spawner",
        arguments=[
            "-name", "bluerov2_heavy",
            "-file", model_sdf_path,
            "-x", "0.0",
            "-y", "0.0",
            "-z", "-2.0",
            "-R", "0.0",
            "-P", "0.0",
            "-Y", "0.0"
        ],
        output="screen",
        parameters=[{"use_sim_time": True}],
        condition=IfCondition(start_sim_choice)
    )

    # 4. Include POSIM BlueROV2 Heavy robot configuration (ArduSub SITL, MAVROS, and telemetry bridge)
    posim_robot_config = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("posim_robot_models"),
                "config",
                "bluerov2_heavy",
                "robot_config.py"
            ])
        ),
        launch_arguments={
            "namespace": "bluerov2_heavy",
            "world_name": "oceans_waves",
            "use_ardusub": "true",
            "use_teleop": "false",
            "use_web_joystick": "false",
            "open_qgc": "false",
            "open_virtual_joystick": "false",
            "zoom_camera": "false"
        }.items(),
        condition=IfCondition(start_sim_choice)
    )

    # Trigger robot configuration once the model spawner completes
    spawn_event_handler = RegisterEventHandler(
        OnProcessExit(
            target_action=gz_spawner,
            on_exit=[
                LogInfo(msg="BlueROV2 Heavy model spawned, launching ArduSub SITL & MAVROS"),
                posim_robot_config
            ]
        ),
        condition=IfCondition(start_sim_choice)
    )

    # 5. Point Cloud ROS-GZ Bridge: Gazebo PointCloudPacked -> ROS 2 sensor_msgs/PointCloud2
    point_cloud_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="point_cloud_bridge",
        arguments=[
            "/model/bluerov2_heavy/depth_camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked"
        ],
        output="screen"
    )

    # 6. Static TF Publisher: vehicle base_link -> depth_camera_optical_frame
    # Matches camera_mount_link pose (x=0.19m, y=0.0m, z=0.067m)
    depth_camera_tf_node = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="depth_camera_tf_publisher",
        arguments=[
            "--x", "0.19",
            "--y", "0.0",
            "--z", "0.067",
            "--roll", "0.0",
            "--pitch", "0.0",
            "--yaw", "0.0",
            "--frame-id", "bluerov2_heavy/base_link",
            "--child-frame-id", "bluerov2_heavy/camera_link/depth_camera"
        ],
        output="screen",
        parameters=[{"use_sim_time": True}]
    )

    # 7. Subsea autonomy core nodes
    motion_controller_node = Node(
        package="subsea_inspection",
        executable="motion_controller",
        name="motion_controller",
        output="screen",
        parameters=[{"use_sim_time": True}]
    )

    robot_state_monitor_node = Node(
        package="subsea_inspection",
        executable="robot_state_monitor",
        name="robot_state_monitor",
        output="screen",
        parameters=[{"use_sim_time": True}]
    )

    obstacle_detector_node = Node(
        package="subsea_inspection",
        executable="obstacle_detector",
        name="obstacle_detector",
        output="screen",
        parameters=[{"use_sim_time": True}]
    )

    # 8. Waypoint navigation action server
    waypoint_navigator_node = Node(
        package="subsea_inspection",
        executable="waypoint_navigator",
        name="waypoint_navigator",
        output="screen",
        parameters=[{"use_sim_time": True}]
    )

    # 9. Mission planner with reactive obstacle avoidance
    mission_planner_node = Node(
        package="subsea_inspection",
        executable="mission_planner",
        name="mission_planner",
        output="screen",
        parameters=[{
            "use_sim_time": True,
            "mission_file": mission_file_choice,
            "avoidance_strategy": avoidance_strategy_choice,
            "avoidance_heave_clearance": avoidance_heave_clearance_choice
        }]
    )

    # 10. Optional obstacle spawner for avoidance testing
    obstacle_sdf = PythonExpression([
        "'<sdf version=\"1.8\"><model name=\"inspection_obstacle\"><static>true</static><link name=\"link\"><pose>' + ",
        "str('", obstacle_x_choice, "') + ' ' + str('", obstacle_y_choice, "') + ' ' + str('", obstacle_z_choice, "') + ' 0 0 0</pose>' + ",
        "'<collision name=\"c\"><geometry><box><size>0.5 0.5 0.5</size></box></geometry></collision>' + ",
        "'<visual name=\"v\"><geometry><box><size>0.5 0.5 0.5</size></box></geometry>' + ",
        "'<material><ambient>1 0 0 1</ambient><diffuse>1 0 0 1</diffuse></material></visual></link></model></sdf>'"
    ])

    obstacle_spawner_node = Node(
        package="ros_gz_sim",
        executable="create",
        name="obstacle_spawner",
        arguments=[
            "-world", "oceans_waves",
            "-name", "inspection_obstacle",
            "-string", obstacle_sdf
        ],
        output="screen",
        condition=IfCondition(spawn_obstacle_choice)
    )

    return LaunchDescription([
        start_sim_arg,
        gui_arg,
        mission_file_arg,
        avoidance_strategy_arg,
        avoidance_heave_clearance_arg,
        spawn_obstacle_arg,
        obstacle_x_arg,
        obstacle_y_arg,
        obstacle_z_arg,
        gz_sim,
        gz_spawner,
        spawn_event_handler,
        point_cloud_bridge,
        depth_camera_tf_node,
        motion_controller_node,
        robot_state_monitor_node,
        obstacle_detector_node,
        waypoint_navigator_node,
        mission_planner_node,
        obstacle_spawner_node,
    ])
