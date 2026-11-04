"""
Launch file for manual teleoperation of BlueROV2 Heavy in POSIM Gazebo.
Supports switching between keyboard and joystick teleoperation modes.
"""

from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, PythonExpression
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare

def generate_launch_description():
    """Generates the launch description for manual ROV simulation testing."""

    # Launch arguments
    teleop_arg = DeclareLaunchArgument(
        "teleop",
        default_value="keyboard",
        description="Teleoperation input method: [keyboard, joystick]",
        choices=["keyboard", "joystick"]
    )

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

    teleop_choice = LaunchConfiguration("teleop")
    start_sim_choice = LaunchConfiguration("start_sim")
    gui_choice = LaunchConfiguration("gui")

    # Launch POSIM ocean waves simulation
    posim_simulation = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution([
                FindPackageShare("posim_demos"),
                "launch",
                "posim_robot.launch.py"
            ])
        ),
        launch_arguments={
            "namespace": "bluerov2_heavy",
            "world_name": "posim_ocean_waves",
            "paused": "false",
            "gui": gui_choice,
            "headless": "false",
            "use_teleop": "false",
            "use_web_joystick": "false",
            "open_qgc": "false",
            "open_virtual_joystick": "false"
        }.items(),
        condition=IfCondition(start_sim_choice)
    )

    # Subsea autonomy core nodes
    motion_controller_node = Node(
        package="subsea_inspection",
        executable="motion_controller",
        name="motion_controller",
        output="screen",
        parameters=[{ "use_sim_time": True }]
    )

    robot_state_monitor_node = Node(
        package="subsea_inspection",
        executable="robot_state_monitor",
        name="robot_state_monitor",
        output="screen",
        parameters=[{ "use_sim_time": True }]
    )

    # Teleoperation interface nodes
    # Keyboard Teleop: Launched in a dedicated GNOME Terminal window so raw
    # keystrokes are captured cleanly without clutter from Gazebo/MAVROS logs.
    keyboard_teleop_node = Node(
        package="subsea_inspection",
        executable="keyboard_teleop",
        name="keyboard_teleop",
        output="screen",
        prefix="gnome-terminal -- ",
        condition=IfCondition(
            PythonExpression(["'", teleop_choice, "' == 'keyboard'"])
        )
    )

    # Joystick Teleop: joy driver reads /dev/input/js0 and joystick_teleop translates to /cmd_vel
    joy_driver_node = Node(
        package="joy",
        executable="joy_node",
        name="joy_node",
        output="screen",
        condition=IfCondition(
            PythonExpression(["'", teleop_choice, "' == 'joystick'"])
        )
    )

    joystick_teleop_node = Node(
        package="subsea_inspection",
        executable="joystick_teleop",
        name="joystick_teleop",
        output="screen",
        condition=IfCondition(
            PythonExpression(["'", teleop_choice, "' == 'joystick'"])
        )
    )

    return LaunchDescription([
        teleop_arg,
        start_sim_arg,
        gui_arg,
        posim_simulation,
        motion_controller_node,
        robot_state_monitor_node,
        keyboard_teleop_node,
        joy_driver_node,
        joystick_teleop_node,
    ])
