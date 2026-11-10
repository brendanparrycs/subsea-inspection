"""Bridge the simulated depth point cloud and publish the camera transform."""

from launch import LaunchDescription
from launch_ros.actions import Node

DEPTH_CAMERA_X_M = 0.19
DEPTH_CAMERA_Y_M = 0.0
DEPTH_CAMERA_Z_M = 0.067
DEPTH_CAMERA_ROLL_RAD = 0.0
DEPTH_CAMERA_PITCH_RAD = 0.0
DEPTH_CAMERA_YAW_RAD = 0.0
DEPTH_CAMERA_PARENT_FRAME = "bluerov2_heavy/base_link"
DEPTH_CAMERA_FRAME = "bluerov2_heavy/camera_link/depth_camera"


def generate_launch_description():
    point_cloud_bridge = Node(
        package="ros_gz_bridge",
        executable="parameter_bridge",
        name="point_cloud_bridge",
        arguments=[
            "/model/bluerov2_heavy/depth_camera/points@sensor_msgs/msg/PointCloud2[gz.msgs.PointCloudPacked"
        ],
        output="screen",
    )

    depth_camera_transform = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="depth_camera_tf_publisher",
        arguments=[
            "--x", str(DEPTH_CAMERA_X_M),
            "--y", str(DEPTH_CAMERA_Y_M),
            "--z", str(DEPTH_CAMERA_Z_M),
            "--roll", str(DEPTH_CAMERA_ROLL_RAD),
            "--pitch", str(DEPTH_CAMERA_PITCH_RAD),
            "--yaw", str(DEPTH_CAMERA_YAW_RAD),
            "--frame-id", DEPTH_CAMERA_PARENT_FRAME,
            "--child-frame-id", DEPTH_CAMERA_FRAME,
        ],
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    return LaunchDescription([point_cloud_bridge, depth_camera_transform])
