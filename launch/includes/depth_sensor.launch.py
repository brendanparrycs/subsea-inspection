"""Bridge the simulated depth point cloud and publish the camera transform."""

from launch import LaunchDescription
from launch_ros.actions import Node


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
            "--x", "0.19",
            "--y", "0.0",
            "--z", "0.067",
            "--roll", "0.0",
            "--pitch", "0.0",
            "--yaw", "0.0",
            "--frame-id", "bluerov2_heavy/base_link",
            "--child-frame-id", "bluerov2_heavy/camera_link/depth_camera",
        ],
        output="screen",
        parameters=[{"use_sim_time": True}],
    )

    return LaunchDescription([point_cloud_bridge, depth_camera_transform])
