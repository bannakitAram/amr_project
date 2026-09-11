#!/usr/bin/env python3
"""
YDLIDAR HP60C depth camera (ascamera driver) + TF bridge into the AMR's
own TF tree published by robot_state_publisher (amr_description).

The ascamera driver publishes its own small TF subtree rooted at
"<namespace>_camera_link_0" (with color/depth optical frames hanging off
it using the vendor's factory extrinsics), but that root frame is not
connected to the rest of the robot - camera_optical_frame in the URDF
already applies the mechanical->optical rotation for the same physical
mount point, so bridging it directly (identity transform) to the
driver's root frame is the correct connection.

Usage:
    ros2 launch amr_bringup depth_camera.launch.py
"""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, DeclareLaunchArgument
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    pkg_ascamera = get_package_share_directory('ascamera')

    camera_namespace = LaunchConfiguration('camera_namespace')
    declare_camera_namespace = DeclareLaunchArgument(
        'camera_namespace', default_value='ascamera_hp60c',
        description='Must match the namespace used in the included ascamera launch file')

    ascamera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ascamera, 'launch', 'hp60c.launch.py'))
    )

    # Bridges the URDF's camera_optical_frame (mounted per amr_description's
    # camera_x/camera_z offsets) to the ascamera driver's own root frame
    # "<namespace>_camera_link_0", onto which the driver hangs its
    # factory-calibrated color/depth optical frames.
    camera_tf_bridge = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_optical_to_ascamera_tf',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            '--roll', '0', '--pitch', '0', '--yaw', '0',
            '--frame-id', 'camera_optical_frame',
            '--child-frame-id', [camera_namespace, '_camera_link_0'],
        ],
    )

    return LaunchDescription([
        declare_camera_namespace,
        ascamera_launch,
        camera_tf_bridge,
    ])
