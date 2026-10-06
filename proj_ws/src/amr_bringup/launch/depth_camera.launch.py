#!/usr/bin/env python3
"""
YDLIDAR HP60C depth camera (ascamera driver), depth-only, for obstacle
avoidance in the Nav2 local costmap / collision_monitor.

Pipeline:
    ascamera_node  --depth0/image_raw (320x240 16UC1 mm) + depth0/camera_info-->
    depth_image_proc::PointCloudXyzNode --> /depth_camera/points (XYZ, optical frame)

Measured on this Jetson + USB2 (2026-10-02):
  - 320x240 is the sensor's native depth resolution (IR fx~212, cx~158);
    640x480 is an upscale: 4x the USB/CPU load for no extra detail.
  - fps:=10 -> ~8 Hz real. fps:=15 produces no frames on this firmware.
  - The driver's own depth0/points topic never publishes, so the cloud is
    built here from the depth image instead.
  - The SDK refuses to start streaming unless the RGB fps equals the depth
    fps, so rgb_* must be set - but RGB is only streamed while something
    subscribes to rgb0/*, so nothing here touches RGB.

TF: pub_tfTree is off. With it on, the driver re-publishes two conflicting
dynamic transforms onto the same child frame (<ns>_color_0), and only while
streaming. With it off every depth message is stamped with
"<ns>_ascamera_0", which is bridged statically (180 deg about optical Z: the camera is mounted upside down) to the URDF's
camera_optical_frame (already carries the mechanical->optical rotation).
The few-cm IR/RGB registration offset this ignores is below costmap
resolution.

Usage:
    ros2 launch amr_bringup depth_camera.launch.py
"""
import glob
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import ComposableNodeContainer, Node
from launch_ros.descriptions import ComposableNode

CAMERA_NAMESPACE = 'ascamera_hp60c'
DEPTH_FRAME = CAMERA_NAMESPACE + '_ascamera_0'
DEPTH_TOPIC_PREFIX = '/' + CAMERA_NAMESPACE + '/camera_publisher/depth0'
POINTS_TOPIC = '/depth_camera/points'


def _ascamera_config_dir():
    # The driver's scanDir() only accepts regular files (DT_REG), so with
    # colcon --symlink-install the installed configurationfiles/ (all
    # symlinks) look empty and it fails with "cannot find config file".
    # Point it at the real directory the symlinks resolve to.
    share_cfg = os.path.join(
        get_package_share_directory('ascamera'), 'configurationfiles')
    for path in glob.glob(os.path.join(share_cfg, 'hp60c_*.json')):
        return os.path.dirname(os.path.realpath(path))
    return share_cfg


def generate_launch_description():
    ascamera_node = Node(
        namespace=CAMERA_NAMESPACE,
        package='ascamera',
        executable='ascamera_node',
        name='camera_publisher',
        respawn=True,
        respawn_delay=2.0,
        output='both',
        parameters=[{
            'usb_bus_no': -1,          # auto: first HP60C found on any bus
            'usb_path': 'null',
            'confiPath': _ascamera_config_dir(),
            'color_pcl': False,
            'pub_tfTree': False,
            'depth_width': 320,
            'depth_height': 240,
            'rgb_width': 640,          # only to satisfy the SDK fps check
            'rgb_height': 480,
            'fps': 10,
        }],
    )

    depth_to_points = ComposableNodeContainer(
        name='depth_camera_container',
        namespace='',
        package='rclcpp_components',
        executable='component_container',
        output='both',
        composable_node_descriptions=[
            ComposableNode(
                package='depth_image_proc',
                plugin='depth_image_proc::PointCloudXyzNode',
                name='depth_to_points',
                remappings=[
                    ('image_rect', DEPTH_TOPIC_PREFIX + '/image_raw'),
                    ('camera_info', DEPTH_TOPIC_PREFIX + '/camera_info'),
                    ('points', POINTS_TOPIC),
                ],
            ),
        ],
    )

    camera_tf_bridge = Node(
        package='tf2_ros',
        executable='static_transform_publisher',
        name='camera_optical_to_ascamera_tf',
        arguments=[
            '--x', '0', '--y', '0', '--z', '0',
            # Camera is mounted upside down (confirmed from the RGB image,
            # 2026-10-05): rotate 180 deg about the optical Z axis (= the
            # viewing direction), i.e. image x/y both flip.
            '--roll', '0', '--pitch', '0', '--yaw', '3.141592653589793',
            '--frame-id', 'camera_optical_frame',
            '--child-frame-id', DEPTH_FRAME,
        ],
    )

    # Cancels Nav2 goals if the driver or the cloud stops (see the node's
    # docstring). Lives here so it only runs when the camera is enabled.
    depth_watchdog = Node(
        package='amr_bringup',
        executable='depth_watchdog_node.py',
        name='depth_watchdog',
        output='screen',
        parameters=[{
            'camera_info_topic': DEPTH_TOPIC_PREFIX + '/camera_info',
            'points_topic': POINTS_TOPIC,
        }],
    )

    return LaunchDescription([
        ascamera_node,
        depth_to_points,
        camera_tf_bridge,
        depth_watchdog,
    ])
