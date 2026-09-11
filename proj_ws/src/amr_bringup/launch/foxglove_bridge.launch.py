#!/usr/bin/env python3
"""
Starts foxglove_bridge so Foxglove Studio (desktop app or foxglove.dev) can
connect over the network to visualize and teleop the AMR.

Connect from Foxglove Studio: "Open connection" -> Foxglove WebSocket ->
ws://<jetson-ip>:8765

To drive the robot, add a "Teleop" panel in Foxglove Studio and set its
publish topic to /cmd_vel (geometry_msgs/Twist) - client-side topic
publishing is enabled by default below.

Usage:
    ros2 launch amr_bringup foxglove_bridge.launch.py
    ros2 launch amr_bringup foxglove_bridge.launch.py port:=8765
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    port = LaunchConfiguration('port')
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_port = DeclareLaunchArgument(
        'port', default_value='8765',
        description='TCP port the Foxglove WebSocket server listens on')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false')

    foxglove_bridge_node = Node(
        package='foxglove_bridge',
        executable='foxglove_bridge',
        name='foxglove_bridge',
        output='screen',
        parameters=[{
            'port': port,
            'address': '0.0.0.0',
            'use_sim_time': use_sim_time,
            # Allow Foxglove panels (Teleop, Publish, Joystick) to publish,
            # e.g. geometry_msgs/Twist on /cmd_vel, back into ROS.
            'capabilities': ['clientPublish', 'parameters', 'parametersSubscribe',
                              'services', 'connectionGraph', 'assets'],
            'client_topic_whitelist': ['.*'],
            'topic_whitelist': ['.*'],
            'service_whitelist': ['.*'],
            'param_whitelist': ['.*'],
        }],
    )

    return LaunchDescription([
        declare_port,
        declare_use_sim_time,
        foxglove_bridge_node,
    ])
