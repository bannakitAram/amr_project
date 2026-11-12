#!/usr/bin/env python3
"""
Starts foxglove_bridge so Foxglove Studio (desktop app or foxglove.dev) can
connect over the network to visualize and teleop the AMR.

Connect from Foxglove Studio: "Open connection" -> Foxglove WebSocket ->
ws://<jetson-ip>:8765

View only: client publishing, parameter access and service calls are OFF
(2026-11-12 risk review). With them on, anyone on the factory WiFi could
publish /cmd_vel or motor/*/cmd (bypassing collision_monitor), change
collision_monitor parameters or call Nav2 lifecycle services - foxglove_bridge
has no authentication. Drive with the gamepad or the rosboard phone page.

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
            # View only - see the module docstring. Do not add
            # clientPublish / parameters / services without auth in front.
            'capabilities': ['connectionGraph', 'assets'],
            'client_topic_whitelist': ['^$'],
            'topic_whitelist': ['.*'],
            'service_whitelist': ['^$'],
            'param_whitelist': ['^$'],
        }],
    )

    return LaunchDescription([
        declare_port,
        declare_use_sim_time,
        foxglove_bridge_node,
    ])
