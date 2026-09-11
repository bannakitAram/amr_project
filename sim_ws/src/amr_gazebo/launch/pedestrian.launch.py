#!/usr/bin/env python3
"""
Brings up the "pedestrian" dynamic obstacle for the complex_world.sdf world:
  1) a ros_gz_bridge topic bridge so ROS 2 can command the pedestrian's
     velocity (ROS /pedestrian/cmd_vel -> Gazebo /model/pedestrian/cmd_vel,
     consumed by the model's VelocityControl plugin)
  2) the pedestrian_walker.py node, which makes it patrol back and forth
     along the corridor automatically.

Requires the world to already be running (amr_gazebo/launch/gazebo.launch.py
or the full amr_bringup/launch/bringup.launch.py), since the "pedestrian"
model must exist in the loaded world before the bridge can find its
Gazebo topic.

Usage:
    ros2 launch amr_gazebo pedestrian.launch.py
    ros2 launch amr_gazebo pedestrian.launch.py speed:=0.8 x_min:=-7.0 x_max:=7.0
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, TimerAction
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_gazebo = get_package_share_directory('amr_gazebo')
    walker_script = os.path.join(pkg_gazebo, 'scripts', 'pedestrian_walker.py')
    bridge_config = os.path.join(pkg_gazebo, 'config', 'pedestrian_bridge.yaml')

    speed = LaunchConfiguration('speed')
    x_min = LaunchConfiguration('x_min')
    x_max = LaunchConfiguration('x_max')
    start_x = LaunchConfiguration('start_x')
    use_sim_time = LaunchConfiguration('use_sim_time')

    declare_speed = DeclareLaunchArgument('speed', default_value='0.5')
    declare_x_min = DeclareLaunchArgument('x_min', default_value='-6.0')
    declare_x_max = DeclareLaunchArgument('x_max', default_value='6.0')
    declare_start_x = DeclareLaunchArgument('start_x', default_value='-4.0')
    declare_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='true')

    # respawn=True + a short delay: this bridge races Gazebo's topic
    # discovery on startup, same reasoning as cmd_vel_direct_bridge in
    # amr_bringup/launch/bringup.launch.py.
    pedestrian_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='pedestrian_bridge',
        output='screen',
        parameters=[{'config_file': bridge_config, 'use_sim_time': use_sim_time}],
        respawn=True,
        respawn_delay=2.0,
    )

    walker_node = ExecuteProcess(
        cmd=[
            'python3', walker_script,
            '--ros-args',
            '-p', ['speed:=', speed],
            '-p', ['x_min:=', x_min],
            '-p', ['x_max:=', x_max],
            '-p', ['start_x:=', start_x],
            '-p', ['use_sim_time:=', use_sim_time],
        ],
        output='screen',
    )

    # Small delay so the bridge is up before the walker starts publishing.
    walker_delayed = TimerAction(period=3.0, actions=[walker_node])

    return LaunchDescription([
        declare_speed,
        declare_x_min,
        declare_x_max,
        declare_start_x,
        declare_use_sim_time,
        pedestrian_bridge,
        walker_delayed,
    ])
