#!/usr/bin/env python3
"""Starts Ignition Gazebo (Fortress) with the complex indoor world and
spawns the AMR (robot_description must already be published, see
amr_bringup/launch/bringup.launch.py which includes this)."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, ExecuteProcess
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node


def generate_launch_description():
    pkg_gazebo = get_package_share_directory('amr_gazebo')
    pkg_ros_gz_sim = get_package_share_directory('ros_gz_sim')

    world_file = LaunchConfiguration('world')
    declare_world = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(pkg_gazebo, 'worlds', 'complex_world.sdf'),
        description='Full path to the SDF world to load')

    gz_sim = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_ros_gz_sim, 'launch', 'gz_sim.launch.py')),
        launch_arguments={'gz_args': [world_file, ' -r -v 3']}.items()
    )

    spawn_robot = Node(
        package='ros_gz_sim',
        executable='create',
        name='spawn_amr',
        arguments=[
            '-name', 'amr',
            '-topic', 'robot_description',
            '-x', '0.0', '-y', '-4.5', '-z', '0.05', '-Y', '1.5708'
        ],
        output='screen'
    )

    return LaunchDescription([
        declare_world,
        gz_sim,
        spawn_robot,
    ])
