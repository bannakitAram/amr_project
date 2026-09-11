#!/usr/bin/env python3
"""
Nav2 stack for the AMR, running against a pre-built map (from the SLAM step).

Starts: map_server -> AMCL (localization_manager) and
        planner_server, controller_server, behavior_server, bt_navigator,
        waypoint_follower, velocity_smoother (navigation_manager).

Assumes the robot (Gazebo/real), ros_gz_bridge, and robot_localization's
ekf_node (odom -> base_footprint) are already running - e.g. from
`amr_bringup navigation.launch.py`, which includes this file. This file on
its own does NOT spawn the robot or start Gazebo.

Usage:
    ros2 launch amr_navigation navigation.launch.py
    ros2 launch amr_navigation navigation.launch.py map:=/path/to/your_map.yaml
    ros2 launch amr_navigation navigation.launch.py rviz:=false
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.descriptions import ParameterFile


def generate_launch_description():
    pkg_nav = get_package_share_directory('amr_navigation')

    default_map = os.path.join(pkg_nav, 'maps', 'amr_map.yaml')
    default_params = os.path.join(pkg_nav, 'config', 'nav2_params.yaml')
    default_rviz = os.path.join(pkg_nav, 'rviz', 'nav2.rviz')

    use_sim_time = LaunchConfiguration('use_sim_time')
    map_yaml = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    autostart = LaunchConfiguration('autostart')
    use_rviz = LaunchConfiguration('rviz')

    declare_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='true')
    declare_map = DeclareLaunchArgument(
        'map', default_value=default_map,
        description='Full path to the map yaml file saved from the SLAM step')
    declare_params_file = DeclareLaunchArgument(
        'params_file', default_value=default_params,
        description='Full path to the Nav2 parameters file')
    declare_autostart = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically bring all lifecycle nodes to the active state')
    declare_use_rviz = DeclareLaunchArgument('rviz', default_value='true')

    # Nav2 params files use per-node 'ros__parameters' keys and need
    # use_sim_time layered in the same way slam_toolbox_params.yaml does.
    configured_params = ParameterFile(params_file, allow_substs=True)

    lifecycle_nodes_localization = ['map_server', 'amcl']
    lifecycle_nodes_navigation = [
        'controller_server',
        'planner_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower',
        'velocity_smoother',
    ]

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[configured_params, {'use_sim_time': use_sim_time, 'yaml_filename': map_yaml}],
    )

    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=[configured_params, {'use_sim_time': use_sim_time}],
    )

    lifecycle_manager_localization = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_localization',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'node_names': lifecycle_nodes_localization,
            'bond_timeout': 10.0,
        }],
    )

    controller_server_node = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=[configured_params, {'use_sim_time': use_sim_time}],
        remappings=[('cmd_vel', 'cmd_vel_nav')],
    )

    planner_server_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=[configured_params, {'use_sim_time': use_sim_time}],
    )

    behavior_server_node = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=[configured_params, {'use_sim_time': use_sim_time}],
    )

    bt_navigator_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=[configured_params, {'use_sim_time': use_sim_time}],
    )

    waypoint_follower_node = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=[configured_params, {'use_sim_time': use_sim_time}],
    )

    velocity_smoother_node = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=[configured_params, {'use_sim_time': use_sim_time}],
        remappings=[('cmd_vel', 'cmd_vel_nav'), ('cmd_vel_smoothed', 'cmd_vel')],
    )

    lifecycle_manager_navigation = Node(
        package='nav2_lifecycle_manager',
        executable='lifecycle_manager',
        name='lifecycle_manager_navigation',
        output='screen',
        parameters=[{
            'use_sim_time': use_sim_time,
            'autostart': autostart,
            'node_names': lifecycle_nodes_navigation,
            'bond_timeout': 10.0,
        }],
    )

    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', default_rviz],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_map,
        declare_params_file,
        declare_autostart,
        declare_use_rviz,
        map_server_node,
        amcl_node,
        lifecycle_manager_localization,
        controller_server_node,
        planner_server_node,
        behavior_server_node,
        bt_navigator_node,
        waypoint_follower_node,
        velocity_smoother_node,
        lifecycle_manager_navigation,
        rviz_node,
    ])
