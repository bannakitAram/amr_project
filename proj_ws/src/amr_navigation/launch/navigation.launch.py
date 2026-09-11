#!/usr/bin/env python3
"""
Nav2 stack (localization + navigation) for the AMR.
Requires a saved map (see amr_slam / nav2_map_server map_saver_cli) and
assumes robot_state_publisher + EKF (odom) + lidar are already running,
e.g. via `ros2 launch amr_bringup bringup.launch.py`.

Usage:
    ros2 launch amr_navigation navigation.launch.py map:=/path/to/map.yaml
    ros2 launch amr_navigation navigation.launch.py map:=/path/to/map.yaml use_sim_time:=true
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_navigation = get_package_share_directory('amr_navigation')
    default_params = os.path.join(pkg_navigation, 'config', 'nav2_params.yaml')

    map_yaml = LaunchConfiguration('map')
    params_file = LaunchConfiguration('params_file')
    use_sim_time = LaunchConfiguration('use_sim_time')
    autostart = LaunchConfiguration('autostart')

    declare_map = DeclareLaunchArgument(
        'map', description='Full path to the map yaml file to load')
    declare_params_file = DeclareLaunchArgument(
        'params_file', default_value=default_params,
        description='Full path to the Nav2 parameters file')
    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false')
    declare_autostart = DeclareLaunchArgument(
        'autostart', default_value='true',
        description='Automatically bring lifecycle nodes up to the active state')

    lifecycle_nodes_localization = ['map_server', 'amcl']
    lifecycle_nodes_navigation = [
        'controller_server',
        'planner_server',
        'smoother_server',
        'behavior_server',
        'bt_navigator',
        'waypoint_follower',
        'velocity_smoother',
    ]

    common_params = [params_file, {'use_sim_time': use_sim_time}]

    map_server_node = Node(
        package='nav2_map_server',
        executable='map_server',
        name='map_server',
        output='screen',
        parameters=[params_file, {'use_sim_time': use_sim_time, 'yaml_filename': map_yaml}],
    )
    amcl_node = Node(
        package='nav2_amcl',
        executable='amcl',
        name='amcl',
        output='screen',
        parameters=common_params,
    )
    controller_node = Node(
        package='nav2_controller',
        executable='controller_server',
        name='controller_server',
        output='screen',
        parameters=common_params,
    )
    planner_node = Node(
        package='nav2_planner',
        executable='planner_server',
        name='planner_server',
        output='screen',
        parameters=common_params,
    )
    smoother_node = Node(
        package='nav2_smoother',
        executable='smoother_server',
        name='smoother_server',
        output='screen',
        parameters=common_params,
    )
    behavior_node = Node(
        package='nav2_behaviors',
        executable='behavior_server',
        name='behavior_server',
        output='screen',
        parameters=common_params,
    )
    bt_navigator_node = Node(
        package='nav2_bt_navigator',
        executable='bt_navigator',
        name='bt_navigator',
        output='screen',
        parameters=common_params,
    )
    waypoint_follower_node = Node(
        package='nav2_waypoint_follower',
        executable='waypoint_follower',
        name='waypoint_follower',
        output='screen',
        parameters=common_params,
    )
    velocity_smoother_node = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=common_params,
        remappings=[('cmd_vel_smoothed', 'cmd_vel')],
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
        }],
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
        }],
    )

    return LaunchDescription([
        declare_map,
        declare_params_file,
        declare_use_sim_time,
        declare_autostart,
        map_server_node,
        amcl_node,
        controller_node,
        planner_node,
        smoother_node,
        behavior_node,
        bt_navigator_node,
        waypoint_follower_node,
        velocity_smoother_node,
        lifecycle_manager_localization,
        lifecycle_manager_navigation,
    ])
