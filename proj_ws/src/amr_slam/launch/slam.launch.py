#!/usr/bin/env python3
"""Starts slam_toolbox (online_async, lifecycle-managed) for 2D lidar SLAM."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch_ros.actions import LifecycleNode
from launch_ros.actions import Node
from launch.actions import EmitEvent, RegisterEventHandler
from launch.events import matches_action
from launch_ros.events.lifecycle import ChangeState
from launch_ros.event_handlers import OnStateTransition
import lifecycle_msgs.msg


def generate_launch_description():
    pkg_slam = get_package_share_directory('amr_slam')
    default_params = os.path.join(pkg_slam, 'config', 'slam_toolbox_params.yaml')
    default_laser_filters_params = os.path.join(pkg_slam, 'config', 'laser_filters.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    slam_params_file = LaunchConfiguration('slam_params_file')
    laser_filters_params_file = LaunchConfiguration('laser_filters_params_file')
    filter_operator_out = LaunchConfiguration('filter_operator_out')

    declare_use_sim_time = DeclareLaunchArgument(
        'use_sim_time', default_value='false')
    declare_params_file = DeclareLaunchArgument(
        'slam_params_file', default_value=default_params,
        description='Full path to the slam_toolbox parameters file')
    declare_laser_filters_params_file = DeclareLaunchArgument(
        'laser_filters_params_file', default_value=default_laser_filters_params,
        description='Full path to the laser_filters parameters file')
    declare_filter_operator_out = DeclareLaunchArgument(
        'filter_operator_out', default_value='true',
        description='Run raw /scan through laser_filters (angular sector '
                     'blank-out) before feeding slam_toolbox, so a teleop '
                     'operator walking behind the robot is not baked into '
                     'the map. Does NOT touch /scan itself - only /scan is '
                     'used for safety/obstacle avoidance and stays raw.')

    # scan_to_scan_filter_chain: laser_filters node that subscribes to raw
    # /scan and republishes the filtered result on /scan_filtered. Only
    # slam_toolbox (mapping) consumes /scan_filtered - safety/obstacle
    # avoidance nodes must keep subscribing to raw /scan.
    scan_filter_node = Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        name='scan_to_scan_filter_chain',
        output='screen',
        parameters=[laser_filters_params_file, {'use_sim_time': use_sim_time}],
        condition=IfCondition(filter_operator_out),
    )

    start_async_slam_toolbox_node = LifecycleNode(
        parameters=[
            slam_params_file,
            {'use_sim_time': use_sim_time},
            # Overrides scan_topic from slam_params_file: /scan_filtered when
            # laser_filters is enabled (default), raw /scan otherwise.
            {'scan_topic': PythonExpression(
                ["'/scan_filtered' if '", filter_operator_out, "' == 'true' else '/scan'"]
            )},
        ],
        package='slam_toolbox',
        executable='async_slam_toolbox_node',
        name='slam_toolbox',
        namespace='',
        output='screen',
    )

    configure_event = EmitEvent(
        event=ChangeState(
            lifecycle_node_matcher=matches_action(start_async_slam_toolbox_node),
            transition_id=lifecycle_msgs.msg.Transition.TRANSITION_CONFIGURE,
        )
    )

    activate_event = RegisterEventHandler(
        OnStateTransition(
            target_lifecycle_node=start_async_slam_toolbox_node,
            start_state='configuring',
            goal_state='inactive',
            entities=[
                EmitEvent(
                    event=ChangeState(
                        lifecycle_node_matcher=matches_action(start_async_slam_toolbox_node),
                        transition_id=lifecycle_msgs.msg.Transition.TRANSITION_ACTIVATE,
                    )
                )
            ],
        )
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_params_file,
        declare_laser_filters_params_file,
        declare_filter_operator_out,
        scan_filter_node,
        start_async_slam_toolbox_node,
        configure_event,
        activate_event,
    ])