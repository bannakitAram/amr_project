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
        'collision_monitor',
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
        # Raw MPPI output must NOT go straight to "cmd_vel": stm32_motor_node
        # listens there, so it would bypass velocity_smoother and
        # collision_monitor (the robot did not stop for a depth-only
        # obstacle, 2026-10-05). Chain: cmd_vel_nav -> velocity_smoother ->
        # cmd_vel_smoothed -> collision_monitor -> cmd_vel.
        remappings=[('cmd_vel', 'cmd_vel_nav')],
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
        # Spin/backup/drive_on_heading also go through the smoother and
        # collision_monitor - see controller_server above.
        remappings=[('cmd_vel', 'cmd_vel_nav')],
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
    # NOTE: output stays on the smoother's own default topic
    # ("cmd_vel_smoothed"), NOT remapped straight to "cmd_vel" - that old
    # remap made this collide with controller_server's own "cmd_vel"
    # publisher (two publishers on the same topic). collision_monitor below
    # is now the sole node that publishes the final "cmd_vel" sent to the
    # robot, per its cmd_vel_in_topic/cmd_vel_out_topic config in
    # nav2_params.yaml.
    velocity_smoother_node = Node(
        package='nav2_velocity_smoother',
        executable='velocity_smoother',
        name='velocity_smoother',
        output='screen',
        parameters=common_params,
        # Input from controller/behavior servers. Without this it subscribed
        # to "cmd_vel" itself, i.e. to collision_monitor's output - a loop.
        remappings=[('cmd_vel', 'cmd_vel_nav')],
    )
    # LiDAR-based independent safety layer, sitting between the smoother and
    # the robot: slows/stops any outgoing cmd_vel if the configured
    # footprint polygon (see nav2_params.yaml's FootprintApproach) predicts
    # a collision from /scan data, regardless of what MPPI/planner decided.
    collision_monitor_node = Node(
        package='nav2_collision_monitor',
        executable='collision_monitor',
        name='collision_monitor',
        output='screen',
        parameters=common_params,
    )

    # Bridges the rosboard goal_picker web page (tap-to-navigate / draw-a-
    # path) to Nav2's navigate_to_pose / navigate_through_poses actions -
    # see nav_goal_executor_node.py's module docstring.
    nav_goal_executor_node = Node(
        package='amr_navigation',
        executable='nav_goal_executor_node.py',
        name='nav_goal_executor_node',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # Halves Nav2's speed (via /speed_limit -> MPPI) while the firmware
    # reports wheel slip - see slip_speed_limiter_node.py's docstring.
    slip_speed_limiter_node = Node(
        package='amr_navigation',
        executable='slip_speed_limiter_node.py',
        name='slip_speed_limiter',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # Cancels all Nav2 goals if /imu/data stops - ekf.yaml takes yaw rate
    # from the gyro only, see imu_watchdog_node.py's docstring.
    imu_watchdog_node = Node(
        package='amr_navigation',
        executable='imu_watchdog_node.py',
        name='imu_watchdog',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
    )

    # Extracts MPPI's optimal local trajectory from /trajectories (needs
    # FollowPath.visualize: true) into a small /mppi_local_plan Path for the
    # goal_picker page - see local_plan_relay_node.py's docstring.
    local_plan_relay_node = Node(
        package='amr_navigation',
        executable='local_plan_relay_node.py',
        name='local_plan_relay',
        output='screen',
        parameters=[{'use_sim_time': use_sim_time}],
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
        collision_monitor_node,
        nav_goal_executor_node,
        slip_speed_limiter_node,
        imu_watchdog_node,
        local_plan_relay_node,
        lifecycle_manager_localization,
        lifecycle_manager_navigation,
    ])
