#!/usr/bin/env python3
"""
Full navigation bring-up: Ignition Gazebo (complex world) + AMR spawn +
ros_gz_bridge + robot_state_publisher + EKF + Nav2 (AMCL on a saved map) +
RViz2. This is the navigation counterpart to bringup.launch.py (which runs
slam_toolbox for mapping instead).

Prerequisite: you must already have a saved map (see the top-level README,
step 5, `map_saver_cli`) copied into
amr_ws/src/amr_navigation/maps/amr_map.yaml (+ .pgm).

Usage:
    ros2 launch amr_bringup navigation.launch.py
    ros2 launch amr_bringup navigation.launch.py map:=/full/path/to/your_map.yaml
    ros2 launch amr_bringup navigation.launch.py rviz:=false   # headless (Jetson w/o display)
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    pkg_description = get_package_share_directory('amr_description')
    pkg_gazebo = get_package_share_directory('amr_gazebo')
    pkg_navigation = get_package_share_directory('amr_navigation')
    pkg_bringup = get_package_share_directory('amr_bringup')

    default_map = os.path.join(pkg_navigation, 'maps', 'amr_map.yaml')

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('rviz')
    world = LaunchConfiguration('world')
    map_yaml = LaunchConfiguration('map')
    use_pedestrian = LaunchConfiguration('pedestrian')

    declare_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='true')
    declare_use_rviz = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Launch RViz2 (set false on headless Jetson over SSH)')
    declare_world = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(pkg_gazebo, 'worlds', 'complex_world.sdf'))
    declare_map = DeclareLaunchArgument(
        'map', default_value=default_map,
        description='Full path to the map yaml file saved from the SLAM step')
    declare_use_pedestrian = DeclareLaunchArgument(
        'pedestrian', default_value='false',
        description='Walk the "pedestrian" dynamic obstacle back and forth '
                     'across the corridor, for Nav2 avoidance testing')

    # 1) robot_description / robot_state_publisher
    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_description, 'launch', 'description.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # 2) Ignition Gazebo + spawn robot
    gazebo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo, 'launch', 'gazebo.launch.py')),
        launch_arguments={'world': world}.items()
    )

    # 3) ROS <-> Gazebo bridge (same bridge used for SLAM - /clock, /cmd_vel,
    #    /odom/wheel, /imu, /scan, /joint_states)
    bridge_config = os.path.join(pkg_bringup, 'config', 'ros_gz_bridge.yaml')
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        parameters=[{'config_file': bridge_config, 'use_sim_time': True}],
    )

    # 3b) direct /cmd_vel bridge - see bringup.launch.py for why this exists
    # alongside the YAML entry (gz-transport discovery race on boot).
    cmd_vel_direct_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='cmd_vel_direct_bridge',
        output='screen',
        arguments=['/cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist'],
        parameters=[{'use_sim_time': use_sim_time}],
        respawn=True,
        respawn_delay=2.0,
    )
    cmd_vel_direct_bridge_delayed = TimerAction(
        period=5.0,
        actions=[cmd_vel_direct_bridge],
    )

    # 4) robot_localization EKF - fuses /odom/wheel + /imu into odom ->
    #    base_footprint. Nav2's AMCL then publishes map -> odom on top of
    #    that, same TF-tree split used during SLAM.
    ekf_config = os.path.join(pkg_bringup, 'config', 'ekf.yaml')
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config, {'use_sim_time': use_sim_time}],
    )

    # 5) Nav2 (map_server + AMCL + planner/controller/behavior/BT navigator),
    #    delayed slightly so the bridge/EKF are already publishing /scan and
    #    /odom by the time AMCL starts localizing.
    navigation_launch = TimerAction(
        period=3.0,
        actions=[IncludeLaunchDescription(
            PythonLaunchDescriptionSource(
                os.path.join(pkg_navigation, 'launch', 'navigation.launch.py')),
            launch_arguments={
                'use_sim_time': use_sim_time,
                'map': map_yaml,
                'rviz': use_rviz,
            }.items()
        )],
    )

    # 6) Optional dynamic-obstacle pedestrian, walking back and forth
    #    across the corridor - the main thing you want running when
    #    testing Nav2's local planner / costmap obstacle avoidance.
    pedestrian_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo, 'launch', 'pedestrian.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
        condition=IfCondition(use_pedestrian),
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_use_rviz,
        declare_world,
        declare_map,
        declare_use_pedestrian,
        description_launch,
        gazebo_launch,
        ros_gz_bridge,
        cmd_vel_direct_bridge_delayed,
        ekf_node,
        navigation_launch,
        pedestrian_launch,
    ])
