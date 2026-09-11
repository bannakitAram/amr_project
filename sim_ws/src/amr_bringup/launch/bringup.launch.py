#!/usr/bin/env python3
"""
Full SLAM bring-up: Ignition Gazebo (complex world) + AMR spawn +
ros_gz_bridge + robot_state_publisher + slam_toolbox + RViz2.

Usage:
    ros2 launch amr_bringup bringup.launch.py
    ros2 launch amr_bringup bringup.launch.py rviz:=false   # headless (Jetson w/o display)
"""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch.conditions import IfCondition
from launch_ros.actions import Node


def generate_launch_description():
    pkg_description = get_package_share_directory('amr_description')
    pkg_gazebo = get_package_share_directory('amr_gazebo')
    pkg_slam = get_package_share_directory('amr_slam')
    pkg_bringup = get_package_share_directory('amr_bringup')

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('rviz')
    world = LaunchConfiguration('world')
    use_pedestrian = LaunchConfiguration('pedestrian')

    declare_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='true')
    declare_use_rviz = DeclareLaunchArgument(
        'rviz', default_value='true',
        description='Launch RViz2 (set false on headless Jetson over SSH)')
    declare_world = DeclareLaunchArgument(
        'world',
        default_value=os.path.join(pkg_gazebo, 'worlds', 'complex_world.sdf'))
    declare_use_pedestrian = DeclareLaunchArgument(
        'pedestrian', default_value='false',
        description='Spawn-time world already includes the "pedestrian" '
                     'dynamic obstacle model; set true to also start it '
                     'walking back and forth (for Nav2 avoidance testing)')

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

    # 3) ROS <-> Gazebo bridge (main config: /clock, /cmd_vel, /odom/wheel,
    #    /imu, /scan, /joint_states - see amr_bringup/config/ros_gz_bridge.yaml)
    bridge_config = os.path.join(pkg_bringup, 'config', 'ros_gz_bridge.yaml')
    ros_gz_bridge = Node(
        package='ros_gz_bridge',
        executable='parameter_bridge',
        name='ros_gz_bridge',
        output='screen',
        parameters=[{'config_file': bridge_config, 'use_sim_time': True}],
    )

    # 3b) Extra direct topic-argument bridge for /cmd_vel, in addition to the
    #     YAML entry above. Useful for teleop/debugging straight against the
    #     unscoped gz topic without going through the /model/amr/... remap.
    #     Equivalent CLI form:
    #       ros2 run ros_gz_bridge parameter_bridge \
    #         /cmd_vel@geometry_msgs/msg/Twist@gz.msgs.Twist
    #     (note: "ignition.msgs.Twist" is the old Ignition-Citadel/Edifice
    #      message namespace; Fortress+ renamed it to "gz.msgs.Twist" - the
    #      same convention already used throughout ros_gz_bridge.yaml. Using
    #      "ignition.msgs.Twist" here would fail to bridge / stay silent.)
    #
    #     respawn=True + a startup delay: this node races Gazebo's
    #     gz-transport discovery on boot. If it starts before Gazebo is
    #     fully up, it fails to find the gz topic and exits - and launch
    #     does NOT auto-restart Nodes by default, so it silently stays
    #     dead unless you happen to run it by hand afterwards (system
    #     already up by then). respawn=True + a few seconds' delay fixes
    #     that so it comes up on its own every time.
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

    # 4) robot_localization EKF - fuses /odom/wheel + /imu into a single,
    #    drift-corrected odom -> base_footprint TF, published as /odom.
    #    This runs BEFORE slam_toolbox so slam_toolbox always sees the
    #    fused, lower-drift transform rather than raw wheel odom.
    ekf_config = os.path.join(pkg_bringup, 'config', 'ekf.yaml')
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config, {'use_sim_time': use_sim_time}],
    )

    # 5) slam_toolbox (2D lidar SLAM)
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slam, 'launch', 'slam.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # 6) Optional dynamic-obstacle pedestrian, walking back and forth
    #    across the corridor - see amr_gazebo/launch/pedestrian.launch.py
    pedestrian_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_gazebo, 'launch', 'pedestrian.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items(),
        condition=IfCondition(use_pedestrian),
    )

    # 7) RViz2
    rviz_config = os.path.join(pkg_description, 'rviz', 'slam.rviz')
    rviz_node = Node(
        package='rviz2',
        executable='rviz2',
        name='rviz2',
        arguments=['-d', rviz_config],
        parameters=[{'use_sim_time': use_sim_time}],
        output='screen',
        condition=IfCondition(use_rviz),
    )

    return LaunchDescription([
        declare_use_sim_time,
        declare_use_rviz,
        declare_world,
        declare_use_pedestrian,
        description_launch,
        gazebo_launch,
        ros_gz_bridge,
        cmd_vel_direct_bridge_delayed,
        ekf_node,
        slam_launch,
        pedestrian_launch,
        rviz_node,
    ])
