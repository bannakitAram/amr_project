#!/usr/bin/env python3
"""
Full AMR bring-up: hardware (micro-ROS + RPLiDAR C1) + robot_state_publisher
+ EKF, plus either SLAM (mapping) or Nav2 (autonomous navigation on a saved
map) - never both at once, since slam_toolbox and AMCL each try to publish
the map -> odom transform and would fight each other.

Usage:
    # Mapping (default): drive the AMR around and build a map with slam_toolbox
    ros2 launch amr_bringup bringup.launch.py
    ros2 launch amr_bringup bringup.launch.py rviz:=false   # headless (Jetson w/o display)

    # Autonomous navigation on a map already saved from a mapping run
    ros2 launch amr_bringup bringup.launch.py slam:=false navigation:=true map:=/path/to/map.yaml
"""
import os
import subprocess
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (DeclareLaunchArgument, IncludeLaunchDescription, LogInfo,
                            SetEnvironmentVariable, TimerAction)
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PythonExpression
from launch.conditions import IfCondition
from launch_ros.actions import Node



def _software_version():
    """`git describe --tags --dirty` of the AMR_Project repo, for the launch log.

    Lets you tell from any bringup log which tagged / tested version the
    robot ran, and whether it had uncommitted changes ("-dirty"), which
    must not be used on the line (see CLAUDE.md, Versioning). The launch file
    is symlinked from src/ with --symlink-install, so realpath finds the repo.
    """
    here = os.path.dirname(os.path.realpath(__file__))
    try:
        return subprocess.run(
            ['git', '-C', here, 'describe', '--tags', '--always', '--dirty'],
            capture_output=True, text=True, timeout=5, check=True).stdout.strip()
    except Exception as e:  # not a git checkout, git missing, ...
        return 'unknown (%s)' % e.__class__.__name__


def _wait_for_time_sync():
    """Block until the system clock is NTP-synchronised (or timeout).

    The Jetson has no RTC battery: it boots with the last saved time and
    systemd-timesyncd later steps the clock forward (seen: +16 h). If that
    jump happens while ROS is running, TF/AMCL/costmaps get stale stamps and
    Nav2 hangs in "Activating planner_server" (station goals ignored), and
    collision_monitor may drop sensor data. So wait before starting nodes.

    Env: AMR_TIME_SYNC_TIMEOUT (s, default 120; 0 = skip the check).
    Returns a status string for the launch log.
    """
    import time
    timeout = float(os.environ.get('AMR_TIME_SYNC_TIMEOUT', '120'))
    if timeout <= 0:
        return 'check skipped (AMR_TIME_SYNC_TIMEOUT=0)'

    def synced():
        try:
            out = subprocess.run(
                ['timedatectl', 'show', '-p', 'NTPSynchronized', '--value'],
                capture_output=True, text=True, timeout=5).stdout.strip()
            return out == 'yes'
        except Exception:
            return None  # cannot tell (no timedatectl)

    t0 = time.monotonic()
    state = synced()
    if state is None:
        return 'unknown (timedatectl unavailable)'
    if not state:
        print('[bringup] waiting for NTP time sync (max %.0f s) ...' % timeout,
              flush=True)
    while not state and time.monotonic() - t0 < timeout:
        time.sleep(2.0)
        state = synced()
    waited = time.monotonic() - t0
    if state:
        return 'synchronized (waited %.0f s)' % waited
    print('\n' + '!' * 70 +
          '\n[bringup] WARNING: clock NOT NTP-synchronised after %.0f s.'
          '\n  If it syncs later the clock will jump and Nav2/TF will break:'
          '\n  restart bringup after `timedatectl` shows synchronized: yes.'
          '\n  NOT for line use in this state.\n' % waited + '!' * 70,
          flush=True)
    return 'NOT synchronized after %.0f s' % waited


def generate_launch_description():
    time_sync_status = _wait_for_time_sync()
    pkg_description = get_package_share_directory('amr_description')
    pkg_slam = get_package_share_directory('amr_slam')
    pkg_navigation = get_package_share_directory('amr_navigation')
    pkg_bringup = get_package_share_directory('amr_bringup')
    pkg_lidar = get_package_share_directory('rplidar_ros')

    use_sim_time = LaunchConfiguration('use_sim_time')
    use_rviz = LaunchConfiguration('rviz')
    use_foxglove = LaunchConfiguration('foxglove')
    foxglove_port = LaunchConfiguration('foxglove_port')
    use_depth_camera = LaunchConfiguration('depth_camera')
    use_slam = LaunchConfiguration('slam')
    use_hook = LaunchConfiguration('hook')
    use_joystick = LaunchConfiguration('joystick')
    hook_port = LaunchConfiguration('hook_port')
    use_navigation = LaunchConfiguration('navigation')
    nav_map = LaunchConfiguration('map')
    slam_start_scanning = LaunchConfiguration('slam_start_scanning')
    filter_operator_out = LaunchConfiguration('filter_operator_out')

    declare_use_sim_time = DeclareLaunchArgument('use_sim_time', default_value='false')
    declare_use_rviz = DeclareLaunchArgument(
        'rviz', default_value='false',
        description='Launch RViz2 (normal monitoring is via rosboard instead; '
                     'set true only when debugging with RViz2 directly)')
    declare_use_foxglove = DeclareLaunchArgument(
        'foxglove', default_value='true',
        description='Launch foxglove_bridge (feeds Lichtblick/Foxglove Studio). Only '
                     'actually starts while slam:=true (mapping) - the Lichtblick/mobile '
                     'monitor QR is a mapping-time tool, not needed during navigation. '
                     'Has no effect when slam:=false.')
    declare_foxglove_port = DeclareLaunchArgument(
        'foxglove_port', default_value='8765',
        description='TCP port for the Foxglove WebSocket server')
    declare_use_depth_camera = DeclareLaunchArgument(
        'depth_camera', default_value='true',
        description='Launch the YDLIDAR HP60C depth camera (depth-only -> /depth_camera/points) '
                     'for Nav2 obstacle avoidance; set false if the camera is unplugged')
    declare_use_hook = DeclareLaunchArgument(
        'hook', default_value='true',
        description='Launch hook_bridge + light_state (Arduino Mega: hook lift <-> /hook/state, '
                    '/hook/cmd; tower light <- /light/cmd); '
                    'set false if the hook controller is unplugged')
    declare_hook_port = DeclareLaunchArgument(
        'hook_port', default_value='/dev/ttyMega',
        description='Serial port of the hook Mega. /dev/ttyMega = any CH340 (VID:PID only, keep one CH340 on the robot); /dev/hook = pinned to a USB port (udev/99-amr-hook.rules)')
    declare_use_joystick = DeclareLaunchArgument(
        'joystick', default_value='false',
        description='Launch joy_node + joy_teleop (USB gamepad -> /cmd_vel, hold LB to '
                    'drive). AMR Control panel Manual mode only: bypasses collision_monitor')
    declare_use_slam = DeclareLaunchArgument(
        'slam', default_value='true',
        description='Launch slam_toolbox for mapping. Mutually exclusive with navigation '
                     '- both publish the map->odom transform and will fight each other.')
    declare_use_navigation = DeclareLaunchArgument(
        'navigation', default_value='false',
        description='Launch Nav2 (amr_navigation) for autonomous navigation on a saved '
                     'map. Requires map:=<path to map.yaml>. Set slam:=false when using this.')
    declare_filter_operator_out = DeclareLaunchArgument(
        'filter_operator_out', default_value='true',
        description='slam:=true only - blank the rear +-60 deg of the SLAM scan '
                    'so an operator walking behind is not mapped. false keeps '
                    'the rear: needed along walls/corridors, where the rear wall '
                    'is the only along-track feature (bag 20261113-11_08_01: '
                    'odom 4.08 m, map 0.42 m). SLAM input only; /scan and '
                    '/scan_filtered (Nav2, collision_monitor) are unchanged.')
    declare_slam_start_scanning = DeclareLaunchArgument(
        'slam_start_scanning', default_value='true',
        description='slam:=true only - false starts SLAM with its scan input gated '
                     'off until scan_gate/set_enabled is called (AMR Control panel '
                     '"Start scan")')
    declare_nav_map = DeclareLaunchArgument(
        'map', default_value='',
        description='Path to the map yaml file - required when navigation:=true')

    # 1) robot_description / robot_state_publisher
    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_description, 'launch', 'description.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time}.items()
    )

    # 2) Hardware Drivers: micro-ROS Agent (Board)
    micro_ros_agent_node = Node(
        package='micro_ros_agent',
        executable='micro_ros_agent',
        name='micro_ros_agent',
        arguments=['serial', '--dev', '/dev/ttyACM0', '-b', '115200'],
        output='screen'
    )

    # 2b) Converts wheel encoder pulse counts into /joint_states so
    #     robot_state_publisher can resolve the two continuous drive-wheel
    #     joints (otherwise RViz/Foxglove show a "no transform" error on
    #     left_drive_wheel_link / right_drive_wheel_link).
    wheel_joint_state_publisher_node = Node(
        package='amr_bringup',
        executable='wheel_joint_state_publisher.py',
        name='wheel_joint_state_publisher',
        output='screen',
    )

    # 3) Hardware Drivers: RPLiDAR C1
    rplidar_c1_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                pkg_lidar,  
                'launch',
                'rplidar_c1_launch.py'                        
            )
        ),
        launch_arguments={
            'serial_port': '/dev/ttyLidar',
            'frame_id': 'lidar_link',
            'inverted': 'false',
            'angle_compensate': 'true',
        }.items()
    )

    # 3b) Drop LiDAR returns that hit the robot's own body -> /scan_filtered
    #     (Nav2 costmaps + collision_monitor use it; SLAM/AMCL keep /scan).
    #     See config/laser_filter.yaml.
    laser_filter_node = Node(
        package='laser_filters',
        executable='scan_to_scan_filter_chain',
        name='scan_to_scan_filter_chain',
        output='screen',
        parameters=[os.path.join(pkg_bringup, 'config', 'laser_filter.yaml'),
                    {'use_sim_time': use_sim_time}],
        remappings=[('scan', 'scan'), ('scan_filtered', 'scan_filtered')],
    )

    # 4) robot_localization EKF (รอ micro-ROS 2 วินาทีเพื่อให้มีข้อมูล /odom/wheel เข้ามาก่อน)
    ekf_config = os.path.join(pkg_bringup, 'config', 'ekf.yaml')
    ekf_node = Node(
        package='robot_localization',
        executable='ekf_node',
        name='ekf_filter_node',
        output='screen',
        parameters=[ekf_config, {'use_sim_time': use_sim_time}],
    )
    delayed_ekf_node = TimerAction(
        period=2.0,
        actions=[ekf_node]
    )

    # 5) slam_toolbox (รอ EKF 4 วินาที เพื่อให้มีพิกัด odom -> base_footprint ก่อน)
    #    Mapping mode - mutually exclusive with navigation (5b), see docstring.
    slam_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_slam, 'launch', 'slam.launch.py')),
        launch_arguments={'use_sim_time': use_sim_time,
                          'scan_gate_start_enabled': slam_start_scanning,
                          'filter_operator_out': filter_operator_out}.items(),
        condition=IfCondition(use_slam),
    )
    delayed_slam_launch = TimerAction(
        period=4.0,
        actions=[slam_launch]
    )

    # 5b) Nav2 (amr_navigation) - autonomous navigation on a saved map.
    #     Mutually exclusive with slam_toolbox (5), see docstring.
    navigation_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_navigation, 'launch', 'navigation.launch.py')),
        launch_arguments={
            'map': nav_map,
            'use_sim_time': use_sim_time,
        }.items(),
        condition=IfCondition(use_navigation),
    )
    delayed_navigation_launch = TimerAction(
        period=4.0,
        actions=[navigation_launch]
    )

    # 6) RViz2 (รอระบบพร้อม 6 วินาทีค่อยเปิดการแสดงผล)
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
    delayed_rviz_node = TimerAction(
        period=6.0,
        actions=[rviz_node]
    )

    # 7) foxglove_bridge (feeds Lichtblick/Foxglove Studio) - mapping-time only.
    #    Gated on slam too so it never runs during navigation (where nobody
    #    connects Lichtblick, and it would just burn CPU the Jetson doesn't
    #    have to spare during localization).
    foxglove_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bringup, 'launch', 'foxglove_bridge.launch.py')),
        launch_arguments={
            'port': foxglove_port,
            'use_sim_time': use_sim_time,
        }.items(),
        condition=IfCondition(PythonExpression(
            ["'", use_foxglove, "' == 'true' and '", use_slam, "' == 'true'"]
        )),
    )

    # 8) YDLIDAR HP60C depth camera (ascamera) + TF bridge into the URDF tree
    depth_camera_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(pkg_bringup, 'launch', 'depth_camera.launch.py')),
        condition=IfCondition(use_depth_camera),
    )

    # 9) Hook lift controller (Arduino Mega, plain serial - no micro-ROS)
    hook_bridge_node = Node(
        package='amr_bringup',
        executable='hook_bridge.py',
        name='hook_bridge',
        parameters=[{'port': hook_port}],
        output='screen',
        condition=IfCondition(use_hook),
    )
    # 9b) Tower light pattern -> /light/cmd -> hook_bridge -> Mega (same
    #     board as the hook, so same on/off switch)
    light_state_node = Node(
        package='amr_bringup',
        executable='light_state_node.py',
        name='light_state',
        output='screen',
        condition=IfCondition(use_hook),
    )

    # 9c) Doors on the nav map: set from goal_picker, relay pulse on pass
    door_manager_node = Node(
        package='amr_bringup',
        executable='door_manager_node.py',
        name='door_manager',
        output='screen',
        condition=IfCondition(use_hook),
    )

    # 10) USB gamepad teleop (Manual mode)
    joy_node = Node(
        package='joy',
        executable='joy_node',
        name='joy_node',
        parameters=[{'deadzone': 0.1, 'autorepeat_rate': 20.0}],
        output='screen',
        condition=IfCondition(use_joystick),
    )
    joy_teleop_node = Node(
        package='amr_bringup',
        executable='joy_teleop_node.py',
        name='joy_teleop',
        output='screen',
        condition=IfCondition(use_joystick),
    )

    version = _software_version()
    version_log = LogInfo(msg='AMR software version: ' + version + (
        '  ** UNCOMMITTED CHANGES - not for line use **'
        if version.endswith('-dirty') or version.startswith('unknown') else ''))

    # Bigger Fast DDS shared-memory segment for every node below - see the
    # comment in config/fastdds_shm_8mb.xml (depth point cloud drops).
    fastdds_profile = SetEnvironmentVariable(
        'FASTRTPS_DEFAULT_PROFILES_FILE',
        os.path.join(pkg_bringup, 'config', 'fastdds_shm_8mb.xml'))
    # ROS on loopback only (the XML above also restricts its UDP transport)
    localhost_only = SetEnvironmentVariable('ROS_LOCALHOST_ONLY', '1')

    return LaunchDescription([
        fastdds_profile,
        localhost_only,
        version_log,
        LogInfo(msg='System clock (NTP): ' + time_sync_status + (
            '' if time_sync_status.startswith(('synchronized', 'check skipped'))
            else '  ** clock may jump - not for line use **')),
        declare_use_sim_time,
        declare_use_rviz,
        declare_use_foxglove,
        declare_foxglove_port,
        declare_use_depth_camera,
        declare_use_slam,
        declare_use_navigation,
        declare_nav_map,
        declare_slam_start_scanning,
        declare_filter_operator_out,
        declare_use_hook,
        declare_hook_port,
        declare_use_joystick,
        description_launch,
        micro_ros_agent_node,
        wheel_joint_state_publisher_node,
        rplidar_c1_launch,
        laser_filter_node,
        delayed_ekf_node,
        delayed_slam_launch,
        delayed_navigation_launch,
        delayed_rviz_node,
        foxglove_launch,
        depth_camera_launch,
        hook_bridge_node,
        light_state_node,
        door_manager_node,
        joy_node,
        joy_teleop_node,
    ])