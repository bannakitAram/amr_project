cmake_minimum_required(VERSION 3.8)
project(amr_bringup)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  launch
  config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/wheel_joint_state_publisher.py
  scripts/amr_control_panel.py
  scripts/move_forward_node.py
  scripts/square_path_node.py
  scripts/circle_path_node.py
  scripts/calib_rotate_node.py
  scripts/calib_straight_node.py
  scripts/amr_calib_panel.py
  scripts/calc_wheel_radius.py
  scripts/gyro_integrate_node.py
  scripts/scan_lines_node.py
  scripts/depth_watchdog_node.py
  scripts/slip_ratio_from_bag.py
  scripts/accel_slip_test_node.py
  scripts/stop_distance_from_bag.py
  scripts/hook_bridge.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
