cmake_minimum_required(VERSION 3.8)
project(amr_navigation)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  behavior_trees
  config
  launch
  maps
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/nav_goal_executor_node.py
  scripts/slip_speed_limiter_node.py
  scripts/slip_localization_node.py
  scripts/imu_watchdog_node.py
  scripts/local_plan_relay_node.py
  scripts/map_edit.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
