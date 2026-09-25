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
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
