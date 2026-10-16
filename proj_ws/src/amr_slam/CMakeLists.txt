cmake_minimum_required(VERSION 3.8)
project(amr_slam)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  config
  launch
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/scan_gate_node.py
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
