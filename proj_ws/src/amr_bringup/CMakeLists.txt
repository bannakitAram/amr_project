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
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
