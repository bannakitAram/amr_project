cmake_minimum_required(VERSION 3.8)
project(amr_gazebo)

find_package(ament_cmake REQUIRED)

install(DIRECTORY
  worlds
  launch
  models
  config
  DESTINATION share/${PROJECT_NAME}
)

install(PROGRAMS
  scripts/pedestrian_walker.py
  DESTINATION share/${PROJECT_NAME}/scripts
)

ament_package()
