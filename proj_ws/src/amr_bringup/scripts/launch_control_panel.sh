#!/bin/bash
# Double-click launcher for the AMR Control Panel GUI.
# ROS on loopback only (2026-11-12 risk review); must match bringup
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID=0
source /opt/ros/humble/setup.bash
source /home/nvidia/AMR_Project/proj_ws/install/setup.bash
exec ros2 run amr_bringup amr_control_panel.py
