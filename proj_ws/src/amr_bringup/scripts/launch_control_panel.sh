#!/bin/bash
# Double-click launcher for the AMR Control Panel GUI.
source /opt/ros/humble/setup.bash
source /home/nvidia/AMR_Project/proj_ws/install/setup.bash
exec ros2 run amr_bringup amr_control_panel.py
