#!/bin/bash
# Always-on hook_bridge (Mega: battery coulometer, hook, relays, tower light)
# so /battery/state is shown even when bringup is not running.
# bringup no longer starts hook_bridge (arg hook_bridge:=false default).
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID=0
source /opt/ros/humble/setup.bash
source /home/nvidia/AMR_Project/proj_ws/install/setup.bash
exec ros2 run amr_bringup hook_bridge.py --ros-args -r __node:=hook_bridge -p port:=/dev/ttyMega
