#!/bin/bash
source /opt/ros/humble/setup.bash
source /home/nvidia/AMR_Project/proj_ws/install/setup.bash
exec ros2 run rosboard rosboard_node
