#!/bin/bash
rosdep update
rosdep install --from-paths src --ignore-src -y
colcon build
source install/setup.bash
