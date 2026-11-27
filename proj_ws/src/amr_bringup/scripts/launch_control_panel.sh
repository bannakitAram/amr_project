#!/bin/bash
# Double-click launcher for the AMR Control Panel GUI.
# Also started at login by ~/.config/autostart/amr_control_panel.desktop
# (GDM auto-login as nvidia -> panel opens after boot). Opening the panel
# starts no robot motion; bringup/navigation still need a button press.
# One instance only: a second start (autostart + double-click) just exits.
if pgrep -u "$USER" -f "amr_bringup/amr_control_panel.py" >/dev/null; then
  exit 0
fi
# ROS on loopback only (2026-11-12 risk review); must match bringup
export ROS_LOCALHOST_ONLY=1
export ROS_DOMAIN_ID=0
source /opt/ros/humble/setup.bash
source /home/nvidia/AMR_Project/proj_ws/install/setup.bash
# keep stderr: the .desktop launch otherwise loses tracebacks / Tk errors
LOG=$HOME/.ros/log/amr_control_panel.log
[ -f "$LOG" ] && [ $(stat -c %s "$LOG") -gt 5000000 ] && mv "$LOG" "$LOG.1"
echo "=== start $(date -Is) ===" >> "$LOG"
export PYTHONFAULTHANDLER=1
exec ros2 run amr_bringup amr_control_panel.py >> "$LOG" 2>&1
