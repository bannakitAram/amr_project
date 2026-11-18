#!/bin/bash
set -e
sudo cp /home/nvidia/AMR_Project/proj_ws/hook_bridge.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now hook_bridge.service
systemctl status hook_bridge --no-pager
