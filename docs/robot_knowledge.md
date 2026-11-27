# Robot knowledge (moved from Claude Code auto-memory, 2026-11-27)

Facts about this robot / Jetson that cost debugging time. Check against the current code
before relying on them - dates say when each was true.

## Encoder PPR - firmware is the source of truth
- Pulse-count -> wheel revolutions: use the STM32 firmware `ENCODER_PPR = 516.85f`
  (`stm/src/motor_control_with_pid_optimized.cpp`), same for both wheels (manual count over
  20 revolutions, after the encoder debounce filter).
- `amr_bringup/scripts/wheel_joint_state_publisher.py` has `LEFT_PPR = 623.3`, `RIGHT_PPR = 558.6`:
  stale pre-debounce noise values, "close enough for visualization" only. Do not copy them.

## HP60C depth camera (ascamera) - verified 2026-10-02/05
- Native depth 320x240 (IR fx ~212); 640x480 is upscaled. fps 10 -> ~8 Hz real; fps 15 = no frames.
- Driver's depth0/points never publishes -> cloud from depth_image_proc::PointCloudXyzNode
  -> `/depth_camera/points` (BEST_EFFORT).
- SDK streams only if rgb fps == depth fps, so rgb_* params must be set.
- scanDir() accepts regular files only -> breaks with colcon --symlink-install; launch resolves realpath.
- pub_tfTree publishes conflicting TFs -> keep off; static bridge camera_optical_frame ->
  ascamera_hp60c_ascamera_0. Camera is mounted UPSIDE DOWN -> yaw = pi about optical Z.
- All-zero depth = lens covered. Factory floor is glossy epoxy -> 0 % depth on a clear floor;
  VoxelLayer cannot clear depth marks.
- STVL (apt 2.3.4) marks 0 voxels inside controller_server's local_costmap (works standalone;
  cause unconfirmed). toggle_enabled never answers - not a health signal.
  Setup: local_costmap = obstacle_layer(/scan) + inflation; depth only in collision_monitor.
  NOTE 2026-11-27: the depth source in collision_monitor is now `enabled: False`
  (commit abd4572, reason not recorded) - LiDAR-only stopping. Decide before line use.
- cmd_vel chain (fixed 2026-10-05): controller/behavior servers remapped to cmd_vel_nav so
  everything goes velocity_smoother -> collision_monitor -> /cmd_vel. rosboard teleop still
  publishes /cmd_vel directly.
- Rear RPLIDAR sees the robot frame (~20 pts) -> laser_filters box filter -> /scan_filtered
  (costmaps + collision_monitor; AMCL/SLAM/goal_picker keep /scan).
- Humble collision_monitor uses max_points (min_points is Iron+); points = flat [x1,y1,...].
  FrontStop (bumper .. +0.35 m) exists because of the camera blind zone ~0.27 m ahead.
- Shell: ~/.bashrc sources an old ~/ros2_ws (stale amr_bringup) - source proj_ws/install/setup.bash.

## Jetson: snapd held at rev 24724 (2026-10-05)
- snapd 2.77.1 breaks every snap (Chromium) on kernel 5.15.148-tegra
  ("snap-confine ... cap_dac_override not found"). Rolled back and `snap refresh --hold snapd`.
- If a snap fails to start: `snap list snapd`. Do not unhold. Edge has no arm64 Linux build.

## Arduino Mega (CH340) upload driver (2026-10-06)
- Stock ch341.ko was replaced on 2026-05-19 by a stub (USB IDs only): `TIOCMGET Inappropriate ioctl`,
  avrdude stk500v2 timeout. Fix: stub moved to ~/ch341/ch341.ko.stub-backup; WCH ch34x.ko in
  /lib/modules/5.15.148-tegra/extra + depmod + /etc/modules-load.d/ch34x.conf (survives reboot).
- Upload fails -> `lsmod | grep ch34`. Port: /dev/serial/by-id/usb-1a86_USB_Serial-if00-port0 (ttyMega).

## Gamepad (ShanWan dongle) - 2026-11-16
- Boots as 045e:028e XInput (usbhid probe -32); after a manual replug it is 2563:0526 Android HID,
  15 buttons, which joy_teleop expects (LB=6, RB=7).
- Tried and failed: kernel xpad, paroj/xpad with ShanWan init, udev authorized toggle,
  USBDEVFS_RESET, uhubctl on hub 1-2.2 (does not cut VBUS). Builds in ~/xpad.
- Workaround: replug after boot. Better: a Linux-native gamepad (then re-map, commit + tag).

## AMR Control Panel autostart (2026-11-27)
- GDM auto-login as nvidia (/etc/gdm3/custom.conf, X11) + `~/.config/autostart/amr_control_panel.desktop`
  (not in git, delay 8 s) -> `proj_ws/src/amr_bringup/scripts/launch_control_panel.sh` opens the panel after boot.
- The launcher exits if a panel is already running (single instance). Opening the panel starts no
  robot motion; bringup / navigation still need a button. Log: ~/.ros/log/amr_control_panel.log
- New machine: copy the Desktop launcher (`~/Desktop/amr_control_panel.desktop`) to `~/.config/autostart/`.
