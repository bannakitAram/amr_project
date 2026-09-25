#!/usr/bin/env python3
"""Wheel-separation (track width) calibration test.

Runs as its own OS process (launched via `ros2 run amr_bringup
calib_rotate_node.py <num_turns> [turn_speed_rad_s]`), same reasoning as
move_forward_node.py / square_path_node.py (no GIL sharing with a GUI).

Commands N full in-place rotations, closed-loop on the RAW /wheel/odom yaw
(the STM32 firmware's own encoder-only pose_theta, published before EKF/IMU
fusion) - NOT /odometry/filtered. This deliberately excludes the IMU
correction that square_path_node.py's turns rely on, because the point here
is to expose the wheel-only odometry's own track-width error, which the IMU
would otherwise mask.

Why this isolates wheel_sep specifically: firmware computes
  w_reported = (v_r - v_l) / wheel_sep_nominal
and integrates pose_theta from w_reported. It stops this node's rotation
once pose_theta_reported == commanded_angle. If the physical track width
(wheel_sep_true) differs from wheel_sep_nominal used in that formula, the
ROBOT'S REAL rotation at that stopping point is:
  angle_actual = commanded_angle * (wheel_sep_nominal / wheel_sep_true)
so measuring angle_actual by hand gives:
  wheel_sep_true = wheel_sep_nominal * commanded_angle / angle_actual
This is the same relationship the classic UMBmark procedure uses to
separate wheelbase error from wheel-diameter error, adapted to this
robot's closed-loop-on-raw-odom turn architecture.

Procedure:
  1. Mark the robot's starting heading on the floor (e.g. tape/laser line
     along one side of the chassis).
  2. Run this node. It performs num_turns full rotations and stops.
  3. Measure the ACTUAL angle turned (compare final heading to the floor
     mark - a laser pointer taped to the chassis projected onto a wall a
     known distance away gives good angular resolution: angle_actual_deg
     = atan(lateral_shift_m / distance_to_wall_m) * 180/pi, accumulated
     over num_turns).
  4. Enter the measured angle in degrees when prompted. The node prints
     the corrected wheel_sep to review against WHEEL_SEP_NOMINAL below,
     stm/src/motor_control_with_pid_optimized.cpp's `wheel_sep` constant,
     and amr_description/urdf/amr.urdf.xacro's `wheel_separation` property
     (all three must be kept equal).

This tool only measures and prints a recommendation - it does NOT edit
firmware or URDF. Apply the correction by hand, re-flash, and re-run this
test to confirm before trusting it in navigation.

Exit codes:
  0 - completed all rotations (calibration prompt follows on stdout)
  3 - timed out mid-rotation (robot stuck / stalled)
  4 - no /wheel/odom data received within the initial wait
Also stops cleanly (exit 0) on SIGINT/SIGTERM, publishing zero velocity
first.
"""
import math
import signal
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node

TURN_SPEED_DEFAULT = 0.3     # rad/s - matches square_path_node.py's default
LEG_TIMEOUT_FACTOR = 3.0
TURN_SLOWDOWN_ANGLE = 0.35   # rad remaining - drop to half speed to reduce overshoot
TURN_TOLERANCE = 0.02        # rad
# Must match stm/src/motor_control_with_pid_optimized.cpp's `wheel_sep` and
# amr_description/urdf/amr.urdf.xacro's `wheel_separation` - update all
# three together if you change this.
WHEEL_SEP_NOMINAL = 0.472


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                       1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def main():
    if len(sys.argv) < 2:
        print('usage: calib_rotate_node.py <num_turns> [turn_speed_rad_s]',
              file=sys.stderr)
        sys.exit(2)
    num_turns = float(sys.argv[1])
    turn_speed = float(sys.argv[2]) if len(sys.argv) > 2 else TURN_SPEED_DEFAULT
    commanded_angle = 2.0 * math.pi * num_turns

    rclpy.init()
    node = Node('amr_calib_rotate')
    pub = node.create_publisher(Twist, '/cmd_vel', 10)

    stop_requested = {'flag': False}

    def on_signal(signum, frame):
        stop_requested['flag'] = True

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    latest_yaw = {'value': None}

    def on_wheel_odom(msg):
        latest_yaw['value'] = yaw_from_quaternion(msg.pose.pose.orientation)

    odom_sub = node.create_subscription(
        Odometry, '/wheel/odom', on_wheel_odom, 10)

    def publish_stop():
        for _ in range(3):
            pub.publish(Twist())
            time.sleep(0.05)

    exit_code = 0
    try:
        wait_start = time.time()
        while latest_yaw['value'] is None and time.time() - wait_start < 3.0:
            if stop_requested['flag']:
                return
            rclpy.spin_once(node, timeout_sec=0.1)
        if latest_yaw['value'] is None:
            print('no /wheel/odom data received', file=sys.stderr)
            exit_code = 4
            sys.exit(exit_code)

        start_yaw = latest_yaw['value']
        last_yaw = start_yaw
        turned = 0.0
        deadline = (time.time() + (abs(commanded_angle) / turn_speed)
                    * LEG_TIMEOUT_FACTOR + 5.0)
        direction = 1.0 if commanded_angle >= 0 else -1.0

        while not stop_requested['flag']:
            rclpy.spin_once(node, timeout_sec=0.05)
            current_yaw = latest_yaw['value']
            turned += normalize_angle(current_yaw - last_yaw) * direction
            last_yaw = current_yaw
            remaining = abs(commanded_angle) - turned
            if remaining <= TURN_TOLERANCE:
                break
            if time.time() > deadline:
                print('timed out mid-rotation', file=sys.stderr)
                exit_code = 3
                break
            twist = Twist()
            twist.angular.z = direction * (
                turn_speed * 0.5 if remaining < TURN_SLOWDOWN_ANGLE else turn_speed)
            pub.publish(twist)
    finally:
        publish_stop()
        node.destroy_subscription(odom_sub)
        node.destroy_publisher(pub)
        node.destroy_node()
        rclpy.shutdown()

    if exit_code != 0:
        sys.exit(exit_code)

    commanded_deg = math.degrees(commanded_angle)
    print(f'\nCommanded rotation (raw /wheel/odom): {commanded_deg:.1f} deg '
          f'({num_turns:g} turns)')
    print('Now measure the ACTUAL angle the robot really turned (see this '
          'file\'s module docstring for a laser-pointer-on-a-wall method).')
    try:
        measured_deg = float(input(
            'Enter measured actual angle in degrees (signed, same sign '
            'convention as commanded): '))
    except (ValueError, EOFError):
        print('no valid measurement entered - skipping calibration output',
              file=sys.stderr)
        sys.exit(0)

    if abs(measured_deg) < 1e-6:
        print('measured angle is ~0, cannot compute a correction',
              file=sys.stderr)
        sys.exit(0)

    corrected_wheel_sep = WHEEL_SEP_NOMINAL * commanded_deg / measured_deg
    print(f'\nWHEEL_SEP_NOMINAL (current): {WHEEL_SEP_NOMINAL:.4f} m')
    print(f'Corrected wheel_sep:          {corrected_wheel_sep:.4f} m')
    print('If this differs meaningfully from the current value, update BOTH:')
    print('  - stm/src/motor_control_with_pid_optimized.cpp: `wheel_sep`')
    print('  - amr_description/urdf/amr.urdf.xacro: `wheel_separation`')
    print('  - this script\'s WHEEL_SEP_NOMINAL (keep all three in sync)')
    print('Re-flash the STM32, then re-run this test to confirm before '
          'trusting it in navigation/localization.')


if __name__ == '__main__':
    main()
