#!/usr/bin/env python3
"""Standalone "move forward by distance" node.

Runs as its own OS process (launched via `ros2 run amr_bringup
move_forward_node.py <distance_m> [speed_mps]`), NOT as a thread inside the
Tkinter control-panel process. It used to run as a background thread in
amr_control_panel.py, sharing the GIL with the Tkinter mainloop. Under GUI
load (QR code rendering, widget updates, subprocess bookkeeping) that
thread could be starved for stretches close to but under the firmware's
CMD_TIMEOUT_MS, so the robot kept coasting at the last-published cmd_vel
without the stop condition being checked. Running this as its own process
removes the GIL-sharing entirely; the control panel just polls the process
exit code instead of trying to share a Python interpreter with it.

Closed-loop on /wheel/raw_distance (a cumulative signed distance published
directly by the STM32 firmware, integrated from raw_speed_l/r - see that
firmware's raw_distance_traveled comment), NOT /odometry/filtered. The EKF-
fused /odometry/filtered goes through the firmware's ODOM_FILTER_ALPHA
low-pass (0.08) before being fused, which measurably lags real distance
during a speed transient (e.g. the accel ramp at the start of a move) -
this was diagnosed as the cause of a fixed ~0.15-0.2m overshoot on "move
forward X meters" tests that persisted even after removing the GIL-sharing
above and tightening the firmware's CMD_TIMEOUT_MS, i.e. an odometry-lag
problem rather than a timing/process one. /wheel/raw_distance skips that
filter, so it tracks real distance far more closely.

Exit codes:
  0 - reached target distance
  1 - timed out (encoder stalled / robot stuck)
  2 - no /wheel/raw_distance data received within the initial wait
Also stops cleanly (exit 0) on SIGINT/SIGTERM (used by the control panel's
Cancel button and app-close handler), publishing zero velocity first.
"""
import signal
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float32

MOVE_FORWARD_SPEED_DEFAULT = 0.15  # m/s - used if no speed arg is given
MOVE_FORWARD_TIMEOUT_FACTOR = 3.0  # abort if it takes > (distance/speed)*this


def main():
    if len(sys.argv) < 2:
        print('usage: move_forward_node.py <distance_m> [speed_mps]', file=sys.stderr)
        sys.exit(2)
    distance = float(sys.argv[1])
    speed = float(sys.argv[2]) if len(sys.argv) > 2 else MOVE_FORWARD_SPEED_DEFAULT

    rclpy.init()
    node = Node('amr_move_forward')
    pub = node.create_publisher(Twist, '/cmd_vel', 10)

    stop_requested = {'flag': False}

    def on_signal(signum, frame):
        stop_requested['flag'] = True

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    latest_distance = {'value': None}

    def on_raw_distance(msg):
        latest_distance['value'] = msg.data

    sub = node.create_subscription(
        Float32, '/wheel/raw_distance', on_raw_distance, 10)

    exit_code = 0
    try:
        wait_start = time.time()
        while latest_distance['value'] is None and time.time() - wait_start < 3.0:
            if stop_requested['flag']:
                exit_code = 0
                return
            rclpy.spin_once(node, timeout_sec=0.1)
        if latest_distance['value'] is None:
            print('no /wheel/raw_distance data received', file=sys.stderr)
            exit_code = 2
            return

        start = latest_distance['value']
        deadline = time.time() + (distance / speed) * MOVE_FORWARD_TIMEOUT_FACTOR + 5.0

        while not stop_requested['flag']:
            rclpy.spin_once(node, timeout_sec=0.05)
            traveled = abs(latest_distance['value'] - start)
            if traveled >= distance:
                break
            if time.time() > deadline:
                print('timed out waiting to reach target distance', file=sys.stderr)
                exit_code = 1
                break
            twist = Twist()
            twist.linear.x = speed
            pub.publish(twist)

        for _ in range(3):
            pub.publish(Twist())
            time.sleep(0.05)
    finally:
        node.destroy_subscription(sub)
        node.destroy_publisher(pub)
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(exit_code)


if __name__ == '__main__':
    main()
