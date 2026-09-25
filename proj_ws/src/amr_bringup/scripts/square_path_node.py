#!/usr/bin/env python3
"""Standalone "drive a square and return to the start point" node.

Runs as its own OS process (launched via `ros2 run amr_bringup
square_path_node.py <side_m> [speed_mps] [turn_speed_rad_s]`), for the same
reason move_forward_node.py does - see that node's module docstring. Four
legs of {drive straight <side_m>, turn 90 deg CCW}, closed-loop the same way
the control panel's other motion commands are:
  - straight legs: /wheel/raw_distance (cumulative signed distance from the
    STM32 firmware) - NOT /odometry/filtered, whose ODOM_FILTER_ALPHA
    low-pass lags real distance during the accel ramp (see
    move_forward_node.py).
  - turns: /odometry/filtered yaw. There is no lower-lag heading source
    available (raw_distance is linear-only), and the EKF's fused heading
    still drifts far less over one 90 degree turn than wheel-only odometry
    would.
Four 90 degree CCW turns sum to 360 degrees, so the heading at the end
matches the start; with no wheel slip the position also returns to the
start point. Real slip/heading error accumulates across the 4 legs same as
any open-loop-heading dead-reckoning move - this is not SLAM-corrected.

Exit codes:
  0 - completed all 4 legs
  1 - timed out on a straight leg (encoder stalled / robot stuck)
  2 - no /wheel/raw_distance data received within the initial wait
  3 - timed out on a turn leg (robot stuck / stalled)
  4 - no /odometry/filtered data received within the initial wait
Also stops cleanly (exit 0) on SIGINT/SIGTERM (used by the control panel's
Cancel button and app-close handler), publishing zero velocity first.
"""
import math
import signal
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float32

SQUARE_SPEED_DEFAULT = 0.15        # m/s - matches move_forward_node's default
SQUARE_TURN_SPEED_DEFAULT = 0.3    # rad/s - conservative in-place turn rate
LEG_TIMEOUT_FACTOR = 3.0           # abort a leg if it takes > (expected time)*this
TURN_ANGLE = math.pi / 2           # 90 degrees per corner, CCW (REP 103 +yaw)
TURN_SLOWDOWN_ANGLE = 0.35         # rad remaining - drop to half speed to reduce overshoot
TURN_TOLERANCE = 0.02              # rad - close enough to call the turn done


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                       1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def main():
    if len(sys.argv) < 2:
        print('usage: square_path_node.py <side_m> [speed_mps] '
              '[turn_speed_rad_s]', file=sys.stderr)
        sys.exit(2)
    side = float(sys.argv[1])
    speed = float(sys.argv[2]) if len(sys.argv) > 2 else SQUARE_SPEED_DEFAULT
    turn_speed = float(sys.argv[3]) if len(sys.argv) > 3 else SQUARE_TURN_SPEED_DEFAULT

    rclpy.init()
    node = Node('amr_square_path')
    pub = node.create_publisher(Twist, '/cmd_vel', 10)

    stop_requested = {'flag': False}

    def on_signal(signum, frame):
        stop_requested['flag'] = True

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    latest_distance = {'value': None}
    latest_yaw = {'value': None}

    def on_raw_distance(msg):
        latest_distance['value'] = msg.data

    def on_odometry(msg):
        latest_yaw['value'] = yaw_from_quaternion(msg.pose.pose.orientation)

    dist_sub = node.create_subscription(
        Float32, '/wheel/raw_distance', on_raw_distance, 10)
    odom_sub = node.create_subscription(
        Odometry, '/odometry/filtered', on_odometry, 10)

    def wait_for(getter, timeout_s):
        wait_start = time.time()
        while getter() is None and time.time() - wait_start < timeout_s:
            if stop_requested['flag']:
                return True
            rclpy.spin_once(node, timeout_sec=0.1)
        return getter() is not None

    def publish_stop():
        for _ in range(3):
            pub.publish(Twist())
            time.sleep(0.05)

    def drive_leg():
        """Returns None on success, or an exit code on failure/abort."""
        start = latest_distance['value']
        deadline = time.time() + (side / speed) * LEG_TIMEOUT_FACTOR + 5.0
        while not stop_requested['flag']:
            rclpy.spin_once(node, timeout_sec=0.05)
            traveled = abs(latest_distance['value'] - start)
            if traveled >= side:
                return None
            if time.time() > deadline:
                print('timed out on straight leg', file=sys.stderr)
                return 1
            twist = Twist()
            twist.linear.x = speed
            pub.publish(twist)
        return 0

    def turn_leg():
        """Returns None on success, or an exit code on failure/abort."""
        start_yaw = latest_yaw['value']
        turned = 0.0
        last_yaw = start_yaw
        deadline = time.time() + (TURN_ANGLE / turn_speed) * LEG_TIMEOUT_FACTOR + 5.0
        while not stop_requested['flag']:
            rclpy.spin_once(node, timeout_sec=0.05)
            current_yaw = latest_yaw['value']
            turned += normalize_angle(current_yaw - last_yaw)
            last_yaw = current_yaw
            remaining = TURN_ANGLE - turned
            if remaining <= TURN_TOLERANCE:
                return None
            if time.time() > deadline:
                print('timed out on turn leg', file=sys.stderr)
                return 3
            twist = Twist()
            twist.angular.z = (turn_speed * 0.5
                                if remaining < TURN_SLOWDOWN_ANGLE else turn_speed)
            pub.publish(twist)
        return 0

    exit_code = 0
    try:
        if not wait_for(lambda: latest_distance['value'], 3.0):
            print('no /wheel/raw_distance data received', file=sys.stderr)
            exit_code = 2
            return
        if stop_requested['flag']:
            return
        if not wait_for(lambda: latest_yaw['value'], 3.0):
            print('no /odometry/filtered data received', file=sys.stderr)
            exit_code = 4
            return

        for _ in range(4):
            if stop_requested['flag']:
                break
            result = drive_leg()
            publish_stop()
            if result is not None:
                exit_code = result
                break
            if stop_requested['flag']:
                break
            result = turn_leg()
            publish_stop()
            if result is not None:
                exit_code = result
                break
    finally:
        publish_stop()
        node.destroy_subscription(dist_sub)
        node.destroy_subscription(odom_sub)
        node.destroy_publisher(pub)
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(exit_code)


if __name__ == '__main__':
    main()
