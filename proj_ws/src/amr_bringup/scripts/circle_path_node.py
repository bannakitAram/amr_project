#!/usr/bin/env python3
"""Standalone "drive an arc/circle of a given radius" node.

Runs as its own OS process (launched via `ros2 run amr_bringup
circle_path_node.py <radius_m> <angle_deg> [angular_speed_rad_s]`), for the
same reason move_forward_node.py/square_path_node.py do - see
move_forward_node.py's module docstring.

Constant-curvature arc: holds angular.z fixed at angular_speed_rad_s and
derives linear.x = angular_speed_rad_s * radius_m so the path radius stays
constant regardless of radius (unlike square_path_node.py's separate
straight/turn legs). angle_deg = 360 drives a full circle back to the start
heading (not necessarily the start point - see below).

Closed-loop on /wheel/odom yaw (the STM32 firmware's own raw encoder-only
odometry), NOT /odometry/filtered. square_path_node.py's turns use the EKF-
fused heading because a 90 degree turn takes several seconds and EKF's lower
long-run drift matters more than its lag; measured on this node, /odometry/
filtered's fusion lag caused a consistent ~8% overshoot on both wheels (e.g.
5.39/7.32 measured vs 4.94/6.78 theoretical revolutions on a 90 degree, 0.2
rad/s arc - a real robot stopped several degrees late because the EKF yaw it
was checking hadn't caught up yet). /wheel/odom has no such fusion lag, at
the cost of the same wheel-slip drift raw_distance_traveled documents for
move_forward_node.py's straight legs - acceptable here since a single arc is
short enough that slip-driven yaw error stays small next to the lag it
avoids. Position is NOT tracked or corrected either way - real wheel slip/
heading error accumulates the same as any open-loop-heading dead-reckoning
move, so a 360 degree arc returns to the start heading but not exactly the
start point. This is not SLAM-corrected.

On completion (any exit code), prints a single line to stdout:
  WHEEL_REVS left=<float> right=<float>
giving each wheel's revolution count over the run (delta /wheel/*/pulse_count
divided by that wheel's encoder PPR), for the control panel to parse and
show the operator - useful for spotting left/right wheel-speed mismatch when
the driven circle comes out non-circular (see amr_control_panel.py's result
dialog after a Circle Path run).

Exit codes:
  0 - completed the requested arc angle
  1 - timed out (robot stuck/stalled, or odom not moving)
  2 - no /wheel/odom data received within the initial wait
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
from std_msgs.msg import Int32

# Encoder PPR (pulses per revolution) - the STM32 firmware's ground-truth
# value (motor_control_with_pid_optimized.cpp's ENCODER_PPR): a manual count
# over 20 revolutions per wheel, taken AFTER the encoder debounce filter was
# added, found the same true PPR on both wheels (identical motor/gearbox).
# wheel_joint_state_publisher.py's LEFT_PPR=623.3/RIGHT_PPR=558.6 look like a
# separate, per-wheel value but are actually the stale PRE-debounce noise-
# inflated readings (619-627 left / 550-568 right) that firmware's own
# comment explicitly says were an artifact, not a real left/right
# difference - not appropriate for a real measurement like this.
ENCODER_PPR = 516.85
LEFT_PPR = ENCODER_PPR
RIGHT_PPR = ENCODER_PPR

CIRCLE_ANGULAR_SPEED_DEFAULT = 0.2  # rad/s - matches the control panel's CIRCLE_ANGULAR_SPEED
LEG_TIMEOUT_FACTOR = 3.0            # abort if it takes > (expected time)*this
# /wheel/odom comes straight from the STM32 over micro-ROS (no EKF TimerAction
# delay to wait on), but the serial link itself still needs a few seconds to
# handshake after Mapping/Navigation is clicked (WAITING_AGENT -> connected
# in the firmware's own state machine) - a short wait here can still time
# out if this node is launched right after clicking instead of after the
# control panel shows "connected".
INITIAL_DATA_WAIT_S = 8.0
ARC_SLOWDOWN_ANGLE = 0.35           # rad remaining - drop to half angular speed to reduce overshoot
ARC_TOLERANCE = 0.02                # rad - close enough to call the arc done


def normalize_angle(angle):
    return math.atan2(math.sin(angle), math.cos(angle))


def yaw_from_quaternion(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                       1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def main():
    if len(sys.argv) < 3:
        print('usage: circle_path_node.py <radius_m> <angle_deg> '
              '[angular_speed_rad_s]', file=sys.stderr)
        sys.exit(2)
    radius = float(sys.argv[1])
    target_angle = math.radians(float(sys.argv[2]))
    angular_speed = (float(sys.argv[3]) if len(sys.argv) > 3
                      else CIRCLE_ANGULAR_SPEED_DEFAULT)

    rclpy.init()
    node = Node('amr_circle_path')
    pub = node.create_publisher(Twist, '/cmd_vel', 10)

    stop_requested = {'flag': False}

    def on_signal(signum, frame):
        stop_requested['flag'] = True

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    latest_yaw = {'value': None}
    pulse_l = {'value': None}
    pulse_r = {'value': None}

    def on_odometry(msg):
        latest_yaw['value'] = yaw_from_quaternion(msg.pose.pose.orientation)

    def on_pulse_l(msg):
        pulse_l['value'] = msg.data

    def on_pulse_r(msg):
        pulse_r['value'] = msg.data

    odom_sub = node.create_subscription(
        Odometry, '/wheel/odom', on_odometry, 10)
    pulse_l_sub = node.create_subscription(
        Int32, '/wheel/left/pulse_count', on_pulse_l, 10)
    pulse_r_sub = node.create_subscription(
        Int32, '/wheel/right/pulse_count', on_pulse_r, 10)

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

    def drive_arc():
        """Returns None on success, or an exit code on failure/abort."""
        start_yaw = latest_yaw['value']
        turned = 0.0
        last_yaw = start_yaw
        deadline = time.time() + (target_angle / angular_speed) * LEG_TIMEOUT_FACTOR + 5.0
        while not stop_requested['flag']:
            rclpy.spin_once(node, timeout_sec=0.05)
            current_yaw = latest_yaw['value']
            turned += normalize_angle(current_yaw - last_yaw)
            last_yaw = current_yaw
            remaining = target_angle - turned
            if remaining <= ARC_TOLERANCE:
                return None
            if time.time() > deadline:
                print('timed out on circle arc', file=sys.stderr)
                return 1
            angular_now = (angular_speed * 0.5
                           if remaining < ARC_SLOWDOWN_ANGLE else angular_speed)
            twist = Twist()
            twist.linear.x = angular_now * radius  # keep curvature (radius) constant through the slowdown
            twist.angular.z = angular_now
            pub.publish(twist)
        return 0

    exit_code = 0
    try:
        if not wait_for(lambda: latest_yaw['value'], INITIAL_DATA_WAIT_S):
            print('no /wheel/odom data received', file=sys.stderr)
            exit_code = 2
            return
        if stop_requested['flag']:
            return

        start_pulse_l = pulse_l['value']
        start_pulse_r = pulse_r['value']

        result = drive_arc()
        publish_stop()
        if result is not None:
            exit_code = result

        # Flush a moment for the last pulse_count updates (published at
        # 5Hz by the firmware) to arrive before reporting the delta.
        flush_deadline = time.time() + 0.5
        while time.time() < flush_deadline:
            rclpy.spin_once(node, timeout_sec=0.1)

        if start_pulse_l is not None and pulse_l['value'] is not None:
            revs_l = (pulse_l['value'] - start_pulse_l) / LEFT_PPR
        else:
            revs_l = None
        if start_pulse_r is not None and pulse_r['value'] is not None:
            revs_r = (pulse_r['value'] - start_pulse_r) / RIGHT_PPR
        else:
            revs_r = None
        if revs_l is not None and revs_r is not None:
            print(f'WHEEL_REVS left={revs_l:.3f} right={revs_r:.3f}')
        else:
            print('WHEEL_REVS unavailable (no /wheel/*/pulse_count data)', file=sys.stderr)
    finally:
        publish_stop()
        node.destroy_subscription(odom_sub)
        node.destroy_subscription(pulse_l_sub)
        node.destroy_subscription(pulse_r_sub)
        node.destroy_publisher(pub)
        node.destroy_node()
        rclpy.shutdown()
        sys.exit(exit_code)


if __name__ == '__main__':
    main()
