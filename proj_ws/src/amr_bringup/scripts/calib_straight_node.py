#!/usr/bin/env python3
"""Left/right wheel-diameter-ratio (trim) calibration test.

Runs as its own OS process (launched via `ros2 run amr_bringup
calib_straight_node.py <distance_m> [speed_mps]`), same reasoning as
move_forward_node.py (no GIL sharing with a GUI).

Drives straight for distance_m, closed-loop on /wheel/raw_distance exactly
like move_forward_node.py (both wheels get the same commanded speed, no
heading correction).

IMPORTANT diagnostic note: the current firmware
(stm/src/motor_control_with_pid_optimized.cpp) uses a single shared
ENCODER_PPR for both wheels (confirmed identical - the earlier apparent PPR
mismatch turned out to be encoder debounce noise) and a PID with an
integral term, which drives each wheel's ENCODER-REPORTED steady-state
speed to match its target regardless of the left_dac_table/right_dac_table
feedforward shape. That means a per-point DAC-table tweak (what recent
commits have been hand-tuning: "trim right table +10 counts", the V9.15
shared-table experiment, the revert to V9.11) can only fix a TRANSIENT
speed mismatch during acceleration/launch - it cannot fix a genuine
steady-state drift, because feedforward shape doesn't change where the PID
integral settles. If this test shows drift that keeps growing linearly
over a long straight run (not just a one-time kick at launch), the real
cause is more likely a small physical wheel-diameter difference that the
single shared wheel_radius constant doesn't account for, which needs a
distinct per-wheel scale factor at the distance/speed conversion, not
another DAC-table edit.

Procedure:
  1. Mark the robot's start point and the direction of travel with a
     straight line/string on the floor.
  2. Run this node with a decently long distance_m (>= 3m; short runs make
     the drift too small to measure accurately, and won't distinguish a
     one-time launch kick from steady drift).
  3. Measure the ACTUAL lateral offset (perpendicular distance from the
     marked straight line to the robot's final position, in meters,
     positive = drifted to the robot's right) at the end.
  4. Enter that offset (and the real distance traveled, if it differs
     noticeably from distance_m) when prompted. The node estimates the
     wheel speed-ratio correction needed.

Geometry/derivation: for a small heading drift, offset y over path length D
gives drift angle dtheta ~= 2*y/D. With wheel_sep from calib_rotate_node.py,
the true right/left speed ratio error is
(e_r - e_l) ~= dtheta * wheel_sep / D. This is only a first-order estimate -
the script deliberately suggests moving only HALFWAY to the computed value
per iteration (apply, re-flash, re-run) rather than the full jump, both
because the small-angle approximation degrades for larger drift and to
avoid overshooting/oscillating a safety-relevant motion parameter.

This tool only measures and prints a recommendation - it does NOT edit
firmware, and it deliberately does NOT tell you which firmware knob to
turn (dac table vs. a new per-wheel distance-scale constant), since that
depends on whether the drift you measured is a launch-transient kick or a
steady-state trend (see the diagnostic note above). Apply whichever
correction fits, re-flash, and re-run this test to confirm before trusting
it in navigation/localization.

Exit codes:
  0 - reached target distance (calibration prompt follows on stdout)
  1 - timed out (encoder stalled / robot stuck)
  2 - no /wheel/raw_distance data received within the initial wait
Also stops cleanly (exit 0) on SIGINT/SIGTERM, publishing zero velocity
first.
"""
import signal
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from rclpy.node import Node
from std_msgs.msg import Float32

SPEED_DEFAULT = 0.15         # m/s - matches move_forward_node.py's default
TIMEOUT_FACTOR = 3.0
# Must match stm/src/motor_control_with_pid_optimized.cpp's `wheel_sep` and
# calib_rotate_node.py's WHEEL_SEP_NOMINAL.
WHEEL_SEP_NOMINAL = 0.472


def main():
    if len(sys.argv) < 2:
        print('usage: calib_straight_node.py <distance_m> [speed_mps]',
              file=sys.stderr)
        sys.exit(2)
    distance = float(sys.argv[1])
    speed = float(sys.argv[2]) if len(sys.argv) > 2 else SPEED_DEFAULT

    rclpy.init()
    node = Node('amr_calib_straight')
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
                return
            rclpy.spin_once(node, timeout_sec=0.1)
        if latest_distance['value'] is None:
            print('no /wheel/raw_distance data received', file=sys.stderr)
            exit_code = 2
            sys.exit(exit_code)

        start = latest_distance['value']
        deadline = time.time() + (distance / speed) * TIMEOUT_FACTOR + 5.0

        while not stop_requested['flag']:
            rclpy.spin_once(node, timeout_sec=0.05)
            traveled = abs(latest_distance['value'] - start)
            if traveled >= distance:
                break
            if time.time() > deadline:
                print('timed out waiting to reach target distance',
                      file=sys.stderr)
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

    if exit_code != 0 or stop_requested['flag']:
        sys.exit(exit_code)

    print(f'\nCommanded straight-line distance: {distance:.2f} m')
    try:
        offset_m = float(input(
            'Enter measured lateral offset in meters (+ = drifted right, '
            '- = drifted left): '))
        real_distance_str = input(
            f'Enter real distance traveled in meters [default {distance:.2f}]: '
        ).strip()
    except (ValueError, EOFError):
        print('no valid measurement entered - skipping calibration output',
              file=sys.stderr)
        sys.exit(0)

    real_distance = float(real_distance_str) if real_distance_str else distance
    if real_distance <= 0:
        print('real distance must be positive', file=sys.stderr)
        sys.exit(0)

    dtheta = 2.0 * offset_m / real_distance   # small-angle approximation
    speed_error_ratio = dtheta * WHEEL_SEP_NOMINAL / real_distance
    # speed_error_ratio > 0 means the right wheel is effectively running
    # FASTER than the left (robot drifted right); apply a correction factor
    # < 1 to whichever side is measured/scaled as "too fast".
    right_scale_factor = 1.0 - speed_error_ratio
    half_step_factor = 1.0 + (right_scale_factor - 1.0) * 0.5

    print(f'\nEstimated heading drift over the run: {dtheta:.4f} rad '
          f'({dtheta * 57.2958:.2f} deg)')
    print(f'Right-wheel effective speed scale needed (full correction): '
          f'{right_scale_factor:.4f}')
    print(f'Recommended next value to TRY (half-step): {half_step_factor:.4f}')
    print()
    print('Where to apply it depends on WHAT KIND of drift this was:')
    print('  - One-time kick right after start/launch, then straight after: '
          'this is a launch/acceleration transient - tune the launch-boost '
          'or startup-balance constants in')
    print('    stm/src/motor_control_with_pid_optimized.cpp instead of the '
          'DAC table.')
    print('  - Drift that keeps growing steadily over the whole run: this '
          'is a steady-state error the PID integral term cannot cancel on '
          'its own (e.g. a real per-wheel diameter difference) - it needs '
          'a distance/speed SCALE correction, not a DAC-table edit. Confirm '
          'with an embedded engineer before adding a new scale constant to '
          'the firmware\'s speed/odometry conversion.')
    print('Re-flash, re-run this test, and repeat until offset_m is small '
          'and stops shrinking further.')


if __name__ == '__main__':
    main()
