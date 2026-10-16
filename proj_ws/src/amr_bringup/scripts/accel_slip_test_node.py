#!/usr/bin/env python3
"""Accel / wheel-slip step test (Calibrate panel section 5).

Runs as its own OS process (launched via `ros2 run amr_bringup
accel_slip_test_node.py ...`), same reasoning as calib_straight_node.py.

For each accel in --accels (ascending), repeats --reps times, alternating
forward / backward so the robot ends near where it started:
  stand still STILL_S (IMU bias) -> ramp /cmd_vel at `accel` up to --vmax
  -> cruise CRUISE_S -> ramp down at --decel -> stand still
It records /wheel/odom vx and /imu/data ax for that accel and runs the same
analysis as slip_ratio_from_bag.py (analyse_arrays): peak gap between
encoder speed and IMU-integrated body speed per ramp, >5% = slip.

The ramp is generated here and published straight to /cmd_vel (like the
other calibration nodes), so it does NOT depend on velocity_smoother and
also bypasses collision_monitor. To compensate, the node watches
/scan_filtered and aborts (zero velocity, exit 5) if anything is inside
the travel corridor within OBSTACLE_STOP_M.
The STM32 ramp (MAX_ACCEL 0.5 / MAX_DECEL 1.2 m/s^2) still applies on top:
accels above 0.5 cannot be tested without a firmware change.

Output (stdout, one line each, parsed by amr_calib_panel.py):
  PROGRESS <text>
  RESULT {"accel": a, "slip_accel": s, "slip_decel": s, "events": n, ...}
  SUMMARY {"a_limit": a|null, "max_ok": a, "suggest_accel": a|null}

Exit codes:
  0 - finished (or stopped on first slipping accel with --stop-on-slip)
  2 - no /wheel/odom or /imu/data within the initial wait
  5 - obstacle in the travel corridor, aborted
  Also stops cleanly on SIGINT/SIGTERM (zero velocity, partial summary).
"""
import argparse
import json
import math
import os
import signal
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
import numpy as np
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu, LaserScan

from slip_ratio_from_bag import SLIP_LIMIT, analyse_arrays

RATE_HZ = 20.0
STILL_S = 2.0
CRUISE_S = 0.5
STM32_MAX_ACCEL = 0.5        # firmware MAX_ACCEL - higher commands are capped
OBSTACLE_STOP_M = 0.6        # from the lidar, along the travel direction
CORRIDOR_HALF_W = 0.32       # m - robot half width + margin
SUGGEST_FACTOR = 0.55        # same as slip_ratio_from_bag.py summary
# Raw odom/imu per accel, for re-checking a strange result offline.
DUMP_DIR = os.path.expanduser('~/.ros/accel_slip_test')


class StopTest(Exception):
    pass


class AccelSlipTest(Node):
    def __init__(self):
        super().__init__('amr_accel_slip_test')
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.odom = []
        self.imu = []
        self.blocked = {1: False, -1: False}   # per travel direction
        self.stop_requested = False
        self.create_subscription(Odometry, '/wheel/odom', self.on_odom, 20)
        self.create_subscription(Imu, '/imu/data', self.on_imu,
                                 qos_profile_sensor_data)
        self.create_subscription(LaserScan, '/scan_filtered', self.on_scan,
                                 qos_profile_sensor_data)

    # Time base: the message header stamp (both come from the STM32, so they
    # share one clock and are not distorted by executor/queue delays);
    # receive time only if a publisher leaves the stamp empty.
    @staticmethod
    def _stamp(msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        return t if t > 0.0 else time.time()

    def on_odom(self, msg):
        self.odom.append((self._stamp(msg), msg.twist.twist.linear.x))

    def on_imu(self, msg):
        self.imu.append((self._stamp(msg), msg.linear_acceleration.x))

    def on_scan(self, msg):
        # Lidar is mounted yawed by pi (amr.urdf.xacro joint_lidar), so a
        # beam at laser angle a points along robot angle a + pi.
        blocked = {1: False, -1: False}
        a = msg.angle_min
        for r in msg.ranges:
            if msg.range_min < r < msg.range_max:
                x = -r * math.cos(a)
                y = -r * math.sin(a)
                if abs(y) < CORRIDOR_HALF_W:
                    for d in (1, -1):
                        if 0.0 < d * x < OBSTACLE_STOP_M:
                            blocked[d] = True
            a += msg.angle_increment
        self.blocked = blocked

    def send(self, vx):
        t = Twist()
        t.linear.x = vx
        self.pub.publish(t)

    def tick(self):
        # Drain every pending callback for one control period. A single
        # spin_once handles only ONE callback, which at 20 Hz could not keep
        # up with /imu/data (50 Hz) + /wheel/odom (20 Hz): samples were
        # dropped and timestamped late, giving bogus >100% slip.
        end = time.monotonic() + 1.0 / RATE_HZ
        while True:
            left = end - time.monotonic()
            if left <= 0.0:
                break
            rclpy.spin_once(self, timeout_sec=min(left, 0.005))
        if self.stop_requested:
            raise StopTest('cancel')

    def hold(self, vx, seconds, direction):
        end = time.monotonic() + seconds
        while time.monotonic() < end:
            self.check_obstacle(direction, vx)
            self.send(vx)
            self.tick()

    def ramp(self, v0, v1, accel, direction):
        t0 = time.monotonic()
        dur = abs(v1 - v0) / accel
        while True:
            k = min((time.monotonic() - t0) / dur, 1.0)
            v = v0 + (v1 - v0) * k
            self.check_obstacle(direction, v)
            self.send(v)
            if k >= 1.0:
                return
            self.tick()

    def check_obstacle(self, direction, vx):
        if vx != 0.0 and self.blocked[direction]:
            raise StopTest('obstacle')

    def zero(self):
        for _ in range(5):
            self.send(0.0)
            time.sleep(0.05)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--accels', default='0.1,0.2,0.3,0.4,0.5',
                    help='comma list, m/s^2, tested in ascending order')
    ap.add_argument('--vmax', type=float, default=0.2)
    ap.add_argument('--decel', type=float, default=0.5)
    ap.add_argument('--reps', type=int, default=2)
    ap.add_argument('--stop-on-slip', action='store_true')
    args = ap.parse_args()
    accels = sorted(float(a) for a in args.accels.split(',') if a.strip())

    rclpy.init()
    node = AccelSlipTest()

    def on_signal(signum, frame):
        node.stop_requested = True

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)

    def say(tag, payload):
        print(f'{tag} {payload}', flush=True)

    results = []
    exit_code = 0
    try:
        wait_end = time.monotonic() + 3.0
        while time.monotonic() < wait_end and not (node.odom and node.imu):
            node.tick()
        if not (node.odom and node.imu):
            say('PROGRESS', 'no /wheel/odom or /imu/data - check bringup')
            exit_code = 2
            raise StopTest('nodata')

        for accel in accels:
            if accel > STM32_MAX_ACCEL:
                say('PROGRESS', f'skip {accel:.2f}: above STM32 MAX_ACCEL '
                    f'{STM32_MAX_ACCEL} (firmware would cap it)')
                continue
            node.odom.clear()
            node.imu.clear()
            for rep in range(args.reps):
                d = 1 if rep % 2 == 0 else -1
                say('PROGRESS', f'accel {accel:.2f} m/s^2 - run {rep + 1}/'
                    f'{args.reps} ({"forward" if d > 0 else "backward"})')
                node.hold(0.0, STILL_S, d)
                node.ramp(0.0, d * args.vmax, accel, d)
                node.hold(d * args.vmax, CRUISE_S, d)
                node.ramp(d * args.vmax, 0.0, args.decel, d)
            node.hold(0.0, STILL_S, 1)

            od = list(zip(*node.odom))
            im = list(zip(*node.imu))
            try:
                os.makedirs(DUMP_DIR, exist_ok=True)
                np.savez(os.path.join(
                    DUMP_DIR, time.strftime('%Y%m%d_%H%M%S') + f'_a{accel:g}.npz'),
                    odom=np.array(node.odom), imu=np.array(node.imu))
            except OSError:
                pass
            # event threshold: half the tested accel (see --event-a note)
            bias, rows = analyse_arrays(od[0], od[1], im[0], im[1],
                                        event_a=min(0.08, accel / 2),
                                        label=f'a={accel}')
            acc = [r['slip'] for r in rows if r['kind'] == 'accel']
            dec = [r['slip'] for r in rows if r['kind'] == 'decel']
            res = {
                'accel': accel,
                'slip_accel': max(acc) if acc else None,
                'slip_decel': max(dec) if dec else None,
                'events': len([r for r in rows if r['kind'] != 'bounce']),
                # wheel speed past zero (other direction) right after a stop
                'stop_overshoot': max([r['overshoot'] for r in rows], default=0.0),
                'expected_events': 2 * args.reps,
                'imu_bias': bias,
            }
            results.append(res)
            say('RESULT', json.dumps(res))
            if (args.stop_on_slip and res['slip_accel'] is not None
                    and res['slip_accel'] > SLIP_LIMIT):
                say('PROGRESS', f'slip at {accel:.2f} - stopping')
                break
    except StopTest as e:
        if str(e) == 'obstacle':
            say('PROGRESS', 'obstacle in travel path - aborted')
            exit_code = 5
    finally:
        node.zero()
        node.destroy_node()
        rclpy.shutdown()

    slipping = [r['accel'] for r in results
                if r['slip_accel'] is not None and r['slip_accel'] > SLIP_LIMIT]
    clean = [r['accel'] for r in results
             if r['slip_accel'] is not None and r['slip_accel'] <= SLIP_LIMIT]
    a_limit = min(slipping) if slipping else None
    say('SUMMARY', json.dumps({
        'a_limit': a_limit,
        'max_ok': max(clean) if clean else None,
        'suggest_accel': round(a_limit * SUGGEST_FACTOR, 2) if a_limit else None,
    }))
    sys.exit(exit_code)


if __name__ == '__main__':
    main()
