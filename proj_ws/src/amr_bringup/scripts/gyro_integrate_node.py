#!/usr/bin/env python3
"""Integrates /imu/data angular_velocity.z until SIGINT/SIGTERM, then prints
'GYRO_DEG=<total degrees> samples=<n>' (CCW positive) on stdout - the
format amr_calib_panel.py's _stop_gyro() parses.

Used by amr_calib_panel.py's gyro-scale section: start it, rotate the robot
an exactly known angle (e.g. 5 full turns by hand against a floor mark,
motors off), stop it. Never commands motion itself.

dt comes from header stamps; a gap outside (0, MAX_DT] (startup, a dropped
burst, or the firmware's periodic time re-sync) falls back to the nominal
period so one bad stamp can't corrupt the total.

Exit codes: 0 ok, 2 no /imu/data received.
"""
import signal
import sys

import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import Imu

NOMINAL_DT = 0.02   # firmware IMU_PUBLISH_PERIOD_MS = 20
MAX_DT = 0.2


def main():
    rclpy.init()
    node = Node('amr_gyro_integrate')
    st = {'deg': 0.0, 'n': 0, 'last': None, 'stop': False}

    def on_imu(msg):
        t = msg.header.stamp.sec + msg.header.stamp.nanosec * 1e-9
        if st['last'] is not None:
            dt = t - st['last']
            if not 0.0 < dt <= MAX_DT:
                dt = NOMINAL_DT
            st['deg'] += msg.angular_velocity.z * dt * 57.29577951308232
        st['last'] = t
        st['n'] += 1

    def on_signal(signum, frame):
        st['stop'] = True

    signal.signal(signal.SIGINT, on_signal)
    signal.signal(signal.SIGTERM, on_signal)
    node.create_subscription(Imu, '/imu/data', on_imu, qos_profile_sensor_data)
    while not st['stop']:
        rclpy.spin_once(node, timeout_sec=0.1)
    print(f"GYRO_DEG={st['deg']:.3f} samples={st['n']}", flush=True)
    node.destroy_node()
    rclpy.try_shutdown()
    sys.exit(0 if st['n'] > 1 else 2)


if __name__ == '__main__':
    main()
