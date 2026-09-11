#!/usr/bin/env python3
"""Converts raw wheel encoder pulse counts (/wheel/left|right/pulse_count)
into /joint_states so robot_state_publisher can compute the TF for the two
continuous drive-wheel joints. Without this, RViz/Foxglove show a "no
transform" error on left_drive_wheel_link and right_drive_wheel_link, since
robot_state_publisher cannot resolve continuous joints on its own.

PPR (pulses per revolution) values below are the per-wheel averages of the
speed-interpolated calibration table baked into the STM32 firmware
(stm/src/calibrate.cpp, ppr_l_table / ppr_r_table) - close enough for
visualization. The firmware's own /wheel/odom (fed into the EKF) remains
the authoritative odometry source; this node only drives the wheel spin
you see in the 3D view.
"""
import math

import rclpy
from rclpy.node import Node
from std_msgs.msg import Int32
from sensor_msgs.msg import JointState

LEFT_PPR = 623.3
RIGHT_PPR = 558.6


class WheelJointStatePublisher(Node):
    def __init__(self):
        super().__init__('wheel_joint_state_publisher')
        self._left_pulse = 0
        self._right_pulse = 0
        self.create_subscription(Int32, 'wheel/left/pulse_count', self._on_left, 10)
        self.create_subscription(Int32, 'wheel/right/pulse_count', self._on_right, 10)
        self._pub = self.create_publisher(JointState, 'joint_states', 10)
        self.create_timer(0.05, self._publish)

    def _on_left(self, msg):
        self._left_pulse = msg.data

    def _on_right(self, msg):
        self._right_pulse = msg.data

    def _publish(self):
        msg = JointState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.name = ['joint_left_drive_wheel', 'joint_right_drive_wheel']
        msg.position = [
            (self._left_pulse / LEFT_PPR) * 2.0 * math.pi,
            (self._right_pulse / RIGHT_PPR) * 2.0 * math.pi,
        ]
        self._pub.publish(msg)


def main():
    rclpy.init()
    node = WheelJointStatePublisher()
    try:
        rclpy.spin(node)
    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
