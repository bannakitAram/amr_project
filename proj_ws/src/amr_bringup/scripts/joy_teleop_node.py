#!/usr/bin/env python3
"""USB gamepad -> /cmd_vel for Manual mode (hold-to-run).

Hold LB and move the left stick: up/down = linear.x, left/right = angular.z.
Limits match the rosboard phone joystick (0.10 m/s, 0.15 rad/s, 0.2 m/s^2),
which drives the same way: straight to /cmd_vel, so collision_monitor is NOT
in this path. The operator must keep the path in sight.

Stops (zero Twist burst, then silent so the phone joystick can still drive):
  - LB released
  - /joy older than joy_timeout_s (gamepad unplugged, joy_node died)
LB pressed with the stick off-centre does not move the robot until the stick
has been centred once, so a stuck or knocked stick cannot start it.

Not a safety function (ISO 3691-4). Stopping is done by E-stop hardware and
the STM32 firmware, whose /cmd_vel timeout also covers this node dying.
"""
import math
import time

import rclpy
import rclpy.executors
from geometry_msgs.msg import Twist
from rclpy.node import Node
from sensor_msgs.msg import Joy

RATE_HZ = 20.0
STOP_BURST = 5          # zero messages sent after every stop
CENTRED = 0.1           # stick fraction that counts as centred


class JoyTeleop(Node):
    def __init__(self):
        super().__init__('joy_teleop')
        self.deadman_button = self.declare_parameter('deadman_button', 6).value  # LB
        # Button numbers differ between the pad's modes (Android mode = 15 buttons,
        # XInput = 11, where 6 is Back): refuse any other layout, do not guess.
        self.expected_buttons = self.declare_parameter('expected_buttons', 15).value
        self.axis_linear = self.declare_parameter('axis_linear', 1).value
        self.axis_angular = self.declare_parameter('axis_angular', 0).value
        self.max_linear = self.declare_parameter('max_linear', 0.10).value
        self.max_angular = self.declare_parameter('max_angular', 0.15).value
        self.max_accel = self.declare_parameter('max_accel', 0.2).value
        self.joy_timeout_s = self.declare_parameter('joy_timeout_s', 0.5).value

        self.joy = None
        self.joy_time = 0.0
        self.armed = False          # stick seen centred since LB went down
        self.out_linear = 0.0
        self.zeros_left = 0
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Joy, '/joy', self.on_joy, 10)
        self.create_timer(1.0 / RATE_HZ, self.tick)

    def on_joy(self, msg):
        needed_axes = max(self.axis_linear, self.axis_angular)
        if len(msg.axes) <= needed_axes or len(msg.buttons) != self.expected_buttons:
            self.get_logger().error(
                'Unexpected gamepad layout: %d axes, %d buttons'
                % (len(msg.axes), len(msg.buttons)), throttle_duration_sec=5.0)
            return
        self.joy = msg
        self.joy_time = time.monotonic()

    def stick(self, axis):
        value = self.joy.axes[axis]
        return max(-1.0, min(1.0, value)) if math.isfinite(value) else 0.0

    def stop(self):
        if self.armed or self.out_linear != 0.0:
            self.zeros_left = STOP_BURST
        self.armed = False
        self.out_linear = 0.0

    def tick(self):
        fresh = time.monotonic() - self.joy_time <= self.joy_timeout_s
        if not fresh or self.joy is None or not self.joy.buttons[self.deadman_button]:
            self.stop()
            if self.zeros_left > 0:
                self.zeros_left -= 1
                self.pub.publish(Twist())
            return

        linear = self.stick(self.axis_linear)
        angular = self.stick(self.axis_angular)
        if not self.armed:
            if abs(linear) > CENTRED or abs(angular) > CENTRED:
                self.get_logger().warn(
                    'LB held with the stick off-centre: centre the stick to drive',
                    throttle_duration_sec=2.0)
                return
            self.armed = True

        # Only speeding up is ramped; slowing down passes through at once
        # (same as the rosboard joystick).
        target = linear * self.max_linear
        step = self.max_accel / RATE_HZ
        if abs(target) > abs(self.out_linear) and target * self.out_linear >= 0.0:
            target = max(self.out_linear - step, min(self.out_linear + step, target))
        self.out_linear = target

        cmd = Twist()
        cmd.linear.x = self.out_linear
        cmd.angular.z = angular * self.max_angular
        self.pub.publish(cmd)


def main():
    rclpy.init()
    node = JoyTeleop()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
