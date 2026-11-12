#!/usr/bin/env python3
"""USB gamepad -> /cmd_vel for Manual mode (hold-to-run).

Hold LB and move the left stick: up/down = linear.x, left/right = angular.z.
Normal max 0.30 m/s (= Nav2 max_velocity), 0.15 rad/s, 0.2 m/s^2. Drives
like the rosboard phone joystick: straight to /cmd_vel, so collision_monitor is NOT
in this path. The operator must keep the path in sight.

Stops (zero Twist burst, then silent so the phone joystick can still drive):
  - LB released
  - /joy older than joy_timeout_s (gamepad unplugged, joy_node died)
Speed: every press starts in Slow (0.10 m/s, 0.05 rad/s). Press RB while
holding LB to toggle Slow <-> Normal (max_linear / max_angular). Releasing LB
or any other stop drops back to Slow. Slow can only be lower than Normal.

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
        self.max_linear = self.declare_parameter('max_linear', 0.30).value
        self.max_angular = self.declare_parameter('max_angular', 0.15).value
        self.speed_button = self.declare_parameter('speed_button', 7).value  # RB
        self.slow_scale = min(1.0, max(0.0, self.declare_parameter('slow_scale', 1.0 / 3.0).value))
        self.max_accel = self.declare_parameter('max_accel', 0.2).value
        self.joy_timeout_s = self.declare_parameter('joy_timeout_s', 0.5).value

        self.joy = None
        self.joy_time = 0.0
        self.armed = False          # stick seen centred since LB went down
        self.out_linear = 0.0
        self.zeros_left = 0
        self.normal = False         # False = Slow
        self.speed_button_prev = 0
        self.pub = self.create_publisher(Twist, '/cmd_vel', 10)
        self.create_subscription(Joy, '/joy', self.on_joy, 10)
        self.create_timer(1.0 / RATE_HZ, self.tick)

    def on_joy(self, msg):
        needed_axes = max(self.axis_linear, self.axis_angular)
        if (len(msg.axes) <= needed_axes or len(msg.buttons) != self.expected_buttons
                or max(self.deadman_button, self.speed_button) >= len(msg.buttons)):
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
        if self.normal:
            self.get_logger().info('Speed: Slow')
        self.normal = False

    def tick(self):
        fresh = time.monotonic() - self.joy_time <= self.joy_timeout_s
        if not fresh or self.joy is None or not self.joy.buttons[self.deadman_button]:
            self.stop()
            self.speed_button_prev = self.joy.buttons[self.speed_button] if fresh and self.joy else 0
            if self.zeros_left > 0:
                self.zeros_left -= 1
                self.pub.publish(Twist())
            return

        pressed = self.joy.buttons[self.speed_button]
        if pressed and not self.speed_button_prev:
            self.normal = not self.normal
            self.get_logger().info('Speed: ' + ('Normal' if self.normal else 'Slow'))
        self.speed_button_prev = pressed
        scale = 1.0 if self.normal else self.slow_scale

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
        target = linear * self.max_linear * scale
        step = self.max_accel / RATE_HZ
        if abs(target) > abs(self.out_linear) and target * self.out_linear >= 0.0:
            target = max(self.out_linear - step, min(self.out_linear + step, target))
        self.out_linear = target

        cmd = Twist()
        cmd.linear.x = self.out_linear
        cmd.angular.z = angular * self.max_angular * scale
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
