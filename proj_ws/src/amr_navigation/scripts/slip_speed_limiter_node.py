#!/usr/bin/env python3
"""Slows Nav2 down while the STM32 firmware reports wheel slip.

Subscribes /wheel/slip_detected (std_msgs/Bool, published every odom cycle
by the firmware: wheel yaw rate vs gyro z mismatch) and publishes
nav2_msgs/SpeedLimit on /speed_limit, which controller_server forwards to
MPPI's setSpeedLimit():

  - slip true            -> limit to `slip_speed_percent` % of max speed
  - slip clear for `release_hold_s` seconds -> remove the limit
  - slip continuously for `fault_after_s` seconds -> log an error (the
    robot is likely stuck or on a very slippery patch; operator check)
  - no /wheel/slip_detected for `stale_timeout_s` -> remove the limit and
    warn once (firmware/micro-ROS down: no data is not evidence of slip)

Monitoring/comfort only - NOT a safety function (ISO 3691-4): it depends
on micro-ROS, the IMU and Nav2 all running. Protective stops must stay in
hardware/firmware and collision_monitor.
"""
import rclpy
import rclpy.executors
from rclpy.node import Node
from std_msgs.msg import Bool
from nav2_msgs.msg import SpeedLimit


class SlipSpeedLimiter(Node):
    def __init__(self):
        super().__init__('slip_speed_limiter')
        self.slip_percent = self.declare_parameter('slip_speed_percent', 50.0).value
        self.release_hold_s = self.declare_parameter('release_hold_s', 2.0).value
        self.fault_after_s = self.declare_parameter('fault_after_s', 3.0).value
        self.stale_timeout_s = self.declare_parameter('stale_timeout_s', 1.0).value

        self.pub = self.create_publisher(SpeedLimit, 'speed_limit', 10)
        self.create_subscription(Bool, '/wheel/slip_detected', self.on_slip, 10)
        self.create_timer(0.1, self.on_timer)

        self.limited = False
        self.slip_since = None      # start of the current continuous slip
        self.last_slip_time = None  # last message with data == True
        self.last_msg_time = None
        self.fault_logged = False
        self.stale_logged = False

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def publish_limit(self, limited):
        msg = SpeedLimit()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.percentage = True
        msg.speed_limit = float(self.slip_percent) if limited else 0.0  # 0.0 = no limit
        self.pub.publish(msg)
        if limited != self.limited:
            if limited:
                self.get_logger().warn(f'wheel slip - limiting speed to {self.slip_percent:.0f}%')
            else:
                self.get_logger().info('wheel slip cleared - speed limit removed')
        self.limited = limited

    def on_slip(self, msg):
        t = self.now_s()
        self.last_msg_time = t
        self.stale_logged = False
        if msg.data:
            self.last_slip_time = t
            if self.slip_since is None:
                self.slip_since = t
            if not self.limited:
                self.publish_limit(True)
        else:
            self.slip_since = None
            self.fault_logged = False

    def on_timer(self):
        t = self.now_s()
        if self.last_msg_time is not None and t - self.last_msg_time > self.stale_timeout_s:
            if not self.stale_logged:
                self.get_logger().warn('/wheel/slip_detected stale - slip limiting inactive')
                self.stale_logged = True
            self.slip_since = None
            if self.limited:
                self.publish_limit(False)
            return

        if (self.slip_since is not None and not self.fault_logged
                and t - self.slip_since > self.fault_after_s):
            self.get_logger().error(
                f'continuous wheel slip > {self.fault_after_s:.0f} s - '
                'robot may be stuck or floor is slippery, operator check required')
            self.fault_logged = True

        if (self.limited and self.slip_since is None and self.last_slip_time is not None
                and t - self.last_slip_time > self.release_hold_s):
            self.publish_limit(False)


def main():
    rclpy.init()
    node = SlipSpeedLimiter()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
