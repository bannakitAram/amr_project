#!/usr/bin/env python3
"""Scan gate in front of slam_toolbox: relays LaserScan `scan_in` (/scan_operator_filtered) ->
`scan_out` only while enabled.

Lets Mapping start with SLAM running but not building a map until the
operator presses "Start scan" (AMR Control panel / phone). slam_toolbox's
own pause_new_measurements service can't do this cleanly: it is a pure
toggle with no state in its response, and the first scan is already
integrated before a pause call can arrive after startup.

Only slam_toolbox consumes `scan_out`; obstacle/safety inputs (/scan,
bringup's /scan_filtered for costmaps + collision_monitor) are untouched.

  ~/set_enabled (std_srvs/SetBool)  - idempotent on/off
  ~/enabled     (std_msgs/Bool)     - current state, latched
  param start_enabled (bool, default true)
"""
import rclpy
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool
from std_srvs.srv import SetBool


class ScanGate(Node):
    def __init__(self):
        super().__init__('scan_gate')
        self.enabled = bool(self.declare_parameter('start_enabled', True).value)
        self.pub = self.create_publisher(LaserScan, 'scan_out', qos_profile_sensor_data)
        self.create_subscription(LaserScan, 'scan_in', self.on_scan, qos_profile_sensor_data)
        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.state_pub = self.create_publisher(Bool, '~/enabled', latched)
        self.create_service(SetBool, '~/set_enabled', self.on_set)
        self.publish_state()
        self.get_logger().info(f'scan gate {"open" if self.enabled else "closed"} at start')

    def on_scan(self, msg):
        if self.enabled:
            self.pub.publish(msg)

    def on_set(self, req, resp):
        if self.enabled != req.data:
            self.enabled = req.data
            self.get_logger().info(f'scan gate {"open" if self.enabled else "closed"}')
        self.publish_state()
        resp.success = True
        resp.message = 'open' if self.enabled else 'closed'
        return resp

    def publish_state(self):
        self.state_pub.publish(Bool(data=self.enabled))


def main():
    rclpy.init()
    node = ScanGate()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
