#!/usr/bin/env python3
"""Cancels all Nav2 navigation when /imu/data stops.

ekf.yaml currently takes yaw rate from the gyro ONLY (wheel vyaw disabled
because of slip while turning). If /imu/data stops, the EKF sees no
rotation at all, heading freezes while the robot actually turns, and Nav2
drives off in the wrong direction. This node:

  - IMU silent for `timeout_s` (or never seen `startup_grace_s` after
    start) -> FAULT: cancel every goal on navigate_to_pose /
    navigate_through_poses / follow_waypoints (cancel-all request, so it
    stops goals from any source: goal picker, RViz, Foxglove), then keep
    re-cancelling every `recancel_period_s` so new goals are refused too.
  - IMU back for `recover_hold_s` -> clear the fault (navigation is NOT
    resumed automatically - the operator re-sends the goal after checking
    localization).
  - Publishes /imu_watchdog/ok (std_msgs/Bool) at 2 Hz for UIs.

Monitoring layer only - NOT a safety function (ISO 3691-4): it depends on
ROS 2 and Nav2 being up. Protective stops stay in hardware/firmware and
collision_monitor.
"""
import rclpy
import rclpy.executors
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from action_msgs.srv import CancelGoal
from sensor_msgs.msg import Imu
from std_msgs.msg import Bool

NAV_ACTIONS = ['navigate_to_pose', 'navigate_through_poses', 'follow_waypoints']


class ImuWatchdog(Node):
    def __init__(self):
        super().__init__('imu_watchdog')
        self.timeout_s = self.declare_parameter('timeout_s', 0.5).value
        self.startup_grace_s = self.declare_parameter('startup_grace_s', 10.0).value
        self.recover_hold_s = self.declare_parameter('recover_hold_s', 2.0).value
        self.recancel_period_s = self.declare_parameter('recancel_period_s', 1.0).value

        self.cancel_clients = {
            name: self.create_client(CancelGoal, f'/{name}/_action/cancel_goal')
            for name in NAV_ACTIONS
        }
        self.ok_pub = self.create_publisher(Bool, '/imu_watchdog/ok', 10)
        self.create_subscription(Imu, '/imu/data', self.on_imu, qos_profile_sensor_data)
        self.create_timer(0.1, self.on_timer)
        self.create_timer(0.5, self.publish_ok)

        self.start_time = self.now_s()
        self.last_imu = None
        self.imu_back_since = None
        self.fault = False
        self.last_cancel = 0.0

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_imu(self, _msg):
        t = self.now_s()
        if self.fault and (self.last_imu is None or t - self.last_imu > self.timeout_s):
            self.imu_back_since = t  # first sample after a gap
        self.last_imu = t

    def cancel_all(self):
        req = CancelGoal.Request()  # zero goal_id + zero stamp = cancel ALL goals
        for name, client in self.cancel_clients.items():
            if client.service_is_ready():
                client.call_async(req)
        self.last_cancel = self.now_s()

    def on_timer(self):
        t = self.now_s()
        if self.last_imu is None:
            silent = t - self.start_time > self.startup_grace_s
        else:
            silent = t - self.last_imu > self.timeout_s

        if silent:
            self.imu_back_since = None
            if not self.fault:
                self.fault = True
                self.get_logger().error(
                    '/imu/data lost - cancelling all navigation (EKF has no yaw source)')
                self.cancel_all()
            elif t - self.last_cancel > self.recancel_period_s:
                self.cancel_all()
        elif self.fault and self.imu_back_since is not None \
                and t - self.imu_back_since > self.recover_hold_s:
            self.fault = False
            self.imu_back_since = None
            self.get_logger().warn(
                '/imu/data restored - navigation allowed again; check localization '
                '(scan vs map) before sending a new goal')

    def publish_ok(self):
        self.ok_pub.publish(Bool(data=not self.fault and self.last_imu is not None))


def main():
    rclpy.init()
    node = ImuWatchdog()
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
