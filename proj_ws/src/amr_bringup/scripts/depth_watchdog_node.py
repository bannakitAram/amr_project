#!/usr/bin/env python3
"""Cancels all Nav2 navigation when the HP60C depth pipeline stops.

The depth camera is the only sensor that sees obstacles below the LiDAR
plane (collision_monitor's FrontStop polygon relies on it). If it stops,
nothing reports an error: collision_monitor just sees no depth points,
which looks exactly like a clear path. This node watches both stages:

  - driver:  depth0/camera_info (published with every depth frame; tiny, so
             cheaper to watch than the image itself)
  - cloud:   /depth_camera/points (depth_image_proc container), subscribed
             raw so the ~900 KB clouds are never deserialized in Python

Either one silent for `timeout_s` (or never seen `startup_grace_s` after
start - the camera needs ~8 s to start streaming) -> FAULT: cancel every
goal on navigate_to_pose / navigate_through_poses / follow_waypoints, and
keep re-cancelling every `recancel_period_s` so new goals are refused too.
Both back for `recover_hold_s` -> clear the fault (navigation is NOT
resumed automatically). Publishes /depth_watchdog/ok (std_msgs/Bool) at 2 Hz.

What it can NOT detect: a covered or dirty lens. The factory floor is
mirror-glossy, so an all-empty depth image is also what a healthy camera
sees on a clear floor. Lens cleaning stays in the PM checklist.

Launched from depth_camera.launch.py, so it only runs when the camera is
enabled (bringup depth_camera:=true).

Monitoring layer only - NOT a safety function (ISO 3691-4): it depends on
ROS 2 and Nav2 being up. Protective stops stay in hardware/firmware and
collision_monitor. Same structure as amr_navigation's imu_watchdog_node.py.
"""
import rclpy
import rclpy.executors
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from action_msgs.srv import CancelGoal
from sensor_msgs.msg import CameraInfo, PointCloud2
from std_msgs.msg import Bool

NAV_ACTIONS = ['navigate_to_pose', 'navigate_through_poses', 'follow_waypoints']


class DepthWatchdog(Node):
    def __init__(self):
        super().__init__('depth_watchdog')
        # Camera runs ~8 Hz (125 ms); 1 s = ~8 missed frames.
        self.timeout_s = self.declare_parameter('timeout_s', 1.0).value
        self.startup_grace_s = self.declare_parameter('startup_grace_s', 20.0).value
        self.recover_hold_s = self.declare_parameter('recover_hold_s', 2.0).value
        self.recancel_period_s = self.declare_parameter('recancel_period_s', 1.0).value
        info_topic = self.declare_parameter(
            'camera_info_topic',
            '/ascamera_hp60c/camera_publisher/depth0/camera_info').value
        points_topic = self.declare_parameter('points_topic', '/depth_camera/points').value

        self.cancel_clients = {
            name: self.create_client(CancelGoal, f'/{name}/_action/cancel_goal')
            for name in NAV_ACTIONS
        }
        self.ok_pub = self.create_publisher(Bool, '/depth_watchdog/ok', 10)
        self.last = {'driver': None, 'cloud': None}
        self.create_subscription(CameraInfo, info_topic,
                                 lambda _m: self.on_msg('driver'), qos_profile_sensor_data)
        self.create_subscription(PointCloud2, points_topic,
                                 lambda _m: self.on_msg('cloud'), qos_profile_sensor_data,
                                 raw=True)
        self.create_timer(0.1, self.on_timer)
        self.create_timer(0.5, self.publish_ok)

        self.start_time = self.now_s()
        self.back_since = None
        self.fault = False
        self.last_cancel = 0.0

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_msg(self, stage):
        self.last[stage] = self.now_s()

    def silent_stages(self, t):
        out = []
        for stage, last in self.last.items():
            if last is None:
                if t - self.start_time > self.startup_grace_s:
                    out.append(stage)
            elif t - last > self.timeout_s:
                out.append(stage)
        return out

    def cancel_all(self):
        req = CancelGoal.Request()  # zero goal_id + zero stamp = cancel ALL goals
        for client in self.cancel_clients.values():
            if client.service_is_ready():
                client.call_async(req)
        self.last_cancel = self.now_s()

    def on_timer(self):
        t = self.now_s()
        silent = self.silent_stages(t)
        if silent:
            self.back_since = None
            if not self.fault:
                self.fault = True
                self.get_logger().error(
                    'depth camera lost (%s silent) - cancelling all navigation '
                    '(obstacles below the LiDAR plane are no longer detected)'
                    % ', '.join(silent))
                self.cancel_all()
            elif t - self.last_cancel > self.recancel_period_s:
                self.cancel_all()
        elif self.fault:
            if self.back_since is None:
                self.back_since = t
            elif t - self.back_since > self.recover_hold_s:
                self.fault = False
                self.back_since = None
                self.get_logger().warn(
                    'depth camera restored - navigation allowed again; '
                    're-send the goal')

    def publish_ok(self):
        seen = all(v is not None for v in self.last.values())
        self.ok_pub.publish(Bool(data=seen and not self.fault))


def main():
    rclpy.init()
    node = DepthWatchdog()
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
