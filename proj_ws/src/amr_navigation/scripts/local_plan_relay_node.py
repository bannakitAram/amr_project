#!/usr/bin/env python3
"""Relays MPPI's chosen (optimal) local trajectory as a small nav_msgs/Path.

With `visualize: true`, MPPI publishes /trajectories (MarkerArray) holding
the optimal trajectory (namespace "Optimal Trajectory", one marker per
point) plus sampled candidate trajectories - thousands of markers, far too
heavy to push to the goal_picker page over WiFi. This node keeps only the
optimal one and republishes it as /mppi_local_plan (nav_msgs/Path, in the
markers' frame - the local costmap's global frame, normally odom).
"""
import rclpy
import rclpy.executors
from rclpy.node import Node
from geometry_msgs.msg import PoseStamped
from nav_msgs.msg import Path
from visualization_msgs.msg import MarkerArray

OPTIMAL_NS = 'Optimal Trajectory'


class LocalPlanRelay(Node):
    def __init__(self):
        super().__init__('local_plan_relay')
        self.pub = self.create_publisher(Path, '/mppi_local_plan', 10)
        self.create_subscription(MarkerArray, '/trajectories', self.on_markers, 10)

    def on_markers(self, msg):
        markers = sorted(
            (m for m in msg.markers if m.ns == OPTIMAL_NS and m.action == m.ADD),
            key=lambda m: m.id)
        if not markers:
            return
        path = Path()
        path.header = markers[0].header
        for m in markers:
            ps = PoseStamped()
            ps.header = m.header
            ps.pose = m.pose
            path.poses.append(ps)
        self.pub.publish(path)


def main():
    rclpy.init()
    node = LocalPlanRelay()
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
