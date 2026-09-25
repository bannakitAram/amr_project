#!/usr/bin/env python3
"""Bridges the rosboard goal_picker web page to Nav2's autonomous
navigation actions.

Long-running node (unlike amr_bringup's move_forward_node.py /
square_path_node.py, which are one-shot processes launched per button
press) - it stays up for the whole Navigation-mode session, launched
alongside the rest of the Nav2 stack by navigation.launch.py, and executes
whatever goal/path the operator sends from their phone next.

Subscribes to the same three topics rosboard.py publishes on behalf of the
goal_picker page (see that page and rosboard/rosboard.py's on_nav_goal/
on_nav_path/on_nav_cancel for the validation/clamping done before anything
reaches here):
  /rosboard_nav_goal   (geometry_msgs/PoseStamped)  - single tap-to-navigate
  /rosboard_nav_path   (nav_msgs/Path)              - drawn path to follow
  /rosboard_nav_cancel (std_msgs/Empty)             - cancel button

A new goal or path always preempts whatever is currently running (cancels
the in-progress task first) - the operator's most recent tap wins, same as
re-pressing "2D Nav Goal" in RViz.
"""
import sys

import rclpy
from geometry_msgs.msg import PoseStamped
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import Path
from std_msgs.msg import Empty


def restamp(pose_stamped, navigator):
    pose_stamped.header.stamp = navigator.get_clock().now().to_msg()
    return pose_stamped


def main():
    rclpy.init()
    navigator = BasicNavigator()
    navigator.get_logger().info('nav_goal_executor_node: waiting for Nav2 to become active...')
    navigator.waitUntilNav2Active()
    navigator.get_logger().info('nav_goal_executor_node: Nav2 active, ready for goals')

    task_active = {'value': False}
    pending = {'value': None}  # ('pose', PoseStamped) | ('path', Path) | ('cancel', None)

    def on_goal(msg):
        pending['value'] = ('pose', msg)

    def on_path(msg):
        pending['value'] = ('path', msg)

    def on_cancel(_msg):
        pending['value'] = ('cancel', None)

    navigator.create_subscription(PoseStamped, '/rosboard_nav_goal', on_goal, 10)
    navigator.create_subscription(Path, '/rosboard_nav_path', on_path, 10)
    navigator.create_subscription(Empty, '/rosboard_nav_cancel', on_cancel, 10)

    try:
        while rclpy.ok():
            rclpy.spin_once(navigator, timeout_sec=0.2)

            if pending['value'] is not None:
                kind, msg = pending['value']
                pending['value'] = None

                if task_active['value']:
                    navigator.cancelTask()
                    task_active['value'] = False

                if kind == 'pose':
                    navigator.get_logger().info(
                        'nav_goal_executor_node: new goal (%.2f, %.2f)'
                        % (msg.pose.position.x, msg.pose.position.y))
                    navigator.goToPose(restamp(msg, navigator))
                    task_active['value'] = True
                elif kind == 'path':
                    if not msg.poses:
                        continue
                    navigator.get_logger().info(
                        'nav_goal_executor_node: new path with %d waypoints'
                        % len(msg.poses))
                    poses = [restamp(p, navigator) for p in msg.poses]
                    navigator.goThroughPoses(poses)
                    task_active['value'] = True
                else:  # 'cancel'
                    navigator.get_logger().info('nav_goal_executor_node: goal cancelled by operator')

            if task_active['value'] and navigator.isTaskComplete():
                result = navigator.getResult()
                if result == TaskResult.SUCCEEDED:
                    navigator.get_logger().info('nav_goal_executor_node: task succeeded')
                elif result == TaskResult.CANCELED:
                    navigator.get_logger().info('nav_goal_executor_node: task canceled')
                else:
                    navigator.get_logger().warn('nav_goal_executor_node: task failed')
                task_active['value'] = False
    except KeyboardInterrupt:
        pass
    finally:
        if task_active['value']:
            navigator.cancelTask()
        navigator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
