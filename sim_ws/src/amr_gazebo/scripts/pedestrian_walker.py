#!/usr/bin/env python3
"""
pedestrian_walker.py

Drives the "pedestrian" model in complex_world.sdf back and forth along the
main corridor (world x-axis, y ~ 0) at walking speed, so it crosses the
AMR's typical path and can be used to test Nav2 dynamic obstacle avoidance.

The pedestrian model never rotates (yaw stays 0), so the VelocityControl
plugin's body-frame linear.x is identical to world-frame x - we only ever
need to flip the sign of linear.x to reverse direction, no angular motion
required.

Publishes: /pedestrian/cmd_vel (geometry_msgs/msg/Twist)
  -> bridged by pedestrian.launch.py to the Gazebo topic
     /model/pedestrian/cmd_vel, which the VelocityControl plugin consumes.

Usage:
    ros2 run ... (or, since this package has no python entry point,
    it is launched directly as a script - see pedestrian.launch.py)

Parameters (ROS 2 params, can be set with --ros-args -p name:=value):
    speed        (float, default 0.5)  walking speed, m/s
    x_min        (float, default -6.0) west turnaround point, m
    x_max        (float, default  6.0) east turnaround point, m
    publish_rate (float, default 20.0) Hz
"""
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import Twist


class PedestrianWalker(Node):

    def __init__(self):
        super().__init__('pedestrian_walker')

        self.declare_parameter('speed', 0.5)
        self.declare_parameter('x_min', -6.0)
        self.declare_parameter('x_max', 6.0)
        self.declare_parameter('publish_rate', 20.0)
        self.declare_parameter('start_x', -4.0)

        self.speed = self.get_parameter('speed').value
        self.x_min = self.get_parameter('x_min').value
        self.x_max = self.get_parameter('x_max').value
        rate = self.get_parameter('publish_rate').value

        # We don't have real odometry feedback for this simple kinematic
        # model, so we dead-reckon its x position from the commanded
        # velocity and the timer period. This is accurate because
        # VelocityControl applies exactly the commanded velocity every
        # step (no slip, no dynamics fighting it).
        self.x = self.get_parameter('start_x').value
        self.direction = 1.0  # +1 = walking toward x_max, -1 = toward x_min

        self.pub = self.create_publisher(Twist, '/pedestrian/cmd_vel', 10)
        self.period = 1.0 / rate
        self.timer = self.create_timer(self.period, self.on_timer)

        self.get_logger().info(
            f'pedestrian_walker: patrolling x in [{self.x_min}, {self.x_max}] '
            f'at {self.speed} m/s, starting at x={self.x}'
        )

    def on_timer(self):
        # Turn around at the ends of the corridor.
        if self.x >= self.x_max:
            self.direction = -1.0
        elif self.x <= self.x_min:
            self.direction = 1.0

        vx = self.direction * self.speed
        self.x += vx * self.period

        msg = Twist()
        msg.linear.x = vx
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = PedestrianWalker()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        # Stop the pedestrian on exit instead of leaving it drifting.
        stop = Twist()
        node.pub.publish(stop)
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
