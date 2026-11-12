#!/usr/bin/env python3
"""Decides the tower light pattern and publishes it on /light/cmd at 2 Hz.

hook_bridge.py forwards it to the Arduino Mega (L<n>), which drives the
24 V R/Y/G tower. Highest priority first:

  FAULT     red steady         a sensor watchdog reports not-ok
                               (/depth_watchdog/ok, /imu_watchdog/ok)
  OBSTACLE  yellow steady      Nav2 goal active and collision_monitor is
                               holding the robot (/cmd_vel_smoothed asks to
                               drive, final /cmd_vel is 0)
  AUTO      yellow blink 1 Hz  Nav2 goal active (also while planning or
                               waiting - the robot can start at any moment)
  MANUAL    yellow blink fast  moving / commanded with no Nav2 goal
                               (phone joystick, Foxglove teleop)
  IDLE      green steady       none of the above

The Mega shows on its own, overriding this: hook FAULT (red steady), hook
moving (yellow + green blink) and link lost (red blink) - the last one if
this node, hook_bridge, ROS or the cable stops: hook_bridge only forwards
while /light/cmd keeps arriving.

Warning device only - NOT a safety function (ISO 3691-4). Stopping is done
by E-stop hardware, collision_monitor and the STM32 firmware.
"""
import rclpy
import rclpy.executors
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile, ReliabilityPolicy
from action_msgs.msg import GoalStatus, GoalStatusArray
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import Bool, String

NAV_ACTIONS = ['navigate_to_pose', 'navigate_through_poses', 'follow_waypoints']
ACTIVE = (GoalStatus.STATUS_ACCEPTED, GoalStatus.STATUS_EXECUTING,
          GoalStatus.STATUS_CANCELING)
MOVING_V = 0.02      # m/s
MOVING_W = 0.05      # rad/s
CMD_EPS = 0.005      # |cmd| below this = zero
FRESH_S = 0.5        # cmd_vel samples older than this are ignored
WATCHDOG_FRESH_S = 2.0
OBSTACLE_HOLD_S = 1.0  # keep OBSTACLE this long after it clears (no flicker)


def nonzero(t):
    return abs(t.linear.x) > CMD_EPS or abs(t.angular.z) > CMD_EPS


class LightState(Node):
    def __init__(self):
        super().__init__('light_state')
        self.pub = self.create_publisher(String, '/light/cmd', 10)
        self.goals = {}         # action -> bool (any goal active)
        self.moving = False
        self.cmd = None         # (nonzero, t) final /cmd_vel
        self.smoothed = None    # (nonzero, t) /cmd_vel_smoothed
        self.watchdog = {}      # name -> (ok, t)
        self.obstacle_until = 0.0
        self.last = None

        status_qos = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                reliability=ReliabilityPolicy.RELIABLE)
        for name in NAV_ACTIONS:
            self.create_subscription(
                GoalStatusArray, f'/{name}/_action/status',
                lambda m, n=name: self.on_status(n, m), status_qos)
        self.create_subscription(Odometry, '/odometry/filtered', self.on_odom, 10)
        self.create_subscription(Twist, '/cmd_vel', lambda m: self.on_twist('cmd', m), 10)
        self.create_subscription(Twist, '/cmd_vel_smoothed',
                                 lambda m: self.on_twist('smoothed', m), 10)
        for name in ('depth', 'imu'):
            self.create_subscription(
                Bool, f'/{name}_watchdog/ok',
                lambda m, n=name: self.watchdog.__setitem__(n, (m.data, self.now_s())), 10)
        self.create_timer(0.5, self.on_timer)

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def on_status(self, name, msg):
        self.goals[name] = any(s.status in ACTIVE for s in msg.status_list)

    def on_odom(self, m):
        t = m.twist.twist
        self.moving = abs(t.linear.x) > MOVING_V or abs(t.angular.z) > MOVING_W

    def on_twist(self, which, m):
        setattr(self, which, (nonzero(m), self.now_s()))

    def fresh(self, sample, t):
        return sample is not None and t - sample[1] < FRESH_S

    def decide(self, t):
        if any(not ok and t - ts < WATCHDOG_FRESH_S for ok, ts in self.watchdog.values()):
            return 'FAULT'
        nav = any(self.goals.values())
        if nav and self.fresh(self.smoothed, t) and self.smoothed[0] \
                and self.fresh(self.cmd, t) and not self.cmd[0]:
            self.obstacle_until = t + OBSTACLE_HOLD_S
        if nav and t < self.obstacle_until:
            return 'OBSTACLE'
        if nav:
            return 'AUTO'
        if self.moving or (self.fresh(self.cmd, t) and self.cmd[0]):
            return 'MANUAL'
        return 'IDLE'

    def on_timer(self):
        state = self.decide(self.now_s())
        if state != self.last:
            self.get_logger().info('light %s' % state)
            self.last = state
        self.pub.publish(String(data=state))


def main():
    rclpy.init()
    node = LightState()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
