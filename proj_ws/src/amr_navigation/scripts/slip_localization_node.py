#!/usr/bin/env python3
"""While the wheels slip, make AMCL lean on the LiDAR instead of odometry.

Yaw already comes from the gyro (ekf.yaml); what slip breaks is the wheel
distance. AMCL doesn't read odometry covariance, only its alpha noise
model, so during slip this node raises AMCL's translation alphas
(alpha3/alpha4) through its parameter service - particles spread further
along the motion and the scan matching pulls the pose back - and restores
the yaml values once slip has been clear for `release_hold_s`.

Slip is either of:
  - /wheel/slip_detected true (firmware: wheel yaw rate vs gyro mismatch,
    sees one-sided slip / failed turns);
  - straight-line slip: over consecutive AMCL updates the EKF odometry
    moved a distance that AMCL doesn't confirm (ratio outside
    [ratio_min, ratio_max] for `ratio_hits` updates in a row). Same check
    used on the bags 2026-10-20. Skipped while /wheel/odom is stale - then
    the EKF just extrapolates (STM32 dropout), which is not slip.

The high alphas are capped at `max_active_s` per episode (a long-diffused
filter can lose itself in a featureless aisle); after that the normal
values come back and a warning is logged.

Publishes /slip_localization/active (std_msgs/Bool, high alphas in use,
for the UI) and /slip_localization/slipping (std_msgs/Bool, either check
saw slip within the last `slipping_window_s`, no release hold) - the
latter drives nav_goal_executor_node's stop-and-retry.

Localization aid only - NOT a safety function (ISO 3691-4). It does not
shorten a slipping stop; speed limiting (slip_speed_limiter_node.py) and
collision_monitor stay as they are.
"""
import math

import rclpy
import rclpy.executors
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.node import Node
from std_msgs.msg import Bool

ALPHAS = ('alpha3', 'alpha4')


class SlipLocalization(Node):
    def __init__(self):
        super().__init__('slip_localization')
        self.slip_alpha = self.declare_parameter('slip_alpha', 0.8).value
        self.release_hold_s = self.declare_parameter('release_hold_s', 3.0).value
        self.max_active_s = self.declare_parameter('max_active_s', 20.0).value
        self.ratio_min = self.declare_parameter('ratio_min', 0.6).value
        self.ratio_max = self.declare_parameter('ratio_max', 1.5).value
        self.ratio_hits = self.declare_parameter('ratio_hits', 2).value
        self.min_move_m = self.declare_parameter('min_move_m', 0.05).value
        self.odom_stale_s = self.declare_parameter('odom_stale_s', 0.3).value
        self.slipping_window_s = self.declare_parameter('slipping_window_s', 1.5).value

        self.get_client = self.create_client(GetParameters, '/amcl/get_parameters')
        self.set_client = self.create_client(SetParameters, '/amcl/set_parameters')
        self.pub = self.create_publisher(Bool, '/slip_localization/active', 10)
        self.slipping_pub = self.create_publisher(Bool, '/slip_localization/slipping', 10)
        self.create_subscription(Bool, '/wheel/slip_detected', self.on_fw_slip, 10)
        self.create_subscription(Odometry, '/wheel/odom', self.on_wheel_odom, 10)
        self.create_subscription(Odometry, '/odometry/filtered', self.on_odom, 10)
        self.create_subscription(PoseWithCovarianceStamped, '/amcl_pose', self.on_amcl, 10)
        self.create_timer(0.2, self.on_timer)

        self.normal = None          # {'alpha3': x, 'alpha4': y} read from AMCL
        self.active = False
        self.active_since = None
        self.last_slip = None       # last time either check saw slip
        self.capped = False         # max_active_s hit; wait for slip to clear
        self.wheel_odom_t = None
        self.odom_xy = None         # latest /odometry/filtered position
        self.prev = None            # (amcl xy, odom xy) at the previous AMCL update
        self.hits = 0

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    # ---------- inputs ----------
    def on_fw_slip(self, msg):
        if msg.data:
            self.last_slip = self.now_s()

    def on_wheel_odom(self, _msg):
        self.wheel_odom_t = self.now_s()

    def on_odom(self, msg):
        p = msg.pose.pose.position
        self.odom_xy = (p.x, p.y)

    def on_amcl(self, msg):
        if self.odom_xy is None:
            return
        p = msg.pose.pose.position
        cur = ((p.x, p.y), self.odom_xy)
        prev, self.prev = self.prev, cur
        fresh = self.wheel_odom_t is not None and \
            self.now_s() - self.wheel_odom_t < self.odom_stale_s
        if prev is None or not fresh:
            self.hits = 0
            return
        d_odom = math.dist(prev[1], cur[1])
        if d_odom < self.min_move_m:
            return  # too short to judge; keep the streak as is
        ratio = math.dist(prev[0], cur[0]) / d_odom
        if ratio < self.ratio_min or ratio > self.ratio_max:
            self.hits += 1
            if self.hits >= self.ratio_hits:
                if not self.active:
                    self.get_logger().warn(
                        'straight-line slip: odom %.3f m, AMCL %.3f m' % (d_odom, d_odom * ratio),
                        throttle_duration_sec=5.0)
                self.last_slip = self.now_s()
        else:
            self.hits = 0

    # ---------- AMCL parameters ----------
    def read_normal(self):
        if not self.get_client.service_is_ready():
            return
        req = GetParameters.Request(names=list(ALPHAS))
        fut = self.get_client.call_async(req)

        def done(f):
            try:
                vals = f.result().values
            except Exception:
                return
            if len(vals) == len(ALPHAS) and all(v.type == ParameterType.PARAMETER_DOUBLE for v in vals):
                self.normal = {n: v.double_value for n, v in zip(ALPHAS, vals)}
                self.get_logger().info('AMCL normal %s' % self.normal)
        fut.add_done_callback(done)
        self._pending_get = fut

    def set_alphas(self, values):
        if not self.set_client.service_is_ready():
            self.get_logger().warn('/amcl/set_parameters not available')
            return False
        req = SetParameters.Request(parameters=[
            Parameter(name=n, value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE,
                                                   double_value=float(values[n])))
            for n in ALPHAS])
        self.set_client.call_async(req)
        return True

    def set_active(self, active):
        values = {n: max(self.slip_alpha, self.normal[n]) for n in ALPHAS} if active else self.normal
        if not self.set_alphas(values):
            return
        self.active = active
        self.active_since = self.now_s() if active else None
        self.pub.publish(Bool(data=active))
        if active:
            self.get_logger().warn('wheel slip - AMCL alpha3/4 -> %.2f (trust LiDAR)' % self.slip_alpha)
        else:
            self.get_logger().info('slip cleared - AMCL alphas restored %s' % self.normal)

    def on_timer(self):
        if self.normal is None:
            if getattr(self, '_pending_get', None) is None or self._pending_get.done():
                self.read_normal()
            return
        t = self.now_s()
        slipping = self.last_slip is not None and t - self.last_slip < self.release_hold_s
        if not slipping:
            self.capped = False
        if self.active:
            if not slipping:
                self.set_active(False)
            elif t - self.active_since > self.max_active_s:
                self.get_logger().warn(
                    'slip for over %.0f s - AMCL alphas restored to avoid a diffused filter; '
                    'check the floor / drive wheels' % self.max_active_s)
                self.set_active(False)
                self.capped = True
        elif slipping and not self.capped:
            self.set_active(True)
        self.pub.publish(Bool(data=self.active))
        self.slipping_pub.publish(Bool(data=self.last_slip is not None and
                                       t - self.last_slip < self.slipping_window_s))

    def restore(self):
        if self.active and self.normal is not None:
            self.set_alphas(self.normal)


def main():
    rclpy.init()
    node = SlipLocalization()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.restore()
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
