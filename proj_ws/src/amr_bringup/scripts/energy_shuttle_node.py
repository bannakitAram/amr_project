#!/usr/bin/env python3
"""Energy-logging shuttle: drive a straight line back and forth at a fixed
speed so the power log gets long constant-speed segments.

Why: logging on a Nav2 route (station to station) gives very little
steady-state data. The robot keeps stopping and changing speed: detours,
RPP slowdowns, approach to the station, the turn at the goal. This node
does not use the planner or the controller. Each leg runs at one speed from
start to end.

  ros2 run amr_bringup energy_shuttle_node.py <distance_m> <speed_mps> <laps> [turn_rad_s]

  one lap = forward <distance_m>, turn 180 deg in place, forward back,
            turn 180 deg (ends at the start pose and heading).

SAFETY: publishes to /cmd_vel_nav, so the command still goes through
velocity_smoother -> collision_monitor -> /cmd_vel. Navigation must be up,
with no active goal. The node refuses to start if nothing subscribes to
/cmd_vel_nav, and it never publishes /cmd_vel directly. If collision_monitor
stops the robot (person in the way), the node keeps commanding and waits.
It resumes when the path is clear and aborts if held for more than HOLD_ABORT_S.

Segment markers go out on /nav_event (JSON, already in the power-log topic
list), so the analysis can cut out the steady-state part of each leg:
  shuttle_start / leg_start / leg_end / turn_start / turn_end / hold / resume / shuttle_end

Exit codes: 0 done or cancelled, 1 held too long or stalled, 2 no data or
no safety chain, 3 bad arguments.
"""
import json
import math
import signal
import sys
import time

import rclpy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from rclpy.node import Node
from std_msgs.msg import Float32, String

MAX_SPEED = 0.40           # m/s, same as velocity_smoother max_velocity
MAX_DISTANCE = 30.0        # m, guard against a typo
TURN_SPEED_DEFAULT = 0.3   # rad/s
TURN_SLOWDOWN = 0.35       # rad remaining: go to half turn speed
TURN_TOL = 0.02            # rad
HOLD_SPEED = 0.01          # m/s measured below this while commanding = held
HOLD_GRACE_S = 2.0         # accel ramp / restart time before calling it held
HOLD_ABORT_S = 60.0
RATE_HZ = 20.0


def yaw_of(q):
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


class Shuttle(Node):
    def __init__(self):
        super().__init__('amr_energy_shuttle')
        self.pub = self.create_publisher(Twist, '/cmd_vel_nav', 10)
        self.ev = self.create_publisher(String, '/nav_event', 10)
        self.dist = None
        self.yaw = None
        self.create_subscription(Float32, '/wheel/raw_distance', self._on_dist, 10)
        self.create_subscription(Odometry, '/odometry/filtered', self._on_odom, 10)
        self.stop_flag = False

    def _on_dist(self, m):
        self.dist = m.data

    def _on_odom(self, m):
        self.yaw = yaw_of(m.pose.pose.orientation)

    def event(self, name, **kw):
        d = {'event': name, 'time': time.strftime('%Y-%m-%d %H:%M:%S')}
        d.update(kw)
        self.ev.publish(String(data=json.dumps(d)))
        self.get_logger().info(json.dumps(d))

    def send(self, v=0.0, w=0.0):
        t = Twist()
        t.linear.x = v
        t.angular.z = w
        self.pub.publish(t)

    def stop(self):
        for _ in range(5):
            self.send()
            time.sleep(0.05)

    def spin_dt(self):
        rclpy.spin_once(self, timeout_sec=1.0 / RATE_HZ)

    def leg(self, idx, distance, speed):
        """Returns True on success. Hold (collision_monitor stop) = wait."""
        start = self.dist
        self.event('leg_start', leg=idx, speed=speed, distance=distance)
        last_d, last_t = self.dist, time.time()
        slow_since = None
        held = False
        t_start = time.time()
        while not self.stop_flag:
            self.spin_dt()
            traveled = abs(self.dist - start)
            if traveled >= distance:
                break
            self.send(speed)
            now = time.time()
            if now - last_t >= 0.5:
                v = abs(self.dist - last_d) / (now - last_t)
                last_d, last_t = self.dist, now
                if v < HOLD_SPEED and now - t_start > HOLD_GRACE_S:
                    slow_since = slow_since or now
                    if not held and now - slow_since > 1.0:
                        held = True
                        self.event('hold', leg=idx, at_m=round(traveled, 2))
                    if now - slow_since > HOLD_ABORT_S:
                        self.event('abort', leg=idx, reason='held_too_long')
                        return False
                else:
                    if held:
                        self.event('resume', leg=idx, at_m=round(traveled, 2))
                    held = False
                    slow_since = None
        self.event('leg_end', leg=idx, traveled=round(abs(self.dist - start), 3),
                   duration_s=round(time.time() - t_start, 2))
        return not self.stop_flag

    def turn(self, idx, angle, rate):
        self.event('turn_start', leg=idx)
        turned, last = 0.0, self.yaw
        deadline = time.time() + angle / rate * 3.0 + HOLD_ABORT_S
        while not self.stop_flag:
            self.spin_dt()
            turned += wrap(self.yaw - last)
            last = self.yaw
            rem = angle - turned
            if rem <= TURN_TOL:
                break
            if time.time() > deadline:
                self.event('abort', leg=idx, reason='turn_timeout')
                return False
            self.send(0.0, rate * 0.5 if rem < TURN_SLOWDOWN else rate)
        self.stop()
        self.event('turn_end', leg=idx)
        return not self.stop_flag


def main():
    try:
        distance, speed, laps = float(sys.argv[1]), float(sys.argv[2]), int(sys.argv[3])
        turn_rate = float(sys.argv[4]) if len(sys.argv) > 4 else TURN_SPEED_DEFAULT
        assert 0.5 <= distance <= MAX_DISTANCE and 0.05 <= speed <= MAX_SPEED
        assert 1 <= laps <= 100 and 0.1 <= turn_rate <= 0.5
    except (IndexError, ValueError, AssertionError):
        print(__doc__.split('SAFETY')[0], file=sys.stderr)
        print('limits: distance 0.5-%g m, speed 0.05-%g m/s, laps 1-100, turn 0.1-0.5 rad/s'
              % (MAX_DISTANCE, MAX_SPEED), file=sys.stderr)
        sys.exit(3)

    rclpy.init()
    n = Shuttle()

    def on_sig(*_):
        n.stop_flag = True
    signal.signal(signal.SIGINT, on_sig)
    signal.signal(signal.SIGTERM, on_sig)

    code = 0
    try:
        t0 = time.time()
        while (n.dist is None or n.yaw is None) and time.time() - t0 < 5.0:
            rclpy.spin_once(n, timeout_sec=0.1)
        if n.dist is None or n.yaw is None:
            print('no /wheel/raw_distance or /odometry/filtered', file=sys.stderr)
            code = 2
            return
        if n.count_subscribers('/cmd_vel_nav') == 0:
            print('nothing subscribes /cmd_vel_nav: start navigation (velocity_smoother '
                  '+ collision_monitor) first. Will not drive without the safety chain.',
                  file=sys.stderr)
            code = 2
            return
        n.event('shuttle_start', distance=distance, speed=speed, laps=laps)
        idx = 0
        for lap in range(laps):
            for _ in range(2):
                idx += 1
                if not n.leg(idx, distance, speed) or not n.turn(idx, math.pi, turn_rate):
                    code = 0 if n.stop_flag else 1
                    raise StopIteration
            n.get_logger().info('lap %d/%d done' % (lap + 1, laps))
    except StopIteration:
        pass
    finally:
        n.stop()
        if code != 2:
            n.event('shuttle_end', code=code, cancelled=n.stop_flag)
            time.sleep(0.2)
        n.destroy_node()
        rclpy.shutdown()
        sys.exit(code)


if __name__ == '__main__':
    main()
