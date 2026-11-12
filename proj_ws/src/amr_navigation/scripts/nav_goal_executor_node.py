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
  /rosboard_nav_station (geometry_msgs/PoseStamped) - park inside a station box

A station goal runs in two parts:
  1. Nav2 to a pre-goal APPROACH_DIST behind the station along its heading
     (normal BT and tolerance);
  2. our own final approach (StationApproach below), because MPPI with a
     tight goal checker hunted at the goal until "Failed to make progress"
     and the recovery Spin turned the robot away: turn onto the station
     line, drive straight along it slowly correcting lateral/heading error,
     stop at the station, turn to the station heading. Pose from TF
     map->base_footprint (AMCL + EKF). Commands go to cmd_vel_nav, i.e.
     through velocity_smoother and collision_monitor like Nav2's own.
If part 1 fails the station goal is dropped (robot stays put). If the
robot is already in the approach corridor (between the pre-goal and the
station, close to the line - e.g. a slip retry after it passed the
pre-goal) part 1 is skipped: sending Nav2 to a pre-goal 0.2 m behind it
left MPPI (no reversing) standing still (bag chain_*, 2026-10-20).

Wheel slip (/slip_localization/slipping from slip_localization_node.py):
if it lasts SLIP_FAULT_S while a goal/path/station is running, the robot
is stopped (task cancelled, no pushing on); once slip has been clear for
SLIP_RESUME_S the same request is sent again, with no retry limit (a
station restarts from its pre-goal). Alerts go out on /nav_alert
(std_msgs/String JSON {"level": "ok"|"warn"|"error", "text": ...}) for the
goal_picker page. In-place turns of the final approach are not counted:
there the firmware flags the stalled turn itself as slip, and
StationApproach has its own stall escalation and timeout.

Blocked (BLOCK_S of Nav2 asking to move while collision_monitor outputs
nothing, or Nav2 giving up on the request): the task is cancelled before
recoveries run and the same request is re-sent every BLOCK_RETRY_S /
FAILED_RETRY_S with no limit, so the robot waits and carries on by itself
once the way is clear. Each retry still goes through collision_monitor.
Operator cancel or a new goal ends the wait. StationApproach is not covered;
it keeps its own timeout.

Fixed routes: if the map has a <map>_routes.yaml (recorded from the control
panel; path in /amr_stations) a station goal follows the recorded route to
that station with FollowPath, from the route point nearest the robot, in
place of Nav2's planner. With routes on the map, a station that has no route
from where the robot is gets an alert and the robot stays put. Maps without
routes, maps switched to the planner on the panel's Auto page (read at every
send), and tapped goals/drawn paths, still use the planner. The final
approach and the waiting above work the same on a route.

A new goal or path always preempts whatever is currently running (cancels
the in-progress task first) - the operator's most recent tap wins, same as
re-pressing "2D Nav Goal" in RViz.
"""
import json
import math
import sys
import time

import rclpy
import yaml
from rclpy.time import Time
from geometry_msgs.msg import PoseStamped, Twist
from lifecycle_msgs.srv import GetState
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import Path
from std_msgs.msg import Bool, Empty, String
from tf2_ros import Buffer, TransformListener


def restamp(pose_stamped, navigator):
    pose_stamped.header.stamp = navigator.get_clock().now().to_msg()
    return pose_stamped


SLIP_FAULT_S = 6.0       # s of continuous slip before the robot is stopped (was 3.0: stopped too often)
SLIP_RESUME_S = 3.0      # s slip-free before the stopped request is resent

BLOCK_S = 2.0            # s of commanded motion stopped by collision_monitor = blocked
BLOCK_RETRY_S = 3.0      # s between cancelling a blocked request and re-sending it
FAILED_RETRY_S = 10.0    # s before re-sending a request Nav2 failed (recoveries already ran)
CHAIN_FRESH_S = 0.5      # s - older cmd_vel_smoothed/cmd_vel counts as not publishing

ROUTE_MAX_OFFSET = 0.5   # m - robot further than this from a route can't join it
STATION_MATCH = 0.05     # m - a station goal belongs to the saved station this close

APPROACH_DIST = 1.0      # m - pre-goal distance behind the station
# Slow curving doesn't turn this robot: at v <= ~0.07 m/s the wheels run
# the differential but the body goes straight (gyro ~0, bag
# 20261020_155840). So heading is corrected by stopping and turning in
# place (stall escalation below), and driving only trims small errors.
APPROACH_V = 0.08        # m/s - final approach speed (was 0.05)
APPROACH_V_MIN = 0.04    # m/s - speed for the last SLOW_DIST (was 0.03)
SLOW_DIST = 0.10         # m
ROT_W_MAX = 0.30         # rad/s
ROT_W_MIN = 0.15         # rad/s - in-place turn floor once the robot is turning
# In-place turns sometimes don't start: bag 20261020_141205 has 40 s of
# cmd/wheel w 0.30 rad/s with gyro 0 (casters/floor hold the body). If the
# heading hasn't moved STALL_YAW in STALL_S, raise the turn floor by
# BOOST_STEP per STALL_S up to ROT_W_KICK; drop the boost once it turns.
STALL_S = 1.0
STALL_YAW = 0.01         # rad
BOOST_STEP = 0.05        # rad/s
ROT_W_KICK = 0.50        # rad/s - = MPPI wz_max / behavior_server limit
K_HEADING = 1.5          # w per rad of heading error (vs. the pure-pursuit heading) while driving
LOOKAHEAD_GAIN = 0.5     # lookahead = this * distance left to the station...
LOOKAHEAD_MIN = 0.15     # m ...but at least this
ALIGN_FIRST = 0.09       # rad (~5 deg) - stop and turn in place if heading is off more (was 0.30)
ALIGN_DONE = 0.03        # rad - in-place alignment finished below this
YAW_TOL = 0.04           # rad (~2.3 deg) - final heading tolerance
HALF_TRACK = 0.236       # m - wheel_sep / 2 (STM32 firmware)
NO_REVERSE = 0.8         # while driving keep |w| <= this * v / HALF_TRACK
CORRIDOR_LATERAL = 0.25  # m - within this of the line (and past the pre-goal) skip leg 1
CORRIDOR_ENTRY = 0.25    # m - leg 1 hands over once within this before the pre-goal (MPPI
                         # stalled 0.18 m from it, then a recovery Spin; bag 20261020_161338)
LATERAL_ABORT = 0.35     # m - too far off the line: stop, give up (0.25 tripped after AMCL jumped at the pre-goal)
APPROACH_TIMEOUT = 90.0  # s


def yaw_of(q):
    return math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))


def wrap(a):
    return math.atan2(math.sin(a), math.cos(a))


def clamp(v, lim):
    return max(-lim, min(lim, v))


def pre_goal(station):
    """Pose APPROACH_DIST behind `station` along its heading, same heading."""
    yaw = yaw_of(station.pose.orientation)
    pre = PoseStamped()
    pre.header.frame_id = station.header.frame_id
    pre.pose.position.x = station.pose.position.x - APPROACH_DIST * math.cos(yaw)
    pre.pose.position.y = station.pose.position.y - APPROACH_DIST * math.sin(yaw)
    pre.pose.orientation = station.pose.orientation
    return pre


def load_routes(path):
    """Routes in a <map>_routes.yaml; [] if there is no such file or the
    operator chose the planner (use_routes: false). Raises on a damaged file,
    so a broken route file never silently means 'use planner'."""
    try:
        with open(path) as f:
            data = yaml.safe_load(f) or {}
        return [] if data.get('use_routes', True) is False else data.get('routes', [])
    except FileNotFoundError:
        return []


def pick_route(routes, dest, x, y):
    """Remaining [x, y, yaw] points of the route to `dest` the robot is
    closest to, starting at its nearest point; None if none is within
    ROUTE_MAX_OFFSET."""
    best = None
    for r in routes:
        if r['to'] != dest or len(r['poses']) < 2:
            continue
        dists = [math.hypot(p[0] - x, p[1] - y) for p in r['poses']]
        i = min(range(len(dists)), key=dists.__getitem__)
        if dists[i] <= ROUTE_MAX_OFFSET and (best is None or dists[i] < best[0]):
            best = (dists[i], r['poses'][min(i, len(dists) - 2):])
    return best[1] if best else None


def route_to_path(points, navigator):
    path = Path()
    path.header.frame_id = 'map'
    path.header.stamp = navigator.get_clock().now().to_msg()
    for px, py, yaw in points:
        pose = PoseStamped()
        pose.header = path.header
        pose.pose.position.x = float(px)
        pose.pose.position.y = float(py)
        pose.pose.orientation.z = math.sin(yaw / 2.0)
        pose.pose.orientation.w = math.cos(yaw / 2.0)
        path.poses.append(pose)
    return path


def station_errors(tf_buffer, sx, sy, sth):
    """Robot pose in the station frame (ex, ey, eth), or None without TF."""
    try:
        t = tf_buffer.lookup_transform('map', 'base_footprint', Time())
    except Exception:
        return None
    dx = t.transform.translation.x - sx
    dy = t.transform.translation.y - sy
    c, s = math.cos(sth), math.sin(sth)
    return (c * dx + s * dy, -s * dx + c * dy,
            wrap(yaw_of(t.transform.rotation) - sth))


class StationApproach:
    """Final approach into a station box, stepped from the main loop.
    Errors are in the station frame: ex along the station heading (negative
    = still before it), ey to the left, eth = robot yaw - station yaw."""

    def __init__(self, navigator, tf_buffer, cmd_pub, station):
        self.nav = navigator
        self.tf = tf_buffer
        self.pub = cmd_pub
        self.sx = station.pose.position.x
        self.sy = station.pose.position.y
        self.sth = yaw_of(station.pose.orientation)
        self.state = 'align'  # align -> drive -> turn -> done | failed
        self.t0 = time.time()
        self.boost = 0.0          # extra in-place turn floor (stall escalation)
        self.stall_ref = None     # (time, heading error) when the turn was last seen moving
        e = self.errors()
        if e is not None:
            navigator.get_logger().info(
                'nav_goal_executor_node: final approach start: along %+.3f m, lateral %+.3f m, '
                'heading %+.1f deg' % (e[0], e[1], math.degrees(e[2])))

    def errors(self):
        return station_errors(self.tf, self.sx, self.sy, self.sth)

    def send(self, v, w):
        m = Twist()
        m.linear.x = float(v)
        m.angular.z = float(w)
        self.pub.publish(m)

    def finish(self, state, msg):
        self.send(0.0, 0.0)
        self.state = state
        # separate calls: rclpy raises if one call site changes severity
        if state == 'done':
            self.nav.get_logger().info('nav_goal_executor_node: station ' + msg)
        else:
            self.nav.get_logger().warn('nav_goal_executor_node: station ' + msg)

    def turn_to(self, eth):
        now = time.time()
        if self.stall_ref is None or abs(wrap(eth - self.stall_ref[1])) > STALL_YAW:
            if self.stall_ref is not None:
                self.boost = max(0.0, self.boost - BOOST_STEP)  # turning: ease off
            self.stall_ref = (now, eth)
        elif now - self.stall_ref[0] > STALL_S:
            was = self.boost
            self.boost = min(ROT_W_KICK - ROT_W_MIN, self.boost + BOOST_STEP)
            self.stall_ref = (now, eth)
            if self.boost != was and self.boost >= ROT_W_KICK - ROT_W_MIN:
                self.nav.get_logger().warn(
                    'nav_goal_executor_node: in-place turn stalled, at %.2f rad/s' % ROT_W_KICK)
        w_min = ROT_W_MIN + self.boost
        w = -1.5 * eth
        w = math.copysign(max(w_min, min(max(ROT_W_MAX, w_min), abs(w))), w)
        self.send(0.0, w)

    def step(self):
        """One control step; returns True while still running."""
        if self.state in ('done', 'failed'):
            return False
        if time.time() - self.t0 > APPROACH_TIMEOUT:
            self.finish('failed', 'approach timed out')
            return False
        e = self.errors()
        if e is None:
            self.send(0.0, 0.0)  # no TF yet / lost: hold still
            return True
        ex, ey, eth = e
        if abs(ey) > LATERAL_ABORT:
            self.finish('failed', 'approach aborted: %.2f m off the line' % ey)
            return False

        # heading toward a point on the line LOOKAHEAD ahead (pure pursuit),
        # so a large lateral offset is still closed before the station
        lookahead = max(LOOKAHEAD_MIN, LOOKAHEAD_GAIN * -ex)
        target = math.atan2(-ey, lookahead)
        err = wrap(eth - target)

        if self.state == 'align':
            if abs(err) < ALIGN_DONE or ex > -0.02:
                self.state = 'drive' if ex <= -0.02 else 'turn'
            else:
                self.turn_to(err)
                return True

        if self.state == 'drive':
            if ex >= -0.005:
                self.send(0.0, 0.0)
                self.state = 'turn'
            elif abs(err) > ALIGN_FIRST:  # same reference as 'align', or they ping-pong
                self.state = 'align'
                return True
            else:
                v = APPROACH_V if ex < -SLOW_DIST else APPROACH_V_MIN
                # never command the inner wheel backwards while rolling
                # forward: the single-channel Hall encoders take their sign
                # from the command, so a dragged inner wheel counts the
                # wrong way (bag 20261020_140110: wheel odom w 0.26 rad/s,
                # gyro 0, lateral error never corrected)
                self.stall_ref = None
                w_lim = min(ROT_W_MAX, NO_REVERSE * v / HALF_TRACK)
                self.send(v, clamp(-K_HEADING * err, w_lim))
                return True

        if self.state == 'turn':
            if abs(eth) <= YAW_TOL:
                self.finish('done', 'parked: along %+.3f m, lateral %+.3f m, heading %+.1f deg'
                            % (ex, ey, math.degrees(eth)))
                return False
            self.turn_to(eth)
        return True


def wait_for_active(navigator, node_name, timeout_s=2.0):
    """Block until `node_name` reports lifecycle state 'active'.

    Replaces BasicNavigator.waitUntilNav2Active(), which (Humble) has two
    problems here:
      * it calls <node>/get_state with spin_until_future_complete() and NO
        timeout - a lost response right after launch hung this node forever
        in "waiting for Nav2 to become active", so it never subscribed to
        /rosboard_nav_goal and every goal from the panel / phone was
        silently dropped;
      * with the default localizer='amcl' it keeps publishing a ZERO pose
        to /initialpose until /amcl_pose arrives, resetting AMCL to (0,0,0)
        - the operator sets the initial pose (panel / goal picker) instead.
    Here every call has a timeout and is simply retried."""
    client = navigator.create_client(GetState, f'{node_name}/get_state')
    while rclpy.ok():
        if not client.wait_for_service(timeout_sec=1.0):
            navigator.get_logger().info(f'{node_name}/get_state not available, waiting...')
            continue
        future = client.call_async(GetState.Request())
        rclpy.spin_until_future_complete(navigator, future, timeout_sec=timeout_s)
        if future.done() and future.result() is not None and \
                future.result().current_state.label == 'active':
            break
        if not future.done():
            future.cancel()
        time.sleep(1.0)
    navigator.destroy_client(client)


def main():
    rclpy.init()
    navigator = BasicNavigator()
    navigator.get_logger().info('nav_goal_executor_node: waiting for Nav2 to become active...')
    wait_for_active(navigator, 'amcl')
    wait_for_active(navigator, 'bt_navigator')
    navigator.get_logger().info('nav_goal_executor_node: Nav2 active, ready for goals')

    task_active = {'value': False}
    pending = {'value': None}  # (kind, msg[, resume]) - kind 'pose'|'station'|'path'|'cancel'
    current = {'req': None}  # running request (kind, msg) for a slip retry
    held = {'value': None}       # request stopped by slip, waiting to be resent
    slip = {'since': None, 'clear_since': None}
    station = {'value': None}   # station PoseStamped while Nav2 drives to its pre-goal
    approach = {'value': None}  # StationApproach while the final approach runs

    stations_info = {'stations': [], 'routes_file': ''}  # from the panel's /amr_stations
    route_plan = {'value': None}  # Path being followed, re-sent on /plan for the pages
    plan_pub = navigator.create_publisher(Path, '/plan', 10)
    chain = {'in': False, 'in_t': 0.0, 'out': False, 'out_t': 0.0}
    blocked = {'req': None, 'resend_at': 0.0, 'since': None, 'alerted': False}

    tf_buffer = Buffer()
    TransformListener(tf_buffer, navigator)
    cmd_pub = navigator.create_publisher(Twist, 'cmd_vel_nav', 10)

    def on_goal(msg):
        pending['value'] = ('pose', msg)

    def on_path(msg):
        pending['value'] = ('path', msg)

    def on_station(msg):
        pending['value'] = ('station', msg)

    def on_cancel(_msg):
        pending['value'] = ('cancel', None)

    def is_moving(msg):
        return abs(msg.linear.x) > 0.001 or abs(msg.angular.z) > 0.001

    def on_smoothed(msg):
        chain['in_t'] = time.time()
        chain['in'] = abs(msg.linear.x) > 0.02 or abs(msg.angular.z) > 0.05

    def on_output(msg):
        chain['out_t'] = time.time()
        chain['out'] = is_moving(msg)

    def wait_for_clear(req, delay, why):
        """Keep `req` and re-send it after `delay` s (no retry limit)."""
        blocked['req'] = req
        blocked['resend_at'] = time.time() + delay
        blocked['since'] = None
        blocked['alerted'] = True
        alert('warn', why)
        navigator.get_logger().info(
            'nav_goal_executor_node: waiting, retry in %.0f s' % delay,
            throttle_duration_sec=30.0)

    def on_stations(msg):
        try:
            d = json.loads(msg.data)
        except ValueError:
            return
        stations_info['stations'] = d.get('stations', [])
        stations_info['routes_file'] = d.get('routes_file', '')

    def station_route(goal):
        """(use_route, points) for a station goal. use_route False = the map
        has no routes; points None = it has, but none usable from here."""
        routes = load_routes(stations_info['routes_file']) if stations_info['routes_file'] else []
        if not routes:
            return False, None
        px, py = goal.pose.position.x, goal.pose.position.y
        dest = next((s['name'] for s in stations_info['stations']
                     if math.hypot(s['x'] - px, s['y'] - py) < STATION_MATCH), None)
        try:
            t = tf_buffer.lookup_transform('map', 'base_footprint', Time())
        except Exception:
            return True, None
        return True, pick_route(routes, dest, t.transform.translation.x,
                                t.transform.translation.y)

    def on_slipping(msg):
        t = time.time()
        if msg.data:
            slip['clear_since'] = None
            if slip['since'] is None:
                slip['since'] = t
        else:
            slip['since'] = None
            if slip['clear_since'] is None:
                slip['clear_since'] = t

    alert_pub = navigator.create_publisher(String, '/nav_alert', 10)
    last_alert = {'data': json.dumps({'level': 'ok', 'text': ''}), 't': 0.0}

    def alert(level, text):
        last_alert['data'] = json.dumps({'level': level, 'text': text})
        last_alert['t'] = time.time()
        alert_pub.publish(String(data=last_alert['data']))
        if level == 'error':
            navigator.get_logger().error('nav_goal_executor_node: ' + text)
        elif level == 'warn':
            navigator.get_logger().warn('nav_goal_executor_node: ' + text)

    def cancel_and_wait(timeout=3.0):
        """Cancel the active Nav2 task and wait until bt_navigator has really
        finished it. A goal sent while the cancel is still being processed is
        queued as a preempt, and the cancel then aborts that new goal too
        (seen 2026-10-26 11:42:44: re-sent station -> 'pre-goal failed')."""
        navigator.cancelTask()
        fut = navigator.result_future
        if fut is not None:
            rclpy.spin_until_future_complete(navigator, fut, timeout_sec=timeout)
            if not fut.done():
                navigator.get_logger().warn(
                    'nav_goal_executor_node: previous task did not finish '
                    'cancelling within %.0f s' % timeout)

    def sent(ok, what):
        """Book-keeping after goToPose/goThroughPoses; False = rejected."""
        if ok:
            task_active['value'] = True
            return True
        station['value'] = None
        current['req'] = None
        alert('error', 'Nav2 rejected the %s - send it again' % what)
        return False

    def stop_all():
        if task_active['value']:
            navigator.cancelTask()
            task_active['value'] = False
        if approach['value'] is not None:
            approach['value'].finish('failed', 'approach stopped')
            approach['value'] = None
        station['value'] = None

    navigator.create_subscription(PoseStamped, '/rosboard_nav_goal', on_goal, 10)
    navigator.create_subscription(Path, '/rosboard_nav_path', on_path, 10)
    navigator.create_subscription(Empty, '/rosboard_nav_cancel', on_cancel, 10)
    navigator.create_subscription(PoseStamped, '/rosboard_nav_station', on_station, 10)
    navigator.create_subscription(Bool, '/slip_localization/slipping', on_slipping, 10)
    navigator.create_subscription(String, '/amr_stations', on_stations, 10)
    navigator.create_subscription(Twist, '/cmd_vel_smoothed', on_smoothed, 10)
    navigator.create_subscription(Twist, '/cmd_vel', on_output, 10)

    try:
        while rclpy.ok():
            # 20 Hz while our own final approach is driving
            rclpy.spin_once(navigator, timeout_sec=0.05 if approach['value'] else 0.2)

            if pending['value'] is not None:
                kind, msg = pending['value'][:2]
                resume = len(pending['value']) > 2
                pending['value'] = None

                if task_active['value']:
                    cancel_and_wait()
                    task_active['value'] = False
                if approach['value'] is not None:
                    approach['value'].finish('failed', 'approach interrupted')
                    approach['value'] = None
                station['value'] = None
                route_plan['value'] = None
                blocked['since'] = None
                if not resume:  # operator's own command: drop any waiting request
                    blocked['req'] = None
                    blocked['alerted'] = False
                    held['value'] = None
                    alert('ok', '')
                current['req'] = (kind, msg) if kind != 'cancel' else None

                if kind == 'pose':
                    navigator.get_logger().info(
                        'nav_goal_executor_node: new goal (%.2f, %.2f)'
                        % (msg.pose.position.x, msg.pose.position.y))
                    sent(navigator.goToPose(restamp(msg, navigator)), 'goal')
                elif kind == 'station':
                    e = station_errors(tf_buffer, msg.pose.position.x, msg.pose.position.y,
                                       yaw_of(msg.pose.orientation))
                    if e is not None and -APPROACH_DIST + 0.15 < e[0] < 0.05 and \
                            abs(e[1]) < CORRIDOR_LATERAL:
                        navigator.get_logger().info(
                            'nav_goal_executor_node: station (%.2f, %.2f) - already in the '
                            'approach corridor, final approach directly'
                            % (msg.pose.position.x, msg.pose.position.y))
                        approach['value'] = StationApproach(navigator, tf_buffer, cmd_pub, msg)
                        continue
                    try:
                        use_route, points = station_route(msg)
                    except (OSError, KeyError, TypeError, AttributeError, yaml.YAMLError) as ex:
                        navigator.get_logger().error(
                            'nav_goal_executor_node: bad routes file: %s' % ex)
                        use_route, points = True, None
                    if use_route:
                        if points is None:
                            current['req'] = None
                            alert('error', 'ไม่มีเส้นทางจากตำแหน่งนี้ไป Station ที่เลือก '
                                           '- หุ่นยนต์ไม่ออกตัว')
                            continue
                        navigator.get_logger().info(
                            'nav_goal_executor_node: station (%.2f, %.2f) - fixed route, '
                            '%d points' % (msg.pose.position.x, msg.pose.position.y, len(points)))
                        station['value'] = msg
                        route_plan['value'] = route_to_path(points, navigator)
                        sent(navigator.followPath(route_plan['value']), 'station route')
                        continue
                    pre = pre_goal(msg)
                    navigator.get_logger().info(
                        'nav_goal_executor_node: station (%.2f, %.2f) - pre-goal (%.2f, %.2f)'
                        % (msg.pose.position.x, msg.pose.position.y,
                           pre.pose.position.x, pre.pose.position.y))
                    station['value'] = msg
                    sent(navigator.goToPose(restamp(pre, navigator)), 'station pre-goal')
                elif kind == 'path':
                    if not msg.poses:
                        continue
                    navigator.get_logger().info(
                        'nav_goal_executor_node: new path with %d waypoints'
                        % len(msg.poses))
                    poses = [restamp(p, navigator) for p in msg.poses]
                    sent(navigator.goThroughPoses(poses), 'path')
                else:  # 'cancel'
                    navigator.get_logger().info('nav_goal_executor_node: goal cancelled by operator')

            # leg 1: hand over to the final approach as soon as the robot is in
            # the corridor - no need for Nav2 to hit the pre-goal exactly
            if station['value'] is not None and task_active['value']:
                g = station['value']
                e = station_errors(tf_buffer, g.pose.position.x, g.pose.position.y,
                                   yaw_of(g.pose.orientation))
                if e is not None and -APPROACH_DIST - CORRIDOR_ENTRY < e[0] < 0.05 and \
                        abs(e[1]) < CORRIDOR_LATERAL:
                    navigator.cancelTask()
                    task_active['value'] = False
                    station['value'] = None
                    navigator.get_logger().info(
                        'nav_goal_executor_node: in the approach corridor - final approach')
                    approach['value'] = StationApproach(navigator, tf_buffer, cmd_pub, g)

            # re-sent at 1 Hz so a page opened later still sees it
            if time.time() - last_alert['t'] > 1.0:
                last_alert['t'] = time.time()
                alert_pub.publish(String(data=last_alert['data']))
                if not task_active['value']:
                    route_plan['value'] = None
                if route_plan['value'] is not None:
                    plan_pub.publish(route_plan['value'])

            # collision_monitor is holding Nav2 back: cancel before the
            # progress checker and recoveries fire, retry until it lets go
            now = time.time()
            wants = chain['in'] and now - chain['in_t'] < CHAIN_FRESH_S
            passes = chain['out'] and now - chain['out_t'] < CHAIN_FRESH_S
            if blocked['alerted'] and passes:
                blocked['alerted'] = False
                alert('ok', '')
            if task_active['value'] and wants and not passes:
                if blocked['since'] is None:
                    blocked['since'] = now
                elif now - blocked['since'] > BLOCK_S and current['req'] is not None:
                    req = current['req']
                    cancel_and_wait()
                    task_active['value'] = False
                    station['value'] = None
                    current['req'] = None
                    wait_for_clear(req, BLOCK_RETRY_S,
                                   'มีสิ่งกีดขวาง - จอดรอ จะไปต่ออัตโนมัติเมื่อทางโล่ง')
            else:
                blocked['since'] = None
            if blocked['req'] is not None and time.time() > blocked['resend_at']:
                pending['value'] = blocked['req'] + (True,)
                blocked['req'] = None

            # sustained wheel slip: stop, then resend when it clears
            busy = task_active['value'] or approach['value'] is not None
            turning_in_place = approach['value'] is not None and \
                approach['value'].state in ('align', 'turn')
            if busy and not turning_in_place and slip['since'] is not None and \
                    time.time() - slip['since'] > SLIP_FAULT_S:
                stop_all()
                if current['req'] is not None:
                    held['value'] = current['req']
                    alert('warn', 'ล้อลื่นต่อเนื่อง - หยุดหุ่นยนต์แล้ว จะไปต่ออัตโนมัติเมื่อหายลื่น')
                current['req'] = None
            if held['value'] is not None and slip['since'] is None and \
                    slip['clear_since'] is not None and \
                    time.time() - slip['clear_since'] > SLIP_RESUME_S:
                navigator.get_logger().info('nav_goal_executor_node: slip cleared - resending request')
                pending['value'] = held['value'] + (True,)
                held['value'] = None
                alert('ok', '')

            if approach['value'] is not None and not approach['value'].step():
                approach['value'] = None
                current['req'] = None

            if task_active['value'] and navigator.isTaskComplete():
                result = navigator.getResult()
                task_active['value'] = False
                if result == TaskResult.FAILED and current['req'] is not None:
                    req = current['req']
                    station['value'] = None
                    current['req'] = None
                    wait_for_clear(req, FAILED_RETRY_S,
                                   'ไปต่อไม่ได้ (ทางถูกบังหรือหาเส้นทางไม่ได้) - จอดรอแล้วลองใหม่อัตโนมัติ')
                    continue
                if station['value'] is not None:
                    goal, station['value'] = station['value'], None
                    if result == TaskResult.SUCCEEDED:
                        navigator.get_logger().info(
                            'nav_goal_executor_node: pre-goal reached - final approach')
                        approach['value'] = StationApproach(navigator, tf_buffer, cmd_pub, goal)
                    else:
                        navigator.get_logger().warn(
                            'nav_goal_executor_node: pre-goal failed - station goal dropped')
                        current['req'] = None
                    continue
                current['req'] = None
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
        if approach['value'] is not None:
            approach['value'].send(0.0, 0.0)
        navigator.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    sys.exit(main())
