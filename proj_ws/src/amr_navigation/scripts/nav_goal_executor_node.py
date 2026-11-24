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

Door wait (/door_hold from amr_bringup door_manager_node.py, JSON at 10 Hz):
while hold is true the running request is stopped (task cancelled, robot
standing) and re-sent when hold goes false or its messages stop for
DOOR_HOLD_STALE_S. A fixed-time wait only - a door still closed is left to
collision_monitor and the blocked-wait above. Operator cancel/new goal ends it.

Ghost clearing: before each blocked/failed retry the local costmap's lethal
cells within GHOST_RADIUS of the robot are checked against the latest
/scan_filtered. A lethal cell with no scan point within GHOST_SUPPORT is a
ghost (a mark raytrace never cleared - no LiDAR return behind it). If at least
GHOST_MIN_CELLS ghosts are found both costmaps are cleared once before the retry. Obstacles
the LiDAR still sees come back on the next costmap update (0.2 s), so real
walls/people are not lost; with no fresh scan nothing is cleared.
Lethal cells inside the keep-out mask (/keepout_filter_mask) are counted
apart: clearing cannot remove them (the filter re-applies them) - a request
that fails with KEEPOUT_MIN_CELLS of them near the robot gets an alert
naming the keep-out zone instead of a plain "blocked".

Backup recovery: when Nav2 fails a request (e.g. RPP "collision ahead"
after a turn put the nose at a wall), there are no ghost cells, the LiDAR
sees a real obstacle within BACKUP_FRONT_M of the bumper and nothing within
BACKUP_REAR_M behind the rear, the robot backs up BACKUP_DIST at
BACKUP_SPEED (Nav2 BackUp behaviour) and the request is re-sent at once.
At most BACKUP_MAX times until the robot has moved BACKUP_RESET_M from the
first backup spot; then the normal wait-and-retry with an operator alert.
While backing: collision_monitor RearStop on, FrontStop/FrontStopShort off
(Humble stop zones ignore direction - a wall at the nose would block the
backup), /nav_backup_active true at 5 Hz so door_manager stops re-sending
its front-zone state; afterwards the saved states are restored. If the
restore fails the request is NOT re-sent (front stop zone may be off).

Fixed routes: if the map has a <map>_routes.yaml (recorded from the control
panel; path in /amr_stations) a station goal follows the recorded route to
that station with FollowPath, from the route point nearest the robot, in
place of Nav2's planner. With routes on the map, a station that has no route
from where the robot is gets an alert and the robot stays put. Maps without
routes, maps switched to the planner on the panel's Auto page (read at every
send), and tapped goals/drawn paths, still use the planner. The final
approach and the waiting above work the same on a route.

Temporary detour (fixed routes only): while following a station route, an
object (scan points not on the saved /map) within DETOUR_BLOCK_R of the route
and DETOUR_EARLY_M of the bumper stops the robot; if it is still there after
DETOUR_AFTER_S (or RPP keeps failing the route there) the route is offset
sideways point by point so every object point beside the footprint stays
DETOUR_SIDE_CLEAR from the robot's side (objects at different places may be
passed on different sides), lane changes no steeper than DETOUR_SLOPE, back
onto the route DETOUR_EXIT_EXTRA after the last object. Every pose is
footprint-checked against /map walls, keep-out and the scan. Too sharp from
where the robot stands -> back up first (rear checked, <= DETOUR_BACK_MAX).
Refused -> the robot stands and tries again every BLOCK_RETRY_S; object gone
-> the route is re-sent. collision_monitor stays as is.

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
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParametersAtomically
from geometry_msgs.msg import PoseStamped, Twist
from lifecycle_msgs.srv import GetState
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav2_msgs.srv import ClearEntireCostmap
from nav_msgs.msg import OccupancyGrid, Path
from rclpy.qos import DurabilityPolicy, QoSProfile, qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Bool, Empty, String
from tf2_ros import Buffer, TransformListener


def restamp(pose_stamped, navigator):
    pose_stamped.header.stamp = navigator.get_clock().now().to_msg()
    return pose_stamped


SLIP_FAULT_S = 6.0       # s of continuous slip before the robot is stopped (was 3.0: stopped too often)
DOOR_HOLD_STALE_S = 1.0  # s without /door_hold = no door wait (door_manager gone)
SLIP_RESUME_S = 3.0      # s slip-free before the stopped request is resent

BLOCK_S = 2.0            # s of commanded motion stopped by collision_monitor = blocked
BLOCK_RETRY_S = 3.0      # s between cancelling a blocked request and re-sending it
FAILED_RETRY_S = 3.0     # s before re-sending a request Nav2 failed (was 10: a door leaf or a
                         # person passing made the operator re-press before the retry, 2026-11-23)
CHAIN_FRESH_S = 0.5      # s - older cmd_vel_smoothed/cmd_vel counts as not publishing

ROUTE_MAX_OFFSET = 0.5   # m - robot further than this from a route can't join it

# Temporary detour on a fixed route: an object that has stopped the robot for
# DETOUR_AFTER_S is passed with a lane shift of the route, back onto the
# route past it. Rejected (normal wait) if it leaves the route corridor.
DETOUR_AFTER_S = 3.0     # s blocked (from collision_monitor stop) before a detour is tried
DETOUR_FRONT_M = 1.0     # m ahead of the bumper - scan points here = object still there
DETOUR_SCAN_M = 5.0      # m - scan points this close to the robot are checked against the route
DETOUR_LOOK_M = 4.5      # m along the route - stretch searched for the blocked part
DETOUR_BLOCK_R = 0.65    # m - scan point this close to a route point = route blocked there
                         # = half width 0.40 + side gap 0.20 + 0.05: with 0.55 an object
                         # 0.54-0.74 m off the route was 'no object' to the detour but
                         # too close for RPP (0.14 m) - robot stuck between (test 16:10)
DETOUR_EARLY_M = 1.6     # m bumper -> object on the route: stop here while following a route
                         # (~0.15 m braking at 0.4 m/s leaves >= 1.4 m: ramp 0.8 + gap 0.15 fit)
                         # (RPP/FrontStop stopped 0.2-0.5 m short - no room left to shift)
DETOUR_EARLY_HZ = 5.0    # early-stop check rate
DETOUR_EARLY_MIN_LEFT = 1.5  # m - no early stop when the route ends closer than this
DETOUR_EXIT_EXTRA = 0.70  # m - stay in the side lane this much longer: RPP's 0.6 m lookahead
                          # cut the way back and swung the rear into the object (test 16:05)
DETOUR_SKIP_START = 0.30  # m along - poses the robot already covers are not footprint-checked
DETOUR_OBJ_WIN = 0.20     # m along - an object point constrains the lane this close to it
DETOUR_REPAIRS = 6         # re-plans after a footprint hit the wall model missed
DETOUR_GRID = 0.02        # m - lateral offset step of the lane search
DETOUR_SLOPE = 0.55       # m sideways per m along (~29 deg) - steepest lane change
DETOUR_REAR_MARGIN = 0.30  # m - behind a detour backup must be clear this much further
DETOUR_SIDE_CLEAR = 0.10 # m - robot side to object gap while passing it (operator choice for
                         # line test; pinch-point range - no people near a detour). 0.15
                         # passed on the robot 16:45; 0.10 at 16:02 was stopped by RPP's own
                         # footprint check (that lane was 0 m - object search radius bug).
DETOUR_RAMP_M = 1.2      # m along the route to move over to the side lane (and back)
DETOUR_RAMP_MIN = 0.8    # m - shortest ramp; less room in front -> back up first
DETOUR_FRONT_GAP = 0.15  # m - lane must be reached this far before the bumper meets the object
DETOUR_BACK_MAX = 0.8    # m - longest backup to make room for the ramp
DETOUR_MAX_OFFSET = 1.2  # m - detour may leave the recorded route by at most this
                         # (was 1.0: Station_a->b test 2026-11-24 15:37 needed 1.03 m left)
DETOUR_FOOT_MARGIN = 0.03  # m - footprint check against /map and scan (was 0.05)
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
CORRIDOR_HEADING = 0.785 # rad - and facing the station's way within this (45 deg): a route
                         # that drives THROUGH the station first (Station_b->a passes it
                         # northbound, loops, comes back south) handed over at the first
                         # pass with heading -168 deg -> approach aborted, 2026-11-23
CORRIDOR_ENTRY = 0.25    # m - leg 1 hands over once within this before the pre-goal (MPPI
                         # stalled 0.18 m from it, then a recovery Spin; bag 20261020_161338)
LATERAL_ABORT = 0.35     # m - too far off the line: stop, give up (0.25 tripped after AMCL jumped at the pre-goal)
APPROACH_TIMEOUT = 90.0  # s
GHOST_RADIUS = 2.0       # m - lethal cells this close to the robot are checked before a retry
GHOST_SUPPORT = 0.15     # m - a lethal cell with a scan point this close is real
GHOST_MIN_CELLS = 3      # ghost cells needed before clearing
KEEPOUT_MIN_CELLS = 1    # keep-out lethal cells near the robot -> "blocked by keep-out" alert
GHOST_SETTLE_S = 1.0     # s - wait after clearing before the retry is sent
SCAN_FRESH_S = 1.0       # s - older /scan_filtered: do not clear anything
FOOT_FRONT = 0.49        # m - base_footprint bumper x (nav2_params.yaml footprint)
FOOT_REAR = -0.87        # m - base_footprint rear x
FOOT_HALF_W = 0.40       # m - footprint half width
BACKUP_FRONT_M = 0.50    # m - real obstacle this close to the bumper -> backup allowed
BACKUP_REAR_M = 0.60     # m - behind the rear must be empty this far (> RearStop 0.30)
BACKUP_SIDE_M = 0.10     # m - extra width of the front/rear check boxes
BACKUP_MIN_POINTS = 3    # scan points that count as an obstacle in a box
BACKUP_DIST = 0.30       # m
BACKUP_SPEED = 0.08      # m/s
BACKUP_TIME_S = 15       # s (int: BasicNavigator puts it in Duration.sec) - BackUp time allowance
BACKUP_MAX = 2           # backups per stuck spot
BACKUP_RESET_M = 1.0     # m - moved this far from the first backup spot: count restarts
CM_PARAMS = '/collision_monitor'


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


def pick_route(routes, dest, x, y, max_offset=ROUTE_MAX_OFFSET):
    """Remaining [x, y, yaw] points of the route to `dest` the robot is
    closest to, starting at its nearest point; None if none is within
    ROUTE_MAX_OFFSET."""
    best = None
    for r in routes:
        if r['to'] != dest or len(r['poses']) < 2:
            continue
        dists = [math.hypot(p[0] - x, p[1] - y) for p in r['poses']]
        i = min(range(len(dists)), key=dists.__getitem__)
        if dists[i] <= max_offset and (best is None or dists[i] < best[0]):
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


DUP_CHECK_S = 3.0        # s of DDS discovery before checking for a second executor


def another_instance(navigator):
    """True if a nav_goal_executor_node is already running. Two executors
    both act on every goal and preempt each other's FollowPath (2026-11-24:
    a hand-started one + the launch's one -> goals aborted every ~0.5 s)."""
    me = navigator.get_name()
    end = time.time() + DUP_CHECK_S
    while time.time() < end:
        rclpy.spin_once(navigator, timeout_sec=0.1)
    n = sum(1 for name, _ns in navigator.get_node_names_and_namespaces() if name == me)
    if n > 1:
        navigator.get_logger().fatal(
            'nav_goal_executor_node: %d nodes named %s are running - this one exits. '
            'Stop the other (pkill -f nav_goal_executor_node.py) and restart navigation.'
            % (n, me))
        return True
    return False


def main():
    rclpy.init()
    navigator = BasicNavigator()
    if another_instance(navigator):
        navigator.destroy_node()
        rclpy.shutdown()
        return 1
    navigator.get_logger().info('nav_goal_executor_node: waiting for Nav2 to become active...')
    wait_for_active(navigator, 'amcl')
    wait_for_active(navigator, 'bt_navigator')
    navigator.get_logger().info('nav_goal_executor_node: Nav2 active, ready for goals')

    task_active = {'value': False}
    pending = {'value': None}  # (kind, msg[, resume]) - kind 'pose'|'station'|'path'|'cancel'
    current = {'req': None}  # running request (kind, msg) for a slip retry
    held = {'value': None}       # request stopped by slip, waiting to be resent
    slip = {'since': None, 'clear_since': None}
    door = {'hold': False, 'name': '', 't': 0.0, 'req': None}  # door wait, see docstring
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

    latest = {'costmap': None, 'scan': None, 'scan_t': 0.0, 'keepout': None, 'gcostmap': None}
    clear_clients = [navigator.create_client(ClearEntireCostmap, srv) for srv in (
        '/local_costmap/clear_entirely_local_costmap',
        '/global_costmap/clear_entirely_global_costmap')]

    def on_costmap(msg):
        latest['costmap'] = msg

    def on_keepout(msg):
        latest['keepout'] = msg

    def in_keepout(x, y):
        """(x, y) in map frame inside the keep-out mask (+-1 cell)."""
        km = latest['keepout']
        if km is None:
            return False
        r = km.info.resolution
        ci = int((x - km.info.origin.position.x) / r)
        cj = int((y - km.info.origin.position.y) / r)
        for j in range(cj - 1, cj + 2):
            for i in range(ci - 1, ci + 2):
                if 0 <= i < km.info.width and 0 <= j < km.info.height and \
                        km.data[j * km.info.width + i] >= 100:
                    return True
        return False

    def on_scan(msg):
        latest['scan'] = msg
        latest['scan_t'] = time.time()

    def tf2d(target, source):
        """(x, y, yaw) of `source` frame in `target`, or None."""
        try:
            t = tf_buffer.lookup_transform(target, source, Time()).transform
        except Exception:  # noqa: BLE001 - any TF failure means "don't know"
            return None
        return t.translation.x, t.translation.y, yaw_of(t.rotation)

    def count_ghosts():
        """Lethal local-costmap cells near the robot that the live scan does not
        support, split into (ghosts, keepout): keepout = cells inside the
        keep-out mask. None = cannot judge (no data / no TF) -> do not clear."""
        cm, scan = latest['costmap'], latest['scan']
        if cm is None or scan is None or time.time() - latest['scan_t'] > SCAN_FRESH_S:
            return None
        frame = cm.header.frame_id
        robot = tf2d(frame, 'base_link')
        lidar = tf2d(frame, scan.header.frame_id)
        to_map = tf2d('map', frame)
        if robot is None or lidar is None:
            return None
        lx, ly, lyaw = lidar
        res = cm.info.resolution
        ox, oy = cm.info.origin.position.x, cm.info.origin.position.y
        # scan points bucketed by cell for a cheap neighbourhood lookup
        cells = set()
        for i, r in enumerate(scan.ranges):
            if not scan.range_min < r < scan.range_max:
                continue
            a = lyaw + scan.angle_min + i * scan.angle_increment
            cells.add((int((lx + r * math.cos(a) - ox) / res),
                       int((ly + r * math.sin(a) - oy) / res)))
        n = int(math.ceil(GHOST_SUPPORT / res))
        rx, ry = int((robot[0] - ox) / res), int((robot[1] - oy) / res)
        rad = int(GHOST_RADIUS / res)
        w, h = cm.info.width, cm.info.height
        ghosts = keepout = 0
        for j in range(max(0, ry - rad), min(h, ry + rad + 1)):
            for i in range(max(0, rx - rad), min(w, rx + rad + 1)):
                if cm.data[j * w + i] < 100:  # 100 = lethal (254); inflation is lower
                    continue
                if any((i + di, j + dj) in cells
                       for di in range(-n, n + 1) for dj in range(-n, n + 1)):
                    continue
                if to_map is not None:
                    cx, cy = ox + (i + 0.5) * res, oy + (j + 0.5) * res
                    mx = to_map[0] + cx * math.cos(to_map[2]) - cy * math.sin(to_map[2])
                    my = to_map[1] + cx * math.sin(to_map[2]) + cy * math.cos(to_map[2])
                    if in_keepout(mx, my):
                        keepout += 1
                        continue
                ghosts += 1
        return ghosts, keepout

    def scan_in_base():
        """Fresh /scan_filtered points in base_footprint, or None."""
        scan = latest['scan']
        if scan is None or time.time() - latest['scan_t'] > SCAN_FRESH_S:
            return None
        lidar = tf2d('base_footprint', scan.header.frame_id)
        if lidar is None:
            return None
        lx, ly, lyaw = lidar
        pts = []
        for i, r in enumerate(scan.ranges):
            if scan.range_min < r < scan.range_max:
                a = lyaw + scan.angle_min + i * scan.angle_increment
                pts.append((lx + r * math.cos(a), ly + r * math.sin(a)))
        return pts

    def backup_wanted():
        """(True, why) if a backup should be tried now."""
        pts = scan_in_base()
        if pts is None:
            return False, 'no fresh scan'
        w = FOOT_HALF_W + BACKUP_SIDE_M
        front = sum(1 for x, y in pts
                    if FOOT_FRONT < x < FOOT_FRONT + BACKUP_FRONT_M and abs(y) < w)
        rear = sum(1 for x, y in pts
                   if FOOT_REAR - BACKUP_REAR_M < x < FOOT_REAR and abs(y) < w)
        if front < BACKUP_MIN_POINTS:
            return False, 'nothing at the bumper (%d pts)' % front
        if rear >= BACKUP_MIN_POINTS:
            return False, 'rear not clear (%d pts)' % rear
        here = tf2d('map', 'base_footprint')
        if here is not None and backup['where'] is not None and math.hypot(
                here[0] - backup['where'][0], here[1] - backup['where'][1]) > BACKUP_RESET_M:
            backup['n'] = 0
            backup['where'] = None
        if backup['n'] >= BACKUP_MAX:
            return False, 'already backed up %d times here' % backup['n']
        return True, 'front %d pts, rear clear' % front

    def cm_call(client, req, timeout=2.0):
        if not client.wait_for_service(timeout_sec=timeout):
            return None
        fut = client.call_async(req)
        rclpy.spin_until_future_complete(navigator, fut, timeout_sec=timeout)
        return fut.result() if fut.done() else None

    def cm_set(values):
        """Set collision_monitor bool params atomically; True on success."""
        req = SetParametersAtomically.Request()
        for name, value in values.items():
            req.parameters.append(Parameter(name=name, value=ParameterValue(
                type=ParameterType.PARAMETER_BOOL, bool_value=value)))
        res = cm_call(cm_set_client, req)
        return res is not None and res.result.successful

    def start_backup(req, dist=BACKUP_DIST):
        """Switch the stop zones, start BackUp. False = not started (zones restored)."""
        names = ['FrontStop.enabled', 'FrontStopShort.enabled', 'RearStop.enabled']
        res = cm_call(cm_get_client, GetParameters.Request(names=names))
        if res is None or len(res.values) != 3 or \
                any(v.type != ParameterType.PARAMETER_BOOL for v in res.values):
            navigator.get_logger().warn('nav_goal_executor_node: backup skipped - '
                                        'cannot read collision_monitor zones')
            return False
        saved = {n: v.bool_value for n, v in zip(names, res.values)}
        backup_pub.publish(Bool(data=True))  # door_manager: hands off the zones
        if not cm_set({'RearStop.enabled': True, 'FrontStop.enabled': False,
                       'FrontStopShort.enabled': False}):
            cm_set(saved)
            navigator.get_logger().warn('nav_goal_executor_node: backup skipped - '
                                        'cannot switch collision_monitor zones')
            return False
        if not navigator.backup(backup_dist=dist, backup_speed=BACKUP_SPEED,
                                time_allowance=BACKUP_TIME_S):
            cm_set(saved)
            navigator.get_logger().warn('nav_goal_executor_node: BackUp goal rejected')
            return False
        here = tf2d('map', 'base_footprint')
        if backup['where'] is None and here is not None:
            backup['where'] = here[:2]
        backup['n'] += 1
        backup.update(active=True, req=req, saved=saved)
        navigator.get_logger().warn('nav_goal_executor_node: blocked at the nose - backing up '
                                    '%.2f m (%d/%d)' % (dist, backup['n'], BACKUP_MAX))
        alert('warn', 'หน้ารถติดสิ่งกีดขวาง - ถอยหลัง %.0f cm แล้วไปต่ออัตโนมัติ'
              % (dist * 100))
        return True

    def end_backup(resend):
        """Restore the zones; re-send the request only if that worked."""
        backup['active'] = False
        req, backup['req'] = backup['req'], None
        ok = cm_set(backup['saved']) or cm_set(backup['saved'])
        backup_pub.publish(Bool(data=False))
        if not ok:
            navigator.get_logger().error(
                'nav_goal_executor_node: could not restore collision_monitor zones %s '
                'after backup - request dropped' % backup['saved'])
            alert('error', 'คืนค่า safety zone หลังถอยไม่สำเร็จ - หยุดรอ operator')
            return
        then_detour = backup.pop('detour', False)
        if resend and req is not None:
            if then_detour:
                # re-sending the route would drive straight back to the object
                # and use up the room just made (test 2026-11-24 15:33)
                why = send_detour(req)
                if why is None:
                    return
                if why.startswith('no object'):
                    pending['value'] = req + (True,)
                else:
                    hold_for_detour(req, why)
                return
            pending['value'] = req + (True,)

    def clear_ghosts():
        counted = count_ghosts()
        if counted is None:
            navigator.get_logger().warn(
                'nav_goal_executor_node: ghost check skipped (no fresh scan/costmap/TF)')
            return False
        ghosts = counted[0]
        if ghosts < GHOST_MIN_CELLS:  # single cells are scan noise
            return False
        navigator.get_logger().warn(
            'nav_goal_executor_node: %d lethal cells not seen by LiDAR - clearing costmaps '
            'before retry' % ghosts)
        for c in clear_clients:
            if c.service_is_ready():
                c.call_async(ClearEntireCostmap.Request())
        return True

    stuck = {'t0': None, 'where': None}  # first failure of a request at this spot

    def stuck_for():
        """s since the running request first failed here (restarts after
        moving BACKUP_RESET_M)."""
        here = tf2d('map', 'base_footprint')
        now = time.time()
        if stuck['t0'] is None or (here is not None and stuck['where'] is not None and math.hypot(
                here[0] - stuck['where'][0], here[1] - stuck['where'][1]) > BACKUP_RESET_M):
            stuck['t0'], stuck['where'] = now, here[:2] if here is not None else None
        return now - stuck['t0']

    backup = {'active': False, 'req': None, 'saved': None, 'n': 0, 'where': None, 'pub_t': 0.0}
    backup_pub = navigator.create_publisher(Bool, '/nav_backup_active', 10)
    cm_get_client = navigator.create_client(GetParameters, CM_PARAMS + '/get_parameters')
    cm_set_client = navigator.create_client(
        SetParametersAtomically, CM_PARAMS + '/set_parameters_atomically')

    def hold_for_detour(req, why):
        """Detour not possible now: stand where we are and try the detour
        again in BLOCK_RETRY_S - re-sending the route drove the robot back
        into the object (test 16:02, 0.02 m from the bumper)."""
        wait_for_clear(req, BLOCK_RETRY_S, 'สิ่งกีดขวางไม่ขยับ และอ้อมไม่ได้ (%s) - จอดรอ' % why)
        blocked['detour'] = True

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

    def station_route(goal, max_offset=ROUTE_MAX_OFFSET):
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
                                t.transform.translation.y, max_offset)

    def object_ahead(depth=DETOUR_FRONT_M):
        """Scan points (base_footprint) of the object in front of the bumper."""
        pts = scan_in_base() or []
        return [(x, y) for x, y in pts if FOOT_FRONT < x < FOOT_FRONT + depth
                and abs(y) < FOOT_HALF_W + BACKUP_SIDE_M]

    def plan_detour(goal):
        """Lane shift around the object on the route to `goal`.
        Returns (Path, info, 0), (None, why, 0) or (None, why, backup_m)."""
        try:
            _, points = station_route(goal, DETOUR_MAX_OFFSET)
        except (OSError, KeyError, TypeError, AttributeError, yaml.YAMLError) as ex:
            return None, 'bad routes file: %s' % ex, 0
        here = tf2d('map', 'base_footprint')
        if points is None or here is None:
            return None, 'robot is more than %.1f m from the route' % DETOUR_MAX_OFFSET, 0
        c, s_ = math.cos(here[2]), math.sin(here[2])
        obs = [o for o in ((here[0] + x * c - y * s_, here[1] + x * s_ + y * c)
                           for x, y in (scan_in_base() or []) if math.hypot(x, y) < DETOUR_SCAN_M)
               if not on_static_map(*o)]
        n = len(points)
        cum = [0.0]
        for i in range(1, n):
            cum.append(cum[-1] + math.hypot(points[i][0] - points[i - 1][0],
                                            points[i][1] - points[i - 1][1]))
        # the route's own heading (recorded yaw can be off on a weave)
        yaws = [math.atan2(points[min(i + 1, n - 1)][1] - points[max(i - 1, 0)][1],
                           points[min(i + 1, n - 1)][0] - points[max(i - 1, 0)][0])
                for i in range(n)]

        def lateral(i, o):
            dx, dy = o[0] - points[i][0], o[1] - points[i][1]
            return -dx * math.sin(yaws[i]) + dy * math.cos(yaws[i])   # + = left
        # object points: those within DETOUR_BLOCK_R of the route ahead
        hits, near = [], []
        for o in obs:
            best = None
            for i in range(n):
                if cum[i] > DETOUR_LOOK_M:
                    break
                d = math.hypot(points[i][0] - o[0], points[i][1] - o[1])
                if best is None or d < best[0]:
                    best = (d, i)
            if best is not None and best[0] < DETOUR_BLOCK_R:
                hits.append(best[1])
                near.append((best[1], lateral(best[1], o)))
        if len(hits) < BACKUP_MIN_POINTS:
            return None, 'no object on the route within %.1f m (scan)' % DETOUR_LOOK_M, 0
        need = FOOT_HALF_W + DETOUR_SIDE_CLEAR
        cur = lateral(0, here[:2])  # robot's offset from the route now (+ = left)
        # object points along the route: (distance along, lateral offset)
        objs = [(cum[i], lt) for i, lt in near]
        first = min(a for a, _ in objs)
        last = max(a for a, _ in objs)
        # walls from the saved map, objects from the live scan: the global
        # costmap also keeps old obstacle marks (moved object, people) and
        # rejected shifts of 0.10 m 'hitting a wall' on the route (test 15:57)
        grid = latest.get('map')
        if grid is None:
            return None, 'no /map', 0
        gw, gh, gr = grid.info.width, grid.info.height, grid.info.resolution
        gox, goy = grid.info.origin.position.x, grid.info.origin.position.y

        hit_at = []

        def footprint_hit(x, y, yaw):
            """Reason string if the footprint at (x, y, yaw) hits something."""
            cy, sy = math.cos(yaw), math.sin(yaw)
            m = DETOUR_FOOT_MARGIN
            fx = FOOT_REAR - m
            while fx <= FOOT_FRONT + m + 1e-6:
                fy = -FOOT_HALF_W - m
                while fy <= FOOT_HALF_W + m + 1e-6:
                    wx, wy = x + fx * cy - fy * sy, y + fx * sy + fy * cy
                    ci, cj = int((wx - gox) / gr), int((wy - goy) / gr)
                    if not (0 <= ci < gw and 0 <= cj < gh):
                        hit_at[:] = [wx, wy]
                        return 'outside the map'
                    v = grid.data[cj * gw + ci]
                    if v >= 65:   # occupied in the saved map
                        hit_at[:] = [wx, wy]
                        return 'wall at (%.2f, %.2f)' % (wx, wy)
                    if v < 0:
                        hit_at[:] = [wx, wy]
                        return 'unknown space at (%.2f, %.2f)' % (wx, wy)
                    if in_keepout(wx, wy):
                        hit_at[:] = [wx, wy]
                        return 'keep-out at (%.2f, %.2f)' % (wx, wy)
                    fy += gr
                fx += gr
            for ox, oy in obs:
                dx, dy = ox - x, oy - y
                lx, ly = dx * cy + dy * sy, -dx * sy + dy * cy
                if FOOT_REAR - m < lx < FOOT_FRONT + m and abs(ly) < FOOT_HALF_W + m:
                    hit_at[:] = [ox, oy]
                    return 'scan point at (%.2f, %.2f)' % (ox, oy)
            return None


        # Lateral offset per route point (+ = left), searched over the whole
        # stretch at once (dynamic programming on a DETOUR_GRID grid): start at
        # the robot's offset, every object point beside the footprint (bumper ..
        # rear, + margin) >= `need` from the centre line, lane changes <=
        # DETOUR_SLOPE, end back on the route; least total offset wins. Objects
        # at different places can be passed on different sides, and the robot
        # starts moving over early enough (a greedy point-by-point choice had
        # to jump sideways at the next object: 'turns too sharply', test 16:32).
        # An object point constrains the lane offset where it is along the route
        # (the robot follows the lane, so its side passes the point at the lane's
        # offset there), +- DETOUR_OBJ_WIN. One offset for the whole 1.4 m body
        # (bumper .. rear window) refused a 2.5 deg lean between clutter behind
        # and the object ahead (snapshot 16:10); every pose's real footprint is
        # checked afterwards anyway.
        lo_a, hi_a = -DETOUR_OBJ_WIN, DETOUR_OBJ_WIN
        end_c = last - FOOT_REAR + DETOUR_EXIT_EXTRA + DETOUR_MAX_OFFSET / DETOUR_SLOPE
        if cum[-1] < end_c:
            return None, 'object too close to the end of the route', 0
        E = next(i for i in range(n) if cum[i] >= end_c)
        G = DETOUR_GRID
        ns = int(round(DETOUR_MAX_OFFSET / G))
        grid_s = [k * G for k in range(-ns, ns + 1)]

        # every unmapped scan point near the stretch (not only those on the route)
        allobj = []
        for o in obs:
            j = min(range(E + 1), key=lambda q: math.hypot(points[q][0] - o[0], points[q][1] - o[1]))
            lt = lateral(j, o)
            if abs(lt) <= DETOUR_MAX_OFFSET + need:
                allobj.append((cum[j], lt))
        # free sideways room to the /map walls (or unknown) at each route point
        def wall_dist(i, side):
            d = 0.0
            while d < DETOUR_MAX_OFFSET + FOOT_HALF_W + 0.3:
                wx = points[i][0] - side * d * math.sin(yaws[i])
                wy = points[i][1] + side * d * math.cos(yaws[i])
                ci, cj = int((wx - gox) / gr), int((wy - goy) / gr)
                if not (0 <= ci < gw and 0 <= cj < gh) or grid.data[cj * gw + ci] >= 65 or \
                        grid.data[cj * gw + ci] < 0 or in_keepout(wx, wy):
                    return d
                d += gr / 2
            return d
        wl = [wall_dist(i, 1.0) for i in range(E + 1)]
        wr = [wall_dist(i, -1.0) for i in range(E + 1)]
        half = FOOT_HALF_W + DETOUR_FOOT_MARGIN

        def allowed(i):
            # walls are continuous: the whole body (rear .. bumper) must fit
            win = [j for j in range(E + 1)
                   if FOOT_REAR - DETOUR_FOOT_MARGIN <= cum[j] - cum[i] <= FOOT_FRONT +
                   DETOUR_FOOT_MARGIN] or [i]
            left_room, right_room = min(wl[j] for j in win), min(wr[j] for j in win)
            iv = [(lt - need, lt + need) for a_, lt in allobj if lo_a <= a_ - cum[i] <= hi_a]
            hold = cum[i] <= last - FOOT_REAR + DETOUR_EXIT_EXTRA
            return [k for k, v in enumerate(grid_s)
                    if v + half <= left_room + 1e-6 and -v + half <= right_room + 1e-6 and
                    all(not (lo + 1e-6 < v < hi - 1e-6) for lo, hi in iv)], hold

        last_bad = None
        for _repair in range(DETOUR_REPAIRS):
            INF = float('inf')
            k_cur = min(range(len(grid_s)), key=lambda k: abs(grid_s[k] - cur))
            cost = [INF] * len(grid_s)
            for k in range(len(grid_s)):
                if abs(grid_s[k] - cur) <= 0.08 + 1e-9:   # RPP takes up a few cm
                    cost[k] = 0.0
            ok0, _ = allowed(0)
            cost = [cost[k] if k in ok0 else INF for k in range(len(grid_s))]
            if min(cost) == INF:
                cost = [INF] * len(grid_s)
                cost[k_cur] = 0.0      # robot already there: trust where it stands
            back_ptr = [[None] * len(grid_s)]
            fail_at = None
            for i in range(1, E + 1):
                step = int(DETOUR_SLOPE * (cum[i] - cum[i - 1]) / G + 1e-6) + 1
                ok, hold = allowed(i)
                okset = set(ok)
                nc, bp = [INF] * len(grid_s), [None] * len(grid_s)
                for k in ok:
                    best, arg = INF, None
                    for kk in range(max(0, k - step), min(len(grid_s), k + step + 1)):
                        if cost[kk] == INF:
                            continue
                        if abs(grid_s[k] - grid_s[kk]) > DETOUR_SLOPE * (cum[i] - cum[i - 1]) + 1e-6:
                            continue
                        c = cost[kk] + abs(grid_s[k]) * (cum[i] - cum[i - 1]) + \
                            (0.0 if k == kk else 0.002)
                        if hold and k != kk:
                            c += 0.05     # keep the lane until the rear has passed (RPP)
                        if c < best:
                            best, arg = c, kk
                    nc[k], bp[k] = best, arg
                cost = nc
                back_ptr.append(bp)
                if min(cost) == INF:
                    fail_at = i
                    break
            k_end = grid_s.index(0.0) if 0.0 in grid_s else ns
            if fail_at is not None or cost[k_end] == INF:
                # would it work if the robot could start anywhere? -> back up for room
                where = cum[fail_at] if fail_at is not None else cum[E]
                if cum[min(fail_at or E, n - 1)] - FOOT_FRONT < DETOUR_RAMP_MIN and \
                        first - FOOT_FRONT < DETOUR_RAMP_MIN:
                    back = min(DETOUR_BACK_MAX, DETOUR_RAMP_MIN - (first - FOOT_FRONT) + 0.05)
                    return None, ('object %.2f m from the bumper, no lane from here - backing up '
                                  '%.2f m for room' % (first - FOOT_FRONT, back)), back
                return None, 'no room: no lane with %.2f m side gaps at %.1f m along' % (
                    DETOUR_SIDE_CLEAR, where), 0
            off = [0.0] * n
            k = k_end
            for i in range(E, -1, -1):
                off[i] = grid_s[k]
                if i > 0:
                    k = back_ptr[i][k]
            off[0] = cur
            pts = [[points[i][0] - off[i] * math.sin(yaws[i]),
                    points[i][1] + off[i] * math.cos(yaws[i])] for i in range(n)]
            for i in range(n):
                j0, j1 = max(i - 1, 0), min(i + 1, n - 1)
                pts[i].append(math.atan2(pts[j1][1] - pts[j0][1], pts[j1][0] - pts[j0][0]))
            pts[-1][2] = points[-1][2]
            bad = None
            for i in range(n):
                if cum[i] > cum[E] + 0.5:
                    break
                if cum[i] < DETOUR_SKIP_START:
                    continue   # where the robot already stands (its real yaw differs
                               # from the route's; touched a map wall, test 16:21)
                bad = footprint_hit(*pts[i])
                if bad:
                    break
            if bad:
                # the real (leaning) footprint touched something the per-point wall
                # model missed: shrink the room on that side around there, re-plan
                hx, hy = hit_at
                side_left = lateral(i, (hx, hy)) > off[i]
                for j in range(E + 1):
                    if abs(cum[j] - cum[i]) <= FOOT_FRONT - FOOT_REAR:
                        if side_left:
                            wl[j] = min(wl[j], max(0.0, off[i] + half - gr))
                        else:
                            wr[j] = min(wr[j], max(0.0, -off[i] + half - gr))
                last_bad = 'lane hits %s at %.1f m along' % (bad, cum[i])
                continue
            big = max(off[:E + 1], key=abs)
            return route_to_path(pts, navigator), (
                'lane shift max %.2f m %s, objects %.1f-%.1f m ahead, robot at %+.2f m, '
                'back on the route %.1f m ahead' % (
                    abs(big), 'left' if big > 0 else 'right', first, last, cur, cum[E])), 0
        return None, 'no room: ' + (last_bad or 'lane search failed'), 0

    def on_static_map(x, y):
        """(x, y) is a wall of the saved map (+-1 cell): not a new object."""
        m = latest.get('map')
        if m is None:
            return False
        r = m.info.resolution
        ci = int((x - m.info.origin.position.x) / r)
        cj = int((y - m.info.origin.position.y) / r)
        for j in range(cj - 1, cj + 2):
            for i in range(ci - 1, ci + 2):
                if 0 <= i < m.info.width and 0 <= j < m.info.height and \
                        m.data[j * m.info.width + i] >= 65:
                    return True
        return False

    early = {'t': 0.0}

    def object_on_route_ahead():
        """(n points, bumper distance) of new objects (not map walls) on the
        route being followed within DETOUR_EARLY_M of the bumper, or None."""
        path = route_plan['value']
        here = tf2d('map', 'base_footprint')
        pts = scan_in_base()
        if path is None or here is None or pts is None or len(path.poses) < 2:
            return None
        xy = [(p.pose.position.x, p.pose.position.y) for p in path.poses]
        k = min(range(len(xy)), key=lambda i: math.hypot(xy[i][0] - here[0], xy[i][1] - here[1]))
        ahead, d = [], 0.0
        for i in range(k, len(xy)):
            if i > k:
                d += math.hypot(xy[i][0] - xy[i - 1][0], xy[i][1] - xy[i - 1][1])
            ahead.append((xy[i], d))
        if ahead[-1][1] < DETOUR_EARLY_MIN_LEFT:
            return None   # close to the end: station area, leave it to RPP/approach
        reach = FOOT_FRONT + DETOUR_EARLY_M
        ahead = [a for a in ahead if a[1] <= reach]
        c, s_ = math.cos(here[2]), math.sin(here[2])
        n, nearest = 0, None
        for x, y in pts:
            if not (FOOT_FRONT - 0.05 < x < reach + DETOUR_BLOCK_R):
                continue
            ox, oy = here[0] + x * c - y * s_, here[1] + x * s_ + y * c
            dist, along = min((math.hypot(px - ox, py - oy), dd) for (px, py), dd in ahead)
            # on a lane shift only points the footprint would really touch: the
            # side of the object, unseen when the shift was planned, came into
            # view within 0.55 m and stopped the robot mid-detour (test 16:00)
            r = FOOT_HALF_W + DETOUR_FOOT_MARGIN if path is early.get('detour') \
                else DETOUR_BLOCK_R
            if dist < r and not on_static_map(ox, oy):
                n += 1   # along = route point beside the object, not one before it
                nearest = along if nearest is None else min(nearest, along)
        return (n, nearest - FOOT_FRONT) if n >= BACKUP_MIN_POINTS else None

    def detour_rear_points(dist):
        """Scan points behind the rear within `dist` + DETOUR_REAR_MARGIN
        (footprint width + BACKUP_SIDE_M); None without a fresh scan."""
        pts = scan_in_base()
        if pts is None:
            return None
        w = FOOT_HALF_W + BACKUP_SIDE_M
        return sum(1 for x, y in pts
                   if FOOT_REAR - dist - DETOUR_REAR_MARGIN < x < FOOT_REAR and abs(y) < w)

    def send_detour(req):
        """Drive the station request `req` around the object; None on
        success, else why not."""
        path, info, back = plan_detour(req[1])
        if path is None and back > 0:
            navigator.get_logger().warn('nav_goal_executor_node: detour - ' + info)
            rear = detour_rear_points(back)
            if rear is None:
                info += ' (backup not possible: no fresh scan)'
            elif rear >= BACKUP_MIN_POINTS:
                # Nav2 BackUp would fail on it and use up a backup (test 15:49)
                info += ' (backup not possible: rear not clear, %d pts within %.2f m)' % (
                    rear, back + DETOUR_REAR_MARGIN)
            elif backup['n'] < BACKUP_MAX and start_backup(req, back):
                backup['detour'] = True   # after it: plan the lane shift, not the route
                return None
            else:
                info += ' (backup not possible: %d/%d backups used here)' % (
                    backup['n'], BACKUP_MAX)
        if path is None:
            navigator.get_logger().warn('nav_goal_executor_node: detour rejected - ' + info)
            return info
        navigator.get_logger().warn('nav_goal_executor_node: temporary detour - ' + info)
        current['req'] = req
        station['value'] = req[1]
        route_plan['value'] = path
        early['detour'] = path
        if not sent(navigator.followPath(path), 'detour'):
            return 'Nav2 rejected the detour'
        alert('warn', 'สิ่งกีดขวางไม่ขยับ - ใช้ทางอ้อมชั่วคราวแล้วกลับเข้าเส้นทางเดิม')
        return None

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

    def on_door_hold(msg):
        try:
            d = json.loads(msg.data)
        except ValueError:
            return
        door['hold'] = bool(d.get('hold'))
        door['name'] = str(d.get('door', ''))
        door['t'] = time.time()

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

    # station trip start/end events on /nav_event so a rosbag (goal_picker
    # "record motor A + battery" preset) shows when each trip began/ended
    event_pub = navigator.create_publisher(String, '/nav_event', 10)
    trip = {'name': None, 't0': 0.0}

    def station_name(goal):
        px, py = goal.pose.position.x, goal.pose.position.y
        return next((s['name'] for s in stations_info['stations']
                     if math.hypot(s['x'] - px, s['y'] - py) < STATION_MATCH),
                    '(%.2f, %.2f)' % (px, py))

    def nav_event(kind, name=None):
        now = time.time()
        if kind == 'start':
            trip['name'], trip['t0'] = name, now
        elif trip['name'] is None:
            return
        d = {'event': kind, 'station': trip['name'],
             'time': time.strftime('%Y-%m-%d %H:%M:%S', time.localtime(now))}
        if kind != 'start':
            d['duration_s'] = round(now - trip['t0'], 1)
            trip['name'] = None
        event_pub.publish(String(data=json.dumps(d, ensure_ascii=False)))
        navigator.get_logger().info('nav_goal_executor_node: event ' + json.dumps(d, ensure_ascii=False))

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
    navigator.create_subscription(String, '/door_hold', on_door_hold, 10)
    navigator.create_subscription(String, '/amr_stations', on_stations, 10)
    navigator.create_subscription(Twist, '/cmd_vel_smoothed', on_smoothed, 10)
    navigator.create_subscription(Twist, '/cmd_vel', on_output, 10)
    navigator.create_subscription(
        OccupancyGrid, '/local_costmap/costmap', on_costmap,
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    navigator.create_subscription(LaserScan, '/scan_filtered', on_scan, qos_profile_sensor_data)
    navigator.create_subscription(
        OccupancyGrid, '/global_costmap/costmap', lambda m: latest.__setitem__('gcostmap', m),
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    navigator.create_subscription(
        OccupancyGrid, '/map', lambda m: latest.__setitem__('map', m),
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
    navigator.create_subscription(
        OccupancyGrid, '/keepout_filter_mask', on_keepout,
        QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))

    try:
        while rclpy.ok():
            # 20 Hz while our own final approach is driving
            rclpy.spin_once(navigator, timeout_sec=0.05 if approach['value'] else 0.2)

            if pending['value'] is not None:
                kind, msg = pending['value'][:2]
                resume = len(pending['value']) > 2
                pending['value'] = None
                if not resume:  # operator command ends any station trip in progress
                    nav_event('canceled' if kind == 'cancel' else 'interrupted')
                    if kind == 'station':
                        nav_event('start', station_name(msg))

                if task_active['value']:
                    cancel_and_wait()
                    task_active['value'] = False
                if backup['active']:
                    cancel_and_wait()
                    end_backup(resend=False)
                if not resume:
                    backup['n'] = 0
                    backup['where'] = None
                    stuck['t0'] = None
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
                    door['req'] = None
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
                            abs(e[1]) < CORRIDOR_LATERAL and \
                            abs(e[2]) < CORRIDOR_HEADING:
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
                        if points is None and resume and send_detour(current['req']) is None:
                            continue  # off the route after a detour: rejoin it
                        if points is None:
                            current['req'] = None
                            nav_event('failed')
                            alert('error', 'ไม่มีเส้นทางจากตำแหน่งนี้ไป Station ที่เลือก '
                                           '- หุ่นยนต์ไม่ออกตัว')
                            continue
                        navigator.get_logger().info(
                            'nav_goal_executor_node: station (%.2f, %.2f) - fixed route, '
                            '%d points' % (msg.pose.position.x, msg.pose.position.y, len(points)))
                        station['value'] = msg
                        route_plan['value'] = route_to_path(points, navigator)
                        if not sent(navigator.followPath(route_plan['value']), 'station route'):
                            nav_event('failed')
                        continue
                    pre = pre_goal(msg)
                    navigator.get_logger().info(
                        'nav_goal_executor_node: station (%.2f, %.2f) - pre-goal (%.2f, %.2f)'
                        % (msg.pose.position.x, msg.pose.position.y,
                           pre.pose.position.x, pre.pose.position.y))
                    station['value'] = msg
                    if not sent(navigator.goToPose(restamp(pre, navigator)), 'station pre-goal'):
                        nav_event('failed')
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
                        abs(e[1]) < CORRIDOR_LATERAL and \
                        abs(e[2]) < CORRIDOR_HEADING:
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
                    on_route = req[0] == 'station' and route_plan['value'] is not None
                    wait_for_clear(req, max(0.0, DETOUR_AFTER_S - BLOCK_S) if on_route
                                   else BLOCK_RETRY_S,
                                   'มีสิ่งกีดขวาง - จอดรอ จะไปต่ออัตโนมัติเมื่อทางโล่ง')
                    blocked['detour'] = on_route
            else:
                blocked['since'] = None
            if blocked['req'] is not None and time.time() > blocked['resend_at']:
                if not blocked.get('cleared') and clear_ghosts():
                    # let the costmaps clear and re-mark from the scan first
                    blocked['cleared'] = True
                    blocked['resend_at'] = time.time() + GHOST_SETTLE_S
                    continue
                blocked['cleared'] = False
                req, blocked['req'] = blocked['req'], None
                if blocked.pop('detour', False):
                    why = send_detour(req)
                    if why is None:
                        continue
                    if not why.startswith('no object'):
                        hold_for_detour(req, why)
                        continue
                    # object gone in the wait: carry on along the route
                pending['value'] = req + (True,)

            # early stop: object on the route ahead -> stand DETOUR_EARLY_M
            # short of it, so a lane shift has room after DETOUR_AFTER_S
            if task_active['value'] and route_plan['value'] is not None and \
                    current['req'] is not None and current['req'][0] == 'station' and \
                    not backup['active'] and time.time() - early['t'] > 1.0 / DETOUR_EARLY_HZ:
                early['t'] = time.time()
                found = object_on_route_ahead()
                if found is not None:
                    req = current['req']
                    cancel_and_wait()
                    task_active['value'] = False
                    station['value'] = None
                    current['req'] = None
                    navigator.get_logger().warn(
                        'nav_goal_executor_node: object on the route %.2f m ahead of the bumper '
                        '(%d pts) - stopping, detour in %.0f s if it stays' % (
                            found[1], found[0], DETOUR_AFTER_S))
                    wait_for_clear(req, DETOUR_AFTER_S,
                                   'มีสิ่งกีดขวางบนเส้นทาง - จอดรอ %.0f วิ ถ้าไม่ขยับจะอ้อม' % DETOUR_AFTER_S)
                    blocked['detour'] = True

            # door wait: stand still while door_manager holds, then carry on
            door_holding = door['hold'] and time.time() - door['t'] < DOOR_HOLD_STALE_S
            if door_holding and (task_active['value'] or approach['value'] is not None):
                stop_all()
                if current['req'] is not None:
                    door['req'] = current['req']
                current['req'] = None
                alert('warn', 'รอประตู %s เปิด - จอดรอ แล้วจะไปต่ออัตโนมัติ' % door['name'])
            if door['req'] is not None and not door_holding:
                navigator.get_logger().info('nav_goal_executor_node: door wait over - resending request')
                pending['value'] = door['req'] + (True,)
                door['req'] = None
                alert('ok', '')

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
                nav_event('arrived' if approach['value'].state == 'done' else 'failed')
                approach['value'] = None
                current['req'] = None

            if backup['active']:
                if time.time() - backup['pub_t'] > 0.2:
                    backup['pub_t'] = time.time()
                    backup_pub.publish(Bool(data=True))
                if navigator.isTaskComplete():
                    navigator.get_logger().info(
                        'nav_goal_executor_node: backup finished (%s) - resending request'
                        % navigator.getResult())
                    end_backup(resend=True)
                continue

            if task_active['value'] and navigator.isTaskComplete():
                result = navigator.getResult()
                task_active['value'] = False
                if result == TaskResult.FAILED and current['req'] is not None:
                    req = current['req']
                    station['value'] = None
                    current['req'] = None
                    counted = count_ghosts()
                    ghosts, kept = counted if counted is not None else (None, 0)
                    want, why = backup_wanted()
                    stuck_s = stuck_for()
                    navigator.get_logger().info(
                        'nav_goal_executor_node: request failed - ghosts %s, keep-out %s, '
                        'stuck %.1f s, backup: %s' % (ghosts, kept, stuck_s, why))
                    # RPP 'collision ahead' aborts FollowPath before collision_monitor
                    # ever stops the robot: a route that keeps failing at the same
                    # spot for DETOUR_AFTER_S is a static object -> go around it
                    on_detour = route_plan['value'] is not None and \
                        route_plan['value'] is early.get('detour')
                    if req[0] == 'station' and on_detour:
                        # the lane shift itself failed (RPP): stand, re-plan in 3 s
                        # instead of a tight re-plan loop or the route into the object
                        hold_for_detour(req, 'lane shift blocked (RPP collision ahead)')
                        continue
                    if req[0] == 'station' and stuck_s >= DETOUR_AFTER_S and \
                            (ghosts is None or ghosts < GHOST_MIN_CELLS):
                        detour_why = send_detour(req)
                        if detour_why is None:
                            continue
                        if not detour_why.startswith('no object'):
                            hold_for_detour(req, detour_why)
                            continue
                    if want and ghosts is not None and ghosts < GHOST_MIN_CELLS and \
                            start_backup(req):
                        continue
                    if kept >= KEEPOUT_MIN_CELLS:
                        navigator.get_logger().warn(
                            'nav_goal_executor_node: %d keep-out cells within %.1f m - the '
                            'path/footprint runs into a keep-out zone' % (kept, GHOST_RADIUS))
                        wait_for_clear(req, FAILED_RETRY_S,
                                       'ไปต่อไม่ได้ - ติด Keepout zone ใกล้หุ่นยนต์ '
                                       '(ตรวจ Map Edit / เส้นทาง) - จอดรอแล้วลองใหม่')
                    else:
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
                        nav_event('failed')
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
