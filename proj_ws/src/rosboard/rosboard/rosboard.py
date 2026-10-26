#!/usr/bin/env python3

import asyncio
import importlib
import os
import random
import signal
import socket
import subprocess
import threading
import time
import tornado, tornado.web, tornado.websocket
import traceback

if os.environ.get("ROS_VERSION") == "1":
    import rospy # ROS1
elif os.environ.get("ROS_VERSION") == "2":
    import rosboard.rospy2 as rospy # ROS2
    from rclpy.qos import HistoryPolicy, QoSProfile, QoSReliabilityPolicy, QoSDurabilityPolicy
else:
    print("ROS not detected. Please source your ROS environment\n(e.g. 'source /opt/ros/DISTRO/setup.bash')")
    exit(1)

import math

from rosgraph_msgs.msg import Log
from geometry_msgs.msg import Twist, PoseStamped, PoseWithCovarianceStamped
from nav_msgs.msg import Path
from std_msgs.msg import Empty, String
import json
import re

from rosboard.serialization import ros2dict
from rosboard.subscribers.dmesg_subscriber import DMesgSubscriber
from rosboard.subscribers.processes_subscriber import ProcessesSubscriber
from rosboard.subscribers.system_stats_subscriber import SystemStatsSubscriber
from rosboard.subscribers.dummy_subscriber import DummySubscriber
from rosboard.handlers import LATCHED_TOPICS, ROSBoardSocketHandler, NoCacheStaticFileHandler


class ROSBoardNode(object):
    instance = None
    def __init__(self, node_name = "rosboard_node"):
        self.__class__.instance = self
        rospy.init_node(node_name)
        self.port = rospy.get_param("~port", 8888)
        self.title = rospy.get_param("~title", socket.gethostname())

        # desired subscriptions of all the websockets connecting to this instance.
        # these remote subs are updated directly by "friend" class ROSBoardSocketHandler.
        # this class will read them and create actual ROS subscribers accordingly.
        # dict of topic_name -> set of sockets
        self.remote_subs = {}

        # actual ROS subscribers.
        # dict of topic_name -> ROS Subscriber
        self.local_subs = {}

        # minimum update interval per topic (throttle rate) amang all subscribers to a particular topic.
        # we can throw data away if it arrives faster than this
        # dict of topic_name -> float (interval in seconds)
        self.update_intervals_by_topic = {}

        # last time data arrived for a particular topic
        # dict of topic_name -> float (time in seconds)
        self.last_data_times_by_topic = {}
        self.ROSOUT_MAX_PER_S = 30
        self.rosout_window_start = 0.0
        self.rosout_count = 0
        # Latched topics (/map, /tf_static) are published once per publisher.
        # They must never be rate-limited (several /tf_static publishers -
        # robot_state_publisher, camera static TF - deliver their one-shot
        # messages within the same second, and the 1 Hz limit dropped all
        # but the first: base_link->lidar_link never reached the browser,
        # so goal_picker could not place the scan). The last data is also
        # kept so a client that connects later still gets it (see
        # handlers.py MSG_SUB) - the ROS subscription is shared and will not
        # receive the latched message again.
        self.latched_msgs = {}

        if rospy.__name__ == "rospy2":
            # ros2 hack: need to subscribe to at least 1 topic
            # before dynamic subscribing will work later.
            # ros2 docs don't explain why but we need this magic.
            self.sub_rosout = rospy.Subscriber("/rosout", Log, lambda x:x)

        tornado_settings = {
            'debug': True,
            'static_path': os.path.join(os.path.dirname(os.path.realpath(__file__)), 'html')
        }

        tornado_handlers = [
                (r"/rosboard/v1", ROSBoardSocketHandler, {
                    "node": self,
                }),
                (r"/(.*)", NoCacheStaticFileHandler, {
                    "path": tornado_settings.get("static_path"),
                    "default_filename": "index.html"
                }),
        ]

        self.event_loop = None
        self.tornado_application = tornado.web.Application(tornado_handlers, **tornado_settings)
        asyncio.set_event_loop(asyncio.new_event_loop())
        self.event_loop = tornado.ioloop.IOLoop()
        self.tornado_application.listen(self.port)

        # allows tornado to log errors to ROS
        self.logwarn = rospy.logwarn
        self.logerr = rospy.logerr

        # tornado event loop. all the web server and web socket stuff happens here
        threading.Thread(target = self.event_loop.start, daemon = True).start()

        # loop to sync remote (websocket) subs with local (ROS) subs
        threading.Thread(target = self.sync_subs_loop, daemon = True).start()

        # loop to keep track of latencies and clock differences for each socket
        threading.Thread(target = self.pingpong_loop, daemon = True).start()

        self.lock = threading.Lock()

        # --- access PIN gate ---
        # Freshly random each time rosboard (re)starts. Anyone on the LAN
        # can reach this port with no login, so this is a basic "you have
        # to be standing at the robot and read the screen" gate, not
        # cryptographic security - see handlers.py for enforcement.
        self.access_pin = "%06d" % random.randint(0, 999999)
        pin_file = os.path.expanduser("~/.rosboard_pin")
        try:
            with open(pin_file, "w") as f:
                f.write(self.access_pin)
            os.chmod(pin_file, 0o600)
        except OSError as e:
            rospy.logwarn("Could not write PIN file %s: %s" % (pin_file, str(e)))
        rospy.loginfo("ROSboard access PIN: %s" % self.access_pin)

        # --- mobile-web teleop (virtual joystick) ---
        # Deliberately hardcoded to /cmd_vel (geometry_msgs/Twist) only - NOT a
        # generic "publish any message to any topic" bridge, to keep the
        # attack/mistake surface small on a channel that's reachable by
        # anyone on the local WiFi with no login.
        self.teleop_pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
        self.teleop_last_cmd_time = 0.0
        self.TELEOP_MAX_LINEAR = 0.1     # m/s - default and the forced limit while mapping is scanning
        # Operator-adjustable linear limit (phone -/+). Joystick teleop goes
        # straight to /cmd_vel - collision_monitor is NOT in this path - so
        # the cap stays low; ISO 3691-4 review before raising it.
        self.TELEOP_LINEAR_MIN = 0.05
        self.TELEOP_LINEAR_CAP = 0.30
        self.TELEOP_LINEAR_STEP = 0.05
        self.teleop_user_max_linear = self.TELEOP_MAX_LINEAR
        # Back to the default once nobody has driven for this long, so the
        # next operator to pick up a phone never inherits a raised limit.
        self.TELEOP_IDLE_RESET_S = 300.0
        self.teleop_last_active = time.time()  # last non-zero drive or -/+ change
        # Forced back to TELEOP_MAX_LINEAR while the AMR Control panel reports
        # mapping "scanning" (fast driving smears SLAM scan-matching). If that
        # state goes stale while it was scanning, keep the lock (fail safe).
        self.mapping_scanning = False
        self.mapping_state_time = 0.0
        self.sub_mapping_state = rospy.Subscriber(
            "/amr_mapping_state", String, self.on_mapping_state, qos=QoSProfile(
                depth=1, history=HistoryPolicy.KEEP_LAST,
                reliability=QoSReliabilityPolicy.RELIABLE))
        self.teleop_state_pub = rospy.Publisher("/rosboard_teleop_state", String, queue_size=1)
        threading.Thread(target=self.teleop_state_loop, daemon=True).start()
        self.TELEOP_MAX_ANGULAR = 0.15   # rad/s - kept low so fast in-place turns during Mapping don't hurt SLAM scan-matching
        self.TELEOP_DEADMAN_TIMEOUT = 0.5  # seconds - stop if no joystick update arrives in time
        # Linear accel ramp for the joystick (it bypasses velocity_smoother).
        # Only speeding up is ramped: slowing down / stop passes through at
        # once so releasing the knob still stops at the STM32 decel/brake.
        # 0.2 = velocity_smoother max_accel with the payload rack.
        self.TELEOP_MAX_ACCEL = 0.2      # m/s^2
        self.TELEOP_RAMP_DT_MAX = 0.15   # s - cap per step (first msg / gaps); joystick sends every 0.1 s
        self.teleop_out_linear = 0.0     # last published linear.x
        threading.Thread(target=self.teleop_watchdog_loop, daemon=True).start()

        # --- goal_picker page: navigation goal / path / cancel ---
        # Same "narrow, validated, dedicated topic" approach as teleop above
        # - NOT a generic publish bridge. These are one-shot commands (the
        # goal_picker page requires an explicit confirm tap before sending),
        # consumed by amr_navigation's nav_goal_executor_node, which is the
        # only thing that actually talks to the Nav2 action servers. rosboard
        # itself never calls into Nav2 - it just republishes a validated
        # PoseStamped/Path, same as it republishes a validated Twist for
        # teleop.
        self.nav_goal_pub = rospy.Publisher("/rosboard_nav_goal", PoseStamped, queue_size=1)
        # station goal: executor parks inside the box (pre-goal + precise leg)
        self.nav_station_pub = rospy.Publisher("/rosboard_nav_station", PoseStamped, queue_size=1)
        self.nav_path_pub = rospy.Publisher("/rosboard_nav_path", Path, queue_size=1)
        self.nav_cancel_pub = rospy.Publisher("/rosboard_nav_cancel", Empty, queue_size=1)
        # Mapping controls (start/pause scan, save, finish) from the phone.
        # Same narrow approach: only these four actions and a sanitized map
        # name are forwarded; the AMR Control panel is the only consumer and
        # owns the actual slam_toolbox pause and map_saver calls.
        self.mapping_cmd_pub = rospy.Publisher("/rosboard_mapping_cmd", String, queue_size=5)
        self.MAPPING_ACTIONS = ("start", "pause", "save", "finish")

        # rosbag recording from the phone: rosboard runs `ros2 bag record`
        # itself on the topics ticked on the goal_picker page. Topics must
        # be ones currently advertised (self.all_topics), so the browser
        # can't inject arguments. State goes out on /rosboard_bag_state.
        self.bag_dir = os.path.expanduser(rospy.get_param("~bag_dir", "~/AMR_Project/bags"))
        self.BAG_MAX_TOPICS = 40
        self.BAG_MAX_DURATION_S = 1800.0  # auto-stop so a forgotten recording can't fill the disk
        self.bag_proc = None
        self.bag_name = ""
        self.bag_topics = []
        self.bag_started = 0.0
        self.bag_error = ""
        # stop-distance analysis of the last bag (amr_bringup's
        # stop_distance_from_bag.py), run on request from the bag panel
        self.stop_proc_busy = False
        self.stop_result = ""
        self.stop_ok = None
        self.bag_lock = threading.Lock()
        self.bag_state_pub = rospy.Publisher("/rosboard_bag_state", String, queue_size=1)
        threading.Thread(target=self.bag_state_loop, daemon=True).start()

        self.NAV_COORD_LIMIT = 1000.0    # m - reject obviously-bogus coordinates; Nav2's own costmap bounds are the real gate
        self.NAV_PATH_MAX_POINTS = 200   # cap a drawn path's waypoint count

        # Published straight to /initialpose (not through a bridge/executor
        # topic like the goal/path above) because nav2_amcl already
        # subscribes to that exact topic natively - there's no Nav2 action
        # to go through, so no reason for nav_goal_executor_node to be
        # involved.
        self.initial_pose_pub = rospy.Publisher(
            "/initialpose", PoseWithCovarianceStamped, queue_size=1)

        rospy.loginfo("ROSboard listening on :%d" % self.port)

    def _nav_pose(self, x, y, theta):
        """Builds a validated PoseStamped in the map frame, or None if x/y/
        theta aren't finite numbers or fall outside NAV_COORD_LIMIT."""
        try:
            x, y, theta = float(x), float(y), float(theta)
        except (TypeError, ValueError):
            return None
        if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(theta)):
            return None
        if abs(x) > self.NAV_COORD_LIMIT or abs(y) > self.NAV_COORD_LIMIT:
            return None

        pose = PoseStamped()
        pose.header.frame_id = "map"
        pose.header.stamp = rospy.Time.now()
        pose.pose.position.x = x
        pose.pose.position.y = y
        pose.pose.orientation.z = math.sin(theta / 2.0)
        pose.pose.orientation.w = math.cos(theta / 2.0)
        return pose

    def on_nav_goal(self, x, y, theta, station=False):
        """Handle a single navigation goal tapped on the goal_picker page's
        map. Publishes a validated PoseStamped on /rosboard_nav_goal for
        nav_goal_executor_node to send to Nav2's navigate_to_pose action.
        station=True ("send to station") goes to /rosboard_nav_station
        instead, for the precise park-in-the-box approach."""
        pose = self._nav_pose(x, y, theta)
        if pose is None:
            print("error: nav_goal: bad coordinates: x=%r y=%r theta=%r" % (x, y, theta))
            return
        (self.nav_station_pub if station is True else self.nav_goal_pub).publish(pose)

    def on_nav_path(self, points):
        """Handle a drawn path from the goal_picker page: a list of
        {x, y, theta} waypoints in the map frame. Publishes a validated
        nav_msgs/Path on /rosboard_nav_path for nav_goal_executor_node to
        send to Nav2's navigate_through_poses action. Malformed individual
        points are skipped rather than rejecting the whole path."""
        if not isinstance(points, list) or not points:
            print("error: nav_path: empty or bad points list")
            return

        poses = []
        for point in points[:self.NAV_PATH_MAX_POINTS]:
            if not isinstance(point, dict):
                continue
            pose = self._nav_pose(
                point.get("x", 0.0), point.get("y", 0.0), point.get("theta", 0.0))
            if pose is not None:
                poses.append(pose)

        if not poses:
            print("error: nav_path: no valid points")
            return

        path = Path()
        path.header.frame_id = "map"
        path.header.stamp = rospy.Time.now()
        path.poses = poses
        self.nav_path_pub.publish(path)

    def on_nav_cancel(self):
        """Handle a cancel request from the goal_picker page - published as
        a signal for nav_goal_executor_node to cancel whatever Nav2 task
        (goal or path) is currently in progress."""
        self.nav_cancel_pub.publish(Empty())

    def on_mapping_cmd(self, action, name):
        """Forward a validated mapping command from the phone as JSON on
        /rosboard_mapping_cmd. Map names are limited to [A-Za-z0-9_-]{0,40},
        same character set the panel's own save dialog allows."""
        if action not in self.MAPPING_ACTIONS:
            print("error: mapping: bad action %r" % (action,))
            return
        name = re.sub(r"[^A-Za-z0-9_\-]", "", str(name or "").strip().replace(" ", "_"))[:40]
        self.mapping_cmd_pub.publish(String(data=json.dumps({"action": action, "name": name})))

    def on_bag_cmd(self, action, topics):
        """Start/stop `ros2 bag record` for the goal_picker page. Only
        currently-advertised topic names are accepted."""
        with self.bag_lock:
            if action == "start":
                if self.bag_proc is not None:
                    return
                if type(topics) is not list:
                    return
                known = getattr(self, "all_topics", {})
                topics = sorted(set(t for t in topics if type(t) is str and t in known
                                    and re.match(r"^/[A-Za-z0-9_/]+$", t)))[:self.BAG_MAX_TOPICS]
                if not topics:
                    self.bag_error = "no valid topics"
                    return
                os.makedirs(self.bag_dir, exist_ok=True)
                self.bag_name = "bag_" + time.strftime("%Y%m%d_%H%M%S")
                try:
                    self.bag_proc = subprocess.Popen(
                        ["ros2", "bag", "record", "-o", os.path.join(self.bag_dir, self.bag_name)] + topics,
                        stdout=subprocess.DEVNULL, stderr=subprocess.DEVNULL, start_new_session=True)
                except OSError as e:
                    self.bag_proc = None
                    self.bag_error = str(e)
                    return
                self.bag_topics = topics
                self.bag_started = time.time()
                self.bag_error = ""
                rospy.loginfo("bag record start: %s %s" % (self.bag_name, " ".join(topics)))
            elif action == "stop":
                self._bag_stop()
            elif action == "stop_dist":
                # last finished bag only; path is ours, never from the browser
                if self.bag_proc is not None or self.stop_proc_busy or not self.bag_name:
                    return
                self.stop_proc_busy = True
                self.stop_result = ""
                self.stop_ok = None
                threading.Thread(target=self._run_stop_dist,
                                 args=(os.path.join(self.bag_dir, self.bag_name),),
                                 daemon=True).start()
        self.publish_bag_state()

    def _run_stop_dist(self, path):
        try:
            r = subprocess.run(
                ["ros2", "run", "amr_bringup", "stop_distance_from_bag.py", path],
                stdout=subprocess.PIPE, stderr=subprocess.STDOUT, timeout=120, text=True)
            # drop rosbag2's [INFO] lines; keep the tail so the message stays small
            lines = [l for l in r.stdout.splitlines() if not l.startswith("[INFO]")]
            text, ok = "\n".join(lines)[-4000:], r.returncode == 0
        except (OSError, subprocess.TimeoutExpired) as e:
            text, ok = "analysis failed: %s" % e, False
        with self.bag_lock:
            self.stop_result, self.stop_ok, self.stop_proc_busy = text, ok, False
        self.publish_bag_state()

    def _bag_stop(self):
        # caller holds bag_lock. SIGINT so the recorder closes the bag cleanly.
        if self.bag_proc is None:
            return
        try:
            os.killpg(self.bag_proc.pid, signal.SIGINT)
            self.bag_proc.wait(timeout=10)
        except (ProcessLookupError, subprocess.TimeoutExpired):
            try:
                os.killpg(self.bag_proc.pid, signal.SIGKILL)
            except ProcessLookupError:
                pass
        rospy.loginfo("bag record stop: %s" % self.bag_name)
        self.bag_proc = None

    def publish_bag_state(self):
        rec = self.bag_proc is not None
        self.bag_state_pub.publish(String(data=json.dumps({
            "recording": rec,
            "name": self.bag_name,
            "topics": self.bag_topics if rec else [],
            "elapsed": round(time.time() - self.bag_started, 1) if rec else 0.0,
            "max_s": self.BAG_MAX_DURATION_S,
            "dir": self.bag_dir,
            "error": self.bag_error,
            "stop_busy": self.stop_proc_busy,
            "stop_result": self.stop_result,
            "stop_ok": self.stop_ok,
        })))

    def bag_state_loop(self):
        while True:
            time.sleep(1.0)
            try:
                with self.bag_lock:
                    if self.bag_proc is not None:
                        if self.bag_proc.poll() is not None:
                            self.bag_error = "recorder exited (code %s)" % self.bag_proc.returncode
                            self.bag_proc = None
                        elif time.time() - self.bag_started > self.BAG_MAX_DURATION_S:
                            self.bag_error = "auto-stopped after %d min" % (self.BAG_MAX_DURATION_S / 60)
                            self._bag_stop()
                self.publish_bag_state()
            except Exception:
                pass

    def on_initial_pose(self, x, y, theta):
        """Handle a "set initial pose" tap from the goal_picker page - where
        the operator indicates the robot's real current position/heading on
        the map, e.g. right after Nav2 starts (AMCL has no idea where the
        robot is until told) or if localization has drifted/gotten lost.
        Publishes directly to /initialpose with the same covariance RViz2's
        own "2D Pose Estimate" tool uses by default, since there's no
        principled way to guess a tighter one from a single tap."""
        pose = self._nav_pose(x, y, theta)
        if pose is None:
            print("error: initial_pose: bad coordinates: x=%r y=%r theta=%r" % (x, y, theta))
            return

        msg = PoseWithCovarianceStamped()
        msg.header = pose.header
        msg.pose.pose = pose.pose
        # RViz2 Displays > Tool Properties > 2D Pose Estimate defaults:
        # 0.25 m^2 variance in x/y (~0.5m std dev), ~0.0685 rad^2 in yaw
        # (~15 degree std dev). AMCL's particle filter uses this as its
        # initial spread to resample around, so it doesn't need to be exact.
        msg.pose.covariance[0] = 0.25   # x
        msg.pose.covariance[7] = 0.25   # y
        msg.pose.covariance[35] = 0.06853892326654787  # yaw
        self.initial_pose_pub.publish(msg)

    def on_teleop_cmd(self, linear, angular):
        """
        Handle a teleop command from a rosboard web client's on-screen
        joystick. Clamps to safe limits and publishes geometry_msgs/Twist
        on /cmd_vel. Values outside the clamp range (or non-finite) are
        capped/rejected rather than trusted as-is, since this comes
        straight from a browser on the network.
        """
        def clamp(value, limit):
            try:
                value = float(value)
            except (TypeError, ValueError):
                return 0.0
            if value != value:  # NaN
                return 0.0
            return max(-limit, min(limit, value))

        twist = Twist()
        twist.linear.x = self.teleop_ramp(clamp(linear, self.teleop_effective_max_linear()))
        twist.angular.z = clamp(angular, self.TELEOP_MAX_ANGULAR)
        if twist.linear.x != 0.0 or twist.angular.z != 0.0:
            self.teleop_last_active = time.time()
        self.teleop_pub.publish(twist)
        self.teleop_last_cmd_time = time.time()

    def teleop_ramp(self, target):
        out = self.teleop_out_linear
        if out * target < 0.0:
            out = 0.0  # direction change: drop to 0 first, then ramp up the other way
        if abs(target) > abs(out):
            last = self.teleop_last_cmd_time
            dt = min(time.time() - last, self.TELEOP_RAMP_DT_MAX) if last else self.TELEOP_RAMP_DT_MAX
            step = self.TELEOP_MAX_ACCEL * dt
            out = min(target, out + step) if target > 0.0 else max(target, out - step)
        else:
            out = target
        self.teleop_out_linear = out
        return out

    def on_mapping_state(self, msg):
        try:
            state = json.loads(msg.data)
        except ValueError:
            return
        self.mapping_scanning = bool(state.get("mode") == "mapping" and state.get("scanning"))
        self.mapping_state_time = time.time()

    def teleop_locked(self):
        # stale state keeps whatever it last said (scanning -> stays locked)
        return self.mapping_scanning

    def teleop_effective_max_linear(self):
        if self.teleop_locked():
            return min(self.teleop_user_max_linear, self.TELEOP_MAX_LINEAR)
        return self.teleop_user_max_linear

    def on_teleop_cfg(self, max_linear):
        """Phone -/+ for the joystick's linear limit, clamped to
        [TELEOP_LINEAR_MIN, TELEOP_LINEAR_CAP] on TELEOP_LINEAR_STEP steps.
        Applies to every client (one robot, one limit)."""
        try:
            v = float(max_linear)
        except (TypeError, ValueError):
            return
        if v != v:
            return
        v = round(v / self.TELEOP_LINEAR_STEP) * self.TELEOP_LINEAR_STEP
        self.teleop_user_max_linear = round(
            max(self.TELEOP_LINEAR_MIN, min(self.TELEOP_LINEAR_CAP, v)), 2)
        self.teleop_last_active = time.time()
        self.publish_teleop_state()

    def publish_teleop_state(self):
        self.teleop_state_pub.publish(String(data=json.dumps({
            "selected": self.teleop_user_max_linear,
            "effective": round(self.teleop_effective_max_linear(), 2),
            "locked": self.teleop_locked(),
            "min": self.TELEOP_LINEAR_MIN, "cap": self.TELEOP_LINEAR_CAP,
            "step": self.TELEOP_LINEAR_STEP, "max_angular": self.TELEOP_MAX_ANGULAR,
            "default": self.TELEOP_MAX_LINEAR,
            "idle_reset_in": (None if self.teleop_user_max_linear == self.TELEOP_MAX_LINEAR
                              else max(0, round(self.TELEOP_IDLE_RESET_S
                                                - (time.time() - self.teleop_last_active)))),
        })))

    def teleop_state_loop(self):
        while True:
            time.sleep(0.5)
            try:
                if (self.teleop_user_max_linear != self.TELEOP_MAX_LINEAR and
                        time.time() - self.teleop_last_active > self.TELEOP_IDLE_RESET_S):
                    rospy.loginfo("teleop limit reset to %.2f m/s (idle %.0f s)" % (
                        self.TELEOP_MAX_LINEAR, self.TELEOP_IDLE_RESET_S))
                    self.teleop_user_max_linear = self.TELEOP_MAX_LINEAR
                self.publish_teleop_state()
            except Exception:
                pass

    def teleop_watchdog_loop(self):
        """
        Deadman switch: if no joystick update has arrived recently (finger
        lifted but the 'stop' message got lost, browser tab closed, WiFi
        drop, etc), force-publish a zero Twist. Defense in depth alongside
        the STM32 firmware's own /cmd_vel receive-timeout.
        """
        while True:
            time.sleep(0.1)
            if self.teleop_last_cmd_time == 0.0:
                continue
            if time.time() - self.teleop_last_cmd_time > self.TELEOP_DEADMAN_TIMEOUT:
                self.teleop_pub.publish(Twist())
                self.teleop_out_linear = 0.0
                self.teleop_last_cmd_time = 0.0  # don't keep re-publishing zero every 100ms

    def start(self):
        try:
            rospy.spin()
        except KeyboardInterrupt:
            pass
        finally:
            # recorder runs in its own session; don't leave it orphaned
            with self.bag_lock:
                self._bag_stop()

    def get_msg_class(self, msg_type):
        """
        Given a ROS message type specified as a string, e.g.
            "std_msgs/Int32"
        or
            "std_msgs/msg/Int32"
        it imports the message class into Python and returns the class, i.e. the actual std_msgs.msg.Int32

        Returns none if the type is invalid (e.g. if user hasn't bash-sourced the message package).
        """
        try:
            msg_module, dummy, msg_class_name = msg_type.replace("/", ".").rpartition(".")
        except ValueError:
            rospy.logerr("invalid type %s" % msg_type)
            return None

        try:
            if not msg_module.endswith(".msg"):
                msg_module = msg_module + ".msg"
            return getattr(importlib.import_module(msg_module), msg_class_name)
        except Exception as e:
            rospy.logerr(str(e))
            return None

    if os.environ.get("ROS_VERSION") == "2":
        def get_topic_qos(self, topic_name: str) -> QoSProfile:
            """!
            Given a topic name, get the QoS profile with which it is being published
            @param topic_name (str) the topic name
            @return QosProfile the qos profile with which the topic is published. If no publishers exist
            for the given topic, it returns the sensor data QoS. returns None in case ROS1 is being used
            """
            if rospy.__name__ == "rospy2":
                # /map (nav2_map_server) and /tf_static are always published
                # exactly once, latched (RELIABLE + TRANSIENT_LOCAL by
                # design in Nav2/tf2) - hardcode that instead of trusting
                # get_publishers_info_by_topic() below for them. That call
                # can return a non-empty result whose qos_profile is still a
                # provisional BEST_EFFORT/VOLATILE placeholder for a brief
                # window right after the publisher's endpoint is first
                # discovered, before DDS has finished propagating its real
                # QoS announcement - hitting that window locks in a
                # subscription that can never receive the one-shot latched
                # publish, and the client is then stuck waiting forever
                # (observed as the goal_picker page never leaving "waiting
                # for /map data" after a fresh Navigation-mode launch).
                latched_topics = ("/map", "/tf_static")
                if topic_name in latched_topics:
                    return QoSProfile(
                            depth=10,
                            reliability=QoSReliabilityPolicy.RELIABLE,
                            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                        )

                topic_info = rospy._node.get_publishers_info_by_topic(topic_name=topic_name)
                if len(topic_info):
                    if topic_info[0].qos_profile.history == HistoryPolicy.UNKNOWN:
                        topic_info[0].qos_profile.history = HistoryPolicy.KEEP_LAST
                    # DDS doesn't propagate history depth across nodes, so a
                    # remote publisher's depth always reads back as 0. Copying
                    # that gave a RELIABLE, near-zero-depth reader, which on
                    # /tf (3 publishers, ~60 msg/s) intermittently stalled for
                    # 1-8s in DDS - the goal_picker "ตำแหน่งล่าสุด" badge
                    # jumping to several seconds while every other /tf
                    # subscriber (depth 100, like tf2's own listener) kept
                    # receiving normally.
                    if topic_info[0].qos_profile.depth < 10:
                        topic_info[0].qos_profile.depth = 100 if topic_name == "/tf" else 10
                    return topic_info[0].qos_profile
                else:
                    rospy.logwarn(f"No publishers available for topic {topic_name}. Returning sensor data QoS")
                    return QoSProfile(
                            depth=10,
                            reliability=QoSReliabilityPolicy.BEST_EFFORT,
                            # reliability=QoSReliabilityPolicy.RELIABLE,
                            durability=QoSDurabilityPolicy.VOLATILE,
                            # durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
                        )
            else:
                rospy.logwarn("QoS profiles are only used in ROS2")
                return None

    def pingpong_loop(self):
        """
        Loop to send pings to all active sockets every 5 seconds.
        """
        while True:
            time.sleep(5)

            if self.event_loop is None:
                continue
            try:
                self.event_loop.add_callback(ROSBoardSocketHandler.send_pings)
            except Exception as e:
                rospy.logwarn(str(e))
                traceback.print_exc()

    def sync_subs_loop(self):
        """
        Periodically calls self.sync_subs(). Intended to be run in a thread.
        """
        while True:
            time.sleep(1)
            self.sync_subs()

    def sync_subs(self):
        """
        Looks at self.remote_subs and makes sure local subscribers exist to match them.
        Also cleans up unused local subscribers for which there are no remote subs interested in them.
        """

        # Acquire lock since either sync_subs_loop or websocket may call this function (from different threads)
        self.lock.acquire()

        try:
            # all topics and their types as strings e.g. {"/foo": "std_msgs/String", "/bar": "std_msgs/Int32"}
            self.all_topics = {}

            for topic_tuple in rospy.get_published_topics():
                topic_name = topic_tuple[0]
                topic_type = topic_tuple[1]
                if type(topic_type) is list:
                    topic_type = topic_type[0] # ROS2
                self.all_topics[topic_name] = topic_type

            self.event_loop.add_callback(
                ROSBoardSocketHandler.broadcast,
                [ROSBoardSocketHandler.MSG_TOPICS, self.all_topics ]
            )

            for topic_name in self.remote_subs:
                if len(self.remote_subs[topic_name]) == 0:
                    continue

                # remote sub special (non-ros) topic: _dmesg
                # handle it separately here
                if topic_name == "_dmesg":
                    if topic_name not in self.local_subs:
                        rospy.loginfo("Subscribing to dmesg [non-ros]")
                        self.local_subs[topic_name] = DMesgSubscriber(self.on_dmesg)
                    continue

                if topic_name == "_system_stats":
                    if topic_name not in self.local_subs:
                        rospy.loginfo("Subscribing to _system_stats [non-ros]")
                        self.local_subs[topic_name] = SystemStatsSubscriber(self.on_system_stats)
                    continue

                if topic_name == "_top":
                    if topic_name not in self.local_subs:
                        rospy.loginfo("Subscribing to _top [non-ros]")
                        self.local_subs[topic_name] = ProcessesSubscriber(self.on_top)
                    continue

                # check if remote sub request is not actually a ROS topic before proceeding
                if topic_name not in self.all_topics:
                    rospy.logwarn("warning: topic %s not found" % topic_name)
                    continue

                # if the local subscriber doesn't exist for the remote sub, create it
                if topic_name not in self.local_subs:
                    topic_type = self.all_topics[topic_name]
                    msg_class = self.get_msg_class(topic_type)

                    if msg_class is None:
                        # invalid message type or custom message package not source-bashed
                        # put a dummy subscriber in to avoid returning to this again.
                        # user needs to re-run rosboard with the custom message files sourced.
                        self.local_subs[topic_name] = DummySubscriber()
                        self.event_loop.add_callback(
                            ROSBoardSocketHandler.broadcast,
                            [
                                ROSBoardSocketHandler.MSG_MSG,
                                {
                                    "_topic_name": topic_name, # special non-ros topics start with _
                                    "_topic_type": topic_type,
                                    "_error": "Could not load message type '%s'. Are the .msg files for it source-bashed?" % topic_type,
                                },
                            ]
                        )
                        continue

                    self.last_data_times_by_topic[topic_name] = 0.0

                    rospy.loginfo("Subscribing to %s" % topic_name)

                    kwargs = {}
                    if rospy.__name__ == "rospy2":
                        # In ros2 we also can pass QoS parameters to the subscriber.
                        # To avoid incompatibilities we subscribe using the same Qos
                        # of the topic's publishers
                        kwargs = {"qos": self.get_topic_qos(topic_name)}
                    self.local_subs[topic_name] = rospy.Subscriber(
                        topic_name,
                        self.get_msg_class(topic_type),
                        self.on_ros_msg,
                        callback_args = (topic_name, topic_type),
                        **kwargs
                    )

            # clean up local subscribers for which remote clients have lost interest
            for topic_name in list(self.local_subs.keys()):
                if topic_name not in self.remote_subs or \
                    len(self.remote_subs[topic_name]) == 0:
                        rospy.loginfo("Unsubscribing from %s" % topic_name)
                        self.local_subs[topic_name].unregister()
                        del(self.local_subs[topic_name])

        except Exception as e:
            rospy.logwarn(str(e))
            traceback.print_exc()

        self.lock.release()

    def on_system_stats(self, system_stats):
        """
        system stats received. send it off to the client as a "fake" ROS message (which could at some point be a real ROS message)
        """
        if self.event_loop is None:
            return

        msg_dict = {
            "_topic_name": "_system_stats", # special non-ros topics start with _
            "_topic_type": "rosboard_msgs/msg/SystemStats",
        }

        for key, value in system_stats.items():
            msg_dict[key] = value

        self.event_loop.add_callback(
            ROSBoardSocketHandler.broadcast,
            [
                ROSBoardSocketHandler.MSG_MSG,
                msg_dict
            ]
        )

    def on_top(self, processes):
        """
        processes list received. send it off to the client as a "fake" ROS message (which could at some point be a real ROS message)
        """
        if self.event_loop is None:
            return

        self.event_loop.add_callback(
            ROSBoardSocketHandler.broadcast,
            [
                ROSBoardSocketHandler.MSG_MSG,
                {
                    "_topic_name": "_top", # special non-ros topics start with _
                    "_topic_type": "rosboard_msgs/msg/ProcessList",
                    "processes": processes,
                },
            ]
        )

    def on_dmesg(self, text):
        """
        dmesg log received. make it look like a rcl_interfaces/msg/Log and send it off
        """
        if self.event_loop is None:
            return

        self.event_loop.add_callback(
            ROSBoardSocketHandler.broadcast,
            [
                ROSBoardSocketHandler.MSG_MSG,
                {
                    "_topic_name": "_dmesg", # special non-ros topics start with _
                    "_topic_type": "rcl_interfaces/msg/Log",
                    "msg": text,
                },
            ]
        )

    def on_ros_msg(self, msg, topic_info):
        """
        ROS messaged received (any topic or type).
        """
        topic_name, topic_type = topic_info
        t = time.time()
        latched = topic_name in LATCHED_TOPICS
        if topic_name == "/rosout":
            # Log lines are discrete events - the normal "latest value at N Hz"
            # throttle would silently drop most of a burst. Pass every line,
            # capped at ROSOUT_MAX_PER_S so a log storm can't flood the WiFi.
            if t - self.rosout_window_start >= 1.0:
                self.rosout_window_start, self.rosout_count = t, 0
            self.rosout_count += 1
            if self.rosout_count > self.ROSOUT_MAX_PER_S:
                return
        elif not latched and \
                t - self.last_data_times_by_topic.get(topic_name, 0) < self.update_intervals_by_topic[topic_name] - 1e-4:
            return

        if self.event_loop is None:
            return

        # convert ROS message into a dict and get it ready for serialization
        ros_msg_dict = ros2dict(msg)

        # add metadata
        ros_msg_dict["_topic_name"] = topic_name
        ros_msg_dict["_topic_type"] = topic_type
        ros_msg_dict["_time"] = time.time() * 1000

        # log last time we received data on this topic
        self.last_data_times_by_topic[topic_name] = t

        if latched:
            if topic_name == "/tf_static":
                # merge every publisher's static transforms, keyed by child
                prev = self.latched_msgs.get(topic_name)
                merged = {tr["child_frame_id"]: tr for tr in (prev or {}).get("transforms", [])}
                for tr in ros_msg_dict.get("transforms", []):
                    merged[tr["child_frame_id"]] = tr
                ros_msg_dict = dict(ros_msg_dict, transforms=list(merged.values()))
            self.latched_msgs[topic_name] = ros_msg_dict

        # broadcast it to the listeners that care
        self.event_loop.add_callback(
            ROSBoardSocketHandler.broadcast,
            [ROSBoardSocketHandler.MSG_MSG, ros_msg_dict]
        )

def main(args=None):
    ROSBoardNode().start()

if __name__ == '__main__':
    main()
