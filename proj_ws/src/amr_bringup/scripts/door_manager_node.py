#!/usr/bin/env python3
"""Doors on the navigation map: set from the phone, relay tick when passing.

Doors are stored per map in <maps>/<map>_doors.yaml (same folder as the
stations file; the map name and folder come from the control panel's
/amr_stations, so doors exist only while Navigation runs on a map):

  doors:
    - {name: A, x: 3.20, y: -1.50, radius: 1.5, relay: 1, pulse_s: 0.5, wait_s: 5.0}

In:
  /rosboard_door_cmd std_msgs/String JSON from rosboard (goal_picker), checked
               again here:
                 {"action": "add", "name": "A", "relay": 1}  door at the
                     robot's current pose (TF map -> base_footprint)
                 {"action": "add", "name": "A", "relay": 1, "x": 3.2, "y": -1.5}
                     door pinned on the map (map frame, m); must lie
                     inside /map
                 {"action": "delete", "name": "A"}
                 {"action": "pulse", "relay": 1}  pulse that relay now
                     (Remote 1/2/3 buttons on the phone's Mapping bar;
                     works in any mode, no map needed)
  /amr_stations std_msgs/String JSON {map, routes_file, ...} from the panel
  /cmd_vel_nav  geometry_msgs/Twist: Nav2/executor output, used to tell Auto
               driving (recent non-zero) from Manual
Out:
  /amr_doors   std_msgs/String JSON {map, doors: [...]} at 1 Hz
  /relay/cmd   std_msgs/String "<n> on" then "<n> off" after pulse_s
  /door_hold   std_msgs/String JSON {"hold": bool, "door": name, "left_s": s}
               at 10 Hz; hold = true for wait_s after a door was triggered.
               nav_goal_executor_node.py stops the running task while it is
               true and re-sends it when it goes false (or goes stale).

Trigger: when the robot enters a door's radius while driving in Auto, the
door's relay is pulsed once and the robot is held still for the door's
wait_s (0 = drive on) so the door has time to open; it re-arms after the
robot has left the zone (radius + 0.3 m hysteresis). The wait is a fixed
time: nothing checks that the door opened - a door still closed after the
wait is an obstacle for collision_monitor (the executor then waits for it).

Convenience function only - NOT a safety function (ISO 3691-4). A door that
does not open is an obstacle for collision_monitor; nothing here checks that
the door actually opened.
"""
import json
import math
import os
import re
import time

import rclpy
import rclpy.executors
import tf2_ros
import yaml
from geometry_msgs.msg import Twist
from nav_msgs.msg import OccupancyGrid
from rclpy.node import Node
from rclpy.qos import DurabilityPolicy, QoSProfile
from rclpy.time import Time
from std_msgs.msg import String

# letters (Thai vowel/tone marks are not \w), digits, space, _ and -
NAME_RE = re.compile(r'^[^\x00-\x1f\x7f"\'\\/<>:;{}\[\]]{1,24}$')
MAX_DOORS = 30
HYSTERESIS_M = 0.3
AUTO_HOLD_S = 1.5        # /cmd_vel_nav non-zero within this = Auto driving


class DoorManager(Node):
    def __init__(self):
        super().__init__('door_manager')
        self.default_radius = float(self.declare_parameter('radius', 1.5).value)
        self.default_pulse_s = float(self.declare_parameter('pulse_s', 0.5).value)
        self.default_wait_s = float(self.declare_parameter('wait_s', 5.0).value)
        self.robot_frame = self.declare_parameter('robot_frame', 'base_footprint').value

        self.map_name = ''
        self.maps_dir = ''
        self.doors = []
        self.inside = set()          # door names the robot is in (armed off)
        self.pending_off = []        # (monotonic time, relay)
        self.nav_moving_t = 0.0
        self.hold_until = 0.0        # monotonic end of the current door wait
        self.hold_door = ''
        self.map_info = None         # /map MapMetaData, to check pinned doors

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.doors_pub = self.create_publisher(String, '/amr_doors', 10)
        self.relay_pub = self.create_publisher(String, '/relay/cmd', 10)
        self.hold_pub = self.create_publisher(String, '/door_hold', 10)
        self.create_subscription(String, '/amr_stations', self.on_stations, 10)
        self.create_subscription(String, '/rosboard_door_cmd', self.on_cmd, 10)
        self.create_subscription(Twist, '/cmd_vel_nav', self.on_nav_cmd, 10)
        self.create_subscription(
            OccupancyGrid, '/map', self.on_map,
            QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL))
        self.create_timer(0.1, self.tick)
        self.create_timer(1.0, self.publish_doors)

    # ---- storage --------------------------------------------------------
    def doors_path(self):
        return os.path.join(self.maps_dir, self.map_name + '_doors.yaml')

    def load(self):
        self.doors = []
        try:
            with open(self.doors_path()) as f:
                data = yaml.safe_load(f) or {}
            for d in data.get('doors', [])[:MAX_DOORS]:
                self.doors.append({
                    'name': str(d['name']), 'x': float(d['x']), 'y': float(d['y']),
                    'radius': float(d.get('radius', self.default_radius)),
                    'relay': int(d['relay']),
                    'pulse_s': float(d.get('pulse_s', self.default_pulse_s)),
                    'wait_s': min(60.0, max(0.0, float(d.get('wait_s', self.default_wait_s))))})
        except FileNotFoundError:
            pass
        except (OSError, KeyError, TypeError, ValueError, yaml.YAMLError) as e:
            self.get_logger().error('bad %s: %s' % (self.doors_path(), e))
        self.doors = [d for d in self.doors if d['relay'] in (1, 2, 3)]
        self.inside.clear()
        self.get_logger().info('map %s: %d door(s)' % (self.map_name, len(self.doors)))

    def save(self):
        tmp = self.doors_path() + '.tmp'
        with open(tmp, 'w') as f:
            yaml.safe_dump({'doors': self.doors}, f, allow_unicode=True, sort_keys=False)
        os.replace(tmp, self.doors_path())

    # ---- inputs ---------------------------------------------------------
    def on_stations(self, msg):
        try:
            d = json.loads(msg.data)
            name = str(d.get('map') or '')
            routes_file = str(d.get('routes_file') or '')
        except (ValueError, AttributeError):
            return
        maps_dir = os.path.dirname(routes_file) if routes_file else ''
        if not re.match(r'^[A-Za-z0-9_\-]{0,64}$', name):
            return
        if (name, maps_dir) != (self.map_name, self.maps_dir):
            self.map_name, self.maps_dir = name, maps_dir
            if name and maps_dir:
                self.load()
            else:
                self.doors = []
                self.inside.clear()

    def on_map(self, msg):
        self.map_info = msg.info

    def pin_xy(self, c):
        """Pinned (x, y) from the command if it lies on the current map,
        else None. Map assumed unrotated (map_server origin yaw 0)."""
        try:
            x, y = float(c['x']), float(c['y'])
        except (KeyError, TypeError, ValueError):
            return None
        i = self.map_info
        if i is None or not (math.isfinite(x) and math.isfinite(y)):
            return None
        x0, y0 = i.origin.position.x, i.origin.position.y
        if not (x0 <= x <= x0 + i.width * i.resolution
                and y0 <= y <= y0 + i.height * i.resolution):
            return None
        return x, y

    def on_nav_cmd(self, msg):
        if abs(msg.linear.x) > 1e-3 or abs(msg.angular.z) > 1e-3:
            self.nav_moving_t = time.monotonic()

    def robot_xy(self):
        try:
            t = self.tf_buffer.lookup_transform('map', self.robot_frame, Time())
        except tf2_ros.TransformException:
            return None
        return t.transform.translation.x, t.transform.translation.y

    def on_cmd(self, msg):
        try:
            c = json.loads(msg.data)
            action = c.get('action')
        except (ValueError, AttributeError):
            return
        if action == 'pulse':
            self.manual_pulse(c.get('relay'))
            return
        if not (self.map_name and self.maps_dir):
            self.get_logger().warn('door cmd ignored: Navigation not running on a map')
            return
        name = str(c.get('name', '')).strip()
        if not NAME_RE.match(name):
            self.get_logger().warn('door cmd ignored: bad name %r' % name)
            return
        if action == 'add':
            relay = c.get('relay')
            if relay not in (1, 2, 3):
                self.get_logger().warn('door cmd ignored: relay must be 1-3')
                return
            pinned = 'x' in c or 'y' in c
            if pinned:
                xy = self.pin_xy(c)
                if xy is None:
                    self.get_logger().warn('door cmd ignored: pin %r, %r not on /map'
                                           % (c.get('x'), c.get('y')))
                    return
            else:
                xy = self.robot_xy()
                if xy is None:
                    self.get_logger().warn('door cmd ignored: no map -> %s TF' % self.robot_frame)
                    return
            self.doors = [d for d in self.doors if d['name'] != name]
            if len(self.doors) >= MAX_DOORS:
                self.get_logger().warn('door cmd ignored: %d doors max' % MAX_DOORS)
                return
            self.doors.append({'name': name, 'x': round(xy[0], 3), 'y': round(xy[1], 3),
                               'radius': self.default_radius, 'relay': relay,
                               'pulse_s': self.default_pulse_s,
                               'wait_s': self.default_wait_s})
            # being at the door when it is set must not tick it at once
            rxy = xy if not pinned else self.robot_xy()
            if rxy is not None and math.hypot(rxy[0] - xy[0], rxy[1] - xy[1]) <= self.default_radius:
                self.inside.add(name)
            else:
                self.inside.discard(name)
            self.get_logger().info('door %s %s at (%.2f, %.2f) relay %d'
                                   % (name, 'pinned' if pinned else 'set', xy[0], xy[1], relay))
        elif action == 'delete':
            self.doors = [d for d in self.doors if d['name'] != name]
            self.inside.discard(name)
            self.get_logger().info('door %s deleted' % name)
        else:
            return
        try:
            self.save()
        except OSError as e:
            self.get_logger().error('save %s failed: %s' % (self.doors_path(), e))
        self.publish_doors()

    def manual_pulse(self, relay):
        """Remote 1/2/3 button on the phone (Mapping bar): pulse one relay now,
        in any mode, no map or door needed. Ignored while that relay's
        previous pulse is still on, so a double tap gives one pulse."""
        if relay not in (1, 2, 3):
            self.get_logger().warn('pulse ignored: relay must be 1-3')
            return
        if any(r == relay for _, r in self.pending_off):
            return
        self.get_logger().info('remote button: pulse relay %d' % relay)
        self.relay_pub.publish(String(data='%d on' % relay))
        self.pending_off.append((time.monotonic() + self.default_pulse_s, relay))

    # ---- outputs --------------------------------------------------------
    def publish_doors(self):
        self.doors_pub.publish(String(data=json.dumps(
            {'map': self.map_name, 'doors': self.doors}, ensure_ascii=False)))
        # short zone only once the robot has had time to slow down
        short = short and self.slow_since is not None and now - self.slow_since >= SLOW_SETTLE_S
        self.set_short(short, now, force=now - self.short_sent_t >= RESEND_S)
        if not slow and not self.short_on and self.slow_since is not None:
            self.slow_since = None
            self.get_logger().info('sensor door: speed limit removed')
            self.publish_door_speed()

    def tick(self):
        now = time.monotonic()
        for t, relay in [p for p in self.pending_off if p[0] <= now]:
            self.relay_pub.publish(String(data='%d off' % relay))
        self.pending_off = [p for p in self.pending_off if p[0] > now]
        left = self.hold_until - now
        self.hold_pub.publish(String(data=json.dumps(
            {'hold': left > 0, 'door': self.hold_door, 'left_s': round(max(0.0, left), 1)},
            ensure_ascii=False)))
        if not self.doors:
            return
        xy = self.robot_xy()
        if xy is None:
            return
        auto = now - self.nav_moving_t < AUTO_HOLD_S
        for d in self.doors:
            dist = math.hypot(xy[0] - d['x'], xy[1] - d['y'])
            if d['name'] in self.inside:
                if dist > d['radius'] + HYSTERESIS_M:
                    self.inside.discard(d['name'])
            elif dist <= d['radius']:
                self.inside.add(d['name'])
                if auto:
                    self.get_logger().info('door %s: pulse relay %d' % (d['name'], d['relay']))
                    self.relay_pub.publish(String(data='%d on' % d['relay']))
                    self.pending_off.append((now + d['pulse_s'], d['relay']))
                    if d['wait_s'] > 0:
                        self.hold_until = now + d['wait_s']
                        self.hold_door = d['name']
                        self.get_logger().info('door %s: hold %.1f s' % (d['name'], d['wait_s']))


def main():
    rclpy.init()
    node = DoorManager()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
