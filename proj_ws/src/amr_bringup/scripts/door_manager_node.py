#!/usr/bin/env python3
"""Doors on the navigation map: set from the phone, relay tick when passing.

Doors are stored per map in <maps>/<map>_doors.yaml (same folder as the
stations file; the map name and folder come from the control panel's
/amr_stations, so doors exist only while Navigation runs on a map):

  doors:
    - {name: A, x: 3.20, y: -1.50, radius: 1.0, relay: 1, pulse_s: 0.5}

In:
  /rosboard_door_cmd std_msgs/String JSON from rosboard (goal_picker), checked
               again here:
                 {"action": "add", "name": "A", "relay": 1}  door at the
                     robot's current pose (TF map -> base_footprint)
                 {"action": "delete", "name": "A"}
  /amr_stations std_msgs/String JSON {map, routes_file, ...} from the panel
  /cmd_vel_nav  geometry_msgs/Twist: Nav2/executor output, used to tell Auto
               driving (recent non-zero) from Manual
Out:
  /amr_doors   std_msgs/String JSON {map, doors: [...]} at 1 Hz
  /relay/cmd   std_msgs/String "<n> on" then "<n> off" after pulse_s

Trigger: when the robot enters a door's radius while driving in Auto, the
door's relay is pulsed once; it re-arms after the robot has left the zone
(radius + 0.3 m hysteresis). The robot does NOT stop or wait for the door.

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
from rclpy.node import Node
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
        self.default_radius = float(self.declare_parameter('radius', 1.0).value)
        self.default_pulse_s = float(self.declare_parameter('pulse_s', 0.5).value)
        self.robot_frame = self.declare_parameter('robot_frame', 'base_footprint').value

        self.map_name = ''
        self.maps_dir = ''
        self.doors = []
        self.inside = set()          # door names the robot is in (armed off)
        self.pending_off = []        # (monotonic time, relay)
        self.nav_moving_t = 0.0

        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)
        self.doors_pub = self.create_publisher(String, '/amr_doors', 10)
        self.relay_pub = self.create_publisher(String, '/relay/cmd', 10)
        self.create_subscription(String, '/amr_stations', self.on_stations, 10)
        self.create_subscription(String, '/rosboard_door_cmd', self.on_cmd, 10)
        self.create_subscription(Twist, '/cmd_vel_nav', self.on_nav_cmd, 10)
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
                    'pulse_s': float(d.get('pulse_s', self.default_pulse_s))})
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
                               'pulse_s': self.default_pulse_s})
            # being at the door when it is set must not tick it at once
            self.inside.add(name)
            self.get_logger().info('door %s set at (%.2f, %.2f) relay %d'
                                   % (name, xy[0], xy[1], relay))
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

    # ---- outputs --------------------------------------------------------
    def publish_doors(self):
        self.doors_pub.publish(String(data=json.dumps(
            {'map': self.map_name, 'doors': self.doors}, ensure_ascii=False)))

    def tick(self):
        now = time.monotonic()
        for t, relay in [p for p in self.pending_off if p[0] <= now]:
            self.relay_pub.publish(String(data='%d off' % relay))
        self.pending_off = [p for p in self.pending_off if p[0] > now]
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
