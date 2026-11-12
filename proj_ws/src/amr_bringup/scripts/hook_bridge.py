#!/usr/bin/env python3
"""ROS 2 <-> serial bridge for the hook lift controller (Arduino Mega,
arduino/src/main.cpp). No micro-ROS: the Mega speaks a plain line protocol.

  Jetson -> Mega:  U / D / S / R / ?   (one char + newline)
  Mega -> Jetson:  STATE <IDLE|UP|DOWN|MOVING_UP|MOVING_DOWN|FAULT> [reason]
                   HOOK READY (after reset), ERR unknown command

Topics:
  /hook/state  std_msgs/String, transient_local (late subscribers get the
               last value). Data is the state word, e.g. "UP", or
               "FAULT TIMEOUT_UP". "UNKNOWN" when the port is not open or
               the Mega has not answered for `timeout_s`.
  /hook/cmd    std_msgs/String in: "U", "D", "S", "R" or "?" (case-insensitive)
  /motor_current/left, /motor_current/right
               std_msgs/Float32 out, A: drive wheel motor current from the
               Mega's ACS712 sensors ("CUR <left> <right>", 5 Hz). Monitor
               only - nothing acts on it. The right motor is mirrored, so
               its raw value is negative when driving forward; it is multiplied
               by `current_right_sign` (-1) so + = driving forward on both.
               Auto-zero: after /wheel/odom has shown the robot still for
               `zero_still_s`, the raw value is averaged (EMA, `zero_tau_s`)
               into a per-wheel offset that is subtracted. The offset is
               clamped to +/-`zero_max_a`; hitting the clamp is logged
               (sensor/wiring fault, or the driver really draws current at
               standstill - auto-zero would hide that up to the clamp).
  /battery/raw std_msgs/String out: raw reply line of the Junctek KG-F
               coulometer ("BAT <reply>", ~1 Hz, RS485 on the Mega's Serial1).
               Monitor only.
  /battery/state sensor_msgs/BatteryState out, decoded from ":r50=" replies:
               field 3 = voltage x0.01 V, field 4 = current x0.01 A,
               field 5 = remaining charge x0.001 Ah, field 12 = direction
               (1 = charging). current is always >= 0 (magnitude, easier to
               read in logs; not the ROS "negative = discharging" sign):
               the direction is in power_supply_status. percentage = charge / `battery_capacity_ah`
               (0..1). Layout from the Junctek KL110F protocol, voltage and
               current checked on the robot; charge scale/direction still to
               be checked against the meter display. Our own ":R50" command
               echoed back is ignored.
  /light/cmd   std_msgs/String in: tower light pattern from light_state_node.py
               (IDLE / AUTO / MANUAL / OBSTACLE / FAULT), forwarded as L<n>
               every `light_period_s`. Stops forwarding if /light/cmd goes
               quiet for `light_timeout_s`, so the Mega's own link watchdog
               turns the tower red-blink when light_state_node dies too.

The Mega only reports on change, so the bridge sends '?' every
`poll_period_s`; any line received counts as alive. Opening the port
toggles DTR, which resets the Mega (~2 s boot) - the first answers arrive
after that. The port is reopened every `reconnect_period_s` if it drops.

Port: /dev/hook from udev/99-amr-hook.rules (install steps there).

Monitoring/command layer only - NOT a safety function (ISO 3691-4).
"UP" means the top limit switch is closed, not that a cart is latched.
Treat "UNKNOWN" as not-latched: never assume the last state held.
"""
import math
import threading
import time

import rclpy
import rclpy.executors
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from nav_msgs.msg import Odometry
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Float32, String
import serial

VALID_CMDS = {'U', 'D', 'S', 'R', '?'}
# must match LightCmd in arduino/src/main.cpp
LIGHT_CODES = {'IDLE': 0, 'AUTO': 1, 'MANUAL': 2, 'OBSTACLE': 3, 'FAULT': 4}


class HookBridge(Node):
    def __init__(self):
        super().__init__('hook_bridge')
        self.port = self.declare_parameter('port', '/dev/ttyMega').value
        self.baud = self.declare_parameter('baud', 115200).value
        self.poll_period_s = self.declare_parameter('poll_period_s', 1.0).value
        # > poll period + Mega boot time after a DTR reset
        self.timeout_s = self.declare_parameter('timeout_s', 3.0).value
        self.reconnect_period_s = self.declare_parameter('reconnect_period_s', 2.0).value
        self.light_period_s = self.declare_parameter('light_period_s', 0.5).value
        # < the Mega's LIGHT_TIMEOUT_MS (2 s) minus one resend
        self.light_timeout_s = self.declare_parameter('light_timeout_s', 1.5).value

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.state_pub = self.create_publisher(String, '/hook/state', latched)
        self.create_subscription(String, '/hook/cmd', self.on_cmd, 10)
        self.current_right_sign = float(
            self.declare_parameter('current_right_sign', -1.0).value)
        self.zero_still_s = self.declare_parameter('zero_still_s', 2.0).value
        self.zero_tau_s = self.declare_parameter('zero_tau_s', 5.0).value
        self.zero_max_a = self.declare_parameter('zero_max_a', 0.5).value
        self.zero = [0.0, 0.0]          # learned raw offset, A (left, right)
        self.zero_learned = False
        self.zero_last = None           # time of the last offset update
        self.still_since = None         # monotonic time odom went still
        self.odom_rx = None
        self.create_subscription(Odometry, '/wheel/odom', self.on_odom, 10)
        self.cur_left_pub = self.create_publisher(Float32, '/motor_current/left', 10)
        self.cur_right_pub = self.create_publisher(Float32, '/motor_current/right', 10)
        self.bat_raw_pub = self.create_publisher(String, '/battery/raw', 10)
        self.bat_seen = False
        # 2 x 12 V 33 Ah in series = 24 V 33 Ah
        self.battery_capacity_ah = float(
            self.declare_parameter('battery_capacity_ah', 33.0).value)
        self.bat_pub = self.create_publisher(BatteryState, '/battery/state', 10)
        self.create_subscription(String, '/light/cmd', self.on_light, 10)
        self.light = None       # latest light code
        self.light_rx = None    # time of the latest /light/cmd

        self.ser = None
        self.lock = threading.Lock()  # guards self.ser writes / close
        self.state = None
        self.last_rx = None
        self.publish_state('UNKNOWN')

        self.create_timer(self.poll_period_s, self.on_poll)
        self.create_timer(self.light_period_s, self.on_light_timer)
        self.create_timer(0.2, self.on_watchdog)
        self.create_timer(self.reconnect_period_s, self.ensure_open)
        self.ensure_open()

        self.reader = threading.Thread(target=self.read_loop, daemon=True)
        self.reader.start()

    def now_s(self):
        return self.get_clock().now().nanoseconds * 1e-9

    def publish_state(self, state):
        if state == self.state:
            return
        if state.startswith('FAULT'):
            self.get_logger().error('hook %s' % state)
        elif state == 'UNKNOWN':
            self.get_logger().warn('hook state UNKNOWN (no answer on %s)' % self.port)
        else:
            self.get_logger().info('hook %s' % state)
        self.state = state
        self.state_pub.publish(String(data=state))

    # ---- serial ---------------------------------------------------------
    def ensure_open(self):
        if self.ser is not None:
            return
        try:
            ser = serial.Serial(self.port, self.baud, timeout=0.5)
        except (serial.SerialException, OSError) as e:
            self.get_logger().warn('cannot open %s: %s' % (self.port, e),
                                   throttle_duration_sec=30.0)
            return
        with self.lock:
            self.ser = ser
        self.last_rx = self.now_s()  # start the timeout from the open (Mega reboots)
        self.get_logger().info('opened %s @ %d' % (self.port, self.baud))

    def close(self, reason):
        with self.lock:
            if self.ser is None:
                return
            try:
                self.ser.close()
            except (serial.SerialException, OSError):
                pass
            self.ser = None
        self.get_logger().error('serial %s closed: %s' % (self.port, reason))
        self.publish_state('UNKNOWN')

    def send(self, cmd):
        with self.lock:
            if self.ser is None:
                return False
            try:
                self.ser.write((cmd + '\n').encode('ascii'))
                return True
            except (serial.SerialException, OSError) as e:
                err = e
        self.close('write failed: %s' % err)
        return False

    def read_loop(self):
        while rclpy.ok():
            ser = self.ser
            if ser is None:
                threading.Event().wait(0.2)
                continue
            try:
                raw = ser.readline()
            except (serial.SerialException, OSError, TypeError) as e:
                if ser is self.ser:
                    self.close('read failed: %s' % e)
                continue
            if not raw:
                continue
            line = raw.decode('ascii', errors='replace').strip()
            if not line:
                continue
            self.last_rx = self.now_s()
            if line.startswith('STATE '):
                self.publish_state(line[len('STATE '):].strip())
            elif line.startswith('CUR '):
                self.on_current(line)
            elif line.startswith('BAT '):
                if not self.bat_seen:
                    self.get_logger().info('first coulometer reply: %s' % line[4:])
                    self.bat_seen = True
                self.bat_raw_pub.publish(String(data=line[4:]))
                self.on_battery(line[4:])
            elif line == 'HOOK READY':
                self.get_logger().info('Mega (re)booted')
            else:
                self.get_logger().warn('Mega: %s' % line)

    def on_current(self, line):
        try:
            left, right = (float(v) for v in line.split()[1:3])
        except ValueError:
            self.get_logger().warn('bad CUR line "%s"' % line,
                                   throttle_duration_sec=10.0)
            return
        self.update_zero(left, right)
        left -= self.zero[0]
        right -= self.zero[1]
        self.cur_left_pub.publish(Float32(data=left))
        self.cur_right_pub.publish(Float32(data=right * self.current_right_sign))

    def on_battery(self, reply):
        if not reply.startswith(':r50='):
            return                      # own command echoed back, or noise
        try:
            f = [int(v) for v in reply[5:].strip(',').split(',')]
            voltage, current = f[2] * 0.01, f[3] * 0.01
            charge = f[4] * 0.001
            charging = f[11] == 1
        except (ValueError, IndexError):
            self.get_logger().warn('bad coulometer reply "%s"' % reply,
                                   throttle_duration_sec=10.0)
            return
        msg = BatteryState()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.voltage = voltage
        msg.current = abs(current)
        msg.temperature = float('nan')
        msg.charge = charge
        msg.capacity = self.battery_capacity_ah
        msg.design_capacity = self.battery_capacity_ah
        msg.percentage = (max(0.0, min(1.0, charge / self.battery_capacity_ah))
                          if self.battery_capacity_ah > 0 else float('nan'))
        msg.power_supply_status = (BatteryState.POWER_SUPPLY_STATUS_CHARGING if charging
                                   else BatteryState.POWER_SUPPLY_STATUS_DISCHARGING)
        msg.power_supply_health = BatteryState.POWER_SUPPLY_HEALTH_UNKNOWN
        msg.power_supply_technology = BatteryState.POWER_SUPPLY_TECHNOLOGY_UNKNOWN
        msg.present = True
        self.bat_pub.publish(msg)

    def on_odom(self, msg):
        now = time.monotonic()
        self.odom_rx = now
        t = msg.twist.twist
        if abs(t.linear.x) < 1e-3 and abs(t.angular.z) < 1e-3:
            if self.still_since is None:
                self.still_since = now
        else:
            self.still_since = None

    def update_zero(self, left, right):
        now = time.monotonic()
        still = (self.odom_rx is not None and now - self.odom_rx < 0.5
                 and self.still_since is not None
                 and now - self.still_since >= self.zero_still_s)
        if not still:
            self.zero_last = None
            return
        if not self.zero_learned:
            new = [left, right]                     # first still: take it as-is
        else:
            dt = now - self.zero_last if self.zero_last else 0.2
            a = 1.0 - math.exp(-dt / self.zero_tau_s)
            new = [z + a * (v - z) for z, v in zip(self.zero, (left, right))]
        for i, name in enumerate(('left', 'right')):
            if abs(new[i]) > self.zero_max_a:
                self.get_logger().warn(
                    'motor_current/%s zero %.2f A at standstill exceeds %.2f A: '
                    'check sensor wiring / driver idle current'
                    % (name, new[i], self.zero_max_a), throttle_duration_sec=30.0)
                new[i] = math.copysign(self.zero_max_a, new[i])
        if not self.zero_learned:
            self.get_logger().info('motor current zero learned: L %.3f A, R %.3f A' % tuple(new))
        self.zero = new
        self.zero_learned = True
        self.zero_last = now

    # ---- timers / callbacks ---------------------------------------------
    def on_poll(self):
        self.send('?')

    def on_watchdog(self):
        if self.ser is None or self.last_rx is None:
            self.publish_state('UNKNOWN')
        elif self.now_s() - self.last_rx > self.timeout_s:
            self.publish_state('UNKNOWN')

    def on_light(self, msg):
        code = LIGHT_CODES.get(msg.data.strip().upper())
        if code is None:
            self.get_logger().warn('ignored /light/cmd "%s"' % msg.data,
                                   throttle_duration_sec=10.0)
            return
        self.light = code
        self.light_rx = self.now_s()

    def on_light_timer(self):
        if self.light is None or self.now_s() - self.light_rx > self.light_timeout_s:
            return  # silence -> Mega shows link-lost (red blink) by itself
        self.send('L%d' % self.light)

    def on_cmd(self, msg):
        cmd = msg.data.strip().upper()
        if cmd not in VALID_CMDS:
            self.get_logger().warn('ignored /hook/cmd "%s" (use U/D/S/R/?)' % msg.data)
            return
        if not self.send(cmd):
            self.get_logger().error('/hook/cmd "%s" not sent: port closed' % cmd)


def main():
    rclpy.init()
    node = HookBridge()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, rclpy.executors.ExternalShutdownException):
        pass
    finally:
        # Context may already be shut down (SIGINT): no logging/publishing here.
        if node.ser is not None:
            try:
                node.ser.write(b'S\n')  # leave the hook stopped if it was moving
                node.ser.close()
            except (serial.SerialException, OSError):
                pass
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
