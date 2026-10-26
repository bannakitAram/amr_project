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

The Mega only reports on change, so the bridge sends '?' every
`poll_period_s`; any line received counts as alive. Opening the port
toggles DTR, which resets the Mega (~2 s boot) - the first answers arrive
after that. The port is reopened every `reconnect_period_s` if it drops.

Port: /dev/hook from udev/99-amr-hook.rules (install steps there).

Monitoring/command layer only - NOT a safety function (ISO 3691-4).
"UP" means the top limit switch is closed, not that a cart is latched.
Treat "UNKNOWN" as not-latched: never assume the last state held.
"""
import threading

import rclpy
import rclpy.executors
from rclpy.node import Node
from rclpy.qos import QoSProfile, DurabilityPolicy
from std_msgs.msg import String
import serial

VALID_CMDS = {'U', 'D', 'S', 'R', '?'}


class HookBridge(Node):
    def __init__(self):
        super().__init__('hook_bridge')
        self.port = self.declare_parameter('port', '/dev/hook').value
        self.baud = self.declare_parameter('baud', 115200).value
        self.poll_period_s = self.declare_parameter('poll_period_s', 1.0).value
        # > poll period + Mega boot time after a DTR reset
        self.timeout_s = self.declare_parameter('timeout_s', 3.0).value
        self.reconnect_period_s = self.declare_parameter('reconnect_period_s', 2.0).value

        latched = QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL)
        self.state_pub = self.create_publisher(String, '/hook/state', latched)
        self.create_subscription(String, '/hook/cmd', self.on_cmd, 10)

        self.ser = None
        self.lock = threading.Lock()  # guards self.ser writes / close
        self.state = None
        self.last_rx = None
        self.publish_state('UNKNOWN')

        self.create_timer(self.poll_period_s, self.on_poll)
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
            elif line == 'HOOK READY':
                self.get_logger().info('Mega (re)booted')
            else:
                self.get_logger().warn('Mega: %s' % line)

    # ---- timers / callbacks ---------------------------------------------
    def on_poll(self):
        self.send('?')

    def on_watchdog(self):
        if self.ser is None or self.last_rx is None:
            self.publish_state('UNKNOWN')
        elif self.now_s() - self.last_rx > self.timeout_s:
            self.publish_state('UNKNOWN')

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
