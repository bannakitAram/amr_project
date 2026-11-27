#!/usr/bin/env python3
"""Smart navigation speed: estimate the payload while driving, then switch the
velocity_smoother speed profile to the GA choice for that payload.

Triggered by the goal_picker page's "⚡" station button: rosboard publishes
{"objective": "auto"} on /rosboard_smart_nav right before the station goal. The
node then picks everything itself: weight from the currents, speed profile from
the GA table given by parameter `table` (default motor_ga = continuous GA optimum,
changes with payload). {"objective": .., "mode": ..} still selects a table
explicitly (for tests). Sequence per trip:

  1. ARMED/WEIGHING  velocity_smoother -> weighing profile (accel 0.15, vmax 0.20),
                     the profile the payload model was trained on (a15_15_v2).
  2. at t_eval s after motion start (weight_linear.yaml, 20 s):
       payload = intercept + c1*I_med + c2*Ib_cru (+ margin_kg)
         I_med  = median(|I_L|+|I_R|) while |v| > 0.03            (motor current)
         Ib_cru = median(battery current) while |a|<0.02, |v|>0.15, |w|<0.1
     lookup ga_speed_table.yaml (first row with payload_kg >= estimate + margin; last row if
     only the margin runs past the table) -> (accel, vmax); estimate above the trained range -> none
  3. APPLIED         velocity_smoother max_velocity[0] / max_accel[0] set to that row.
  4. on /nav_event arrived / failed / canceled / interrupted (or timeout, or node
     exit) the original velocity_smoother values are restored.

SAFETY
- Only LOWERS limits: every value is min(GA value, value velocity_smoother had when
  the trip started). The committed nav2_params.yaml (max_velocity 0.40, max_accel
  0.15) stays the upper bound. max_decel (stopping) is never touched.
- No estimate (not enough moving / cruise samples, model out of range) -> the trip
  stays on the weighing profile (0.20 m/s).
- collision_monitor, speed mask zones (/speed_limit) and slip limiting still act
  downstream / in the controller exactly as for a normal goal.
- Changes runtime speed limits: safety-related. Needs a tag + test before line use
  (see CLAUDE.md). The payload estimate is NOT an overload check.

Topics: in  /rosboard_smart_nav, /nav_event, /wheel/odom, /motor_current/left|right,
            /battery/state
        out /smart_nav/state (JSON, 2 Hz), /nav_event (smart_* events, for bags)
Params: model_dir (default ~/AMR_Project/ml/models), timeout_s (900),
        table (motor_ga | motor_tested | battery_tested | battery_ga, default motor_ga)
"""
import json
import os
import time

import numpy as np
import rclpy
import yaml
from nav_msgs.msg import Odometry
from rcl_interfaces.msg import Parameter, ParameterType, ParameterValue
from rcl_interfaces.srv import GetParameters, SetParameters
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.node import Node
from sensor_msgs.msg import BatteryState
from std_msgs.msg import Float32, String

SMOOTHER = '/velocity_smoother'
RATE_HZ = 20.0
MOVE_V = 0.03
EXTRA_WAIT_S = 20.0           # wait this long past t_eval for enough cruise samples
END_EVENTS = ('arrived', 'failed', 'canceled', 'interrupted')


class SmartSpeed(Node):
    def __init__(self):
        super().__init__('smart_speed')
        md = os.path.expanduser(self.declare_parameter('model_dir', '~/AMR_Project/ml/models').value)
        self.timeout_s = float(self.declare_parameter('timeout_s', 900.0).value)
        self.auto_table = self.declare_parameter('table', 'motor_ga').value
        self.wm = yaml.safe_load(open(os.path.join(md, 'weight_linear.yaml')))
        self.table = yaml.safe_load(open(os.path.join(md, 'ga_speed_table.yaml')))
        self.weigh = self.table['weighing_profile']
        self.t_eval = float(self.wm['t_eval_s'])
        # own group: the callbacks below wait on these clients, which must be served meanwhile
        cg = ReentrantCallbackGroup()
        self.get_cli = self.create_client(GetParameters, SMOOTHER + '/get_parameters', callback_group=cg)
        self.set_cli = self.create_client(SetParameters, SMOOTHER + '/set_parameters', callback_group=cg)
        self.state_pub = self.create_publisher(String, '/smart_nav/state', 10)
        self.ev_pub = self.create_publisher(String, '/nav_event', 10)
        self.create_subscription(String, '/rosboard_smart_nav', self.on_start, 10)
        self.create_subscription(String, '/nav_event', self.on_nav_event, 10)
        self.create_subscription(Odometry, '/wheel/odom', self.on_odom, 20)
        self.create_subscription(Float32, '/motor_current/left', lambda m: self._cur(0, m), 20)
        self.create_subscription(Float32, '/motor_current/right', lambda m: self._cur(1, m), 20)
        self.create_subscription(BatteryState, '/battery/state', self.on_bat, 20)
        self.v = self.w = 0.0
        self.i_lr = [0.0, 0.0]
        self.bat_i = 0.0
        self.have = set()
        self.reset('idle', 'รอคำสั่ง Smart')
        self.create_timer(1.0 / RATE_HZ, self.tick)
        self.create_timer(0.5, self.publish_state)
        self.get_logger().info('smart_speed: t_eval %.0f s, margin %.2f kg, tables %s'
                               % (self.t_eval, self.wm['margin_kg'], list(self.table['tables'])))

    # ---------- inputs ----------
    def on_odom(self, m):
        self.v, self.w = m.twist.twist.linear.x, m.twist.twist.angular.z
        self.have.add('odom')

    def _cur(self, i, m):
        self.i_lr[i] = m.data
        self.have.add('cur%d' % i)

    def on_bat(self, m):
        self.bat_i = m.current
        self.have.add('bat')

    # ---------- state ----------
    def reset(self, phase, msg):
        self.phase, self.msg = phase, msg
        self.objective = self.mode = None
        self.orig = None              # (max_velocity list, max_accel list) before the trip
        self.started = None           # wall time of the trigger
        self.t0 = None                # wall time of motion start
        self.samples = []             # (v, w, i_sum, bat_i)
        self.est = self.used = None
        self.applied = None

    def publish_state(self):
        el = (time.time() - self.t0) if self.t0 else 0.0
        d = dict(phase=self.phase, msg=self.msg, objective=self.objective, mode=self.mode,
                 t_eval=self.t_eval, elapsed=round(el, 1),
                 payload_est=None if self.est is None else round(self.est, 2),
                 payload_used=None if self.used is None else round(self.used, 2),
                 applied=self.applied)
        self.state_pub.publish(String(data=json.dumps(d)))

    def event(self, name, **kw):
        d = dict(event=name, time=time.strftime('%Y-%m-%d %H:%M:%S'), **kw)
        self.ev_pub.publish(String(data=json.dumps(d)))

    # ---------- velocity_smoother params ----------
    def _call(self, cli, req, timeout=2.0):
        if not cli.wait_for_service(timeout_sec=timeout):
            return None
        fut = cli.call_async(req)
        t = time.time()
        while not fut.done() and time.time() - t < timeout:
            time.sleep(0.02)
        return fut.result() if fut.done() else None

    def get_limits(self):
        r = self._call(self.get_cli, GetParameters.Request(names=['max_velocity', 'max_accel']))
        if r is None or len(r.values) != 2:
            return None
        return list(r.values[0].double_array_value), list(r.values[1].double_array_value)

    def set_limits(self, vmax, accel):
        """vmax/accel for x only; capped by the values at trip start (never raises)."""
        mv, ma = list(self.orig[0]), list(self.orig[1])
        mv[0] = min(float(vmax), self.orig[0][0])
        ma[0] = min(float(accel), self.orig[1][0])
        return self._set(mv, ma), mv[0], ma[0]

    def _set(self, mv, ma):
        def p(name, arr):
            return Parameter(name=name, value=ParameterValue(type=ParameterType.PARAMETER_DOUBLE_ARRAY,
                                                             double_array_value=[float(x) for x in arr]))
        r = self._call(self.set_cli, SetParameters.Request(parameters=[p('max_velocity', mv), p('max_accel', ma)]))
        return r is not None and all(x.successful for x in r.results)

    def restore(self, why):
        if self.orig is not None:
            ok = self._set(self.orig[0], self.orig[1])
            self.get_logger().info('smart_speed: restore velocity_smoother (%s) %s' % (why, 'ok' if ok else 'FAILED'))
            self.event('smart_restore', reason=why, ok=ok)
            if not ok:
                self.get_logger().error('smart_speed: could not restore velocity_smoother limits - check %s' % SMOOTHER)
        self.reset('idle', 'จบ: ' + why)

    # ---------- trigger / end ----------
    def on_start(self, m):
        try:
            d = json.loads(m.data)
        except ValueError:
            d = {}
        obj = d.get('objective', 'auto')
        mode = d.get('mode', 'tested')
        if obj == 'auto':
            obj, mode = self.auto_table.split('_', 1)
        if '%s_%s' % (obj, mode) not in self.table['tables']:
            self.get_logger().warn('smart_speed: bad request %r' % m.data)
            return
        if self.orig is not None:
            self.restore('new smart goal')
        lim = self.get_limits()
        if lim is None:
            self.phase, self.msg = 'error', 'ติดต่อ velocity_smoother ไม่ได้ - ใช้ profile ปกติ'
            self.get_logger().error('smart_speed: velocity_smoother parameters not reachable')
            return
        self.orig = lim
        self.objective, self.mode, self.started = obj, mode, time.time()
        ok, v, a = self.set_limits(self.weigh['vmax'], self.weigh['accel'])
        if not ok:
            self.get_logger().error('smart_speed: velocity_smoother refused the weighing profile')
            self.orig = None
            self.phase, self.msg = 'error', 'ตั้ง profile ชั่งน้ำหนักไม่ได้ - ใช้ profile ปกติ'
            return
        self.applied = dict(vmax=v, accel=a, profile='weighing')
        self.phase, self.msg = 'weighing', 'ชั่งน้ำหนัก (v %.2f, a %.2f)' % (v, a)
        self.event('smart_start', objective=obj, mode=mode, vmax=v, accel=a)
        self.get_logger().info('smart_speed: start (%s/%s), weighing profile v %.2f a %.2f' % (obj, mode, v, a))

    def on_nav_event(self, m):
        try:
            ev = json.loads(m.data).get('event')
        except (ValueError, AttributeError):
            return
        if ev in END_EVENTS and self.orig is not None:
            self.restore(ev)

    # ---------- weighing ----------
    def tick(self):
        if self.orig is None:
            return
        now = time.time()
        if now - self.started > self.timeout_s:
            self.restore('timeout')
            return
        if self.phase != 'weighing':
            return
        if not {'odom', 'cur0', 'cur1', 'bat'} <= self.have:
            self.msg = 'รอข้อมูล odom / กระแส / แบต'
            return
        if self.t0 is None:
            if abs(self.v) <= MOVE_V:
                return
            self.t0 = now
        self.samples.append((self.v, self.w, abs(self.i_lr[0]) + abs(self.i_lr[1]), self.bat_i))
        el = now - self.t0
        self.msg = 'ชั่งน้ำหนัก %.0f/%.0f s' % (el, self.t_eval)
        if el >= self.t_eval:
            self.estimate(final=el >= self.t_eval + EXTRA_WAIT_S)

    def features(self):
        s = np.array(self.samples)
        v, w, i_sum, bi = np.abs(s[:, 0]), np.abs(s[:, 1]), s[:, 2], s[:, 3]
        vs = np.convolve(v, np.ones(5) / 5, 'same')       # same smoothing as ml/extract_bags.py
        a = np.abs(np.gradient(vs, 1.0 / RATE_HZ))
        mov = v > MOVE_V
        cru = (a < 0.02) & (v > 0.15) & (w < 0.1)
        if mov.sum() < RATE_HZ or cru.sum() < RATE_HZ:
            return None
        return float(np.median(i_sum[mov])), float(np.median(bi[cru]))

    def estimate(self, final):
        f = self.features()
        if f is None:
            if final:
                self.phase, self.msg = 'no_estimate', 'ข้อมูล cruise ไม่พอ - คง profile ชั่งน้ำหนัก (ปลอดภัย)'
                self.event('smart_no_estimate')
            return
        i_med, ib_cru = f
        c = self.wm['coef']
        self.est = self.wm['intercept'] + c['I_med'] * i_med + c['Ib_cru'] * ib_cru
        self.used = self.est + float(self.wm['margin_kg'])
        rows = self.table['tables']['%s_%s' % (self.objective, self.mode)]
        lo, hi = self.wm['payload_range_kg']
        row = next((r for r in rows if r['payload_kg'] >= self.used), None)
        if row is None and self.est <= hi:
            row = rows[-1]   # estimate inside the trained range, only the margin runs past the table
        if row is None:
            self.phase = 'out_of_range'
            self.msg = 'น้ำหนัก %.1f kg เกินช่วงที่โมเดลรู้จัก (%.0f-%.0f) - คง profile ชั่งน้ำหนัก' % (self.est, lo, hi)
            self.event('smart_weight', payload_est=round(self.est, 2), applied=False)
            self.get_logger().warn('smart_speed: ' + self.msg)
            return
        ok, v, a = self.set_limits(row['vmax'], row['accel'])
        if not ok:
            self.phase, self.msg = 'error', 'velocity_smoother ไม่รับ profile GA - คง profile ชั่งน้ำหนัก'
            self.get_logger().error('smart_speed: set GA profile failed')
            return
        self.applied = dict(vmax=v, accel=a, profile='%s_%s' % (self.objective, self.mode))
        self.phase = 'applied'
        self.msg = 'น้ำหนัก %.1f kg (+%.1f) -> v %.2f a %.2f' % (self.est, self.wm['margin_kg'], v, a)
        self.event('smart_weight', payload_est=round(self.est, 2), payload_used=round(self.used, 2),
                   I_med=round(i_med, 4), Ib_cru=round(ib_cru, 4), vmax=v, accel=a, applied=True)
        self.get_logger().info('smart_speed: ' + self.msg)


def main():
    rclpy.init()
    node = SmartSpeed()
    from rclpy.executors import MultiThreadedExecutor
    ex = MultiThreadedExecutor(num_threads=3)   # service calls are waited on from callbacks
    ex.add_node(node)
    try:
        ex.spin()
    except KeyboardInterrupt:
        pass
    finally:
        if node.orig is not None:
            node.restore('node exit')
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
