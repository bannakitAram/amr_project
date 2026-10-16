#!/usr/bin/env python3
"""Offline: wheel slip ratio per accel/decel event from a rosbag.

Compares the speed change the wheels report (/wheel/odom, encoder based)
with the speed change the body really had (/imu/data linear_acceleration.x
integrated over the same window). If the wheels gained (or lost) more speed
than the body, the difference is slip.

Also estimates the floor's effective traction a_limit (and mu_eff) from the
first slipping event, so no force-gauge / friction measurement is needed.

Record (straight line, full payload, repeat 0 -> vx_max -> 0 a few times
per smoother max_accel/max_decel setting, robot stationary ~2 s at start
so the IMU bias can be measured):
  ros2 bag record -o slip_a03 /wheel/odom /imu/data /cmd_vel

Run (no ROS nodes needed, just a sourced ROS 2 env for rosbag2_py):
  slip_ratio_from_bag.py slip_a03 [slip_a04 ...] [--csv out.csv]

Per event:
  dv_wheel  speed change from encoder odom over the event window
  dv_imu    integral of (imu ax - bias) over the same window
  slip      peak (v_wheel - v_body) / |dv_wheel| during the ramp, where
            v_body = start speed + integral of IMU ax ; >5% = slipping
            accel: wheels spun faster than body moved
            decel: wheels slowed faster than body (skid)
"""
import argparse
import csv
import sys

import numpy as np

try:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
except ImportError:
    sys.exit('source /opt/ros/humble/setup.bash first (needs rosbag2_py)')

G = 9.81
ODOM_TOPIC = '/wheel/odom'
IMU_TOPIC = '/imu/data'
STATIONARY_V = 0.005     # m/s - below this the robot is treated as stopped
EVENT_A = 0.08           # m/s^2 - |a_wheel| above this starts an event
MIN_EVENT_DV = 0.05      # m/s - ignore tiny speed corrections
SLIP_LIMIT = 0.05        # 5 %
SETTLE_S = 0.5           # s - look past the ramp end for the slip peak
_trapz = getattr(np, "trapezoid", None) or np.trapz  # numpy 2 renamed it


def read_bag(path):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=path, storage_id=''),
                rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    for need in (ODOM_TOPIC, IMU_TOPIC):
        if need not in types:
            sys.exit(f'{path}: topic {need} not in bag (has: {sorted(types)})')
    odom, imu = [], []
    odom_cls = get_message(types[ODOM_TOPIC])
    imu_cls = get_message(types[IMU_TOPIC])
    # Bag receive time for both streams: one clock, no MCU/Jetson clock skew.
    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        t = t_ns * 1e-9
        if topic == ODOM_TOPIC:
            m = deserialize_message(data, odom_cls)
            odom.append((t, m.twist.twist.linear.x))
        elif topic == IMU_TOPIC:
            m = deserialize_message(data, imu_cls)
            imu.append((t, m.linear_acceleration.x))
    if len(odom) < 20 or len(imu) < 20:
        sys.exit(f'{path}: too few messages (odom {len(odom)}, imu {len(imu)})')
    return np.array(odom), np.array(imu)


def smooth(x, n=3):
    if len(x) < n:
        return x
    return np.convolve(x, np.ones(n) / n, mode='same')


def find_events(t, v):
    """Contiguous windows where the wheel speed is ramping one way."""
    a = np.gradient(smooth(v), t)
    sign = np.where(a > EVENT_A, 1, np.where(a < -EVENT_A, -1, 0))
    events, i = [], 0
    while i < len(sign):
        if sign[i] == 0:
            i += 1
            continue
        j = i
        while j + 1 < len(sign) and sign[j + 1] == sign[i]:
            j += 1
        # pad one sample each side so the window covers the whole ramp
        s, e = max(i - 1, 0), min(j + 1, len(t) - 1)
        if abs(v[e] - v[s]) >= MIN_EVENT_DV:
            events.append((s, e))
        i = j + 1
    return events


def analyse(path):
    odom, imu = read_bag(path)
    to, vo = odom[:, 0], odom[:, 1]
    ti, ai = imu[:, 0], imu[:, 1]

    # IMU bias: mean ax while the wheels report standstill.
    v_at_imu = np.interp(ti, to, vo)
    still = np.abs(v_at_imu) < STATIONARY_V
    if still.sum() < 10:
        print(f'  WARNING {path}: <10 stationary IMU samples, bias = 0 '
              '(keep the robot still ~2 s at the start of each recording)')
        bias = 0.0
    else:
        bias = float(np.median(ai[still]))
    ac = ai - bias

    rows = []
    for s, e in find_events(to, vo):
        t0, t1 = to[s], to[e]
        dv_wheel = vo[e] - vo[s]
        m = (ti >= t0) & (ti <= t1)
        if m.sum() < 3:
            continue
        dv_imu = float(_trapz(ac[m], ti[m]))
        # Slip is transient: once the wheel speed stops ramping the body
        # catches up, so the end-to-end dv hides it. Integrate the body speed
        # from the event start (through SETTLE_S after it) and take the
        # largest wheel-ahead (accel) / wheel-behind (decel) gap.
        mw = (ti >= t0) & (ti <= t1 + SETTLE_S)
        tb = ti[mw]
        v_body = vo[s] + np.concatenate(
            ([0.0], np.cumsum(np.diff(tb) * 0.5 * (ac[mw][1:] + ac[mw][:-1]))))
        gap = (np.interp(tb, to, vo) - v_body) * np.sign(dv_wheel)
        kind = 'accel' if abs(vo[e]) > abs(vo[s]) else 'decel'
        rows.append({
            'bag': path,
            't': t0 - to[0],
            'kind': kind,
            'dur': t1 - t0,
            'v0': vo[s], 'v1': vo[e],
            'a_wheel': abs(dv_wheel) / (t1 - t0),
            'a_body': abs(dv_imu) / (t1 - t0),
            'a_imu_peak': float(np.max(np.abs(ac[m]))),
            'dv_wheel': dv_wheel,
            'dv_imu': dv_imu,
            'slip': float(np.max(gap)) / abs(dv_wheel),
        })
    return bias, rows


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument('bags', nargs='+')
    ap.add_argument('--csv', help='write per-event table to this file')
    ap.add_argument('--drive-load', type=float, default=None,
                    help='share of robot weight on the drive wheels (0-1); '
                         'if given, also prints mu_eff = a_limit/(g*share)')
    args = ap.parse_args()

    all_rows = []
    for b in args.bags:
        bias, rows = analyse(b)
        print(f'\n== {b}  (imu ax bias {bias:+.3f} m/s^2, {len(rows)} events)')
        print(f'  {"t[s]":>6} {"kind":5} {"v0->v1 [m/s]":>14} {"a_wheel":>8} '
              f'{"a_imu_pk":>8} {"dv_wheel":>8} {"dv_imu":>8} {"slip":>7}')
        for r in rows:
            flag = '  <-- SLIP' if abs(r['slip']) > SLIP_LIMIT else ''
            print(f'  {r["t"]:6.1f} {r["kind"]:5} {r["v0"]:+6.3f}->{r["v1"]:+6.3f} '
                  f'{r["a_wheel"]:8.3f} {r["a_imu_peak"]:8.3f} '
                  f'{r["dv_wheel"]:+8.3f} {r["dv_imu"]:+8.3f} '
                  f'{r["slip"] * 100:6.1f}%{flag}')
        all_rows += rows

    if not all_rows:
        sys.exit('\nno accel/decel events found - did the robot move?')

    print('\n== Summary (all bags)')
    for kind, keep in (('accel', 0.55), ('decel', 0.65)):
        ev = [r for r in all_rows if r['kind'] == kind]
        if not ev:
            print(f'  {kind}: no events')
            continue
        slipping = [r for r in ev if abs(r['slip']) > SLIP_LIMIT]
        clean = [r for r in ev if abs(r['slip']) <= SLIP_LIMIT]
        med = np.median([r['slip'] for r in ev]) * 100
        print(f'  {kind}: {len(ev)} events, median slip {med:.1f}%, '
              f'{len(slipping)} over {SLIP_LIMIT * 100:.0f}%')
        if slipping:
            a_lim = min(r['a_wheel'] for r in slipping)
            print(f'    a_limit (lowest a_wheel that slipped) = {a_lim:.2f} m/s^2')
            print(f'    suggested max_{kind} = {keep:.0%} x a_limit = '
                  f'{a_lim * keep:.2f} m/s^2')
            a_trac = np.median([r['a_body'] for r in slipping])
            print(f'    body accel while slipping (rough traction limit) ~ '
                  f'{a_trac:.2f} m/s^2')
            if args.drive_load:
                print(f'    mu_eff ~ {a_trac / (G * args.drive_load):.2f}')
        else:
            a_ok = max(r['a_wheel'] for r in clean)
            print(f'    no slip up to a_wheel = {a_ok:.2f} m/s^2 '
                  f'(a_limit is higher - test a larger setting)')
        # IMU noise check: dv_imu is unreliable if events are very short
        short = [r for r in ev if r['dur'] < 0.3]
        if short:
            print(f'    note: {len(short)} events < 0.3 s - IMU at ~50 Hz gives '
                  'few samples, treat their slip value as rough')

    if args.csv:
        with open(args.csv, 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=list(all_rows[0]))
            w.writeheader()
            w.writerows(all_rows)
        print(f'\nper-event table written to {args.csv}')


if __name__ == '__main__':
    main()
