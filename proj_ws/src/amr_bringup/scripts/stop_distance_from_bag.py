#!/usr/bin/env python3
"""Offline: stop distance per stop event from a rosbag.

A stop event = the robot is moving forward and the final /cmd_vel (after
collision_monitor) drops to ~0. From that moment (t_trig) the odometry speed
is integrated until the robot stands still: that is the stop distance,
including every latency after the command (ROS -> STM32 -> motors + braking).

Cause per event, from the cmd_vel chain
(cmd_vel_nav -> velocity_smoother -> cmd_vel_smoothed -> collision_monitor
-> cmd_vel):
  collision   cmd_vel_smoothed still asked to drive, cmd_vel was 0
              -> collision_monitor stopped it (the FrontStop test case)
  command     the request itself went to 0 (goal reached, cancel, teleop)

If /scan_filtered + /tf_static are in the bag, also prints the gap from the
front bumper to the nearest obstacle in the driving corridor at the trigger
and after the stop (tape-measure cross-check).

Record (Navigation running, goal behind a cardboard box, see test procedure):
  ros2 bag record -o stop_v030 /cmd_vel /cmd_vel_smoothed /cmd_vel_nav \\
      /odometry/filtered /wheel/odom /scan_filtered /tf_static

Run (no ROS nodes needed, just a sourced ROS 2 env for rosbag2_py):
  stop_distance_from_bag.py stop_v030 [more bags ...] [--csv out.csv]
  stop_distance_from_bag.py stop_v030 --front 0.60 --half-width 0.35

--front / --half-width / --zone default to the footprint front edge and the
FrontStop polygon in the installed nav2_params.yaml (so a payload footprint
set from the control panel is picked up). Pass/fail uses --zone (FrontStop
depth) and --limit (share of the zone the stop may use, 0.8 = same warning
the control panel shows). Also run from the goal picker's bag panel.
"""
import argparse
import csv
import math
import os
import re
import sys

import numpy as np

try:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
except ImportError:
    sys.exit('source /opt/ros/humble/setup.bash first (needs rosbag2_py)')

CMD_TOPIC = '/cmd_vel'
REQ_TOPIC = '/cmd_vel_smoothed'
ODOM_TOPICS = ('/odometry/filtered', '/wheel/odom')  # first found is primary
SCAN_TOPIC = '/scan_filtered'
BASE_FRAME = 'base_footprint'

MOVING_V = 0.05          # m/s - robot counts as driving above this
CMD_ZERO = 0.01          # m/s - |cmd_vel.x| below this = stop command
STATIONARY_V = 0.005     # m/s - below this the robot is treated as stopped
STILL_S = 0.3            # s it has to stay below STATIONARY_V
MAX_STOP_S = 5.0         # s - give up on an event that never settles
# Firmware forces odom to 0 below STOP_SPEED_THRESHOLD (0.035 m/s) when the
# command is stop; the last few mm are lost, negligible next to the margin.


def nav_geometry(path=None):
    """(front, half_width, zone) from nav2_params.yaml: footprint front edge,
    FrontStop half width, FrontStop depth ahead of the bumper. None if the
    file can't be read."""
    if path is None:
        try:
            from ament_index_python.packages import get_package_share_directory
            path = os.path.join(get_package_share_directory('amr_navigation'),
                                'config', 'nav2_params.yaml')
        except Exception:
            return None
    try:
        text = open(path).read()
        fp = re.search(r'^\s*footprint:\s*"\[\[([^"]*)\]\]"', text, re.M).group(1)
        front = max(float(p.split(',')[0]) for p in fp.split('], ['))
        pts = [float(v) for v in re.search(
            r'^\s*points:\s*\[([^\]]*)\]', text, re.M).group(1).split(',')]
    except (OSError, AttributeError, ValueError):
        return None
    xs, ys = pts[0::2], pts[1::2]
    return front, max(abs(y) for y in ys), max(xs) - front


def read_bag(path):
    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=path, storage_id=''),
                rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    if CMD_TOPIC not in types:
        sys.exit(f'{path}: {CMD_TOPIC} not in bag (has: {sorted(types)})')
    odom_topic = next((t for t in ODOM_TOPICS if t in types), None)
    if odom_topic is None:
        sys.exit(f'{path}: none of {ODOM_TOPICS} in bag')
    cls = {t: get_message(types[t]) for t in
           (CMD_TOPIC, REQ_TOPIC, odom_topic, SCAN_TOPIC, '/tf_static') if t in types}
    cmd, req, odom, scans, tfs = [], [], [], [], []
    # bag receive time for every stream: one clock, no MCU/Jetson skew
    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        if topic not in cls:
            continue
        t = t_ns * 1e-9
        m = deserialize_message(data, cls[topic])
        if topic == CMD_TOPIC:
            cmd.append((t, m.linear.x))
        elif topic == REQ_TOPIC:
            req.append((t, m.linear.x))
        elif topic == odom_topic:
            odom.append((t, m.twist.twist.linear.x))
        elif topic == SCAN_TOPIC:
            scans.append((t, m))
        else:
            tfs += list(m.transforms)
    if len(odom) < 20 or len(cmd) < 5:
        sys.exit(f'{path}: too few messages (odom {len(odom)}, cmd {len(cmd)})')
    return (odom_topic, np.array(cmd), np.array(req) if req else None,
            np.array(odom), scans, tfs)


def lidar_pose(tfs, frame):
    """(x, y, yaw) of `frame` in BASE_FRAME from /tf_static, or None."""
    parent = {t.child_frame_id: t for t in tfs}
    x, y, yaw, f = 0.0, 0.0, 0.0, frame
    while f != BASE_FRAME:
        if f not in parent:
            return None
        tr = parent[f].transform
        q = tr.rotation
        pyaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y ** 2 + q.z ** 2))
        # pose of f in parent: p_parent = T + R(pyaw) p_f
        x, y = (tr.translation.x + math.cos(pyaw) * x - math.sin(pyaw) * y,
                tr.translation.y + math.sin(pyaw) * x + math.cos(pyaw) * y)
        yaw += pyaw
        f = parent[f].header.frame_id
    return x, y, yaw


def corridor_gap(scan, pose, front, half_width):
    """Distance from the front bumper to the nearest scan point ahead of it
    inside |y| <= half_width (base frame), or None if the corridor is clear."""
    r = np.asarray(scan.ranges)
    a = scan.angle_min + np.arange(len(r)) * scan.angle_increment
    ok = np.isfinite(r) & (r > scan.range_min) & (r < scan.range_max)
    lx, ly, lyaw = pose
    x = lx + r[ok] * np.cos(a[ok] + lyaw)
    y = ly + r[ok] * np.sin(a[ok] + lyaw)
    sel = (x > front) & (np.abs(y) <= half_width)
    return float(x[sel].min() - front) if sel.any() else None


def scan_at(scans, t):
    if not scans:
        return None
    ts = np.array([s[0] for s in scans])
    i = int(np.searchsorted(ts, t))
    i = min(max(i, 0), len(scans) - 1)
    return scans[i][1] if abs(ts[i] - t) < 0.5 else None


def find_stops(cmd, odom):
    """Indices into cmd of each moving -> stop-command transition."""
    to, vo = odom[:, 0], odom[:, 1]
    events, last_end = [], -1.0
    for k in range(1, len(cmd)):
        t, c = cmd[k]
        if abs(c) >= CMD_ZERO or cmd[k - 1, 1] < MOVING_V or t < last_end:
            continue
        v = np.interp(t, to, vo)
        if v < MOVING_V:
            continue
        events.append(k)
        last_end = t + 1.0  # one event per stop, skip repeated zero cmds
    return events


def analyse(path, front, half_width):
    odom_topic, cmd, req, odom, scans, tfs = read_bag(path)
    to, vo = odom[:, 0], odom[:, 1]
    pose = None
    if scans:
        pose = lidar_pose(tfs, scans[0][1].header.frame_id)
    rows = []
    for k in find_stops(cmd, odom):
        t0 = cmd[k, 0]
        v0 = float(np.interp(t0, to, vo))
        # integrate odom from t_trig until STILL_S below STATIONARY_V
        i0 = int(np.searchsorted(to, t0))
        dist, t_stop, still_since = 0.0, None, None
        for i in range(max(i0, 1), len(to)):
            ta, tb = max(to[i - 1], t0), to[i]
            if tb > ta:
                dist += 0.5 * (vo[i - 1] + vo[i]) * (tb - ta)
            if abs(vo[i]) < STATIONARY_V:
                still_since = still_since if still_since is not None else to[i]
                if to[i] - still_since >= STILL_S:
                    t_stop = still_since
                    break
            else:
                still_since = None
            if to[i] - t0 > MAX_STOP_S:
                break
        if t_stop is None:
            continue  # cmd went 0 but the robot kept going / bag ended
        cause = 'command'
        if req is not None:
            # request just before the trigger (collision_monitor's input)
            j = int(np.searchsorted(req[:, 0], t0)) - 1
            if 0 <= j < len(req) and req[j, 1] >= MOVING_V and t0 - req[j, 0] < 0.5:
                cause = 'collision'
        row = {'bag': path, 't': round(t0 - to[0], 2), 'cause': cause,
               'v0': round(v0, 3), 'dist': round(dist, 3),
               'time': round(t_stop - t0, 2),
               'decel': round(v0 / (t_stop - t0), 2) if t_stop > t0 else None,
               'gap_trig': None, 'gap_stop': None}
        if pose is not None:
            s0, s1 = scan_at(scans, t0), scan_at(scans, t_stop + STILL_S)
            if s0 is not None:
                row['gap_trig'] = corridor_gap(s0, pose, front, half_width)
            if s1 is not None:
                row['gap_stop'] = corridor_gap(s1, pose, front, half_width)
        rows.append(row)
    return odom_topic, pose, rows


def fmt(x, f='{:.3f}'):
    return '  -  ' if x is None else f.format(x)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument('bags', nargs='+')
    ap.add_argument('--params', help='nav2_params.yaml (default: installed amr_navigation)')
    ap.add_argument('--front', type=float,
                    help='centre -> front edge of robot/payload [m] (default: footprint)')
    ap.add_argument('--half-width', type=float,
                    help='corridor half width for the scan gap [m] (default: FrontStop)')
    ap.add_argument('--zone', type=float,
                    help='FrontStop depth ahead of the bumper [m] (default: FrontStop)')
    ap.add_argument('--limit', type=float, default=0.8,
                    help='max share of --zone a stop may use (default 0.8)')
    ap.add_argument('--csv', help='write per-event table to this file')
    args = ap.parse_args()
    geo = nav_geometry(args.params) or (0.49, 0.29, 0.35)
    src = 'nav2_params.yaml' if nav_geometry(args.params) else 'built-in defaults'
    args.front = args.front if args.front is not None else geo[0]
    args.half_width = args.half_width if args.half_width is not None else geo[1]
    args.zone = args.zone if args.zone is not None else geo[2]
    print(f'geometry ({src} unless overridden): front {args.front:.2f} m, '
          f'corridor +-{args.half_width:.2f} m, FrontStop {args.zone:.2f} m')

    all_rows = []
    for b in args.bags:
        odom_topic, pose, rows = analyse(b, args.front, args.half_width)
        lid = (f'lidar ({pose[0]:+.3f}, {pose[1]:+.3f}, yaw {math.degrees(pose[2]):+.0f} deg)'
               if pose else 'no /scan_filtered + /tf_static: gap columns empty')
        print(f'\n== {b}  (speed from {odom_topic}; {lid}; {len(rows)} stops)')
        print(f'  {"t[s]":>6} {"cause":9} {"v0[m/s]":>8} {"dist[m]":>8} '
              f'{"time[s]":>7} {"decel":>6} {"gap@trig":>8} {"gap@stop":>8}')
        for r in rows:
            print(f'  {r["t"]:6.1f} {r["cause"]:9} {r["v0"]:8.3f} {r["dist"]:8.3f} '
                  f'{r["time"]:7.2f} {fmt(r["decel"], "{:6.2f}")} '
                  f'{fmt(r["gap_trig"]):>8} {fmt(r["gap_stop"]):>8}')
        all_rows += rows

    if not all_rows:
        sys.exit('\nno stop events found - did the robot drive and get stopped?')

    print('\n== Summary (all bags)')
    budget = args.zone * args.limit
    ok = True
    for cause in ('collision', 'command'):
        ev = [r for r in all_rows if r['cause'] == cause]
        if not ev:
            print(f'  {cause}: no events')
            continue
        worst = max(ev, key=lambda r: r['dist'])
        print(f'  {cause}: {len(ev)} stops, v0 max {max(r["v0"] for r in ev):.3f} m/s, '
              f'stop distance median {np.median([r["dist"] for r in ev]):.3f} / '
              f'WORST {worst["dist"]:.3f} m (t={worst["t"]:.1f} s, {worst["bag"]})')
        gaps = [r['gap_stop'] for r in ev if r['gap_stop'] is not None]
        if gaps:
            print(f'    smallest gap bumper -> obstacle after stop: {min(gaps):.3f} m')
        if cause == 'collision':
            if worst['dist'] > budget:
                ok = False
                print(f'    FAIL: {worst["dist"]:.3f} m > {args.limit:.0%} of FrontStop '
                      f'{args.zone:.2f} m ({budget:.3f} m) - lower v_max, raise decel '
                      'or deepen FrontStop, then re-test')
            else:
                print(f'    PASS: worst {worst["dist"]:.3f} m <= {budget:.3f} m '
                      f'({args.limit:.0%} of FrontStop {args.zone:.2f} m)')
    if not any(r['cause'] == 'collision' for r in all_rows):
        ok = False
        print('  no collision_monitor stops - FrontStop was not exercised '
              '(record /cmd_vel_smoothed, drive at an obstacle)')
    print('  Cross-check with the tape measure; worst case counts, not the median.')

    if args.csv:
        with open(args.csv, 'w', newline='') as f:
            w = csv.DictWriter(f, fieldnames=list(all_rows[0]))
            w.writeheader()
            w.writerows(all_rows)
        print(f'  per-stop table written to {args.csv}')
    sys.exit(0 if ok else 1)


if __name__ == '__main__':
    main()
