#!/usr/bin/env python3
"""Offline: which sensor put points into the FrontStop polygon (false stops).

For every /scan_filtered and /depth_camera/points message the points are
transformed to base_footprint, filtered the way collision_monitor does
(depth: min_height..max_height), and counted inside the stop polygon.
collision_monitor stops when MORE than max_points are inside (Humble), so
per source the script reports when it alone exceeded max_points, the
positions of those points and - for depth - a z histogram, which separates:
  floor returns (robot pitch / camera tilt)  z just above min_height, spread
  speckle / flying pixels                    few points, scattered x/y/z
  real wall / object                         compact cluster, also in scan

Also prints /rosout lines that can stop the robot without collision_monitor
(controller/planner failures, progress checker, depth/imu watchdog cancels,
behavior_server recoveries, bt_navigator aborts), so one run covers every
stop cause - not only FrontStop trips.

cmd_vel chain (/cmd_vel_smoothed = what Nav2 asks, /cmd_vel = what
collision_monitor lets through): finds every interval where Nav2 asked to
move but the output was zero (same test as nav_goal_executor's
"มีสิ่งกีดขวาง" alert: >0.02 m/s or >0.05 rad/s in, ~0 out; >= 2 s = alert)
or throttled below half (FootprintApproach slowing). Each interval is tagged
rotate / forward / reverse from the request - a rotate-in-place start in a
narrow aisle points at FootprintApproach (the long rear swinging into the
walls), forward with FrontStop trips points at the sensor that tripped.

Record (in the narrow spot, from standstill, start a goal):
  ros2 bag record -o fs_narrow /scan_filtered /depth_camera/points \\
      /collision_monitor_state /tf /tf_static /odometry/filtered /rosout \\
      /cmd_vel_smoothed /cmd_vel

Run (sourced ROS 2 env, no nodes needed):
  front_stop_source_from_bag.py fs_narrow
  front_stop_source_from_bag.py fs_narrow --polygon FrontStopShort --csv out.csv

Polygon, max_points and depth heights are read from the installed
nav2_params.yaml (override with --params).
"""
import argparse
import csv
import math
import os
import sys

import numpy as np
import yaml

try:
    import rosbag2_py
    from rclpy.serialization import deserialize_message
    from rosidl_runtime_py.utilities import get_message
    from sensor_msgs_py import point_cloud2
except ImportError:
    sys.exit('source /opt/ros/humble/setup.bash first (needs rosbag2_py)')

BASE = 'base_footprint'
SCAN_TOPIC = '/scan_filtered'
DEPTH_TOPIC = '/depth_camera/points'
STATE_TOPIC = '/collision_monitor_state'
ODOM_TOPICS = ('/odometry/filtered', '/wheel/odom')
ROSOUT_TOPIC = '/rosout'
REQ_TOPIC = '/cmd_vel_smoothed'
OUT_TOPIC = '/cmd_vel'
CHAIN_FRESH_S = 0.5   # nav_goal_executor: older output counts as silent
BLOCK_S = 2.0         # nav_goal_executor: blocked this long -> alert
MIN_GAP_S = 0.3       # report held/throttled intervals at least this long
# Nodes whose WARN+ lines can explain a stop; plus keyword hits at any level.
ROSOUT_NODES = ('controller_server', 'planner_server', 'bt_navigator',
                'behavior_server', 'collision_monitor', 'velocity_smoother',
                'depth_watchdog', 'imu_watchdog', 'door_manager',
                'zone_manager', 'slip_speed_limiter')
ROSOUT_KEYWORDS = ('fail', 'abort', 'cancel', 'collision', 'progress',
                   'no valid', 'timeout', 'lost', 'stop', 'keepout',
                   'out of bounds', 'occupied', 'recovery', 'unable')
LEVEL_NAME = {10: 'DEBUG', 20: 'INFO', 30: 'WARN', 40: 'ERROR', 50: 'FATAL'}


def rosout_relevant(log):
    name = log.name.split('.')[-1]
    text = log.msg.lower()
    node_hit = any(n in name for n in ROSOUT_NODES)
    kw_hit = any(k in text for k in ROSOUT_KEYWORDS)
    return (node_hit and log.level >= 30) or (kw_hit and log.level >= 20
                                              and (node_hit or log.level >= 30))


def default_params():
    try:
        from ament_index_python.packages import get_package_share_directory
        return os.path.join(get_package_share_directory('amr_navigation'),
                            'config', 'nav2_params.yaml')
    except Exception:
        here = os.path.dirname(os.path.realpath(__file__))
        return os.path.join(here, '..', '..', 'amr_navigation', 'config',
                            'nav2_params.yaml')


def load_cm(path, polygon):
    with open(path) as f:
        p = yaml.safe_load(f)['collision_monitor']['ros__parameters']
    poly = p[polygon]
    pts = np.array(poly['points'], dtype=float).reshape(-1, 2)
    d = p.get('depth', {})
    return pts, int(poly.get('max_points', 3)), \
        float(d.get('min_height', 0.05)), float(d.get('max_height', 0.5))


def quat_to_mat(q):
    x, y, z, w = q.x, q.y, q.z, q.w
    return np.array([
        [1 - 2 * (y * y + z * z), 2 * (x * y - z * w), 2 * (x * z + y * w)],
        [2 * (x * y + z * w), 1 - 2 * (x * x + z * z), 2 * (y * z - x * w)],
        [2 * (x * z - y * w), 2 * (y * z + x * w), 1 - 2 * (x * x + y * y)]])


class Tf:
    """Static/latest TF tree; enough for sensor->base (all fixed mounts)."""

    def __init__(self):
        self.parent = {}  # child -> (parent, 4x4)

    def add(self, msg):
        for t in msg.transforms:
            m = np.eye(4)
            m[:3, :3] = quat_to_mat(t.transform.rotation)
            tr = t.transform.translation
            m[:3, 3] = [tr.x, tr.y, tr.z]
            self.parent[t.child_frame_id.lstrip('/')] = (
                t.header.frame_id.lstrip('/'), m)

    def to_base(self, frame):
        frame = frame.lstrip('/')
        m = np.eye(4)
        seen = set()
        while frame != BASE:
            if frame not in self.parent or frame in seen:
                return None
            seen.add(frame)
            par, t = self.parent[frame]
            m = t @ m
            frame = par
        return m


def in_polygon(xy, poly):
    """Even-odd ray casting, vectorised. xy: (N,2)."""
    x, y = xy[:, 0], xy[:, 1]
    inside = np.zeros(len(xy), dtype=bool)
    n = len(poly)
    for i in range(n):
        x1, y1 = poly[i]
        x2, y2 = poly[(i + 1) % n]
        cond = (y1 > y) != (y2 > y)
        with np.errstate(divide='ignore', invalid='ignore'):
            xc = (x2 - x1) * (y - y1) / (y2 - y1) + x1
        inside ^= cond & (x < xc)
    return inside


def scan_points(msg):
    r = np.asarray(msg.ranges, dtype=float)
    a = msg.angle_min + np.arange(len(r)) * msg.angle_increment
    ok = np.isfinite(r) & (r >= msg.range_min) & (r <= msg.range_max)
    r, a = r[ok], a[ok]
    return np.stack([r * np.cos(a), r * np.sin(a), np.zeros_like(r)], 1)


def cloud_points(msg):
    pts = point_cloud2.read_points_numpy(
        msg, field_names=('x', 'y', 'z'), skip_nans=True)
    return np.asarray(pts, dtype=float).reshape(-1, 3)


def transform(pts, m):
    return pts @ m[:3, :3].T + m[:3, 3]


def report_chain(chain, states, scan_trip_t, depth_trip_t, logs):
    """Intervals where Nav2 asked to move but collision_monitor held it."""
    if not chain:
        print(f'cmd_vel chain: {REQ_TOPIC} not recorded')
        return
    # group consecutive non-ok/idle samples of the same family
    groups = []
    for rel, v, w, kind in chain:
        bad = kind in ('held', 'silent', 'throttled')
        if bad and groups and groups[-1]['open'] and rel - groups[-1]['t1'] < 0.5:
            g = groups[-1]
        elif bad:
            g = {'t0': rel, 't1': rel, 'v': [], 'w': [], 'kinds': [], 'open': True}
            groups.append(g)
        else:
            if groups:
                groups[-1]['open'] = False
            continue
        g['t1'] = rel
        g['v'].append(v)
        g['w'].append(w)
        g['kinds'].append(kind)
    groups = [g for g in groups if g['t1'] - g['t0'] >= MIN_GAP_S]

    print(f'\ncmd_vel chain: {len(groups)} interval(s) >= {MIN_GAP_S}s where '
          f'Nav2 asked to move and the output was held/throttled')
    for g in groups:
        dur = g['t1'] - g['t0']
        v = float(np.median(g['v']))
        w = float(np.median(g['w']))
        if abs(v) < 0.03 and abs(w) > 0.05:
            motion = 'ROTATE'
        elif v < 0:
            motion = 'REVERSE'
        else:
            motion = 'FORWARD'
        kinds = {k: g['kinds'].count(k) for k in set(g['kinds'])}
        main_kind = max(kinds, key=kinds.get)
        lo, hi = g['t0'] - 0.3, g['t1'] + 0.05
        # polygon active at the start of the interval + any that fired in it
        before = [x for x in states if x[0] <= g['t0']]
        cand = ([before[-1]] if before else []) + \
            [x for x in states if g['t0'] < x[0] <= hi]
        polys = sorted({p for _, a, p in cand if a != 0 and p})
        src = [n for n, lst in (('scan', scan_trip_t), ('depth', depth_trip_t))
               if any(lo <= x <= hi for x in lst)]
        nlogs = [l for l in logs if lo - 1.0 <= l[0] <= hi + 1.0]
        alert = '  ** executor alert "มีสิ่งกีดขวาง" **' \
            if main_kind in ('held', 'silent') and dur >= BLOCK_S else ''
        print(f'  [{g["t0"]:7.2f}-{g["t1"]:7.2f}s] {dur:4.1f}s {main_kind:9s} '
              f'{motion:7s} req v={v:+.2f} w={w:+.2f}  '
              f'polygon={",".join(polys) or "-"}  trips={",".join(src) or "-"}'
              f'{alert}')
        if main_kind == 'silent':
            hint = 'collision_monitor not publishing /cmd_vel (source timeout / node down)'
        elif 'FootprintApproach' in polys or (motion == 'ROTATE' and not src):
            hint = ('FootprintApproach: footprint sweep hits walls/obstacles - '
                    'rotating in a narrow aisle (rear 0.87 m)')
        elif src:
            hint = f'FrontStop from {"+".join(src)} - see TRIP lines / z histogram'
        elif polys:
            hint = f'{",".join(polys)}'
        else:
            hint = ('no FrontStop trip from scan/depth -> FootprintApproach '
                    '(Humble has no /collision_monitor_state to confirm); '
                    'if /tf_static missing, trips cannot be computed')
        print(f'      -> {hint}')
        for l in nlogs[:3]:
            print(f'         log {l[0]:7.2f}s {l[1]} {l[2]}: {l[3][:110]}')

    tot = sum(1 for c in chain if c[3] != 'idle')
    if tot:
        cnt = {k: sum(1 for c in chain if c[3] == k)
               for k in ('ok', 'throttled', 'held', 'silent')}
        print('  moving-request samples: ' + ', '.join(
            f'{k} {100 * n / tot:.0f}%' for k, n in cnt.items()))


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawTextHelpFormatter)
    ap.add_argument('bag')
    ap.add_argument('--params', default=default_params())
    ap.add_argument('--polygon', default='FrontStop')
    ap.add_argument('--csv', help='per-message counts to CSV')
    ap.add_argument('--max-events', type=int, default=20,
                    help='detailed dumps to print per source')
    args = ap.parse_args()

    poly, max_pts, dmin, dmax = load_cm(args.params, args.polygon)
    print(f'{args.polygon}: {poly.tolist()}  stop if > {max_pts} pts; '
          f'depth z {dmin}..{dmax} m')

    reader = rosbag2_py.SequentialReader()
    reader.open(rosbag2_py.StorageOptions(uri=args.bag, storage_id=''),
                rosbag2_py.ConverterOptions('', ''))
    types = {t.name: t.type for t in reader.get_all_topics_and_types()}
    for need in (SCAN_TOPIC, DEPTH_TOPIC, '/tf_static', ROSOUT_TOPIC,
                 REQ_TOPIC, OUT_TOPIC):
        if need not in types:
            print(f'WARN: {need} not in bag')
    msg_cls = {n: get_message(t) for n, t in types.items()}

    tf = Tf()
    t0 = None
    rows = []
    speed = 0.0
    state = ''
    dumps = {'scan': 0, 'depth': 0}
    z_all = []
    depth_trip_t = []
    scan_trip_t = []
    state_stop_t = []
    logs = []  # (rel, level, node, msg)
    states = []  # (rel, action_type, polygon)
    out = {'t': None, 'v': 0.0, 'w': 0.0}
    chain = []  # per request msg: (rel, v_req, w_req, kind) kind=ok/held/throttled
    last_log = {}

    while reader.has_next():
        topic, data, t_ns = reader.read_next()
        if topic not in msg_cls:
            continue
        t = t_ns * 1e-9
        t0 = t if t0 is None else t0
        rel = t - t0
        if topic in ('/tf_static', '/tf'):
            tf.add(deserialize_message(data, msg_cls[topic]))
            continue
        if topic == OUT_TOPIC:
            c = deserialize_message(data, msg_cls[topic])
            out.update(t=rel, v=c.linear.x, w=c.angular.z)
            continue
        if topic == REQ_TOPIC:
            c = deserialize_message(data, msg_cls[topic])
            v, w = c.linear.x, c.angular.z
            wants = abs(v) > 0.02 or abs(w) > 0.05
            fresh = out['t'] is not None and rel - out['t'] < CHAIN_FRESH_S
            ov, ow = (out['v'], out['w']) if fresh else (0.0, 0.0)
            if not wants:
                kind = 'idle'
            elif abs(ov) <= 0.001 and abs(ow) <= 0.001:
                kind = 'held' if fresh else 'silent'
            elif (abs(v) > 0.02 and abs(ov) < 0.5 * abs(v)) or \
                    (abs(v) <= 0.02 and abs(ow) < 0.5 * abs(w)):
                kind = 'throttled'
            else:
                kind = 'ok'
            chain.append((rel, v, w, kind))
            continue
        if topic == ROSOUT_TOPIC:
            log = deserialize_message(data, msg_cls[topic])
            if rosout_relevant(log):
                key = (log.name, log.msg)
                # collapse repeats (controllers log the same line at 20 Hz)
                if rel - last_log.get(key, -1e9) > 2.0:
                    lv = LEVEL_NAME.get(log.level, str(log.level))
                    print(f'[{rel:7.2f}s] rosout {lv:5s} {log.name}: '
                          f'{log.msg[:160]}')
                    logs.append((rel, lv, log.name, log.msg))
                last_log[key] = rel
            continue
        if topic in ODOM_TOPICS:
            o = deserialize_message(data, msg_cls[topic])
            speed = o.twist.twist.linear.x
            continue
        if topic == STATE_TOPIC:
            s = deserialize_message(data, msg_cls[topic])
            new = f'{s.action_type}:{s.polygon_name}'
            if new != state:
                states.append((rel, s.action_type, s.polygon_name))
                print(f'[{rel:7.2f}s] collision_monitor_state -> '
                      f'action_type={s.action_type} polygon={s.polygon_name}')
                if s.action_type == 1:  # STOP
                    state_stop_t.append(rel)
                state = new
            continue
        if topic not in (SCAN_TOPIC, DEPTH_TOPIC):
            continue

        msg = deserialize_message(data, msg_cls[topic])
        m = tf.to_base(msg.header.frame_id)
        if m is None:
            continue
        if topic == SCAN_TOPIC:
            src = 'scan'
            pts = transform(scan_points(msg), m)
        else:
            src = 'depth'
            pts = transform(cloud_points(msg), m)
            pts = pts[(pts[:, 2] >= dmin) & (pts[:, 2] <= dmax)]
        inside = pts[in_polygon(pts[:, :2], poly)] if len(pts) else pts
        n = len(inside)
        rows.append((round(rel, 3), src, n, round(speed, 3)))
        trip = n > max_pts
        if trip:
            (depth_trip_t if src == 'depth' else scan_trip_t).append(rel)
            if src == 'depth':
                z_all.extend(inside[:, 2].tolist())
            if dumps[src] < args.max_events:
                dumps[src] += 1
                print(f'[{rel:7.2f}s] {src:5s} TRIP {n:5d} pts  v={speed:+.2f}'
                      f'  x {inside[:,0].min():.2f}..{inside[:,0].max():.2f}'
                      f'  y {inside[:,1].min():+.2f}..{inside[:,1].max():+.2f}'
                      f'  z {inside[:,2].min():.2f}..{inside[:,2].max():.2f}'
                      f'  z_med {np.median(inside[:,2]):.2f}')

    if t0 is None:
        sys.exit('bag empty / no usable topics')

    print('\n=== summary ===')
    for src, lst in (('scan', scan_trip_t), ('depth', depth_trip_t)):
        tot = sum(1 for r in rows if r[1] == src)
        print(f'{src:5s}: {len(lst)}/{tot} msgs over max_points'
              + (f'  (first {lst[0]:.2f}s, last {lst[-1]:.2f}s)' if lst else ''))
    print(f'collision_monitor STOP transitions: '
          f'{", ".join(f"{x:.2f}s" for x in state_stop_t) or "none/not recorded"}')

    # Attribute each STOP to the source(s) tripping within 0.3 s before it.
    for ts in state_stop_t:
        who = [s for s, lst in (('scan', scan_trip_t), ('depth', depth_trip_t))
               if any(ts - 0.3 <= x <= ts + 0.05 for x in lst)]
        print(f'  STOP @{ts:.2f}s <- {", ".join(who) or "?? (no source tripped: TF/timing?)"}')

    report_chain(chain, states, scan_trip_t, depth_trip_t, logs)

    print(f'/rosout: {len(logs)} relevant lines (repeats within 2 s collapsed)')
    by_node = {}
    for _, lv, node, _m in logs:
        by_node.setdefault(node, []).append(lv)
    for node, lvs in sorted(by_node.items(), key=lambda kv: -len(kv[1])):
        print(f'  {node:30s} {len(lvs):4d}  ' + ', '.join(
            f'{l}x{lvs.count(l)}' for l in sorted(set(lvs))))
    if not state_stop_t and logs:
        print('  -> no collision_monitor STOP but stop-related logs: cause is '
              'likely Nav2/watchdog (see lines above), not FrontStop')

    if z_all:
        z = np.array(z_all)
        print(f'\ndepth z histogram of points in {args.polygon} (all trips):')
        edges = np.arange(dmin, min(dmax, z.max()) + 0.05, 0.05)
        if len(edges) < 2:
            edges = np.array([dmin, dmin + 0.05])
        h, e = np.histogram(z, bins=edges)
        for c, lo in zip(h, e[:-1]):
            print(f'  {lo:4.2f}-{lo+0.05:4.2f} m {c:7d} ' +
                  '#' * int(50 * c / max(h.max(), 1)))
        low = np.mean(z < dmin + 0.05)
        if low > 0.6:
            print(f'  -> {low:.0%} within 5 cm of min_height: floor returns '
                  '(pitch on start / camera tilt) likely')

    if args.csv:
        with open(args.csv, 'w', newline='') as f:
            w = csv.writer(f)
            w.writerow(['t_s', 'source', 'pts_in_polygon', 'v_mps'])
            w.writerows(rows)
        print(f'\nCSV: {args.csv}')


if __name__ == '__main__':
    main()
