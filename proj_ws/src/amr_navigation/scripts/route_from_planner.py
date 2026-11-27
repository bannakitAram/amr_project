#!/usr/bin/env python3
"""Compute a fixed route with Nav2's own planner (Smac Hybrid-A*) and save it.

  python3 route_from_planner.py <maps_dir> <map_name> <from> <to> [--apply] [--no-clear]
  (ROS 2 + workspace sourced, Navigation running)

Needs Navigation running on <map_name>. Asks planner_server
(/compute_path_to_pose, planner "GridBased" = SmacPlannerHybrid: real
footprint, minimum turning radius, forward-only, cost_penalty keeps to the
middle) for a path from station <from> to station <to> - start given
explicitly, so the robot does not have to stand at <from>.

The global costmap also holds live obstacles (people, carts): by default it
is cleared first (clear_entirely_global_costmap) so the route follows the
map + keep-out only; --no-clear keeps the current marks.

The path is resampled every STEP_M with tangent yaw (last pose = station
pose), its footprint clearance to /map walls / keep-out checked (as
route_center.py), and compared with the existing recorded route if any.
Dry run by default; --apply replaces/adds the <from> -> <to> route in
<map>_routes.yaml (old file -> .bak; the control panel picks it up by
mtime). Run route_audit.py and test-drive before line use (ISO 3691-4:
verify every new path). Not a safety function.
"""
import argparse
import math
import os
import shutil
import sys

import numpy as np
import rclpy
import yaml
from geometry_msgs.msg import PoseStamped
from nav2_msgs.action import ComputePathToPose
from nav2_msgs.srv import ClearEntireCostmap
from rclpy.action import ActionClient

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from route_audit import LOADER, load_grid  # noqa: E402
from route_center import clearance, distance_map, outline  # noqa: E402

STEP_M = 0.05
MIN_FOOT_CLEAR = 0.05   # m - below: refuse (planner footprint vs our map check)
PLANNER_ID = 'GridBased'


def station_pose(stations, s):
    for st in stations:
        if st['name'] == s:
            return float(st['x']), float(st['y']), float(st['theta'])
    raise SystemExit('unknown station %r' % s)


def pose(node, x, y, th):
    p = PoseStamped()
    p.header.frame_id = 'map'
    p.header.stamp = node.get_clock().now().to_msg()
    p.pose.position.x, p.pose.position.y = x, y
    p.pose.orientation.z, p.pose.orientation.w = math.sin(th / 2), math.cos(th / 2)
    return p


def resample(xy, step):
    seg = np.hypot(np.diff(xy[:, 0]), np.diff(xy[:, 1]))
    keep = np.concatenate(([True], seg > 1e-6))
    xy = xy[keep]
    seg = np.hypot(np.diff(xy[:, 0]), np.diff(xy[:, 1]))
    cum = np.concatenate(([0.0], np.cumsum(seg)))
    s = np.append(np.arange(0.0, cum[-1], step), cum[-1])
    return np.column_stack((np.interp(s, cum, xy[:, 0]), np.interp(s, cum, xy[:, 1])))


def stats(dm, out, P):
    cl = clearance(dm, out, P[:, 0], P[:, 1], P[:, 2])
    seg = np.hypot(np.diff(P[:, 0]), np.diff(P[:, 1]))
    cum = np.concatenate(([0.0], np.cumsum(seg)))
    body = (cum > 0.3) & (cum[-1] - cum > 0.3)   # not the station poses themselves
    i = int(np.argmin(np.where(body, cl, np.inf)))
    return cum[-1], cl[i], cum[i], P[i, 0], P[i, 1], int((body & (cl < 0.25)).sum())


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('maps_dir')
    ap.add_argument('map_name')
    ap.add_argument('frm')
    ap.add_argument('to')
    ap.add_argument('--apply', action='store_true')
    ap.add_argument('--no-clear', action='store_true')
    a = ap.parse_args(rclpy.utilities.remove_ros_args(sys.argv)[1:])
    base = os.path.join(a.maps_dir, a.map_name)
    with open(base + '_stations.yaml') as f:
        stations = (yaml.safe_load(f) or {}).get('stations', [])
    s = station_pose(stations, a.frm)
    g = station_pose(stations, a.to)

    rclpy.init()
    node = rclpy.create_node('route_from_planner')
    try:
        if not a.no_clear:
            cli = node.create_client(ClearEntireCostmap, '/global_costmap/clear_entirely_global_costmap')
            if cli.wait_for_service(timeout_sec=5.0):
                fut = cli.call_async(ClearEntireCostmap.Request())
                rclpy.spin_until_future_complete(node, fut, timeout_sec=5.0)
                rclpy.spin_once(node, timeout_sec=1.0)   # let the static layer repaint
            else:
                print('warning: global costmap clear service not available')
        ac = ActionClient(node, ComputePathToPose, '/compute_path_to_pose')
        if not ac.wait_for_server(timeout_sec=10.0):
            raise SystemExit('planner_server not available - is Navigation running?')
        goal = ComputePathToPose.Goal()
        goal.start = pose(node, *s)
        goal.goal = pose(node, *g)
        goal.use_start = True
        goal.planner_id = PLANNER_ID
        fut = ac.send_goal_async(goal)
        rclpy.spin_until_future_complete(node, fut, timeout_sec=10.0)
        gh = fut.result()
        if gh is None or not gh.accepted:
            raise SystemExit('planner refused the request')
        rf = gh.get_result_async()
        rclpy.spin_until_future_complete(node, rf, timeout_sec=30.0)
        res = rf.result()
        if res is None or not res.result.path.poses:
            raise SystemExit('no path from %s to %s (planner failed)' % (a.frm, a.to))
        raw = np.array([[p.pose.position.x, p.pose.position.y] for p in res.result.path.poses])
    finally:
        node.destroy_node()
        rclpy.try_shutdown()

    xy = resample(np.vstack(([s[:2]], raw, [g[:2]])), STEP_M)
    n = len(xy)
    yaw = np.empty(n)
    for i in range(n):
        j0, j1 = max(i - 2, 0), min(i + 2, n - 1)
        yaw[i] = math.atan2(xy[j1, 1] - xy[j0, 1], xy[j1, 0] - xy[j0, 0])
    yaw[0], yaw[-1] = s[2], g[2]
    P = np.column_stack((xy, yaw))

    grids = [load_grid(base + '.yaml')]
    if os.path.exists(base + '_keepout.yaml'):
        grids.append(load_grid(base + '_keepout.yaml'))
    dm = distance_map(grids)
    out = outline(dm[1] / 2)
    L, cmin, at, mx, my, nt = stats(dm, out, P)
    print('planner  %s -> %s: %d poses, %.1f m, narrowest %.2f m at %.1f m (%.2f, %.2f), '
          '%d poses < 0.25 m' % (a.frm, a.to, n, L, cmin, at, mx, my, nt))
    path_f = base + '_routes.yaml'
    with open(path_f) as f:
        data = yaml.load(f, Loader=LOADER) or {}
    old = next((r for r in data.get('routes', [])
                if r.get('from') == a.frm and r.get('to') == a.to), None)
    if old:
        O = np.array([p[:3] for p in old['poses']], float)
        L2, c2, at2, x2, y2, nt2 = stats(dm, out, O)
        print('recorded %s -> %s: %d poses, %.1f m, narrowest %.2f m at %.1f m (%.2f, %.2f), '
              '%d poses < 0.25 m' % (a.frm, a.to, len(O), L2, c2, at2, x2, y2, nt2))
    if cmin < MIN_FOOT_CLEAR:
        raise SystemExit('refused: footprint within %.2f m of a wall/keep-out (< %.2f)'
                         % (cmin, MIN_FOOT_CLEAR))
    if not a.apply:
        print('dry run - add --apply to write')
        return
    poses = [[round(float(x), 3), round(float(y), 3), round(float(t), 4)] for x, y, t in P]
    routes = [r for r in data.get('routes', [])
              if not (r.get('from') == a.frm and r.get('to') == a.to)]
    routes.append({'from': a.frm, 'to': a.to, 'poses': poses})
    data['routes'] = routes
    shutil.copy2(path_f, path_f + '.bak')
    tmp = path_f + '.tmp'
    with open(tmp, 'w') as f:
        yaml.safe_dump(data, f, allow_unicode=True, sort_keys=False)
    os.replace(tmp, path_f)
    print('written %s (old file .bak)' % path_f)


if __name__ == '__main__':
    main()
