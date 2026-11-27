#!/usr/bin/env python3
"""Offline check of recorded routes for places where the robot will (likely) stop.

  route_audit.py <maps_dir> <map_name> [--wall-margin 0.08] [--from A] [--to B]

Walks every pose of every route in <map>_routes.yaml with the real footprint
and reports stretches where:
  WALL      footprint + wall-margin touches a /map wall or unknown space
            (RPP 'collision ahead' as soon as AMCL is a few cm off)
  KEEPOUT   footprint touches the keep-out mask (planner/RPP refuse)
  FRONTSTOP a /map wall lies in collision_monitor's FrontStop zone
            (bumper .. +0.35 m, +-0.43 m): the LiDAR sees it -> hard stop.
            Not reported in the last END_SKIP_M of a route (station
            approach) or inside a door radius (door_manager swaps zones).
  TIGHT     side gap to walls below --tight (default 0.25 m): passes, but
            any localization error or parked object stops it
Read-only: writes nothing. Not a safety function - a planning aid.
"""
import argparse
import math
import os

import numpy as np
import yaml
from PIL import Image

FOOT_FRONT, FOOT_REAR, FOOT_HALF_W = 0.49, -0.87, 0.40
FS_DEPTH, FS_HALF_W = 0.35, 0.43
END_SKIP_M = 1.5
LOADER = getattr(yaml, 'CSafeLoader', yaml.SafeLoader)


def load_grid(path_yaml):
    with open(path_yaml) as f:
        m = yaml.safe_load(f)
    img = np.array(Image.open(os.path.join(os.path.dirname(path_yaml), m['image'])))
    if img.ndim == 3:
        img = img[..., 0]
    occ = (255 - img.astype(float)) / 255.0
    if m.get('negate'):
        occ = 1.0 - occ
    occupied = occ > m.get('occupied_thresh', 0.65)
    unknown = (~occupied) & (occ > m.get('free_thresh', 0.25))
    return occupied, unknown, m['resolution'], m['origin'][0], m['origin'][1]


def cells(grid, pts):
    occ, unk, res, ox, oy = grid
    h, w = occ.shape
    c = ((pts[:, 0] - ox) / res).astype(int)
    r = h - 1 - ((pts[:, 1] - oy) / res).astype(int)
    out = (c < 0) | (c >= w) | (r < 0) | (r >= h)
    c, r = np.clip(c, 0, w - 1), np.clip(r, 0, h - 1)
    return occ[r, c] & ~out, (unk[r, c] | out)


def rect(x0, x1, y0, y1, step):
    xs = np.arange(x0, x1 + 1e-9, step)
    ys = np.arange(y0, y1 + 1e-9, step)
    g = np.array([(x, y) for x in xs for y in ys])
    return g


def to_world(local, x, y, yaw):
    c, s = math.cos(yaw), math.sin(yaw)
    return np.column_stack((x + local[:, 0] * c - local[:, 1] * s,
                            y + local[:, 0] * s + local[:, 1] * c))


def side_gap(grid, x, y, yaw, side, step, limit=1.5):
    d = FOOT_HALF_W
    while d < limit:
        px = x - side * d * math.sin(yaw)
        py = y + side * d * math.cos(yaw)
        o, u = cells(grid, np.array([[px, py]]))
        if o[0] or u[0]:
            return d - FOOT_HALF_W
        d += step / 2
    return limit


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('maps_dir')
    ap.add_argument('map_name')
    ap.add_argument('--wall-margin', type=float, default=0.08)
    ap.add_argument('--tight', type=float, default=0.25)
    ap.add_argument('--from', dest='frm')
    ap.add_argument('--to')
    a = ap.parse_args()
    base = os.path.join(a.maps_dir, a.map_name)
    grid = load_grid(base + '.yaml')
    keep = load_grid(base + '_keepout.yaml') if os.path.exists(base + '_keepout.yaml') else None
    doors = []
    if os.path.exists(base + '_doors.yaml'):
        with open(base + '_doors.yaml') as f:
            doors = (yaml.safe_load(f) or {}).get('doors', [])
    with open(base + '_routes.yaml') as f:
        routes = (yaml.load(f, Loader=LOADER) or {}).get('routes', [])
    res = grid[2]
    m = a.wall_margin
    foot = rect(FOOT_REAR - m, FOOT_FRONT + m, -FOOT_HALF_W - m, FOOT_HALF_W + m, res / 2)
    foot0 = rect(FOOT_REAR, FOOT_FRONT, -FOOT_HALF_W, FOOT_HALF_W, res / 2)
    fs = rect(FOOT_FRONT + 0.02, FOOT_FRONT + FS_DEPTH, -FS_HALF_W, FS_HALF_W, res / 2)

    for rt in routes:
        if a.frm and rt.get('from') != a.frm or a.to and rt.get('to') != a.to:
            continue
        poses = rt.get('poses', [])
        if len(poses) < 2:
            continue
        cum = [0.0]
        for i in range(1, len(poses)):
            cum.append(cum[-1] + math.hypot(poses[i][0] - poses[i - 1][0],
                                            poses[i][1] - poses[i - 1][1]))
        total = cum[-1]
        issues = []   # (kind, along, x, y, detail)
        for i, (x, y, yaw) in enumerate(p[:3] for p in poses):
            o, u = cells(grid, to_world(foot, x, y, yaw))
            if o.any() or u.any():
                o0, u0 = cells(grid, to_world(foot0, x, y, yaw))
                issues.append(('WALL', cum[i], x, y,
                               'footprint on wall' if (o0.any() or u0.any())
                               else 'within %.2f m of wall' % m))
            if keep is not None:
                ko, _ = cells(keep, to_world(foot0, x, y, yaw))
                if ko.any():
                    issues.append(('KEEPOUT', cum[i], x, y, 'footprint in keep-out'))
            in_door = any(math.hypot(x - d['x'], y - d['y']) <= d.get('radius', 1.5)
                          for d in doors)
            if total - cum[i] > END_SKIP_M and not in_door:
                fo, _ = cells(grid, to_world(fs, x, y, yaw))
                if fo.any():
                    issues.append(('FRONTSTOP', cum[i], x, y, 'wall in FrontStop zone'))
            gl = side_gap(grid, x, y, yaw, 1.0, res)
            gr = side_gap(grid, x, y, yaw, -1.0, res)
            if min(gl, gr) < a.tight:
                issues.append(('TIGHT', cum[i], x, y, 'side gap L %.2f R %.2f' % (gl, gr)))
        print('== %s -> %s  (%d poses, %.1f m)' % (rt.get('from'), rt.get('to'), len(poses), total))
        if not issues:
            print('   OK')
            continue
        # merge consecutive issues of one kind into stretches
        for kind in ('KEEPOUT', 'WALL', 'FRONTSTOP', 'TIGHT'):
            st = None
            rows = [it for it in issues if it[0] == kind] + [None]
            for it in rows:
                if st is not None and (it is None or it[1] - st[1] > 0.3):
                    print('   %-9s %6.1f-%6.1f m  at (%.2f, %.2f)  %s'
                          % (kind, st[0][1], st[1], st[0][2], st[0][3], st[0][4]))
                    st = None
                if it is not None:
                    st = [it, it[1]] if st is None else [st[0], it[1]]


if __name__ == '__main__':
    main()
