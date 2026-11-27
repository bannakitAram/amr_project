#!/usr/bin/env python3
"""Compute a route between two stations from the map (no driving/recording).

  route_compute.py <maps_dir> <map_name> <from> <to> [--apply] [--png out.png]

  <from>/<to>: station names from <map>_stations.yaml (or "x,y,yaw").

Search: A* on the map grid (8-connected) over a distance map to /map walls,
unknown space and keep-out. Cells closer than HARD_M are forbidden (body half
width + margin); closer than PREF_M cost extra, so the path keeps to the
middle of passages and only hugs a wall where it must. The last APPROACH_M
is a straight line along the goal station's heading (docking as recorded
routes do); the first LEAVE_M leaves the start station straight ahead if
there is room, else it starts turning at once.

Then: smoothed, resampled every STEP_M with tangent yaw, and every pose's
real footprint (rectangle, at that yaw) checked against the distance map.
Reported: length, narrowest clearance and where. Dry run by default;
--apply replaces/adds the <from> -> <to> route in <map>_routes.yaml
(old file -> .bak). Run route_audit.py and test-drive before line use
(ISO 3691-4: verify every new path). Not a safety function.
"""
import argparse
import heapq
import math
import os
import shutil

import numpy as np
import yaml

from route_audit import LOADER, load_grid
from route_center import clearance, distance_map, outline
import route_center

HARD_M = 0.45      # m - path cell this close to a wall: forbidden (half width 0.40 + 5 cm)
PREF_M = 1.0       # m - closer than this costs extra (centre of passages)
WEIGHT = 4.0       # extra cost factor at HARD_M
APPROACH_M = 1.5   # m - straight docking into the goal station
LEAVE_M = 0.6      # m - straight out of the start station, if free
STEP_M = 0.05      # m - pose spacing (as recorded routes)
SMOOTH_M = 0.8     # m - moving-average window
REPAIRS = 12
REPAIR_R = 0.4     # m - area made dearer around a pose whose body touched a wall
REPAIR_COST = 3.0
MIN_FOOT_CLEAR = 0.08  # m - footprint outline to wall: below -> refuse


def station_pose(stations, s):
    for st in stations:
        if st['name'] == s:
            return float(st['x']), float(st['y']), float(st['theta'])
    try:
        x, y, t = (float(v) for v in s.split(','))
        return x, y, t
    except ValueError:
        raise SystemExit('unknown station %r' % s)


def astar(dist, res, start, goal, extra=None):
    h, w = dist.shape
    cost_map = (0.0 if extra is None else extra) + np.where(dist < HARD_M, np.inf,
                        1.0 + WEIGHT * np.clip((PREF_M - dist) / (PREF_M - HARD_M), 0, 1))
    sr, sc = start
    gr, gc = goal
    if not np.isfinite(cost_map[sr, sc]) or not np.isfinite(cost_map[gr, gc]):
        return None
    g = {start: 0.0}
    came = {}
    pq = [(0.0, start)]
    nb = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
          (-1, -1, 1.4142), (-1, 1, 1.4142), (1, -1, 1.4142), (1, 1, 1.4142)]
    closed = set()
    while pq:
        _, cur = heapq.heappop(pq)
        if cur in closed:
            continue
        if cur == goal:
            path = [cur]
            while cur in came:
                cur = came[cur]
                path.append(cur)
            return path[::-1]
        closed.add(cur)
        r, c = cur
        gc_ = g[cur]
        for dr, dc, l in nb:
            rr, cc = r + dr, c + dc
            if not (0 <= rr < h and 0 <= cc < w):
                continue
            cm = cost_map[rr, cc]
            if not np.isfinite(cm):
                continue
            ng = gc_ + l * cm
            nxt = (rr, cc)
            if ng < g.get(nxt, np.inf):
                g[nxt] = ng
                came[nxt] = cur
                heapq.heappush(pq, (ng + math.hypot(rr - gr, cc - gc), nxt))
    return None


def resample(xy, step):
    seg = np.hypot(np.diff(xy[:, 0]), np.diff(xy[:, 1]))
    cum = np.concatenate(([0.0], np.cumsum(seg)))
    s = np.arange(0.0, cum[-1], step)
    s = np.append(s, cum[-1])
    return np.column_stack((np.interp(s, cum, xy[:, 0]), np.interp(s, cum, xy[:, 1])))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('maps_dir')
    ap.add_argument('map_name')
    ap.add_argument('frm')
    ap.add_argument('to')
    ap.add_argument('--apply', action='store_true')
    a = ap.parse_args()
    base = os.path.join(a.maps_dir, a.map_name)
    with open(base + '_stations.yaml') as f:
        stations = (yaml.safe_load(f) or {}).get('stations', [])
    sx, sy, sth = station_pose(stations, a.frm)
    gx, gy, gth = station_pose(stations, a.to)
    grids = [load_grid(base + '.yaml')]
    if os.path.exists(base + '_keepout.yaml'):
        grids.append(load_grid(base + '_keepout.yaml'))
    route_center.LIMIT = PREF_M + 0.1
    dm = distance_map(grids)
    dist, res, ox, oy = dm
    h = dist.shape[0]

    def cell(x, y):
        return h - 1 - int((y - oy) / res), int((x - ox) / res)

    def world(rc):
        return ox + (rc[1] + 0.5) * res, oy + (h - 1 - rc[0] + 0.5) * res

    out = outline(res / 2)
    # straight docking leg into the goal, straight leave from the start
    for app in (APPROACH_M, 1.2, 0.9, 0.6, 0.3):
        pre = (gx - app * math.cos(gth), gy - app * math.sin(gth))
        dock = np.array([[pre[0] + t * math.cos(gth), pre[1] + t * math.sin(gth)]
                         for t in np.arange(0.0, app + 1e-9, STEP_M)])
        if clearance(dm, out, dock[:, 0], dock[:, 1], np.full(len(dock), gth)).min() > 0.0 \
                and dist[cell(*pre)] >= HARD_M:
            break
    else:
        raise SystemExit('no straight docking leg (>= 0.3 m) into %s is free' % a.to)
    if app < APPROACH_M:
        print('  docking leg shortened to %.1f m (wall/keep-out within %.1f m of %s)'
              % (app, APPROACH_M, a.to))
    leave_ok = False
    for lm in (1.5, 1.2, 0.9, 0.6):
        lv = (sx + lm * math.cos(sth), sy + lm * math.sin(sth))
        leg = np.array([[sx + t * math.cos(sth), sy + t * math.sin(sth)]
                        for t in np.arange(0.0, lm + 1e-9, STEP_M)])
        if clearance(dm, out, leg[:, 0], leg[:, 1], np.full(len(leg), sth)).min() > 0.1 \
                and dist[cell(*lv)] >= HARD_M:
            leave_ok = True
            break
    s0 = lv if leave_ok else (sx, sy)
    extra = np.zeros_like(dist)
    for it in range(REPAIRS):
        path = astar(dist, res, cell(*s0), cell(*pre), extra)
        if path is None:
            raise SystemExit('no path keeping %.2f m from walls/keep-out between %s and %s'
                             % (HARD_M, a.frm, a.to))
        mid = np.array([world(p) for p in path])
        xy = np.vstack(((leg if leave_ok else [[sx, sy]]), mid, dock[1:]))
        xy = resample(xy, STEP_M)
        # smooth (ends fixed: start pose and the straight docking leg)
        k = max(1, int(SMOOTH_M / STEP_M / 2))
        n_dock = int(app / STEP_M)
        sm = xy.copy()
        n_leave = int(lm / STEP_M) if leave_ok else 0
        for i in range(max(k, n_leave), len(xy) - n_dock):
            sm[i] = xy[max(0, i - k):i + k + 1].mean(axis=0)
        xy = resample(sm, STEP_M)
        n = len(xy)
        yaw = np.empty(n)
        for i in range(n):
            j0, j1 = max(i - 3, 0), min(i + 3, n - 1)
            yaw[i] = math.atan2(xy[j1, 1] - xy[j0, 1], xy[j1, 0] - xy[j0, 0])
        yaw[-n_dock:] = gth
        if n_leave:
            yaw[:n_leave] = sth
        cl = clearance(dm, out, xy[:, 0], xy[:, 1], yaw)
        seg = np.hypot(np.diff(xy[:, 0]), np.diff(xy[:, 1]))
        cum = np.concatenate(([0.0], np.cumsum(seg)))
        body = cum > 0.3   # the start pose itself is where the robot stands
        i_min = int(np.argmin(np.where(body, cl, np.inf)))
        print('%s -> %s: %d poses, %.1f m, narrowest footprint clearance %.2f m at %.1f m (%.2f, %.2f)'
              % (a.frm, a.to, n, cum[-1], cl[i_min], cum[i_min], xy[i_min, 0], xy[i_min, 1]))
        tight = np.nonzero(body & (cl < 0.25))[0]
        if len(tight):
            print('  clearance < 0.25 m at %d poses (%.1f-%.1f m along)'
                  % (len(tight), cum[tight[0]], cum[tight[-1]]))
        bad = np.nonzero(body & (cl < MIN_FOOT_CLEAR))[0]
        if len(bad) and it < REPAIRS - 1:
            # the body (rear swing) touches a wall here: make this area dearer
            # so A* turns elsewhere / keeps further off, and plan again
            rr, cc = np.ogrid[:h, :dist.shape[1]]
            for i in bad[::4]:
                r0, c0 = cell(xy[i, 0], xy[i, 1])
                rad = int(REPAIR_R / res)
                extra[max(0, r0 - rad):r0 + rad + 1, max(0, c0 - rad):c0 + rad + 1] += REPAIR_COST
            print('  repair %d: %d poses touch walls near %.1f m along - re-planning'
                  % (it + 1, len(bad), cum[bad[0]]))
            continue
        if cl[i_min] < MIN_FOOT_CLEAR:
            raise SystemExit('refused: footprint gets within %.2f m of a wall (< %.2f)'
                             % (cl[i_min], MIN_FOOT_CLEAR))
        break
    poses = [[round(float(x), 3), round(float(y), 3), round(float(t), 4)]
             for (x, y), t in zip(xy, yaw)]
    poses[-1] = [round(gx, 3), round(gy, 3), round(gth, 4)]
    path_f = base + '_routes.yaml'
    if not a.apply:
        print('dry run - add --apply to write')
        return
    with open(path_f) as f:
        data = yaml.load(f, Loader=LOADER) or {}
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
