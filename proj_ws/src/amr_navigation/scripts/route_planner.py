#!/usr/bin/env python3
"""Route planner for fixed routes: fewest likely stops, not the shortest path.

  route_planner.py <maps_dir> <map_name> <from> <to> [--apply] [--json]

<from>/<to>: station names (<map>_stations.yaml) or "x,y,yaw".

Search: Hybrid-A* over (x, y, heading) with this robot's real rectangle
footprint (FOOT_* below, base_footprint), forward arcs of radius >= R_MIN
and in-place turns (diff drive), checked against a distance map of /map
walls, unknown space and the keep-out mask. Cost of a move =
    length * (1 + W_CLEAR * how far the footprint is inside PREF_M of a wall)
    + turn penalties
so the route keeps to the middle of passages and only goes near a wall
where there is no other way; a pose with less than HARD_M between the
footprint and a wall is never used. Heuristic: clearance-weighted Dijkstra
from the goal on a coarse grid (admissible-ish: plain distance x 1).

Ends: leaves the start station along its heading when there is room, and
docks the last APPROACH_M straight along the goal station's heading
(shortened when a wall is closer) - the same shape as recorded routes.

Result: poses every STEP_M with tangent yaw, footprint clearance checked
again, and compared with the recorded <from> -> <to> route (length,
narrowest gap, poses closer than TIGHT_M). Dry run by default;
--apply writes it into <map>_routes.yaml (old file -> .bak).
--json prints one JSON line (for the control panel button).

The planner knows only the map: people, parked carts and anything not in
the map are not in it. Test-drive every new route before line use
(ISO 3691-4). Not a safety function.
"""
import argparse
import heapq
import json
import math
import os
import shutil
import sys
import time

import numpy as np
import yaml

sys.path.insert(0, os.path.dirname(os.path.realpath(__file__)))
from route_audit import FOOT_FRONT, FOOT_HALF_W, FOOT_REAR, LOADER, load_grid  # noqa: E402
import route_center  # noqa: E402
from route_center import clearance, distance_map, outline  # noqa: E402

HARD_M = 0.18        # m - footprint-to-wall below this: never (set per attempt)
HARD_LEVELS = (0.30, 0.25, 0.20, 0.18, 0.15, 0.12, 0.10, 0.07)  # widest first
PREF_M = 0.60        # m - footprint-to-wall below this costs extra
W_CLEAR = 6.0        # extra cost factor (x length) at HARD_M
TIGHT_M = 0.25       # m - reported as a narrow pose
R_MIN = 0.60         # m - smallest forward turning radius
PRIM_LEN = 0.20      # m - forward motion primitive length
TURN_BIN = 5         # heading bins per in-place turn step (5 x 5 deg)
N_HEAD = 72          # heading bins (5 deg)
XY_RES = 0.10        # m - search cell
W_TURN = 0.15        # cost per arc primitive (prefers straight lines)
W_SPIN = 0.6         # cost per in-place turn step (rear swings, slow)
APPROACH_M = 1.5     # m - straight docking leg into the goal station
LEAVE_M = 1.0        # m - straight out of the start station, when there is room
STEP_M = 0.05        # m - pose spacing (as recorded routes)
GOAL_XY = 0.10       # m - search goal tolerance at the docking leg start
GOAL_TH = math.radians(10)
EPS = 1.5            # weighted A*: heuristic x EPS (much faster, near-optimal)
CONNECT_M = 2.5      # m - try a direct smooth curve to the goal within this
DOCK_MIN_M = 0.05    # m - docking leg footprint may come this close (station is near a wall)
MAX_EXPAND = 25000   # per attempt (a found path needs ~2-3k)
TIME_LIMIT_S = 240.0


def station_pose(stations, s):
    for st in stations:
        if st['name'] == s:
            return float(st['x']), float(st['y']), float(st['theta'])
    try:
        x, y, t = (float(v) for v in s.split(','))
        return x, y, t
    except ValueError:
        raise SystemExit('unknown station %r' % s)


def wrap(a):
    return (a + math.pi) % (2 * math.pi) - math.pi


class Planner:
    def __init__(self, base):
        grids = [load_grid(base + '.yaml')]
        if os.path.exists(base + '_keepout.yaml'):
            grids.append(load_grid(base + '_keepout.yaml'))
        route_center.LIMIT = PREF_M + 0.2
        self.dm = distance_map(grids)
        self.dist, self.res, self.ox, self.oy = self.dm
        self.h, self.w = self.dist.shape
        self.out = outline(self.res / 2)
        # forward primitives: (dx, dy, dth, n_samples) in the robot frame
        self.prims = []
        for curv in (0.0, 1.0 / R_MIN, -1.0 / R_MIN, 0.5 / R_MIN, -0.5 / R_MIN):
            pts = []
            for s in np.linspace(PRIM_LEN / 4, PRIM_LEN, 4):
                if curv == 0.0:
                    pts.append((s, 0.0, 0.0))
                else:
                    r = 1.0 / curv
                    th = s * curv
                    pts.append((r * math.sin(th), r * (1 - math.cos(th)), th))
            self.prims.append((curv, np.array(pts)))

    def clear(self, x, y, th):
        return clearance(self.dm, self.out, np.atleast_1d(x), np.atleast_1d(y), np.atleast_1d(th))

    def step_cost(self, length, cl):
        c = np.clip((PREF_M - cl) / (PREF_M - HARD_M), 0.0, 1.0)
        return float(length * (1.0 + W_CLEAR * c.mean()))

    def connect(self, a, b):
        """Cubic Bezier from pose a to pose b (analytic expansion): list of
        (x, y, yaw) after a, or None if too sharp or it hits a wall."""
        L = math.hypot(b[0] - a[0], b[1] - a[1])
        if L < 1e-3:
            return [b] if abs(wrap(a[2] - b[2])) <= GOAL_TH else None
        if abs(wrap(math.atan2(b[1] - a[1], b[0] - a[0]) - a[2])) > math.radians(60):
            return None
        k = L / 3
        P0 = np.array(a[:2]); P3 = np.array(b[:2])
        P1 = P0 + k * np.array([math.cos(a[2]), math.sin(a[2])])
        P2 = P3 - k * np.array([math.cos(b[2]), math.sin(b[2])])
        t = np.linspace(0, 1, max(3, int(L / 0.05)))[:, None]
        pts = (1 - t) ** 3 * P0 + 3 * (1 - t) ** 2 * t * P1 + 3 * (1 - t) * t ** 2 * P2 + t ** 3 * P3
        d1 = 3 * (1 - t) ** 2 * (P1 - P0) + 6 * (1 - t) * t * (P2 - P1) + 3 * t ** 2 * (P3 - P2)
        d2 = 6 * (1 - t) * (P2 - 2 * P1 + P0) + 6 * t * (P3 - 2 * P2 + P1)
        sp = np.hypot(d1[:, 0], d1[:, 1])
        curv = np.abs(d1[:, 0] * d2[:, 1] - d1[:, 1] * d2[:, 0]) / np.maximum(sp, 1e-6) ** 3
        if curv.max() > 1.0 / (0.8 * R_MIN):
            return None
        yaw = np.arctan2(d1[:, 1], d1[:, 0])
        if self.clear(pts[:, 0], pts[:, 1], yaw).min() < HARD_M:
            return None
        return [(float(px), float(py), float(pt)) for (px, py), pt in zip(pts[1:], yaw[1:])]

    def heuristic_field(self, gx, gy):
        """Clearance-weighted cost-to-go on a coarse grid (point robot,
        cells with no room for the body's half width excluded)."""
        f = int(round(XY_RES / self.res))
        H, W = self.h // f, self.w // f
        d = self.dist[:H * f, :W * f].reshape(H, f, W, f).min(axis=(1, 3))
        free = d >= FOOT_HALF_W * 0.9
        # same scale as step_cost: body side ~ point distance - half width
        w = 1.0 + W_CLEAR * np.clip((PREF_M - (d - FOOT_HALF_W)) / (PREF_M - HARD_M), 0, 1)
        cost = np.full((H, W), np.inf)
        r0, c0 = (self.h - 1 - int((gy - self.oy) / self.res)) // f, int((gx - self.ox) / self.res) // f
        if not (0 <= r0 < H and 0 <= c0 < W):
            return None, f
        cost[r0, c0] = 0.0
        pq = [(0.0, r0, c0)]
        nb = [(-1, 0, 1.0), (1, 0, 1.0), (0, -1, 1.0), (0, 1, 1.0),
              (-1, -1, 1.414), (-1, 1, 1.414), (1, -1, 1.414), (1, 1, 1.414)]
        while pq:
            cst, r, c = heapq.heappop(pq)
            if cst > cost[r, c]:
                continue
            for dr, dc, l in nb:
                rr, cc = r + dr, c + dc
                if 0 <= rr < H and 0 <= cc < W and free[rr, cc]:
                    n = cst + l * XY_RES * w[rr, cc]
                    if n < cost[rr, cc]:
                        cost[rr, cc] = n
                        heapq.heappush(pq, (n, rr, cc))
        return cost, f

    def search(self, start, goal, progress=None):
        sx, sy, sth = start
        gx, gy, gth = goal
        hf, f = self.heuristic_field(gx, gy)
        if hf is None:
            return None, 'goal outside the map'

        def hcost(x, y):
            r = (self.h - 1 - int((y - self.oy) / self.res)) // f
            c = int((x - self.ox) / self.res) // f
            if 0 <= r < hf.shape[0] and 0 <= c < hf.shape[1]:
                return hf[r, c]
            return np.inf

        if not np.isfinite(hcost(sx, sy)):
            return None, 'no corridor wide enough from %s to the goal' % 'start'

        def key(x, y, th):
            return (int(math.floor(x / XY_RES)), int(math.floor(y / XY_RES)),
                    int(round(wrap(th) / (2 * math.pi / N_HEAD))) % N_HEAD)

        t0 = time.time()
        k0 = key(sx, sy, sth)
        nodes = {k0: (sx, sy, sth, 0.0, None, 'start')}
        pq = [(hcost(sx, sy), 0.0, k0)]
        closed = set()
        dth_spin = TURN_BIN * 2 * math.pi / N_HEAD
        n_exp = 0
        while pq:
            _, g, k = heapq.heappop(pq)
            if k in closed:
                continue
            closed.add(k)
            x, y, th, gk, parent, how = nodes[k]
            tail = None
            if math.hypot(x - gx, y - gy) <= CONNECT_M:
                tail = self.connect((x, y, th), goal)
            if tail is not None or (math.hypot(x - gx, y - gy) <= GOAL_XY and
                                    abs(wrap(th - gth)) <= GOAL_TH):
                path = [] if tail is None else [(px, py, pt, 'connect') for px, py, pt in tail[::-1]]
                while k is not None:
                    path.append(nodes[k][:3] + ('node',))
                    if isinstance(nodes[k][5], tuple):   # arc: its inner samples
                        path.extend((px, py, pt, 'arc') for px, py, pt in nodes[k][5][::-1])
                    k = nodes[k][4]
                return path[::-1], 'ok (%d expansions, %.0f s)' % (n_exp, time.time() - t0)
            n_exp += 1
            if n_exp > MAX_EXPAND or time.time() - t0 > TIME_LIMIT_S:
                return None, 'search limit reached (%d expansions)' % n_exp
            if progress and n_exp % 5000 == 0:
                progress('%d expansions, %.0f s' % (n_exp, time.time() - t0))
            c, s = math.cos(th), math.sin(th)
            # forward arcs
            for curv, P in self.prims:
                wx = x + P[:, 0] * c - P[:, 1] * s
                wy = y + P[:, 0] * s + P[:, 1] * c
                wt = th + P[:, 2]
                cl = self.clear(wx, wy, wt)
                if cl.min() < HARD_M:
                    continue
                ng = g + self.step_cost(PRIM_LEN, cl) + (W_TURN if curv else 0.0)
                nk = key(wx[-1], wy[-1], wt[-1])
                if nk in closed:
                    continue
                if nk not in nodes or ng < nodes[nk][3]:
                    nodes[nk] = (wx[-1], wy[-1], wt[-1], ng, k,
                                 tuple(zip(wx[:-1], wy[:-1], wt[:-1])))
                    heapq.heappush(pq, (ng + EPS * hcost(wx[-1], wy[-1]), ng, nk))
            # in-place turns (the whole sweep must be clear)
            for sgn in (1, -1):
                ths = th + sgn * np.linspace(dth_spin / 5, dth_spin, 5)
                cl = self.clear(np.full(5, x), np.full(5, y), ths)
                if cl.min() < HARD_M:
                    continue
                ng = g + W_SPIN + self.step_cost(0.05, cl)
                nk = key(x, y, ths[-1])
                if nk in closed:
                    continue
                if nk not in nodes or ng < nodes[nk][3]:
                    nodes[nk] = (x, y, ths[-1], ng, k, 'spin')
                    heapq.heappush(pq, (ng + EPS * hcost(x, y), ng, nk))
        return None, 'no path (all reachable states explored)'


def densify(path, step):
    """Positions every `step` m along the arc path; in-place turns keep
    their position (the route then has a sharp corner there - RPP turns)."""
    xy = [path[0][:2]]
    for (x0, y0, _, _), (x1, y1, _, _) in zip(path, path[1:]):
        d = math.hypot(x1 - x0, y1 - y0)
        if d < 1e-6:
            continue
        n = max(1, int(round(d / step)))
        for i in range(1, n + 1):
            xy.append((x0 + (x1 - x0) * i / n, y0 + (y1 - y0) * i / n))
    xy = np.array(xy)
    seg = np.hypot(np.diff(xy[:, 0]), np.diff(xy[:, 1]))
    cum = np.concatenate(([0.0], np.cumsum(seg)))
    s = np.append(np.arange(0.0, cum[-1], step), cum[-1])
    return np.column_stack((np.interp(s, cum, xy[:, 0]), np.interp(s, cum, xy[:, 1])))


def route_stats(pl, P):
    cl = pl.clear(P[:, 0], P[:, 1], P[:, 2])
    seg = np.hypot(np.diff(P[:, 0]), np.diff(P[:, 1]))
    cum = np.concatenate(([0.0], np.cumsum(seg)))
    body = (cum > 0.3) & (cum[-1] - cum > 0.3)
    i = int(np.argmin(np.where(body, cl, np.inf)))
    return {'poses': int(len(P)), 'length_m': round(float(cum[-1]), 1),
            'narrowest_m': round(float(cl[i]), 2),
            'narrowest_at': [round(float(P[i, 0]), 2), round(float(P[i, 1]), 2)],
            'tight_poses': int((body & (cl < TIGHT_M)).sum())}


def plan(maps_dir, map_name, frm, to, progress=None):
    """Plan frm -> to with the largest wall clearance that still has a path
    (HARD_LEVELS, widest first). Returns dict (ok, msg, poses, stats, ...)."""
    global HARD_M
    base = os.path.join(maps_dir, map_name)
    with open(base + '_stations.yaml') as f:
        stations = (yaml.safe_load(f) or {}).get('stations', [])
    s = station_pose(stations, frm)
    g = station_pose(stations, to)
    pl = Planner(base)
    tried = []
    for hm in HARD_LEVELS:
        HARD_M = hm
        if progress:
            progress('clearance >= %.2f m' % hm)
        res = _plan_once(pl, base, frm, to, s, g, progress)
        if res['ok']:
            res['clearance_target_m'] = hm
            res['msg'] += '; clearance target %.2f m (tried wider: %s)' % (
                hm, ', '.join('%.2f' % t for t in tried) or '-')
            return res
        tried.append(hm)
    return res


def _plan_once(pl, base, frm, to, s, g, progress):
    # docking leg: longest straight leg (<= APPROACH_M) that is free
    for app in (APPROACH_M, 1.2, 0.9, 0.6, 0.3, 0.0):
        pre = (g[0] - app * math.cos(g[2]), g[1] - app * math.sin(g[2]), g[2])
        ts = np.arange(0.0, app + 1e-9, STEP_M)
        if pl.clear(pre[0] + ts * math.cos(g[2]), pre[1] + ts * math.sin(g[2]),
                    np.full(len(ts), g[2])).min() >= DOCK_MIN_M and \
                pl.clear(*pre)[0] >= HARD_M:
            break
    else:
        return {'ok': False, 'msg': 'no free straight approach into %s' % to}
    # leaving leg: straight ahead out of the start station when free
    lv_len = 0.0
    for lm in (LEAVE_M, 0.6, 0.3):
        ts = np.arange(0.0, lm + 1e-9, STEP_M)
        if pl.clear(s[0] + ts * math.cos(s[2]), s[1] + ts * math.sin(s[2]),
                    np.full(len(ts), s[2]))[1:].min() >= HARD_M:
            lv_len = lm
            break
    lv = (s[0] + lv_len * math.cos(s[2]), s[1] + lv_len * math.sin(s[2]), s[2])
    path, why = pl.search(lv, pre, progress)
    if path is None:
        return {'ok': False, 'msg': why}
    pts = [(s[0], s[1], s[2], 'start')] + path + [(g[0], g[1], g[2], 'dock')]
    xy = densify(pts, STEP_M)
    n = len(xy)
    yaw = np.empty(n)
    for i in range(n):
        j0, j1 = max(i - 2, 0), min(i + 2, n - 1)
        yaw[i] = math.atan2(xy[j1, 1] - xy[j0, 1], xy[j1, 0] - xy[j0, 0]) \
            if (j1 > j0 and np.hypot(*(xy[j1] - xy[j0])) > 1e-6) else s[2]
    yaw[0] = s[2]
    seg = np.hypot(np.diff(xy[:, 0]), np.diff(xy[:, 1]))
    cum = np.concatenate(([0.0], np.cumsum(seg)))
    yaw[cum >= cum[-1] - app - 1e-6] = g[2]
    P = np.column_stack((xy, yaw))
    # second pass: move toward the middle where it is still narrow
    cen = route_center.center_route(P, pl.dm, pl.out, 0.30, 0.30, '%s -> %s' % (frm, to),
                                    log=lambda *_: None)
    if cen is not None:
        P = cen
    st = route_stats(pl, P)
    rec = None
    with open(base + '_routes.yaml') as f:
        data = yaml.load(f, Loader=LOADER) or {}
    old = next((r for r in data.get('routes', [])
                if r.get('from') == frm and r.get('to') == to), None)
    if old and len(old.get('poses', [])) > 2:
        rec = route_stats(pl, np.array([p[:3] for p in old['poses']], float))
    poses = [[round(float(x), 3), round(float(y), 3), round(float(t), 4)] for x, y, t in P]
    poses[-1] = [round(g[0], 3), round(g[1], 3), round(g[2], 4)]
    better = rec is None or (st['narrowest_m'] >= rec['narrowest_m'] - 0.02 and
                             st['tight_poses'] <= rec['tight_poses'])
    return {'ok': True, 'msg': why, 'from': frm, 'to': to, 'stats': st, 'recorded': rec,
            'better': bool(better), 'approach_m': app, 'leave_m': lv_len, 'poses': poses}


def save(maps_dir, map_name, res):
    path_f = os.path.join(maps_dir, map_name + '_routes.yaml')
    with open(path_f) as f:
        data = yaml.load(f, Loader=LOADER) or {}
    routes = [r for r in data.get('routes', [])
              if not (r.get('from') == res['from'] and r.get('to') == res['to'])]
    routes.append({'from': res['from'], 'to': res['to'], 'poses': res['poses']})
    data['routes'] = routes
    shutil.copy2(path_f, path_f + '.bak')
    tmp = path_f + '.tmp'
    with open(tmp, 'w') as f:
        yaml.safe_dump(data, f, allow_unicode=True, sort_keys=False)
    os.replace(tmp, path_f)
    return path_f


def _score(st):
    """Sort key: wider narrowest gap first (2 cm steps), then fewer tight poses."""
    return (round(st['narrowest_m'] / 0.02), -st['tight_poses'])


def best_route(maps_dir, map_name, start, end, progress=None):
    """Control panel button: compute A -> B two ways and keep the better one.
      1. this module's clearance-weighted Hybrid-A* (+ centring pass),
      2. route_tools.plan_route (amr_bringup lattice planner) + centring pass.
    start/end: station dicts {name, x, y, theta}. Returns (poses, notes,
    warns) like route_tools.plan_route; poses None if neither found a path."""
    base = os.path.join(maps_dir, map_name)
    pl = Planner(base)
    cands, notes = [], []
    say = progress or (lambda *_: None)
    say('คำนวณแบบ 1/2 (Hybrid-A*)...')
    try:
        r1 = plan(maps_dir, map_name, start['name'], end['name'])
        if r1['ok']:
            cands.append(('Hybrid-A*', np.array(r1['poses'], float), r1['stats']))
        else:
            notes.append('Hybrid-A*: ' + r1['msg'])
    except Exception as e:  # noqa: BLE001
        notes.append('Hybrid-A*: %s' % e)
    say('คำนวณแบบ 2/2 (lattice)...')
    try:
        bdir = os.path.join(os.path.dirname(os.path.realpath(__file__)), '..', '..',
                            'amr_bringup', 'scripts')
        if bdir not in sys.path:
            sys.path.insert(0, bdir)
        import route_tools
        p2, n2, _ = route_tools.plan_route(base + '.yaml', start, end)
        if p2 is not None:
            P2 = np.array([q[:3] for q in p2], float)
            c2 = route_center.center_route(P2, pl.dm, pl.out, 0.30, 0.30, log=lambda *_: None)
            P2 = c2 if c2 is not None else P2
            cands.append(('lattice', P2, route_stats(pl, P2)))
        else:
            notes.append('lattice: %s' % (n2[0] if n2 else 'no path'))
    except Exception as e:  # noqa: BLE001
        notes.append('lattice: %s' % e)
    if not cands:
        return None, notes or ['no path'], []
    name, P, st = max(cands, key=lambda c: _score(c[2]))
    notes = ['เลือกแบบ %s: ยาว %.1f m, ใกล้ผนังสุด %.2f m ที่ (%.1f, %.1f), จุดแคบ < %.2f m: %d จุด'
             % (name, st['length_m'], st['narrowest_m'], st['narrowest_at'][0],
                st['narrowest_at'][1], TIGHT_M, st['tight_poses'])]
    for other in cands:
        if other[0] != name:
            notes.append('(แบบ %s: ใกล้ผนังสุด %.2f m, จุดแคบ %d จุด)'
                         % (other[0], other[2]['narrowest_m'], other[2]['tight_poses']))
    with open(base + '_routes.yaml') as f:
        data = yaml.load(f, Loader=LOADER) or {}
    old = next((r for r in data.get('routes', [])
                if r.get('from') == start['name'] and r.get('to') == end['name']), None)
    warns = []
    if old and len(old.get('poses', [])) > 2:
        so = route_stats(pl, np.array([q[:3] for q in old['poses']], float))
        notes.append('เส้นเดิม: ใกล้ผนังสุด %.2f m, จุดแคบ %d จุด' % (so['narrowest_m'], so['tight_poses']))
        if _score(so) > _score(st):
            warns.append('เส้นเดิมดีกว่าเส้นที่คำนวณ - แนะนำให้ไม่บันทึกทับ')
    poses = [[round(float(x), 3), round(float(y), 3), round(float(t), 4)] for x, y, t in P]
    poses[-1] = [round(float(end['x']), 3), round(float(end['y']), 3), round(float(end['theta']), 4)]
    return poses, notes, warns


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('maps_dir')
    ap.add_argument('map_name')
    ap.add_argument('frm')
    ap.add_argument('to')
    ap.add_argument('--apply', action='store_true')
    ap.add_argument('--json', action='store_true')
    a = ap.parse_args()
    prog = None if a.json else (lambda m: print('  ... ' + m, flush=True))
    res = plan(a.maps_dir, a.map_name, a.frm, a.to, prog)
    if a.json:
        print(json.dumps({k: v for k, v in res.items() if k != 'poses'}))
    else:
        if not res['ok']:
            raise SystemExit('%s -> %s: %s' % (a.frm, a.to, res['msg']))
        print('%s -> %s: %s, approach %.1f m, leave %.1f m' % (
            a.frm, a.to, res['msg'], res['approach_m'], res['leave_m']))
        print('  planned : %s' % res['stats'])
        print('  recorded: %s' % res['recorded'])
        print('  planned is %s' % ('BETTER or equal' if res['better'] else 'WORSE'))
    if a.apply and res['ok']:
        print('written %s (old file .bak)' % save(a.maps_dir, a.map_name, res))


if __name__ == '__main__':
    main()
