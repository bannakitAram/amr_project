#!/usr/bin/env python3
"""Fixed-route post-processing shared by amr_control_panel.py (record / draw)
and usable from the shell on an existing <map>_routes.yaml.

Hand-recorded routes carry the driver's weave: FR2_e1_e1 HOME->doo drifted
12 cm toward the side wall within the first 1.3 m and then turned hard, so
the 0.87 m rear overhang swung into the wall and RPP stopped with "collision
ahead" right at the start (bag_20261124_092846, t=122.7 s). This module:

  1. smooths the route (moving average over ~SMOOTH_M),
  2. replaces the start with a straight EXIT_M leg along the start station's
     heading, joined back to the route by a smooth curve,
  3. sweeps the real footprint along the route over the saved map and
     reports every stretch with less than CLEAR_WARN_M to a wall.

The map check only sees the saved map, not pallets/carts: every route must
still be test-driven at low speed before line use.

Shell use (writes <routes>.bak first):
  route_tools.py <map.yaml> [--plan] [--apply] [--from HOME] [--to doo]
  --plan  replace each route by a computed one (recorded line = guide)
  route_tools.py <map.yaml> --new FROM TO [--apply]   compute a route, no guide
"""
import math
import os
import shutil
import sys

import numpy as np
import yaml
from PIL import Image

# Footprint in base_footprint (nav2_params.yaml local/global costmap).
FP_FRONT, FP_REAR, FP_HALF_W = 0.49, -0.87, 0.42
STEP_M = 0.05          # m between route points
SMOOTH_M = 0.5         # m moving-average window (removes hand weave)
EXIT_M = 1.0           # m straight leg out of the start station
JOIN_M = 1.0           # m length of the curve joining the exit leg to the route
STRAIGHT_TOL_M = 0.10  # m - a stretch deviating less than this is made straight
# Tolerance: AMCL +-3-5 cm, heading +-2 deg on the 0.87 m rear overhang ~3 cm,
# RPP tracking 2-5 cm, map cells +-2.5 cm -> ~10-15 cm. A computed route must
# keep the footprint at least ROUTE_MIN_CLEAR_M from every wall/keep-out cell
# of the saved map, else it is refused (point highlighted on the phone).
ROUTE_MIN_CLEAR_M = 0.12
GOAL_TOL_CELLS = 4   # lattice goal tolerance (x 5 cm) = one move
GOAL_HEADING_TOL = 2  # bins (x 11.25 deg); the straight approach leg aligns the rest
SPIN_COST_M = 2.0   # A* cost of turning on the spot at a station (prefer driving out)
CLEAR_WARN_M = ROUTE_MIN_CLEAR_M  # recorded/drawn lines: warning below this
# The footprint sweep above already catches a rear swing into a wall; this
# only flags turns the robot cannot follow (Smac minimum_turning_radius 0.50).
MIN_RADIUS_WARN_M = 0.5  # m


# ---- map -----------------------------------------------------------------
def load_occ(map_yaml):
    """(resolution, origin_x, origin_y, occ bool grid with row 0 = bottom)."""
    with open(map_yaml) as f:
        meta = yaml.safe_load(f)
    img = meta['image']
    if not os.path.isabs(img):
        img = os.path.join(os.path.dirname(map_yaml), img)
    g = np.asarray(Image.open(img).convert('L'), dtype=np.float32) / 255.0
    occ = g if meta.get('negate', 0) else 1.0 - g
    occ = occ > meta.get('occupied_thresh', 0.65)
    return (float(meta['resolution']), float(meta['origin'][0]),
            float(meta['origin'][1]), occ[::-1])


def footprint_clearance(grid, x, y, yaw, search=0.30):
    """Smallest distance (m) from the footprint rectangle at (x, y, yaw) to an
    occupied cell, capped at `search`; 0.0 = a wall cell inside the footprint."""
    res, ox, oy, occ = grid
    c, s = math.cos(yaw), math.sin(yaw)
    reach = max(abs(FP_FRONT), abs(FP_REAR), FP_HALF_W) * 1.5 + search
    c0 = int((x - reach - ox) / res)
    c1 = int((x + reach - ox) / res) + 1
    r0 = int((y - reach - oy) / res)
    r1 = int((y + reach - oy) / res) + 1
    c0, r0 = max(c0, 0), max(r0, 0)
    c1, r1 = min(c1, occ.shape[1]), min(r1, occ.shape[0])
    if c0 >= c1 or r0 >= r1:
        return 0.0
    rows, cols = np.nonzero(occ[r0:r1, c0:c1])
    if rows.size == 0:
        return search
    wx = ox + (cols + c0 + 0.5) * res - x
    wy = oy + (rows + r0 + 0.5) * res - y
    lx = wx * c + wy * s          # into the robot frame
    ly = -wx * s + wy * c
    dx = np.maximum(np.maximum(FP_REAR - lx, lx - FP_FRONT), 0.0)
    dy = np.maximum(np.abs(ly) - FP_HALF_W, 0.0)
    d = np.hypot(dx, dy) - res / 2  # cell centre -> cell edge
    return float(min(max(d.min(), 0.0), search))


# ---- geometry --------------------------------------------------------------
def _resample(xy, step=STEP_M):
    out = [xy[0]]
    carry = 0.0
    for (x0, y0), (x1, y1) in zip(xy, xy[1:]):
        seg = math.hypot(x1 - x0, y1 - y0)
        if seg < 1e-9:
            continue
        d = step - carry
        while d <= seg:
            t = d / seg
            out.append((x0 + (x1 - x0) * t, y0 + (y1 - y0) * t))
            d += step
        carry = seg - (d - step)
    if math.hypot(out[-1][0] - xy[-1][0], out[-1][1] - xy[-1][1]) > 0.01:
        out.append(tuple(xy[-1]))
    return out


def _smooth(xy, window_m=SMOOTH_M):
    n = len(xy)
    k = max(1, int(round(window_m / STEP_M / 2)))
    out = []
    for i in range(n):
        w = min(k, i, n - 1 - i)        # ends stay where they were
        seg = xy[i - w:i + w + 1]
        out.append((sum(p[0] for p in seg) / len(seg), sum(p[1] for p in seg) / len(seg)))
    return out


def _straighten(xy, tol=STRAIGHT_TOL_M):
    """Douglas-Peucker: stretches that stay within tol of a line become that
    line (the corners are then rounded again by _smooth)."""
    def rec(i, j):
        (x0, y0), (x1, y1) = xy[i], xy[j]
        L = math.hypot(x1 - x0, y1 - y0) or 1e-9
        best, bi = 0.0, None
        for k in range(i + 1, j):
            d = abs((x1 - x0) * (y0 - xy[k][1]) - (x0 - xy[k][0]) * (y1 - y0)) / L
            if d > best:
                best, bi = d, k
        if bi is None or best <= tol:
            return [i, j]
        return rec(i, bi)[:-1] + rec(bi, j)
    idx = rec(0, len(xy) - 1)
    return [xy[i] for i in idx]


def _with_yaw(xy, k=1):
    """Poses with yaw = direction from point i-k to i+k (k > 1: the yaw a
    real robot can follow - no spike at a 2 cm grid jog)."""
    poses = []
    n = len(xy)
    for i, (x, y) in enumerate(xy):
        ax, ay = xy[max(i - k, 0)]
        bx, by = xy[min(i + k, n - 1)]
        poses.append([round(x, 3), round(y, 3), round(math.atan2(by - ay, bx - ax), 3)])
    return poses


def _exit_leg(xy, start):
    """Straight EXIT_M along the start heading, cubic Bezier into the route."""
    th = start.get('theta', 0.0)
    sx, sy = start['x'], start['y']
    ex, ey = sx + EXIT_M * math.cos(th), sy + EXIT_M * math.sin(th)
    # first route point at least EXIT_M + JOIN_M along the route
    dist, j = 0.0, None
    for i in range(1, len(xy)):
        dist += math.hypot(xy[i][0] - xy[i - 1][0], xy[i][1] - xy[i - 1][1])
        if dist >= EXIT_M + JOIN_M:
            j = i
            break
    if j is None or j + 1 >= len(xy):
        return None                      # route too short for an exit leg
    jx, jy = xy[j]
    tx, ty = xy[j + 1][0] - xy[j - 1][0], xy[j + 1][1] - xy[j - 1][1]
    tl = math.hypot(tx, ty) or 1e-9
    tx, ty = tx / tl, ty / tl
    h = math.hypot(jx - ex, jy - ey) / 3.0
    p1 = (ex + h * math.cos(th), ey + h * math.sin(th))
    p2 = (jx - h * tx, jy - h * ty)
    curve = []
    for k in range(1, 41):
        t = k / 40.0
        a, b, c, d = (1 - t) ** 3, 3 * t * (1 - t) ** 2, 3 * t * t * (1 - t), t ** 3
        curve.append((a * ex + b * p1[0] + c * p2[0] + d * jx,
                      a * ey + b * p1[1] + c * p2[1] + d * jy))
    return _resample([(sx, sy), (ex, ey)] + curve + list(xy[j + 1:]))


def _min_radius(xy, span=6):
    """[(index, radius)] for turns tighter than MIN_RADIUS_WARN_M."""
    out = []
    for i in range(span, len(xy) - span):
        (x0, y0), (x1, y1), (x2, y2) = xy[i - span], xy[i], xy[i + span]
        a = math.hypot(x1 - x0, y1 - y0)
        b = math.hypot(x2 - x1, y2 - y1)
        c = math.hypot(x2 - x0, y2 - y0)
        area2 = abs((x1 - x0) * (y2 - y0) - (x2 - x0) * (y1 - y0))
        if area2 < 1e-9:
            continue
        r = a * b * c / (2 * area2)
        if r < MIN_RADIUS_WARN_M:
            out.append((i, r))
    return out


# ---- public ----------------------------------------------------------------
def check_route(grid, poses):
    """Warnings (Thai) for footprint-to-wall clearance and tight turns."""
    warns = []
    tight = None
    for i, (x, y, yaw) in enumerate(poses):
        d = footprint_clearance(grid, x, y, yaw)
        if d < CLEAR_WARN_M:
            if tight is None or d < tight[2]:
                tight = (i, (x, y), d)          # worst point of this stretch
        elif tight is not None:
            warns.append(_clear_msg(poses, tight))
            tight = None
    if tight is not None:
        warns.append(_clear_msg(poses, tight))
    xy = [(p[0], p[1]) for p in poses]
    last = -99
    for i, r in _min_radius(xy):
        if i - last > 10:
            warns.append('เลี้ยวแคบ รัศมี %.2f ม. ที่ระยะ %.1f ม. (%.1f, %.1f) - ท้ายรถเหวี่ยง'
                         % (r, _along(poses, i), xy[i][0], xy[i][1]))
        last = i
    return warns


def _along(poses, i):
    return sum(math.hypot(poses[k][0] - poses[k - 1][0], poses[k][1] - poses[k - 1][1])
               for k in range(1, i + 1))


def _clear_msg(poses, tight):
    i, (x, y), d = tight
    what = 'footprint ทับผนังบนแผนที่' if d <= 0.0 else 'ห่างผนัง %.0f ซม.' % (d * 100)
    return '%s ที่ระยะ %.1f ม. (%.1f, %.1f)' % (what, _along(poses, i), x, y)


def _clear_score(grid, poses):
    """(min clearance, number of poses under CLEAR_WARN_M) - higher/lower is better."""
    d = [footprint_clearance(grid, x, y, yaw) for x, y, yaw in poses]
    return min(d), sum(1 for v in d if v < CLEAR_WARN_M)


def _not_worse(grid, new, old):
    if grid is None:
        return True
    (nmin, nlow), (omin, olow) = _clear_score(grid, new), _clear_score(grid, old)
    return nmin >= omin - 0.01 and nlow <= olow


def improve_route(poses, start, grid=None):
    """Smoothed route with a straight exit leg. Returns (poses, notes, warns):
    notes = what was changed, warns = clearance/turn problems left (if grid).
    With a grid a step is kept only if it does not bring the footprint closer
    to a wall (e.g. the driver steered away from a wall on purpose)."""
    def finish(xy):
        out = _with_yaw(xy)
        out[-1][2] = round(poses[-1][2], 3)  # keep the recorded arrival heading
        return out
    best = [list(p) for p in poses]
    notes = []
    xy = _resample([(p[0], p[1]) for p in poses])
    xy = _resample(_straighten(_smooth(xy)))
    xy = _smooth(xy, SMOOTH_M)           # round the corners left by straightening
    cand = finish(xy)
    if _not_worse(grid, cand, best):
        best = cand
        notes.append('เกลาเส้นทาง (ตัดการส่ายของมือ)')
    leg = _exit_leg([(p[0], p[1]) for p in best], start)
    if leg is not None:
        cand = finish(leg)
        if _not_worse(grid, cand, best):
            best = cand
            notes.append('ออกจาก %s ตรง %.1f ม. ตามทิศ Station' % (start['name'], EXIT_M))
        else:
            notes.append('ไม่ได้ทำทางออกตรง (ชิดผนังกว่าเส้นที่ขับ)')
    warns = check_route(grid, best) if grid is not None else []
    return best, notes, warns


STATION_WARN_M = 0.15   # m footprint-to-wall at a station: less = hard to leave


def check_station(grid, st):
    """Thai warning if the parked footprint is too close to a wall, else None."""
    d = footprint_clearance(grid, st['x'], st['y'], st.get('theta', 0.0))
    if d >= STATION_WARN_M:
        return None
    return ('Station %s ห่างผนังบนแผนที่ %.0f ซม. (แนะนำ ≥ %.0f ซม.) - '
            'หุ่นอาจหยุดตอนออกตัว ("collision ahead")' % (st['name'], d * 100, STATION_WARN_M * 100))


# ---- planner: compute a route that keeps the footprint clear ---------------
# A* over the saved map (unknown + keep-out count as wall), cost rising as the
# centreline gets closer to a wall than PLAN_PREF_M, so the route runs down
# the middle of aisles. With a guide (recorded/drawn line) the search stays
# within PLAN_CORRIDOR_M of it - the operator still chooses WHICH way to go.
# The result is smoothed, given straight legs out of / into the stations, then
# the real footprint is swept along it and points still too close to a wall
# are pushed sideways. Map only: pallets/carts are not on the map.
PLAN_MIN_CENTER_M = FP_HALF_W + 0.01  # m centreline to wall: footprint just fits
PLAN_PREF_M = 1.0        # m - closer than this costs extra
PLAN_WEIGHT = 8.0        # how hard the middle of the aisle is preferred
PLAN_CORRIDOR_M = 1.0    # m around the guide line
APPROACH_M = 0.6         # m straight leg into the end station
DIST_CAP_M = 1.6


def _load_blocked(map_yaml):
    """(res, ox, oy, blocked) with row 0 = bottom; blocked = wall, unknown,
    or keep-out (<map>_keepout.yaml next to the map, same grid)."""
    with open(map_yaml) as f:
        meta = yaml.safe_load(f)
    img = meta['image']
    if not os.path.isabs(img):
        img = os.path.join(os.path.dirname(map_yaml), img)
    g = np.asarray(Image.open(img).convert('L'), dtype=np.float32) / 255.0
    occ = g if meta.get('negate', 0) else 1.0 - g
    blocked = ~(occ < meta.get('free_thresh', 0.25))
    kp = map_yaml[:-5] + '_keepout.yaml'
    if os.path.exists(kp):
        try:
            kres, kox, koy, kocc = load_occ(kp)
            if (kocc.shape == blocked.shape and abs(kres - meta['resolution']) < 1e-6
                    and abs(kox - meta['origin'][0]) < 1e-6 and abs(koy - meta['origin'][1]) < 1e-6):
                blocked |= kocc[::-1]
        except (OSError, KeyError, ValueError, yaml.YAMLError):
            pass
    return (float(meta['resolution']), float(meta['origin'][0]),
            float(meta['origin'][1]), blocked[::-1])


def _chamfer(seed, res, cap):
    """Distance (m) to the nearest seed cell, capped. Two-pass chamfer with a
    5x5 mask (steps 1, sqrt2, sqrt5): within ~2 % of Euclidean."""
    import itertools
    d = np.where(seed, 0.0, cap).astype(np.float32)
    steps = [(dr, dc, res * math.hypot(dr, dc)) for dr, dc in itertools.product(range(-2, 3), repeat=2)
             if (dr, dc) != (0, 0) and math.gcd(abs(dr), abs(dc)) == 1]
    rows, cols = d.shape
    for _ in range(int(cap / res) + 2):
        old = d.copy()
        for dr, dc, w in steps:
            dst = d[max(dr, 0):rows + min(dr, 0), max(dc, 0):cols + min(dc, 0)]
            src = d[max(-dr, 0):rows + min(-dr, 0), max(-dc, 0):cols + min(-dc, 0)]
            np.minimum(dst, src + w, out=dst)
        if np.array_equal(old, d):
            break
    return np.minimum(d, cap)


PLAN_RES_M = 0.10        # m planning grid
PLAN_FP_MARGINS_M = (0.20, 0.16, 0.13)  # m footprint margin, tried in order (>= ROUTE_MIN_CLEAR_M)
PLAN_TURN_COST_M = 0.15  # m extra per 11.25 deg heading change (prefers wide turns)
# 32 headings (11.25 deg), one move = ~4 cells (20 cm) along the heading.
# 8 headings (45 deg) could not shift sideways in a 1.95 m aisle and 16 could
# only in one travel direction (FR2 doo->HOME "stuck" at x=9.1). The skipped
# cells are covered: consecutive footprints (1.36 m long) overlap, and the
# exact sweep in plan_route checks the final route anyway.
LATTICE_N, LATTICE_STEP = 32, 4
HEADINGS = []
for _k in range(LATTICE_N):
    _a = 2 * math.pi * _k / LATTICE_N
    HEADINGS.append((int(round(LATTICE_STEP * math.sin(_a))), int(round(LATTICE_STEP * math.cos(_a)))))
NH = len(HEADINGS)


FP_DISKS = 8   # footprint covered by this many disks along its axis


def _footprint_free(dist, res, margin):
    """free[h][r, c]: footprint (+ margin) centred on cell (r, c) facing
    HEADINGS[h] is clear. The rectangle is covered by FP_DISKS disks on its
    long axis (radius covers the corners: ~1 cm over the sides with 8), so
    the test is 8 look-ups in the wall-distance map. Outside the crop =
    blocked. The exact rectangle is checked again afterwards (check_route)."""
    L = FP_FRONT - FP_REAR
    sp = L / FP_DISKS
    rad = math.hypot(FP_HALF_W, sp / 2) + margin
    xs = [FP_REAR + sp * (k + 0.5) for k in range(FP_DISKS)]
    rows, cols = dist.shape
    pad = int(math.ceil(max(abs(FP_REAR), FP_FRONT) / res)) + 1
    dp = np.pad(dist, pad, constant_values=0.0)
    out = []
    for dr, dc in HEADINGS:
        yaw = math.atan2(dr, dc)
        ok = np.ones(dist.shape, bool)
        for x in xs:
            orow = int(round(x * math.sin(yaw) / res))
            ocol = int(round(x * math.cos(yaw) / res))
            ok &= dp[pad + orow:pad + orow + rows, pad + ocol:pad + ocol + cols] >= rad
        out.append(ok)
    return out


def _astar(free, clear, start, goal, res, corridor=None):
    """A* over (row, col, heading); moves one cell along the current heading
    or a neighbouring one (45 deg), and only onto cells where the footprint
    facing that way is clear. start/goal = (row, col, heading); goal heading
    None = arrive in any direction (via points).
    Returns (path, None), or (None, (row, col)) = the reached cell closest
    to the goal, i.e. where the robot gets stuck."""
    import heapq
    rows, cols = clear.shape
    pen = 1.0 + PLAN_WEIGHT * np.square(np.maximum(PLAN_PREF_M - clear, 0.0) / PLAN_PREF_M)
    if corridor is not None:
        pen = np.where(corridor, pen, np.inf)
    gr, gc, gh = goal
    starts = start if isinstance(start, list) else [(start, 0.0)]
    gs = {st: g for st, g in starts}
    came = {}
    heap = [(g, st) for st, g in starts]
    heapq.heapify(heap)
    start = starts[0][0]
    done = set()
    stuck, stuck_d = start[:2], math.hypot(start[0] - gr, start[1] - gc)
    while heap:
        _, cur = heapq.heappop(heap)
        if cur in done:
            continue
        r, c, h = cur
        dg = math.hypot(r - gr, c - gc)
        if dg < stuck_d:
            stuck, stuck_d = (r, c), dg
        # within GOAL_TOL_CELLS of the goal is enough: hitting the exact cell
        # with the right heading in a narrow aisle needs a sideways shift the
        # long body may not fit (FR2 doo->HOME failed 0.6 m from the goal);
        # the last bit is joined straight and the exact check runs afterwards
        if dg <= GOAL_TOL_CELLS and (gh is None or min((h - gh) % NH, (gh - h) % NH) <= GOAL_HEADING_TOL):
            path = [cur]
            while cur in came:
                cur = came[cur]
                path.append(cur)
            return path[::-1], None
        done.add(cur)
        g0 = gs[cur]
        for dh in (0, -1, 1):
            nh = (h + dh) % NH
            dr, dc = HEADINGS[nh]
            nr, nc = r + dr, c + dc
            if not (0 <= nr < rows and 0 <= nc < cols) or not free[nh][nr, nc]:
                continue
            if dh and not free[nh][r, c]:          # turning on the spot cell
                continue
            step = math.hypot(dr, dc) * res
            ng = g0 + step * pen[nr, nc] + (PLAN_TURN_COST_M if dh else 0.0)
            if not ng < 1e17:
                continue
            nxt = (nr, nc, nh)
            if ng < gs.get(nxt, 1e18):
                gs[nxt] = ng
                came[nxt] = cur
                heapq.heappush(heap, (ng + math.hypot(nr - gr, nc - gc) * res, nxt))
    return None, stuck


def _push_clear(grid, poses, iters=4):
    """Shift poses closer than CLEAR_WARN_M to a wall sideways (<= 0.20 m)
    to the best clearance, re-smooth, keep the result only if it is better."""
    best = poses
    for _ in range(iters):
        xy = [(p[0], p[1]) for p in best]
        moved = False
        for i, (x, y, yaw) in enumerate(best):
            if footprint_clearance(grid, x, y, yaw) >= CLEAR_WARN_M:
                continue
            nx, ny = -math.sin(yaw), math.cos(yaw)
            cand = max(((footprint_clearance(grid, x + k * 0.02 * nx, y + k * 0.02 * ny, yaw), -abs(k), k)
                        for k in range(-10, 11)))
            if cand[2]:
                xy[i] = (x + cand[2] * 0.02 * nx, y + cand[2] * 0.02 * ny)
                moved = True
        if not moved:
            break
        cand = _with_yaw(_resample(_smooth(xy, 0.3)))
        cand[0], cand[-1] = list(best[0]), list(best[-1])
        if _clear_score(grid, cand) > _clear_score(grid, best) or (
                _clear_score(grid, cand)[1] < _clear_score(grid, best)[1]):
            best = cand
        else:
            break
    return best


def _heading_bin(theta):
    """Index of the HEADINGS entry closest to theta."""
    return min(range(NH), key=lambda h: abs(math.atan2(
        math.sin(theta - math.atan2(HEADINGS[h][0], HEADINGS[h][1])),
        math.cos(theta - math.atan2(HEADINGS[h][0], HEADINGS[h][1])))))


def plan_route(map_yaml, start, end, guide=None, via=None):
    """plan_route_once with the widest footprint margin that finds a route.
    via: [(x, y), ...] points the route must pass through, in order (the
    phone's tapped points); the route is computed leg by leg between them.
    Failure: (None, Thai reason, bad) - bad = {'points': [via indices, 0-based],
    'leg': [i, j] in the sequence start=0, via 1..n, end=n+1, 'at': [x, y]
    where it fails, 'station': name}; keys only when known (for the phone
    to highlight what to move/delete)."""
    why, bad = '', {}
    grid = load_occ(map_yaml)
    for m in PLAN_FP_MARGINS_M:
        poses, notes, warns = plan_route_once(map_yaml, start, end, guide, m, via)
        if poses is None:
            # The heading lattice is coarse (22.5 deg): in a tight aisle that
            # drifts sideways only one travel direction may be found (FR2
            # HOME->doo yes, doo->HOME no). Plan the reverse trip and drive it
            # backwards along the same line - the exact check below decides.
            flip = lambda st: dict(st, theta=st.get('theta', 0.0) + math.pi)
            rp, rn, _ = plan_route_once(map_yaml, flip(end), flip(start),
                                        guide[::-1] if guide else None, m,
                                        list(via or [])[::-1])
            if rp is not None:
                poses = [[x, y, round(math.atan2(math.sin(t + math.pi), math.cos(t + math.pi)), 3)]
                         for x, y, t in rp[::-1]]
                notes = [n.replace(end['name'], '\0').replace(start['name'], end['name'])
                         .replace('\0', start['name']) for n in rn] + ['คำนวณจากปลายทางย้อนกลับ']
                warns = check_route(load_occ(map_yaml), poses)
        if poses is not None:
            d = [footprint_clearance(grid, *q) for q in poses]
            k = int(np.argmin(d))
            if d[k] < ROUTE_MIN_CLEAR_M - 1e-3:
                q = poses[k]
                poses, notes = None, ('ตัวรถห่างผนังแค่ %.0f ซม. ที่ (%.1f, %.1f) - ต้อง ≥ %.0f ซม. '
                                      '(เผื่อความคลาดเคลื่อน)' % (d[k] * 100, q[0], q[1],
                                                                ROUTE_MIN_CLEAR_M * 100))
                warns = {'at': [round(q[0], 2), round(q[1], 2)]}
        if poses is not None:
            if m < PLAN_FP_MARGINS_M[0]:
                notes.append('ทางแคบ: ห่างผนังขั้นต่ำ %.0f ซม.' % (ROUTE_MIN_CLEAR_M * 100))
            return poses, notes, warns
        why, bad = notes, warns or {}
    return None, why, bad


def plan_route_once(map_yaml, start, end, guide, margin, via=None):
    """Compute a fixed route start -> end on the saved map: the real
    footprint (+PLAN_FP_MARGIN_M) is checked at every step and heading.
    Returns (poses, notes, warns) or (None, reason, []) when none exists."""
    res0, ox0, oy0, blocked0 = _load_blocked(map_yaml)
    f = 1   # map resolution: a coarser grid closes doors/narrow aisles
    res = res0 * f
    via = [(float(p[0]), float(p[1])) for p in (via or [])]
    pts = [(start['x'], start['y']), (end['x'], end['y'])] + [(p[0], p[1]) for p in (guide or [])] + via
    crop = (PLAN_CORRIDOR_M if guide else 15.0) + DIST_CAP_M + 1.5
    c0 = max(int((min(p[0] for p in pts) - crop - ox0) / res0), 0) // f * f
    r0 = max(int((min(p[1] for p in pts) - crop - oy0) / res0), 0) // f * f
    c1 = min(int((max(p[0] for p in pts) + crop - ox0) / res0) + 1, blocked0.shape[1])
    r1 = min(int((max(p[1] for p in pts) + crop - oy0) / res0) + 1, blocked0.shape[0])
    b = blocked0[r0:r1, c0:c1]
    b = b[:b.shape[0] // f * f, :b.shape[1] // f * f]
    if f > 1:
        b = b.reshape(b.shape[0] // f, f, b.shape[1] // f, f).any(axis=(1, 3))
    ox, oy = ox0 + c0 * res0, oy0 + r0 * res0
    clear = _chamfer(b, res, DIST_CAP_M)  # DIST_CAP_M > disk radius
    free = _footprint_free(clear, res, margin)
    corridor = None
    if guide:
        gseed = np.zeros_like(b)
        for x, y, *_ in guide:
            r, c = int((y - oy) / res), int((x - ox) / res)
            if 0 <= r < b.shape[0] and 0 <= c < b.shape[1]:
                gseed[r, c] = True
        corridor = _chamfer(gseed, res, PLAN_CORRIDOR_M + res) <= PLAN_CORRIDOR_M

    def cell(x, y):
        return int((y - oy) / res), int((x - ox) / res)

    def leg_ok(ax, ay, bx, by, h):
        n = max(2, int(math.hypot(bx - ax, by - ay) / res) + 1)
        for k in range(n + 1):
            r, c = cell(ax + (bx - ax) * k / n, ay + (by - ay) * k / n)
            if not (0 <= r < b.shape[0] and 0 <= c < b.shape[1]) or not free[h][r, c]:
                return False
        return True

    notes = []
    sth, eth = start.get('theta', 0.0), end.get('theta', 0.0)
    sh, eh = _heading_bin(sth), _heading_bin(eth)
    a = None
    for L in (EXIT_M, 0.6, 0.3, 0.0):            # straight leg out of the station
        ax, ay = start['x'] + L * math.cos(sth), start['y'] + L * math.sin(sth)
        if leg_ok(start['x'], start['y'], ax, ay, sh):
            a = (ax, ay)
            if L:
                notes.append('ออกจาก %s ตรง %.1f ม.' % (start['name'], L))
            break
    e = None
    for L in (APPROACH_M, 0.3, 0.0):              # straight leg into the station
        ex, ey = end['x'] - L * math.cos(eth), end['y'] - L * math.sin(eth)
        if leg_ok(ex, ey, end['x'], end['y'], eh):
            e = (ex, ey)
            break
    if a is None or e is None:
        bad_st = start if a is None else end
        return None, ('Station %s ชิดผนังเกินไป (ต้องห่าง ≥ 12 ซม.) '
                      '(ขยับ Station ออกจากผนัง แล้วบันทึกทับ)' % bad_st['name']), \
            {'station': bad_st['name'], 'at': [bad_st['x'], bad_st['y']]}
    for k, (vx, vy) in enumerate(via):
        r, c = cell(vx, vy)
        if not (0 <= r < b.shape[0] and 0 <= c < b.shape[1]) or not any(fr[r, c] for fr in free):
            return None, ('จุดที่ %d ชิดผนัง/อยู่นอกทาง - ตัวรถวางลงไม่ได้ '
                          '(ลบจุดนี้แล้ววางใหม่กลางทาง)' % (k + 1)), \
                {'points': [k], 'at': [vx, vy]}
    grid = load_occ(map_yaml)

    def can_spin(st):
        # the whole body (rear corner 0.97 m from the centre) sweeps a circle
        return all(footprint_clearance(grid, st['x'], st['y'], k * math.pi / 16) >= ROUTE_MIN_CLEAR_M
                   for k in range(32))
    spin_start, spin_end = can_spin(start), can_spin(end)
    path, keep_cells = [], []
    cur = [(cell(*a) + (sh,), 0.0)]
    if spin_start:      # or turn on the spot at the start station, any heading
        scell = cell(start['x'], start['y'])
        cur += [(scell + (h,), SPIN_COST_M) for h in range(NH) if h != sh and free[h][scell]]
    targets = [cell(*v) + (None,) for v in via] + [cell(*e) + (eh,)]
    for k, tgt in enumerate(targets):
        leg, stuck = _astar(free, clear, cur, tgt, res, corridor)
        if leg is None and k == len(targets) - 1 and spin_end:
            # arrive in any direction and turn on the spot at the end station
            ecell = cell(end['x'], end['y'])
            leg, stuck2 = _astar(free, clear, cur, ecell + (None,), res, corridor)
            if leg is not None:
                e = (end['x'], end['y'])
                notes.append('หมุนกลับที่ %s' % end['name'])
        if leg is None:
            name = lambda i: (start['name'] if i == 0 else end['name'] if i == len(via) + 1
                              else 'จุดที่ %d' % i)
            sx, sy = ox + (stuck[1] + 0.5) * res, oy + (stuck[0] + 0.5) * res
            bad = {'leg': [k, k + 1], 'points': [i - 1 for i in (k, k + 1) if 1 <= i <= len(via)],
                   'at': [round(sx, 2), round(sy, 2)]}
            if k == 0 and not spin_start and math.hypot(sx - start['x'], sy - start['y']) < 2.0:
                return None, ('ออกจาก %s ไปทางนั้นไม่ได้ - ต้องกลับรถ แต่รอบ Station ไม่มีที่ให้หมุนตัว '
                              '(ต้องว่างรัศมี ~1.1 ม.) - ย้าย Station หรือตั้งทิศ Station ให้หันออกทางที่จะไป'
                              % start['name']), {'station': start['name'], 'at': [start['x'], start['y']]}
            return None, ('ไปจาก %s ถึง %s ไม่ได้ - ตัวรถติดที่ (%.1f, %.1f) '
                          '(ทางแคบกว่าตัวรถ + %.0f ซม. ต่อข้าง หรือกั้นด้วยโซนห้ามเข้า)%s'
                          % (name(k), name(k + 1), sx, sy, ROUTE_MIN_CLEAR_M * 100,
                             ' - ลบ/ย้ายจุดที่ไฮไลต์ หรือเพิ่มจุดพาอ้อมไปทางอื่น' if via else '')
                          + (' ในแนวเส้นที่ขับ (±%.1f ม.)' % PLAN_CORRIDOR_M if guide else '')), bad
        path += leg if not path else leg[1:]
        if k < len(via):
            keep_cells.append(len(path) - 1)
        cur = leg[-1]
    cells = [(ox + (c + 0.5) * res, oy + (r + 0.5) * res,
              math.atan2(*HEADINGS[h])) for r, c, h in path]
    if spin_start and path[0][:2] == cell(start['x'], start['y']) and path[0][2] != sh:
        a = (start['x'], start['y'])          # turned on the spot instead of the exit leg
        notes = [n for n in notes if not n.startswith('ออกจาก')] + ['หมุนกลับที่ %s' % start['name']]
    lattice = ([(start['x'], start['y'], sth)] + [(a[0], a[1], sth)] + cells[1:-1]
               + [(e[0], e[1], eth), (end['x'], end['y'], eth)])
    # path[i] -> lattice[i + 1] (lattice = start, a, path[1:-1], e, end)
    poses = _shape_path(grid, lattice, n_head=2, n_tail=2,
                        must=[i + 1 for i in keep_cells])
    if via:
        notes.append('ผ่าน %d จุดที่กำหนด' % len(via))
    notes.insert(0, 'คำนวณเส้นทางให้ตัวรถพ้นผนัง (%.0f ม.)' % _along(poses, len(poses) - 1))
    return poses, notes, check_route(grid, poses)


SHAPE_TARGET_M = 0.20     # m clearance a shortcut/fillet must keep (or what the lattice had)
SHAPE_MIN_M = ROUTE_MIN_CLEAR_M  # m - shortcuts/fillets never come closer than this
FILLET_RADII = (2.0, 1.5, 1.0, 0.7, 0.5, 0.3)


def _seg_poses(p, q, step=STEP_M):
    yaw = math.atan2(q[1] - p[1], q[0] - p[0])
    n = max(1, int(math.hypot(q[0] - p[0], q[1] - p[1]) / step))
    return [(p[0] + (q[0] - p[0]) * k / n, p[1] + (q[1] - p[1]) * k / n, yaw) for k in range(n + 1)]


def _poses_ok(grid, poses, need):
    """need: one clearance for all poses, or a function of the pose index."""
    f = need if callable(need) else (lambda k: need)
    return all(footprint_clearance(grid, x, y, t) >= f(k) - 1e-3
               for k, (x, y, t) in enumerate(poses))


def _shape_path(grid, lattice, n_head, n_tail, must=()):
    """Lattice cells -> straight segments + rounded corners. Each shortcut and
    each fillet is accepted only if the exact footprint keeps at least
    min(SHAPE_TARGET_M, what the lattice path itself had there) to every wall.
    The first n_head / last n_tail points (station legs) are kept as they are,
    and so are the indices in `must` (via points): shortcuts never skip them."""
    ref = [footprint_clearance(grid, x, y, t) for x, y, t in lattice]
    pts = [(x, y) for x, y, _ in lattice]
    n = len(pts)
    # string pulling, greedy farthest reachable point (binary search)
    keep = list(range(n_head))
    i = n_head - 1
    must = sorted(m for m in must if n_head <= m < n - n_tail)
    while i < n - n_tail:
        lo, hi = i + 1, min([m for m in must if m > i] + [n - n_tail])
        best = lo
        while lo <= hi:
            mid = (lo + hi) // 2
            seg = _seg_poses(pts[i], pts[mid])
            # local need: what the lattice had at the matching place (+-0.5 m)
            def need(k, i=i, mid=mid, m=len(seg)):
                c = i + round(k * (mid - i) / max(m - 1, 1))
                return max(SHAPE_MIN_M, min(SHAPE_TARGET_M, min(ref[max(c - 5, i):min(c + 6, mid + 1)])))
            if _poses_ok(grid, seg, need):
                best, lo = mid, mid + 1
            else:
                hi = mid - 1
        keep.append(best)
        i = best
    keep += list(range(n - n_tail + 1, n))
    keep = sorted(set(keep))
    verts = [pts[k] for k in keep]
    vref = [min(ref[keep[j]:keep[j + 1] + 1]) if j + 1 < len(keep) else ref[keep[j]]
            for j in range(len(keep))]
    # fillets at the inner vertices
    out = [verts[0]]
    for j in range(1, len(verts) - 1):
        p0, p1, p2 = out[-1], verts[j], verts[j + 1]
        h0 = math.atan2(p1[1] - p0[1], p1[0] - p0[0])
        h1 = math.atan2(p2[1] - p1[1], p2[0] - p1[0])
        turn = math.atan2(math.sin(h1 - h0), math.cos(h1 - h0))
        done = False
        if abs(turn) > 1e-3 and j >= n_head - 1 and j < len(verts) - n_tail:
            l0 = math.hypot(p1[0] - p0[0], p1[1] - p0[1])
            l1 = math.hypot(p2[0] - p1[0], p2[1] - p1[1])
            need = max(SHAPE_MIN_M, min(SHAPE_TARGET_M, vref[j - 1], vref[j]))
            for R in FILLET_RADII:
                t = R * math.tan(abs(turn) / 2)
                if t > 0.5 * l0 or t > 0.5 * l1:
                    continue
                a0 = (p1[0] - t * math.cos(h0), p1[1] - t * math.sin(h0))
                sgn = 1 if turn > 0 else -1
                cx, cy = a0[0] - sgn * R * math.sin(h0), a0[1] + sgn * R * math.cos(h0)
                m = max(2, int(R * abs(turn) / STEP_M))
                arc = []
                for k in range(m + 1):
                    hh = h0 + turn * k / m
                    arc.append((cx + sgn * R * math.sin(hh), cy - sgn * R * math.cos(hh), hh))
                if _poses_ok(grid, _seg_poses(p0, a0)[:-1] + arc, need):
                    out += [(x, y) for x, y, _ in arc]
                    done = True
                    break
        if not done:
            out.append(p1)
    out.append(verts[-1])
    dense = _resample(out)
    return _with_yaw(dense, k=5)


def main(argv):
    args = [a for a in argv if not a.startswith('--')]
    if '--new' in argv:
        args = args[:1]
    if len(args) != 1:
        print(__doc__)
        return 2
    map_yaml = args[0]
    apply = '--apply' in argv
    only_from = argv[argv.index('--from') + 1] if '--from' in argv else None
    only_to = argv[argv.index('--to') + 1] if '--to' in argv else None
    base = map_yaml[:-5]
    with open(base + '_stations.yaml') as f:
        stations = {s['name']: s for s in (yaml.safe_load(f) or {}).get('stations', [])}
    rpath = base + '_routes.yaml'
    doc = {}
    if os.path.exists(rpath):
        with open(rpath) as f:
            doc = yaml.safe_load(f) or {}
    grid = load_occ(map_yaml)
    if '--new' in argv:
        a, b = argv[argv.index('--new') + 1:argv.index('--new') + 3]
        poses, notes, warns = plan_route(map_yaml, stations[a], stations[b])
        if poses is None:
            print('plan failed: ' + notes)
            return 1
        print('%s -> %s: %s' % (a, b, ', '.join(notes)))
        for w in warns:
            print('  ! ' + w)
        if apply:
            doc['routes'] = [r for r in doc.get('routes', []) if (r['from'], r['to']) != (a, b)]
            doc['routes'].append({'from': a, 'to': b, 'poses': poses})
            doc.setdefault('use_routes', True)
            if os.path.exists(rpath):
                shutil.copy2(rpath, rpath + '.bak')
            with open(rpath, 'w') as f:
                yaml.safe_dump(doc, f, sort_keys=False, default_flow_style=None)
            print('written ' + rpath)
        return 0
    for st in stations.values():
        w = check_station(grid, st)
        if w:
            print('!! ' + w)
    for rt in doc.get('routes', []):
        if (only_from and rt['from'] != only_from) or (only_to and rt['to'] != only_to):
            continue
        print('== %s -> %s (%d pts)' % (rt['from'], rt['to'], len(rt['poses'])))
        before = check_route(grid, rt['poses'])
        print('  before: ' + ('; '.join(before) if before else 'OK'))
        if rt['from'] not in stations:
            print('  station %s not found - skipped' % rt['from'])
            continue
        if '--plan' in argv:
            new, notes, warns = plan_route(map_yaml, stations[rt['from']], stations[rt['to']],
                                           guide=rt['poses'])
            if new is None:
                print('  plan failed: ' + notes)
                continue
        else:
            new, notes, warns = improve_route(rt['poses'], stations[rt['from']], grid)
        print('  after:  ' + ('; '.join(warns) if warns else 'OK') + '  [%s]' % ', '.join(notes))
        dev = max(min(math.hypot(p[0] - q[0], p[1] - q[1]) for q in rt['poses']) for p in new)
        print('  max shift from recorded line: %.2f m' % dev)
        if apply:
            rt['poses'] = new
    if apply:
        shutil.copy2(rpath, rpath + '.bak')
        with open(rpath, 'w') as f:
            yaml.safe_dump(doc, f, sort_keys=False, default_flow_style=None)
        print('written %s (backup %s.bak)' % (rpath, rpath))
    return 0


if __name__ == '__main__':
    sys.exit(main(sys.argv[1:]))
