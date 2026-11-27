#!/usr/bin/env python3
"""Move recorded routes toward the middle of narrow passages.

  route_center.py <maps_dir> <map_name> [--tight 0.30] [--max-shift 0.30]
                  [--from A] [--to B] [--apply]

Clearance of a pose = smallest distance from the footprint outline to a
/map wall, unknown space or keep-out cell (distance map, map resolution).
Poses within REACH_M (along) of a pose whose clearance is below --tight
may shift sideways by up to --max-shift. A dynamic programme picks one
offset per pose that maximises sum(min(clearance, tight)), with:
  - lane change <= SLOPE m per m, and the footprint evaluated at the lean
    that lane change gives (the rear swings out while shifting),
  - a heavy cost for any pose ending below its own original clearance,
  - the first START_KEEP_M and last END_KEEP_M (station approach) fixed.
A route is left unchanged if any pose still ends narrower than before
(and below --tight) by more than one map cell.

Dry run by default. --apply writes <map>_routes.yaml (old -> .bak).
Re-run route_audit.py and test-drive every changed route before line use
(ISO 3691-4: verify after any path change).
"""
import argparse
import math
import os
import shutil

import numpy as np
import yaml

from route_audit import FOOT_FRONT, FOOT_HALF_W, FOOT_REAR, LOADER, load_grid

START_KEEP_M = 0.5
END_KEEP_M = 1.5
SLOPE = 0.15          # m sideways per m along (~8.5 deg lean)
GRID = 0.01           # m - lateral offset step
REACH_M = 2.5         # m - poses this close (along) to a narrow pose may move
YAW_WIN = 5           # poses each side for the new heading (~0.25 m: the
                      # 1.4 m body cannot follow a 5 cm wiggle)
RETRIES = 6
LIMIT = 1.0           # m - distance map cap


def distance_map(grids):
    """Approx. Euclidean distance (m) to the nearest blocked cell, capped."""
    occ, unk, res, ox, oy = grids[0]
    blocked = occ | unk
    for g in grids[1:]:
        if g[0].shape == blocked.shape:
            blocked = blocked | g[0]
    big = 1e9
    d = np.where(blocked, 0.0, big)
    steps = int(LIMIT / res) + 2
    diag = math.sqrt(2.0)
    for _ in range(steps):
        p = np.pad(d, 1, constant_values=big)
        cand = [p[1:-1, :-2] + 1, p[1:-1, 2:] + 1, p[:-2, 1:-1] + 1, p[2:, 1:-1] + 1,
                p[:-2, :-2] + diag, p[:-2, 2:] + diag, p[2:, :-2] + diag, p[2:, 2:] + diag]
        nd = np.minimum(d, np.minimum.reduce(cand))
        if np.array_equal(nd, d):
            break
        d = nd
    return np.minimum(d * res, LIMIT), res, ox, oy


def outline(step):
    pts = []
    for x in np.arange(FOOT_REAR, FOOT_FRONT + 1e-9, step):
        pts += [(x, FOOT_HALF_W), (x, -FOOT_HALF_W)]
    for y in np.arange(-FOOT_HALF_W, FOOT_HALF_W + 1e-9, step):
        pts += [(FOOT_FRONT, y), (FOOT_REAR, y)]
    return np.array(pts)


def clearance(dm, out, x, y, yaw):
    """Min distance map value over the footprint outline; x, y, yaw arrays."""
    d, res, ox, oy = dm
    h, w = d.shape
    x, y, yaw = np.atleast_1d(x), np.atleast_1d(y), np.atleast_1d(yaw)
    c, s = np.cos(yaw)[:, None], np.sin(yaw)[:, None]
    wx = x[:, None] + out[None, :, 0] * c - out[None, :, 1] * s
    wy = y[:, None] + out[None, :, 0] * s + out[None, :, 1] * c
    ci = ((wx - ox) / res).astype(int)
    rj = h - 1 - ((wy - oy) / res).astype(int)
    bad = (ci < 0) | (ci >= w) | (rj < 0) | (rj >= h)
    v = d[np.clip(rj, 0, h - 1), np.clip(ci, 0, w - 1)]
    v[bad] = 0.0
    return v.min(axis=1)


def center_route(P, dm, out, tight=0.30, max_shift=0.30, name='route', log=print):
    """Shift poses P (n x 3) toward the middle of narrow passages.
    Returns the new n x 3 array, or None (nothing to do / not better)."""
    ks = np.round(np.arange(-max_shift, max_shift + 1e-9, GRID), 3)
    k0 = int(np.argmin(np.abs(ks)))
    NEG = -1e9
    n = len(P)
    if n < 3:
        return None
    seg = np.hypot(np.diff(P[:, 0]), np.diff(P[:, 1]))
    cum = np.concatenate(([0.0], np.cumsum(seg)))
    tang = np.empty(n)
    for i in range(n):
        j0, j1 = max(i - 2, 0), min(i + 2, n - 1)
        tang[i] = math.atan2(P[j1, 1] - P[j0, 1], P[j1, 0] - P[j0, 0])
    g0 = clearance(dm, out, P[:, 0], P[:, 1], tang)
    pinned = (cum < START_KEEP_M) | (cum[-1] - cum < END_KEEP_M)
    tight_idx = np.nonzero((g0 < tight) & ~pinned)[0]
    if len(tight_idx) == 0:
        log('== %s: no narrow stretch' % name)
        return None
    free = np.zeros(n, bool)
    for i in tight_idx:
        free |= np.abs(cum - cum[i]) <= REACH_M
    free &= ~pinned
    for attempt in range(RETRIES):
        # DP over offsets; transition kk -> k gives a lean of atan(dk / ds)
        best = np.full(len(ks), NEG)
        best[k0] = 0.0
        back = [None]
        for i in range(1, n):
            ds = max(seg[i - 1], 1e-3)
            span = max(1, int(round(SLOPE * ds / GRID)))
            nb = np.full(len(ks), NEG)
            bp = np.zeros(len(ks), int)
            allowed = range(len(ks)) if free[i] else [k0]
            base_g = min(g0[i], tight)
            sn, cs = math.sin(tang[i]), math.cos(tang[i])
            for k in allowed:
                lo, hi = max(0, k - span), min(len(ks) - 1, k + span)
                prev = np.arange(lo, hi + 1)
                prev = prev[best[prev] > NEG / 2]
                if len(prev) == 0:
                    continue
                o = ks[k]
                x, y = P[i, 0] - o * sn, P[i, 1] + o * cs
                if free[i]:
                    lean = np.arctan2(ks[k] - ks[prev], ds)
                    gg = clearance(dm, out, np.full(len(prev), x), np.full(len(prev), y),
                                   tang[i] + lean)
                    sc = np.minimum(gg, tight) - 20.0 * np.maximum(0.0, base_g - gg) \
                        - 0.02 * abs(o) - 0.05 * (prev != k)
                else:
                    sc = np.full(len(prev), base_g)
                tot = best[prev] + sc
                j = int(np.argmax(tot))
                nb[k], bp[k] = tot[j], prev[j]
            best = nb
            back.append(bp)
        if best[k0] <= NEG / 2:
            log('== %s: no feasible lane - unchanged' % name)
            new = None
            break
        off = np.zeros(n)
        k = k0
        for i in range(n - 1, -1, -1):
            off[i] = ks[k]
            if i > 0:
                k = back[i][k]
        new = np.column_stack((P[:, 0] - off * np.sin(tang), P[:, 1] + off * np.cos(tang), P[:, 2]))
        moved = np.abs(off) >= 0.005
        if not moved.any():
            log('== %s: narrow, but no better lane' % name)
            new = None
            break
        ntang = tang.copy()
        for i in range(n):
            j0, j1 = max(i - YAW_WIN, 0), min(i + YAW_WIN, n - 1)
            if moved[j0:j1 + 1].any():
                ntang[i] = math.atan2(new[j1, 1] - new[j0, 1], new[j1, 0] - new[j0, 0])
                new[i, 2] = ntang[i]
        g1 = clearance(dm, out, new[:, 0], new[:, 1], ntang)
        worse = np.nonzero(g1 + dm[1] + 1e-6 < np.minimum(g0, tight))[0]
        if len(worse):
            for i in worse[:3]:
                log('   narrower at %.1f m (%.2f, %.2f): %.2f -> %.2f m'
                      % (cum[i], P[i, 0], P[i, 1], g0[i], g1[i]))
            if attempt == RETRIES - 1:
                log('== %s: %d poses would get narrower - unchanged' % (name, len(worse)))
                new = None
                break
            for i in worse:   # keep the original line around them, re-plan
                free &= np.abs(cum - cum[i]) > 1.0
            continue
        break
    if new is None:
        return None
    log('== %s: %d poses shifted, max %.2f m; narrowest %.2f -> %.2f m' % (
        name, int(moved.sum()), float(np.abs(off).max()),
        float(g0[~pinned].min()), float(g1[~pinned].min())))
    idx = np.nonzero(moved)[0]
    st = idx[0]
    for p_, q in zip(idx, list(idx[1:]) + [None]):
        if q is None or cum[q] - cum[p_] > 0.3:
            sl = slice(st, p_ + 1)
            big = off[sl][np.argmax(np.abs(off[sl]))]
            log('   %6.1f-%6.1f m at (%.2f, %.2f): shift %+.2f m, clearance %.2f -> %.2f m'
                  % (cum[st], cum[p_], P[st, 0], P[st, 1], big,
                     float(g0[sl].min()), float(g1[sl].min())))
            st = q
    return new


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('maps_dir')
    ap.add_argument('map_name')
    ap.add_argument('--tight', type=float, default=0.30)
    ap.add_argument('--max-shift', type=float, default=0.30)
    ap.add_argument('--from', dest='frm')
    ap.add_argument('--to')
    ap.add_argument('--apply', action='store_true')
    a = ap.parse_args()
    base = os.path.join(a.maps_dir, a.map_name)
    grids = [load_grid(base + '.yaml')]
    if os.path.exists(base + '_keepout.yaml'):
        grids.append(load_grid(base + '_keepout.yaml'))
    dm = distance_map(grids)
    out = outline(dm[1] / 2)
    path = base + '_routes.yaml'
    with open(path) as f:
        data = yaml.load(f, Loader=LOADER) or {}
    changed = 0
    for rt in data.get('routes', []):
        if a.frm and rt.get('from') != a.frm or a.to and rt.get('to') != a.to:
            continue
        P = np.array([p[:3] for p in rt.get('poses', [])], float)
        new = center_route(P, dm, out, a.tight, a.max_shift,
                           '%s -> %s' % (rt.get('from'), rt.get('to')))
        if new is None:
            continue
        rt['poses'] = [[round(float(p[0]), 3), round(float(p[1]), 3), round(float(p[2]), 4)]
                       for p in new]
        changed += 1
    if a.apply and changed:
        shutil.copy2(path, path + '.bak')
        tmp = path + '.tmp'
        with open(tmp, 'w') as f:
            yaml.safe_dump(data, f, allow_unicode=True, sort_keys=False)
        os.replace(tmp, path)
        print('written %s (%d routes changed, old file .bak)' % (path, changed))
    elif changed:
        print('dry run - add --apply to write')


if __name__ == '__main__':
    main()
