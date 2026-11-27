#!/usr/bin/env python3
"""Per-segment speed profile by GA along the recorded route (start -> Station_b, ~14.3 m straight).
1. Every bag is cut into segments by distance travelled from motion start (--seg-len m).
   Last part (Station speed zone, v capped by the speed mask) is kept fixed, not optimised.
2. Per segment and KPI a surrogate is fitted on the a0.15 bags:
      KPI = b0 + b1 v + b2 v^2 + b3 kg + b4 v*kg     (v = tested vmax 0.2/0.3/0.4)
3. GA chooses v for each segment (continuous in [0.2, 0.4], or --tested to use 0.2/0.3/0.4 only).
   Speed changes between segments add ramp time (v2-v1)^2 / (2 a v2) (energy of the ramp ignored).
   Cost = objective (motor mAh | battery mWh | time) with optional --t-max trip-time constraint.
   With --tested the exhaustive optimum is also computed to check the GA."""
import os, argparse, itertools, json, numpy as np, pandas as pd
ROOT = os.path.dirname(os.path.abspath(__file__)); DT = 0.05
KPIS = ['t', 'mAh', 'mWh', 'i_p99']

def segment_table(seg_len, zone_start):
    runs = pd.read_csv(os.path.join(ROOT, 'data', 'runs.csv'))
    runs = runs[np.isclose(runs.accel, 0.15)]
    edges = list(np.arange(0, zone_start - 1e-6, seg_len)) + [zone_start, 1e9]
    rows = []
    for _, r in runs.iterrows():
        d = np.load(os.path.join(ROOT, 'data', 'ts', r.bag + '.npz'))
        k0 = np.argmax(np.abs(d['v']) > 0.03)
        s = np.cumsum(np.abs(d['v'])) * DT; s -= s[k0]
        im = 0.5 * (np.abs(d['i_l']) + np.abs(d['i_r']))
        pw = d['bat_v'] * np.clip(d['bat_i'], 0, None)
        idx = np.arange(len(s))
        for i in range(len(edges) - 1):
            m = (s >= edges[i]) & (s < edges[i + 1]) & (idx >= k0)
            if m.sum() < 2: continue
            if i == len(edges) - 2:   # stop counting after the robot has stopped for good
                last = np.where(np.abs(d['v']) > 0.01)[0][-1]; m &= idx <= last
            rows.append(dict(seg=i, kg=r.payload_kg, v=r.vmax, t=m.sum() * DT, mAh=im[m].sum() * DT / 3.6,
                             mWh=pw[m].sum() * DT / 3.6, i_p99=np.percentile(im[m], 99)))
    return pd.DataFrame(rows), edges

def fit(S):
    """coef[seg][kpi] for KPI = b . [1, v, v^2, kg, v*kg]"""
    C = {}
    for seg, g in S.groupby('seg'):
        A = np.column_stack([np.ones(len(g)), g.v, g.v ** 2, g.kg, g.v * g.kg])
        C[seg] = {k: np.linalg.lstsq(A, g[k].values, rcond=None)[0] for k in KPIS}
    return C

def predict(C, seg, v, kg, k):
    return C[seg][k] @ np.array([1, v, v * v, kg, v * kg])

def evaluate(C, V, kg, nseg, accel, v_zone):
    tot = {k: 0.0 for k in KPIS}; tot['i_p99'] = 0.0
    for i in range(nseg):
        for k in ('t', 'mAh', 'mWh'): tot[k] += predict(C, i, V[i], kg, k)
        tot['i_p99'] = max(tot['i_p99'], predict(C, i, V[i], kg, 'i_p99'))
        if i > 0 and V[i] > V[i - 1]:                     # extra ramp time (speed-up); slow-down ~free on time
            tot['t'] += (V[i] - V[i - 1]) ** 2 / (2 * accel * V[i])
    for k in ('t', 'mAh', 'mWh'): tot[k] += predict(C, nseg, v_zone, kg, k)   # fixed station zone
    return tot

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--payload', type=float, required=True)
    ap.add_argument('--objective', choices=['mAh', 'mWh', 't'], default='mAh', help='mAh motor | mWh battery | t time')
    ap.add_argument('--t-max', type=float, default=None, help='max trip time [s] (constraint)')
    ap.add_argument('--seg-len', type=float, default=2.0); ap.add_argument('--zone-start', type=float, default=13.0)
    ap.add_argument('--i-peak-max', type=float, default=10.42)
    ap.add_argument('--tested', action='store_true', help='only tested speeds 0.2/0.3/0.4 per segment')
    ap.add_argument('--pop', type=int, default=60); ap.add_argument('--gens', type=int, default=150)
    a = ap.parse_args()
    S, edges = segment_table(a.seg_len, a.zone_start)
    C = fit(S); nseg = len(edges) - 2
    lo, hi, accel = 0.2, 0.4, 0.15
    grid = np.array([0.2, 0.3, 0.4])
    def cost(V):
        if a.tested: V = grid[np.abs(V[:, None] - grid).argmin(1)]
        e = evaluate(C, V, a.payload, nseg, accel, 0.2)
        c = e[a.objective]
        if a.t_max: c += 1e3 * max(0, e['t'] - a.t_max)
        if e['i_p99'] > a.i_peak_max: c += 1e3
        return c, e, V
    rng = np.random.default_rng(0)
    P = rng.uniform(lo, hi, (a.pop, nseg)); best, bc = None, np.inf
    for _ in range(a.gens):
        c = np.array([cost(p)[0] for p in P])
        i = c.argmin()
        if c[i] < bc: bc, best = c[i], P[i].copy()
        x, y = rng.integers(0, a.pop, (2, a.pop)); par = P[np.where(c[x] < c[y], x, y)]
        al = rng.uniform(-0.3, 1.3, par.shape); ch = par + al * (par[rng.permutation(a.pop)] - par)
        ch += (rng.random(ch.shape) < 0.15) * rng.normal(0, 0.03, ch.shape)
        P = np.clip(ch, lo, hi); P[0] = best
    c, e, V = cost(best)
    res = dict(payload_kg=a.payload, objective=a.objective, t_max=a.t_max,
               segments=[f'{edges[i]:.0f}-{edges[i+1]:.0f} m' for i in range(nseg)] + [f'{edges[nseg]:.0f} m-stop (zone, fixed)'],
               v=[round(float(x), 3) for x in V] + ['zone'],
               predicted={k: round(float(v), 3) for k, v in e.items()})
    for vu in (0.2, 0.3, 0.4):
        u = evaluate(C, np.full(nseg, vu), a.payload, nseg, accel, 0.2)
        res[f'uniform_v{vu}'] = {k: round(float(v), 3) for k, v in u.items()}
    if a.tested:
        bx = min(itertools.product(grid, repeat=nseg), key=lambda V: cost(np.array(V))[0])
        res['exhaustive_v'] = list(bx); res['ga_matches_exhaustive'] = bool(np.allclose(bx, V))
    print(json.dumps(res, indent=1))

if __name__ == '__main__':
    main()
