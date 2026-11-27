#!/usr/bin/env python3
"""GA over the whole route: per-segment cruise speed + station approach (start -> Station_b, ~14.3 m).
Energy model (from all a0.15 bags, 20 Hz samples, straight driving |w|<0.15):
    y = b . [1, v, v^2, kg, v*kg, a+, a+*kg, a-, a-*kg]      y = motor current [A] or battery power [W]
  -> any v(s) profile can be simulated (incl. station-zone speeds 0.04-0.08 that are in the data).
Profile = target speed per region, then forward/backward passes limit accel (0.15) and decel (variable):
    [0, D-L_zone): cruise segments of --seg-len m, v_i in [vmin, vmax]
    [D-L_zone, D-L_creep): station zone speed v_z   (default fixed max 0.08 = current speed mask)
    [D-L_creep, D):        creep speed v_c, then stop
GA variables: v_1..v_n, v_z, v_c, L_creep, a_dec.  Baseline (a15_15_v4) is simulated with the same model
and also shown measured, so the comparison is model-consistent."""
import os, argparse, json, numpy as np, pandas as pd
ROOT = os.path.dirname(os.path.abspath(__file__)); DT = 0.05
F = lambda v, a, kg: np.column_stack([np.ones_like(v), v, v * v, np.full_like(v, kg), v * kg,
                                      np.clip(a, 0, None), np.clip(a, 0, None) * kg, np.clip(a, None, 0), np.clip(a, None, 0) * kg])

def fit_energy_model():
    runs = pd.read_csv(os.path.join(ROOT, 'data', 'runs.csv'))
    X, Im, Pb = [], [], []
    for _, r in runs[np.isclose(runs.accel, 0.15)].iterrows():
        d = np.load(os.path.join(ROOT, 'data', 'ts', r.bag + '.npz'))
        v = np.abs(d['v']); m = (v > 0.015) & (np.abs(d['w']) < 0.15)
        X.append(F(v[m], d['acc'][m] * np.sign(d['v'][m] + 1e-9), r.payload_kg))
        Im.append((0.5 * (np.abs(d['i_l']) + np.abs(d['i_r'])))[m]); Pb.append((d['bat_v'] * np.clip(d['bat_i'], 0, None))[m])
    X = np.vstack(X)
    return {k: np.linalg.lstsq(X, np.concatenate(y), rcond=None)[0] for k, y in (('I', Im), ('P', Pb))}

def build_profile(cruise, v_z, v_c, L_creep, a_dec, D, L_zone, a_acc=0.15, ds=0.005):
    s = np.arange(0, D + ds / 2, ds); n = len(cruise); s_z = D - L_zone
    edges = np.linspace(0, s_z, n + 1)
    vt = np.where(s < s_z, np.asarray(cruise)[np.clip(np.searchsorted(edges, s, 'right') - 1, 0, n - 1)],
                  np.where(s < D - L_creep, v_z, v_c))
    v = vt.copy(); v[-1] = 0.0
    v[0] = 0.03
    for i in range(1, len(s)):        # accel limit (forward)
        v[i] = min(v[i], np.sqrt(v[i - 1] ** 2 + 2 * a_acc * ds))
    for i in range(len(s) - 2, -1, -1):   # decel limit (backward)
        v[i] = min(v[i], np.sqrt(v[i + 1] ** 2 + 2 * a_dec * ds))
    vm = np.maximum(0.5 * (v[1:] + v[:-1]), 1e-3)
    dt = ds / vm; t = np.concatenate([[0], np.cumsum(dt)])
    a = np.diff(v) / dt
    return s, v, t, vm, dt, a

def energy(M, vm, dt, a, kg):
    X = F(vm, a, kg)
    return dict(t=float(dt.sum()), mAh=float(np.sum(X @ M['I'] * dt) / 3.6), Wh=float(np.sum(X @ M['P'] * dt) / 3600))

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--payload', type=float, required=True)
    ap.add_argument('--objective', choices=['mAh', 'Wh'], default='Wh')
    ap.add_argument('--seg-len', type=float, default=2.0)
    ap.add_argument('--D', type=float, default=14.28, help='route length [m] (measured)')
    ap.add_argument('--L-zone', type=float, default=1.03, help='station zone length [m] (measured, kept fixed)')
    ap.add_argument('--vmin', type=float, default=0.2); ap.add_argument('--vmax', type=float, default=0.4)
    ap.add_argument('--zone-vmax', type=float, default=0.08, help='max zone speed; 0.08 = current speed mask (safety)')
    ap.add_argument('--creep-v', nargs=2, type=float, default=[0.03, 0.06])
    ap.add_argument('--creep-L', nargs=2, type=float, default=[0.10, 0.30], help='creep length bounds [m]')
    ap.add_argument('--decel', nargs=2, type=float, default=[0.10, 0.25], help='decel bounds [m/s^2] (measured 0.10-0.39)')
    ap.add_argument('--pop', type=int, default=50); ap.add_argument('--gens', type=int, default=120)
    ap.add_argument('--plot', action='store_true')
    a = ap.parse_args()
    M = fit_energy_model()
    n = int(round((a.D - a.L_zone) / a.seg_len))
    lo = np.array([a.vmin] * n + [0.04, a.creep_v[0], a.creep_L[0], a.decel[0]])
    hi = np.array([a.vmax] * n + [a.zone_vmax, a.creep_v[1], a.creep_L[1], a.decel[1]])
    def sim(x):
        p = build_profile(x[:n], x[n], min(x[n + 1], x[n]), x[n + 2], x[n + 3], a.D, a.L_zone)
        return p, energy(M, p[3], p[4], p[5], a.payload)
    def cost(x): return sim(x)[1][a.objective]
    rng = np.random.default_rng(0)
    P = rng.uniform(lo, hi, (a.pop, len(lo))); best, bc = None, np.inf
    for _ in range(a.gens):
        c = np.array([cost(p) for p in P]); i = c.argmin()
        if c[i] < bc: bc, best = c[i], P[i].copy()
        x, y = rng.integers(0, a.pop, (2, a.pop)); par = P[np.where(c[x] < c[y], x, y)]
        al = rng.uniform(-0.3, 1.3, par.shape); ch = par + al * (par[rng.permutation(a.pop)] - par)
        ch += (rng.random(ch.shape) < 0.15) * rng.normal(0, 0.05 * (hi - lo), ch.shape)
        P = np.clip(ch, lo, hi); P[0] = best
    best = np.round(best, 3)
    base_x = np.array([0.4] * n + [0.08, 0.04, 0.15, 0.13])      # a15_15_v4 as configured (measured approach)
    (pb, eb), (pg, eg) = sim(base_x), sim(best)
    res = dict(payload_kg=a.payload, objective=a.objective,
               ga=dict(cruise=list(best[:n]), zone_v=best[n], creep_v=min(best[n + 1], best[n]), creep_L=best[n + 2], decel=best[n + 3], **{k: round(v, 3) for k, v in eg.items()}),
               baseline_model={k: round(v, 3) for k, v in eb.items()},
               diff_pct={k: round(100 * (eg[k] / eb[k] - 1), 1) for k in eg})
    print(json.dumps(res, indent=1, default=float))
    if a.plot:
        import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
        fig, ax = plt.subplots(1, 2, figsize=(14, 4.2))
        for (s, v, t, *_), lab, col, ls in ((pb, f'baseline a15_15_v4 (model) {eb["t"]:.0f}s {eb["Wh"]:.3f}Wh {eb["mAh"]:.2f}mAh', '#d62728', '-'),
                                           (pg, f'GA {eg["t"]:.0f}s {eg["Wh"]:.3f}Wh {eg["mAh"]:.2f}mAh', '#9467bd', '--')):
            ax[0].plot(t, v, color=col, ls=ls, lw=2, label=lab); ax[1].plot(s, v, color=col, ls=ls, lw=2, label=lab)
        ax[1].axvspan(a.D - a.L_zone, a.D, color='#eee', zorder=0)
        for x_, xl in zip(ax, ('time [s]', 'distance [m]')): x_.set(xlabel=xl, ylabel='v [m/s]', ylim=(0, 0.45)); x_.grid(alpha=.3); x_.legend(fontsize=8)
        fig.suptitle(f'Per-segment + station-approach GA, payload {a.payload} kg, objective {a.objective} (model, not run on robot)')
        fig.tight_layout(); out = os.path.join(ROOT, 'figures', f'station_ga_{a.objective}_{a.payload:g}kg.png'); fig.savefig(out, dpi=110); print(out)

if __name__ == '__main__':
    main()
