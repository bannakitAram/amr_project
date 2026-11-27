#!/usr/bin/env python3
"""Speed profile + energy: baseline a15_15_v4 vs station_ga.py results (model, not run on the robot).
GA parameters are the station_ga.py optima (all at bounds, identical for every payload):
  GA zone<=0.08: cruise 0.40, zone 0.08, creep 0.06 m/s over 0.10 m, decel 0.25
  GA zone<=0.12: cruise 0.40, zone 0.12, creep 0.06 m/s over 0.10 m, decel 0.25   (needs risk assessment)"""
import os, numpy as np, pandas as pd, matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt
import station_ga as S
ROOT = os.path.dirname(os.path.abspath(__file__))
D, LZ, N = 14.28, 1.03, 7
CASES = {'baseline a15_15_v4 (model)': ([0.4] * N, 0.08, 0.04, 0.15, 0.13, '#d62728', '-'),
         'GA, zone <= 0.08 (safety unchanged)': ([0.4] * N, 0.08, 0.06, 0.10, 0.25, '#9467bd', '--'),
         'GA, zone <= 0.12 (needs risk assessment)': ([0.4] * N, 0.12, 0.06, 0.10, 0.25, '#ff7f0e', ':')}
KG = [0.0, 6.44, 12.88, 19.32, 25.77]
M = S.fit_energy_model(); runs = pd.read_csv(os.path.join(ROOT, 'data', 'runs.csv'))
prof = {k: S.build_profile(c[0], c[1], c[2], c[3], c[4], D, LZ) for k, c in CASES.items()}
E = {k: [S.energy(M, p[3], p[4], p[5], kg) for kg in KG] for k, p in prof.items()}
meas = [runs[(runs.profile == 'a15_15_v4') & np.isclose(runs.payload_kg, kg)] for kg in KG]

fig = plt.figure(figsize=(15, 11)); gs = fig.add_gridspec(3, 2, height_ratios=[1, 1, 1.1])
a1, a2 = fig.add_subplot(gs[0, 0]), fig.add_subplot(gs[0, 1]); a3 = fig.add_subplot(gs[1, :])
a4, a5 = fig.add_subplot(gs[2, 0]), fig.add_subplot(gs[2, 1])
kg_show = 25.77
for k, (s, v, t, vm, dt, a) in prof.items():
    col, ls = CASES[k][5], CASES[k][6]; e = S.energy(M, vm, dt, a, kg_show)
    lab = f'{k}: {e["t"]:.1f} s'
    a1.plot(t, v, color=col, ls=ls, lw=2, label=lab); a2.plot(s, v, color=col, ls=ls, lw=2, label=lab)
    X = S.F(vm, a, kg_show); a3.plot(t[1:], np.cumsum(X @ M['P'] * dt) / 3.6, color=col, ls=ls, lw=2,
                                     label=f'{k}: {e["Wh"]*1000:.0f} mWh')
a2.axvspan(D - LZ, D, color='#eee', zorder=0); a2.text(D - LZ / 2, 0.42, 'station zone', ha='center', fontsize=8)
a2.set_xlim(11.5, D + 0.1); a2.set_title('distance view, zoom on station approach (last 2.8 m)')
a1.set_title('speed vs time'); a3.set_title(f'cumulative battery energy, payload {kg_show} kg')
for ax_, xl in ((a1, 'time [s]'), (a2, 'distance [m]')): ax_.set(xlabel=xl, ylabel='v [m/s]', ylim=(0, 0.45)); ax_.grid(alpha=.3); ax_.legend(fontsize=8)
a3.set(xlabel='time [s]', ylabel='energy [mWh]'); a3.grid(alpha=.3); a3.legend(fontsize=8)
x = np.arange(len(KG)); w = 0.2
for ax_, key, sc, lab in ((a4, 'Wh', 1, 'battery energy per trip [Wh] (total)'), (a5, 'mAh', 1, 'motor charge per trip [mAh] (motor only)')):
    mcol = 'energy_Wh' if key == 'Wh' else 'motor_Ah'; msc = 1 if key == 'Wh' else 1e3
    # measured moving-part energy is not stored per run; show measured trip value as reference marker
    for j, (k, c) in enumerate(CASES.items()):
        vals = [e[key] for e in E[k]]; base = [e[key] for e in E['baseline a15_15_v4 (model)']]
        b = ax_.bar(x + (j - 1) * w, vals, w, color=c[5], hatch='//' if j else None, alpha=.85, label=k)
        if j:
            for xi, (vv, bb) in enumerate(zip(vals, base)):
                ax_.text(x[xi] + (j - 1) * w, vv, f'{100*(vv/bb-1):+.1f}%', ha='center', va='bottom', fontsize=7, rotation=90)
    ax_.set(xticks=x, xticklabels=[f'{k} kg' for k in KG], ylabel=lab); ax_.grid(axis='y', alpha=.3)
lo4 = min(min(e['Wh'] for e in E[k]) for k in E); hi4 = max(max(e['Wh'] for e in E[k]) for k in E)
a4.set_ylim(lo4 * 0.9, hi4 * 1.06); lo5 = min(min(e['mAh'] for e in E[k]) for k in E); hi5 = max(max(e['mAh'] for e in E[k]) for k in E)
a5.set_ylim(lo5 * 0.95, hi5 * 1.03); a4.legend(fontsize=7, loc='upper left')
fig.suptitle('Per-segment + station-approach GA vs baseline a15_15_v4 (start -> Station_b, 14.3 m)\n'
             'all curves/bars = energy model (validated on measured runs: Wh +-1.2 %, mAh +-2.3 %), GA not run on the robot', fontsize=11)
fig.tight_layout(); out = os.path.join(ROOT, 'figures', 'station_ga_profile_energy.png'); fig.savefig(out, dpi=105); print(out)
rows = [dict(case=k, kg=kg, **e) for k in E for kg, e in zip(KG, E[k])]
pd.DataFrame(rows).to_csv(os.path.join(ROOT, 'models', 'station_ga_energy.csv'), index=False)
print(pd.DataFrame(rows).pivot_table(index='kg', columns='case', values=['t', 'Wh', 'mAh']).round(3).to_string())
