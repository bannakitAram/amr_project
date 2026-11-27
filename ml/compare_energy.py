#!/usr/bin/env python3
"""Energy per trip: fixed profiles (measured, mean +- std of 3 runs) vs GA profiles found with the smooth
poly surrogate (predicted, 0.01 resolution). Bias check: the same surrogate evaluated at the tested profiles
vs their measured mean is reported (mean +- sd)."""
import os, sys, json, subprocess, numpy as np, pandas as pd, joblib, matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT = os.path.dirname(os.path.abspath(__file__)); PY = sys.executable
runs = pd.read_csv(os.path.join(ROOT, 'data', 'runs.csv'))
poly = joblib.load(os.path.join(ROOT, 'models', 'fitness_poly.joblib'))
YC = poly['y_cols']
KG = [0.0, 6.44, 12.88, 19.32, 25.77]
FIXED = {'a15_15_v2': 'a0.15 v0.20 (current)', 'a5_5_v4': 'a0.50 v0.40', 'a15_15_v3': 'a0.15 v0.30', 'a15_15_v4': 'a0.15 v0.40 (base)'}
def pred(a, v, kg): return dict(zip(YC, poly['model'].predict([[a, v, kg]])[0]))
rows = []
for kg in KG:
    for p, name in FIXED.items():
        g = runs[(runs.profile == p) & np.isclose(runs.payload_kg, kg)]
        a, v = g.accel.iloc[0], g.vmax.iloc[0]; P = pred(a, v, kg)
        rows.append(dict(kg=kg, name=name, kind='measured', a=a, v=v, t=g.nav_time_s.mean(),
                         mAh=g.motor_Ah.mean() * 1e3, mAh_sd=g.motor_Ah.std() * 1e3, Wh=g.energy_Wh.mean(), Wh_sd=g.energy_Wh.std(),
                         bias_mAh=P['motor_Ah'] * 1e3 - g.motor_Ah.mean() * 1e3, bias_Wh=P['energy_Wh'] - g.energy_Wh.mean()))
    for obj, name in (('motor', 'GA poly, motor objective'), ('battery', 'GA poly, battery objective')):
        o = json.loads(subprocess.check_output([PY, os.path.join(ROOT, 'ga_speed_profile.py'), '--payload', str(kg), '--objective', obj,
                                                '--i-peak-max', '10.42', '--surrogate', 'poly', '--no-snap']))
        P = pred(o['accel'], o['vmax'], kg)          # full precision (GA JSON is rounded)
        rows.append(dict(kg=kg, name=name, kind='predicted', a=o['accel'], v=o['vmax'], t=P['nav_time_s'],
                         mAh=P['motor_Ah'] * 1e3, Wh=P['energy_Wh']))
T = pd.DataFrame(rows)
meas = T[T.kind == 'measured']
# GA rows: raw surrogate prediction (bias at tested profiles is ~0 on average, reported below);
# per-point correction with the nearest tested profile was dropped: it imports single-run outliers.
T['mAh_corr'] = T.mAh; T['Wh_corr'] = T.Wh
base = T[T.name == FIXED['a15_15_v4']].set_index('kg')
for c in ('mAh_corr', 'Wh_corr'):
    T[c.split('_')[0] + '_vs_base_%'] = [100 * (r[c] / base.loc[r.kg, c] - 1) for _, r in T.iterrows()]
T.to_csv(os.path.join(ROOT, 'models', 'energy_comparison.csv'), index=False)
pd.set_option('display.width', 250)
print(T[['kg', 'name', 'kind', 'a', 'v', 't', 'mAh', 'mAh_sd', 'mAh_corr', 'Wh', 'Wh_sd', 'Wh_corr', 'mAh_vs_base_%', 'Wh_vs_base_%']].round(3).to_string(index=False))
print('\nsurrogate bias at tested profiles (pred - measured): mAh %.3f +- %.3f, Wh %.3f +- %.3f'
      % (meas.bias_mAh.mean(), meas.bias_mAh.std(), meas.bias_Wh.mean(), meas.bias_Wh.std()))
names = list(dict.fromkeys(T.name)); x = np.arange(len(KG)); w = 0.8 / len(names)
fig, ax = plt.subplots(2, 1, figsize=(13, 9))
for j, n in enumerate(names):
    g = T[T.name == n].set_index('kg').loc[KG]
    lab = n + ('' if g.kind.iloc[0] == 'measured' else '  [' + ', '.join(f'v{v:.2f}' for v in g.v) + ']')
    for k, (c, sd, unit) in enumerate((('mAh_corr', 'mAh_sd', 'mAh'), ('Wh_corr', 'Wh_sd', 'Wh'))):
        b = ax[k].bar(x + (j - len(names) / 2 + .5) * w, g[c], w, yerr=g.get(sd, pd.Series(0, index=g.index)).fillna(0),
                      capsize=2, label=lab, hatch='//' if g.kind.iloc[0] == 'predicted' else None, alpha=.85)
        pct = g[c.split('_')[0] + '_vs_base_%']
        for xi, (hv, pv) in enumerate(zip(g[c], pct)):
            ax[k].text(x[xi] + (j - len(names) / 2 + .5) * w, hv, f'{pv:+.0f}%', ha='center', va='bottom', fontsize=6, rotation=90)
ax[0].set(ylabel='motor charge per trip [mAh]', ylim=(9, 13.3), xticks=x, xticklabels=[f'{k} kg' for k in KG])
ax[1].set(ylabel='battery energy per trip [Wh]', ylim=(0.8, 1.65), xticks=x, xticklabels=[f'{k} kg' for k in KG])
for a_ in ax: a_.grid(axis='y', alpha=.3)
ax[0].legend(fontsize=7, ncol=2, loc='upper left')
fig.suptitle('Energy per trip, start -> Station_b (14.3 m).  Solid = measured (std of 3 runs), hatched = GA (poly surrogate prediction).  % vs base a0.15 v0.40')
fig.tight_layout(); fig.savefig(os.path.join(ROOT, 'figures', 'energy_comparison.png'), dpi=110)
