#!/usr/bin/env python3
"""GA (poly2 surrogate, motor objective) vs baseline a15_15_v4: motor charge and battery energy per trip.
Base / v0.30 = measured (mean, error bar = std of 3 runs). GA = surrogate prediction, error bar = 95% bootstrap CI
(runs resampled within each profile x payload cell, poly2 refitted, GA v kept fixed)."""
import os, sys, json, subprocess, numpy as np, pandas as pd, joblib, matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt
from sklearn.pipeline import make_pipeline
from sklearn.preprocessing import PolynomialFeatures, StandardScaler
from sklearn.linear_model import Ridge
ROOT = os.path.dirname(os.path.abspath(__file__))
d = pd.read_csv(os.path.join(ROOT, 'data', 'runs.csv')); d['mAh'] = d.motor_Ah * 1e3
KG = [0.0, 6.44, 12.88, 19.32, 25.77]
X = d[['accel', 'vmax', 'payload_kg']].values; Y = d[['mAh', 'energy_Wh']].values
mk = lambda: make_pipeline(StandardScaler(), PolynomialFeatures(2), Ridge(alpha=1e-3))
ga = {}
for kg in KG:
    o = json.loads(subprocess.check_output([sys.executable, os.path.join(ROOT, 'ga_speed_profile.py'), '--payload', str(kg),
                                            '--objective', 'motor', '--i-peak-max', '10.42', '--surrogate', 'poly', '--no-snap']))
    ga[kg] = (o['accel'], o['vmax'])
full = mk().fit(X, Y); rng = np.random.default_rng(0); boot = []
cells = [g.index.values for _, g in d.groupby(['profile', 'payload_kg'])]
for _ in range(500):
    idx = np.concatenate([rng.choice(c, len(c)) for c in cells])
    m = mk().fit(X[idx], Y[idx]); boot.append(m.predict([[*ga[k], k] for k in KG]))
boot = np.array(boot)
rows = []
for i, kg in enumerate(KG):
    r = dict(kg=kg, ga_v=ga[kg][1])
    for p, tag in (('a15_15_v4', 'base'), ('a15_15_v3', 'v030')):
        g = d[(d.profile == p) & np.isclose(d.payload_kg, kg)]
        r.update({f'{tag}_mAh': g.mAh.mean(), f'{tag}_mAh_sd': g.mAh.std(), f'{tag}_Wh': g.energy_Wh.mean(), f'{tag}_Wh_sd': g.energy_Wh.std()})
    pr = full.predict([[*ga[kg], kg]])[0]
    r.update(ga_mAh=pr[0], ga_Wh=pr[1], ga_mAh_lo=np.percentile(boot[:, i, 0], 2.5), ga_mAh_hi=np.percentile(boot[:, i, 0], 97.5),
             ga_Wh_lo=np.percentile(boot[:, i, 1], 2.5), ga_Wh_hi=np.percentile(boot[:, i, 1], 97.5))
    rows.append(r)
T = pd.DataFrame(rows)
for u in ('mAh', 'Wh'):
    T[f'ga_{u}_%'] = 100 * (T[f'ga_{u}'] / T[f'base_{u}'] - 1); T[f'v030_{u}_%'] = 100 * (T[f'v030_{u}'] / T[f'base_{u}'] - 1)
T.to_csv(os.path.join(ROOT, 'models', 'ga_vs_base.csv'), index=False)
print(T[['kg', 'ga_v', 'base_mAh', 'ga_mAh', 'ga_mAh_%', 'v030_mAh_%', 'base_Wh', 'ga_Wh', 'ga_Wh_%', 'v030_Wh_%']].round(3).to_string(index=False))

x = np.arange(len(KG))
C = dict(base='#d62728', ga='#9467bd', v030='#2ca02c')
SPEC = (('mAh', 'motor', 'Motor only: motor charge per trip [mAh]', (9.3, 12.2)),
        ('Wh', 'total', 'Total (battery, whole robot): energy per trip [Wh]', (0.85, 1.35)))
for (u, tag, lab, lim), simple in [(sp, sm) for sm in (False, True) for sp in SPEC]:
    fig, (a, b) = plt.subplots(2, 1, figsize=(10, 8), gridspec_kw=dict(height_ratios=[2, 1]))
    w = 0.36 if simple else 0.26; off = w / 2 if simple else w
    a.bar(x - off, T[f'base_{u}'], w, yerr=T[f'base_{u}_sd'], capsize=3, color=C['base'], label='baseline a0.15 v0.40 (measured)')
    a.bar(x + (off if simple else 0), T[f'ga_{u}'], w, yerr=[T[f'ga_{u}'] - T[f'ga_{u}_lo'], T[f'ga_{u}_hi'] - T[f'ga_{u}']], capsize=3, color=C['ga'],
          hatch='//', label='GA poly2, motor objective (predicted, 95% CI)')
    if not simple: a.bar(x + w, T[f'v030_{u}'], w, yerr=T[f'v030_{u}_sd'], capsize=3, color=C['v030'], alpha=.6, label='a0.15 v0.30 (measured, reference)')
    for i in range(len(KG)):
        a.text(x[i] + (off if simple else 0), T[f'ga_{u}_hi'][i] + (lim[1] - lim[0]) * .01, f'v{T.ga_v[i]:.2f}', ha='center', fontsize=8, color=C['ga'])
    a.set(ylabel=lab.split(': ')[1], ylim=lim, xticks=x, xticklabels=[f'{k_} kg' for k_ in KG], title=lab.split(': ')[0])
    a.grid(axis='y', alpha=.3); a.legend(fontsize=8, loc='upper left')
    b.axhline(0, color=C['base'], lw=1.5, label='baseline')
    b.plot(x, T[f'ga_{u}_%'], 'o-', color=C['ga'], label='GA vs baseline')
    if not simple: b.plot(x, T[f'v030_{u}_%'], 's--', color=C['v030'], alpha=.7, label='v0.30 vs baseline')
    for i in range(len(KG)):
        b.annotate(f'{T[f"ga_{u}_%"][i]:+.1f}%', (x[i], T[f'ga_{u}_%'][i]), textcoords='offset points', xytext=(0, 6), ha='center', fontsize=8, color=C['ga'])
    b.set(ylabel='difference vs baseline [%]', xticks=x, xticklabels=[f'{k_} kg' for k_ in KG]); b.grid(alpha=.3); b.legend(fontsize=8)
    fig.suptitle(f'GA vs baseline a15_15_v4 - {lab.split(":")[0]} (start -> Station_b, 14.3 m), lower is better')
    if simple:
        err = '+-1 %' if u == 'mAh' else '+-4 %'
        fig.text(0.5, 0.005, f'GA = surrogate prediction (poly2), v0.33-0.37 not run on the robot. Surrogate error at tested profiles ~{err}.',
                 ha='center', fontsize=8, style='italic')
    fig.tight_layout(rect=(0, 0.02, 1, 1)); out = os.path.join(ROOT, 'figures', f'ga_vs_base_{tag}{"_simple" if simple else ""}.png'); fig.savefig(out, dpi=110); print(out)
