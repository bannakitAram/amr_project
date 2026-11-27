#!/usr/bin/env python3
"""Real robot: compare only the part AFTER the Smart/GA switch.
For each GA run, s_switch = distance driven when the GA profile was applied (20 s after motion start).
GA and baselines are then integrated over the SAME distance window [s_switch, stop] - a time cut at 20 s
would compare different stretches of route (v0.40 is ~4 m further than v0.20 after 20 s)."""
import os, numpy as np, pandas as pd, matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT = os.path.dirname(os.path.abspath(__file__)); DT = 0.05; T_SW = 20.0

def series(bag):
    d = np.load(os.path.join(ROOT, 'data', 'ts', bag + '.npz')); v = np.abs(d['v'])
    if not (v > 0.03).any(): return None
    k0 = np.argmax(v > 0.03); k1 = np.where(v > 0.005)[0][-1] + 1; sl = slice(k0, k1)
    return dict(v=v[sl], s=np.cumsum(v[sl]) * DT, im=0.5 * (np.abs(d['i_l']) + np.abs(d['i_r']))[sl],
                p=(d['bat_v'] * np.clip(d['bat_i'], 0, None))[sl])

def window(x, s0):
    m = x['s'] >= s0
    return dict(t=m.sum() * DT, mAh=x['im'][m].sum() * DT / 3.6, Wh=x['p'][m].sum() * DT / 3600, dist=x['s'][-1] - s0)

ga = pd.read_csv(os.path.join(ROOT, 'data', 'ga_runs.csv')); runs = pd.read_csv(os.path.join(ROOT, 'data', 'runs.csv'))
ga = ga[ga.smart_started & ga.ga_vmax.notna()]
rows = []
for _, r in ga.iterrows():
    x = series(r.bag); s0 = x['s'][int(T_SW / DT)]
    rows.append(dict(case='GA (Smart)', kg=r.payload_kg, s0=s0, vga=r.ga_vmax, **window(x, s0)))
    for p, name in (('a15_15_v4', 'baseline a0.15 v0.40'), ('a15_15_v2', 'current a0.15 v0.20')):
        for _, b in runs[(runs.profile == p) & np.isclose(runs.payload_kg, r.payload_kg)].iterrows():
            rows.append(dict(case=name, kg=r.payload_kg, s0=s0, **window(series(b.bag), s0)))
T = pd.DataFrame(rows)
S = T.groupby(['case', 'kg']).agg(s0=('s0', 'mean'), dist=('dist', 'mean'), t=('t', 'mean'), mAh=('mAh', 'mean'),
                                  mAh_sd=('mAh', 'std'), Wh=('Wh', 'mean'), Wh_sd=('Wh', 'std'), n=('t', 'size')).reset_index()
S.to_csv(os.path.join(ROOT, 'models', 'ga_real_after_switch.csv'), index=False)
base = S[S.case == 'baseline a0.15 v0.40'].set_index('kg')
for c in ('Wh', 'mAh', 't'):
    S[c + '_vs_v040_%'] = [100 * (r[c] / base.loc[r.kg, c] - 1) if r.kg in base.index else np.nan for _, r in S.iterrows()]
pd.set_option('display.width', 220)
print(S[['case', 'kg', 's0', 'dist', 't', 'Wh', 'Wh_sd', 'Wh_vs_v040_%', 'mAh', 'mAh_sd', 'mAh_vs_v040_%']].round(3).to_string(index=False))

KG = sorted(S.kg.unique()); x = np.arange(len(KG)); w = 0.27
CASES = [('current a0.15 v0.20', '#1f77b4'), ('baseline a0.15 v0.40', '#d62728'), ('GA (Smart)', '#9467bd')]
fig, ax = plt.subplots(1, 3, figsize=(17, 5))
for k, (key, lab) in enumerate((('Wh', 'Total (battery) energy after switch [Wh]'), ('mAh', 'Motor charge after switch [mAh]'), ('t', 'Time after switch [s]'))):
    a = ax[k]
    for j, (c, col) in enumerate(CASES):
        sub = S[S.case == c].set_index('kg').reindex(KG)
        a.bar(x + (j - 1) * w, sub[key], w, yerr=sub.get(key + '_sd', pd.Series(0, index=sub.index)).fillna(0), capsize=2,
              color=col, hatch='//' if c.startswith('GA') else None, label=c)
        if c.startswith('GA'):
            for xi, (vv, pc) in enumerate(zip(sub[key], sub[key + '_vs_v040_%'])):
                if np.isfinite(pc): a.text(x[xi] + (j - 1) * w, vv, f'{pc:+.0f}%', ha='center', va='bottom', fontsize=7)
    a.set(title=lab, xticks=x, xticklabels=[f'{v:g} kg' for v in KG]); a.grid(axis='y', alpha=.3)
    a.set_ylim(S[key].min() * 0.85, S[key].max() * 1.08)
ax[0].legend(fontsize=8, loc='upper left')
fig.suptitle('Real robot, AFTER the GA switch only: same distance window [s at 20 s of the GA run (~4 m) -> stop] for every profile\n'
             '% = GA vs baseline a0.15 v0.40;  GA 1 run, baselines mean of 3 (error bar = std)')
fig.tight_layout(); out = os.path.join(ROOT, 'figures', 'ga_real_after_switch.png'); fig.savefig(out, dpi=100); print(out)
