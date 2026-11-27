#!/usr/bin/env python3
"""Real-robot comparison: Smart/GA runs (2026-11-27, ⚡ button) vs baselines recorded 11-25/26
(a15_15_v4 = best fixed profile, a15_15_v2 = current/weighing profile). Same route start -> Station_b.
Energy is integrated over the MOTION window only (first |v|>0.03 to last |v|>0.005), so idle time
before/after the trip inside a bag does not count."""
import os, numpy as np, pandas as pd, matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT = os.path.dirname(os.path.abspath(__file__)); DT = 0.05

def motion(bag):
    d = np.load(os.path.join(ROOT, 'data', 'ts', bag + '.npz')); v = np.abs(d['v'])
    if not (v > 0.03).any(): return None
    k0 = np.argmax(v > 0.03); k1 = np.where(v > 0.005)[0][-1] + 1; sl = slice(k0, k1)
    im = 0.5 * (np.abs(d['i_l']) + np.abs(d['i_r']))[sl]
    return dict(t=(k1 - k0) * DT, mAh=im.sum() * DT / 3.6, Wh=np.sum((d['bat_v'] * np.clip(d['bat_i'], 0, None))[sl]) * DT / 3600,
                trace_t=np.arange(k1 - k0) * DT, trace_v=v[sl], dist=v[sl].sum() * DT)

ga = pd.read_csv(os.path.join(ROOT, 'data', 'ga_runs.csv')); runs = pd.read_csv(os.path.join(ROOT, 'data', 'runs.csv'))
rows = []
for _, r in ga.iterrows():
    m = motion(r.bag)
    if m is None or not r.smart_started: continue
    rows.append(dict(case='GA (Smart)', kg=r.payload_kg, bag=r.bag, est=r.est_kg, vga=r.ga_vmax, **{k: m[k] for k in ('t', 'mAh', 'Wh', 'dist')}))
for p, name in (('a15_15_v4', 'baseline a0.15 v0.40'), ('a15_15_v2', 'current a0.15 v0.20')):
    for _, r in runs[runs.profile == p].iterrows():
        m = motion(r.bag)
        if m: rows.append(dict(case=name, kg=r.payload_kg, bag=r.bag, **{k: m[k] for k in ('t', 'mAh', 'Wh', 'dist')}))
T = pd.DataFrame(rows)
KG = sorted(T[T.case == 'GA (Smart)'].kg.unique())
S = T[T.kg.isin(KG)].groupby(['case', 'kg']).agg(t=('t', 'mean'), mAh=('mAh', 'mean'), mAh_sd=('mAh', 'std'),
                                                Wh=('Wh', 'mean'), Wh_sd=('Wh', 'std'), n=('t', 'size')).reset_index()
S.to_csv(os.path.join(ROOT, 'models', 'ga_real_vs_base.csv'), index=False)
piv = S.pivot(index='kg', columns='case', values=['t', 'mAh', 'Wh']).round(3)
print(piv.to_string())
g = T[T.case == 'GA (Smart)'].set_index('kg')
print(g[['bag', 'est', 'vga']].to_string())

CASES = [('current a0.15 v0.20', '#1f77b4'), ('baseline a0.15 v0.40', '#d62728'), ('GA (Smart)', '#9467bd')]
fig = plt.figure(figsize=(15, 12)); gs = fig.add_gridspec(3, 2, height_ratios=[1, 1, 1.1])
x = np.arange(len(KG)); w = 0.27
for k, (key, lab) in enumerate((('Wh', 'Total (battery) energy per trip [Wh]'), ('mAh', 'Motor charge per trip [mAh]'))):
    a = fig.add_subplot(gs[0, k])
    for j, (c, col) in enumerate(CASES):
        sub = S[S.case == c].set_index('kg').reindex(KG)
        a.bar(x + (j - 1) * w, sub[key], w, yerr=sub[f'{key}_sd'].fillna(0), capsize=2, color=col,
              hatch='//' if c.startswith('GA') else None, label=c + (' (1 run each)' if c.startswith('GA') else ' (mean of 3)'))
        if c.startswith('GA'):
            base = S[S.case == 'baseline a0.15 v0.40'].set_index('kg').reindex(KG)[key]
            for xi, (vv, bb) in enumerate(zip(sub[key], base)):
                if np.isfinite(bb): a.text(x[xi] + (j - 1) * w, vv, f'{100*(vv/bb-1):+.0f}%', ha='center', va='bottom', fontsize=7)
                else: a.text(x[xi] + (j - 1) * w, vv, 'no base', ha='center', va='bottom', fontsize=6)
    a.set(title=lab, xticks=x, xticklabels=[f'{v:g} kg' for v in KG]); a.grid(axis='y', alpha=.3)
    a.set_ylim(S[key].min() * 0.85, S[key].max() * 1.08)
    if k == 0: a.legend(fontsize=7, loc='upper left')
a = fig.add_subplot(gs[1, 0])
for j, (c, col) in enumerate(CASES):
    sub = S[S.case == c].set_index('kg').reindex(KG); a.bar(x + (j - 1) * w, sub['t'], w, color=col, hatch='//' if c.startswith('GA') else None)
a.set(title='Trip time (motion) [s]', xticks=x, xticklabels=[f'{v:g} kg' for v in KG]); a.grid(axis='y', alpha=.3)
a = fig.add_subplot(gs[1, 1])
gg = g.reset_index()
a.plot(gg.kg, gg.est, 'o', color='#9467bd', label='Smart estimate at 20 s'); a.plot([0, 36], [0, 36], 'k--', lw=1, label='ideal')
for _, r in gg.iterrows(): a.annotate('v%.2f' % r.vga if np.isfinite(r.vga) else 'kept 0.20\n(out of range)', (r.kg, r.est), xytext=(4, -12), textcoords='offset points', fontsize=7)
a.axvspan(25.77, 36, color='#eee'); a.text(30.5, 2, 'beyond trained range', ha='center', fontsize=8)
a.set(title='Payload estimate on the robot', xlabel='real payload [kg]', ylabel='estimated [kg]'); a.grid(alpha=.3); a.legend(fontsize=8)
for k, kg in enumerate((12.88, 25.77)):
    a = fig.add_subplot(gs[2, k])
    for c, col in CASES:
        r = T[(T.case == c) & np.isclose(T.kg, kg)]
        if len(r): m = motion(r.bag.iloc[0]); a.plot(m['trace_t'], m['trace_v'], color=col, lw=1.3, label=f'{c}: {m["t"]:.0f} s')
    a.axvline(20, color='#9467bd', ls=':', lw=1); a.text(20.3, 0.42, 'GA switch (20 s)', fontsize=7, color='#9467bd')
    a.set(title=f'Speed profile, {kg} kg (wheel odom)', xlabel='time from motion start [s]', ylabel='v [m/s]', ylim=(0, 0.45)); a.grid(alpha=.3); a.legend(fontsize=8)
fig.suptitle('Real robot: Smart/GA (⚡, 2026-11-27) vs baselines (2026-11-25/26), start -> Station_b 14.3 m\n'
             'energy over the motion window only; GA = 1 run per payload, baselines = mean of 3 (error bar = std)')
fig.tight_layout(); out = os.path.join(ROOT, 'figures', 'ga_real_vs_base.png'); fig.savefig(out, dpi=100); print(out)
