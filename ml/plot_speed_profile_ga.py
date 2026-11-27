#!/usr/bin/env python3
"""Speed profile: baseline a15_15_v4 (measured, wheel odom) vs GA profile (constructed, not run on the robot).
GA profile in the distance domain:  v_GA(s) = min( sqrt(v0^2 + 2*a*s),  v_cruise_GA,  v_base_tail(s) )
  - ramp with the same accel 0.15 m/s^2, cruise at the GA speed for that payload,
  - the station speed-zone tail (0.08 -> 0.04 m/s -> stop) copied from the measured baseline run.
Time axis from t(s) = integral ds / v."""
import os, json, sys, subprocess, numpy as np, pandas as pd, matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt
ROOT = os.path.dirname(os.path.abspath(__file__)); DT = 0.05
runs = pd.read_csv(os.path.join(ROOT, 'data', 'runs.csv'))
KG = [0.0, 12.88, 25.77]; ACC = 0.15

def baseline(kg):
    r = runs[(runs.profile == 'a15_15_v4') & np.isclose(runs.payload_kg, kg)].iloc[0]
    d = np.load(os.path.join(ROOT, 'data', 'ts', r.bag + '.npz'))
    v = np.abs(d['v']); k0 = np.argmax(v > 0.03); k1 = np.where(v > 0.005)[0][-1] + 1
    v = v[k0:k1]; t = np.arange(len(v)) * DT; s = np.concatenate([[0], np.cumsum(v[:-1]) * DT])
    return t, s, v, r

def ga_speed(kg):
    o = json.loads(subprocess.check_output([sys.executable, os.path.join(ROOT, 'ga_speed_profile.py'), '--payload', str(kg),
                    '--objective', 'motor', '--i-peak-max', '10.42', '--surrogate', 'poly', '--no-snap']))
    return o['vmax'], o['accel']

def ga_profile(tb, sb, vb, v_ga):
    sg = np.linspace(0, sb[-1], 4000)
    vbs = np.interp(sg, sb, vb)
    k_end = np.where(vbs > 0.7 * vb.max())[0][-1]               # end of baseline cruise
    tail = np.where(np.arange(len(sg)) > k_end, vbs, np.inf)     # station zone from baseline
    # approach to the zone: decelerate from v_ga with the baseline's own decel shape (min with tail backwards)
    vg = np.minimum.reduce([np.sqrt(vb[0] ** 2 + 2 * ACC * sg), np.full_like(sg, v_ga), tail])
    dec = np.where(np.arange(len(sg)) <= k_end, np.inf, 0)
    ds = np.diff(sg); vm = np.maximum(0.5 * (vg[1:] + vg[:-1]), 0.01)
    tg = np.concatenate([[0], np.cumsum(ds / vm)])
    return tg, sg, vg

fig, ax = plt.subplots(len(KG), 2, figsize=(14, 3.6 * len(KG)))
summary = []
for i, kg in enumerate(KG):
    tb, sb, vb, r = baseline(kg); v_ga, a_ga = ga_speed(kg)
    tg, sg, vg = ga_profile(tb, sb, vb, v_ga)
    summary.append((kg, v_ga, tb[-1], tg[-1]))
    for j, (xb, xg, xl) in enumerate(((tb, tg, 'time from motion start [s]'), (sb, sg, 'distance from start [m]'))):
        a = ax[i, j]
        a.plot(xb, vb, color='#d62728', lw=1.2, label=f'baseline a0.15 v0.40 (measured)  trip {tb[-1]:.0f} s')
        a.plot(xg, vg, color='#9467bd', lw=2, ls='--', label=f'GA a{a_ga:.2f} v{v_ga:.2f} (constructed)  trip {tg[-1]:.0f} s')
        a.set(xlabel=xl, ylabel='v [m/s]', ylim=(0, 0.45), title=f'payload {kg} kg'); a.grid(alpha=.3); a.legend(fontsize=8, loc='upper right')
        if j == 1: a.axvspan(sb[np.where(vb > 0.7 * vb.max())[0][-1]], sb[-1], color='#eeeeee', zorder=0); a.text(sb[-1] - 0.6, 0.02, 'station\nzone', ha='center', fontsize=8)
fig.suptitle('Speed profile: baseline a15_15_v4 vs GA (motor objective), start -> Station_b\n'
             'GA = ramp 0.15 m/s^2 + GA cruise speed + measured station-zone tail; not run on the robot', fontsize=11)
fig.tight_layout(); out = os.path.join(ROOT, 'figures', 'speed_profile_ga_vs_base.png'); fig.savefig(out, dpi=110); print(out)
for kg, v, tb_, tg_ in summary: print(f'{kg:5.2f} kg  GA v={v:.2f}  trip baseline {tb_:.1f} s  GA {tg_:.1f} s')
