#!/usr/bin/env python3
"""Plot what the selected speed profiles look like on the robot (real bags) + the proposed online sequence."""
import os, numpy as np, pandas as pd, matplotlib; matplotlib.use('Agg')
import matplotlib.pyplot as plt, yaml
ROOT = os.path.dirname(os.path.abspath(__file__))
runs = pd.read_csv(os.path.join(ROOT, 'data', 'runs.csv'))
lin = yaml.safe_load(open(os.path.join(ROOT, 'models', 'weight_linear.yaml')))
C = {'a15_15_v2': '#888888', 'a15_15_v3': '#1f77b4', 'a15_15_v4': '#d62728'}
LBL = {'a15_15_v2': 'a15_15_v2 (now, weighing)', 'a15_15_v3': 'a15_15_v3 (GA: min motor Ah)', 'a15_15_v4': 'a15_15_v4 (GA: min battery Wh)'}

def trace(profile, kg):
    r = runs[(runs.profile == profile) & (np.isclose(runs.payload_kg, kg))].iloc[0]
    d = np.load(os.path.join(ROOT, 'data', 'ts', r.bag + '.npz'))
    k0 = np.argmax(np.abs(d['v']) > 0.03)
    return d['t'][k0:] - d['t'][k0], d['v'][k0:], d['v_cmd'][k0:], r

fig, ax = plt.subplots(3, 1, figsize=(11, 11))
for p in C:
    t, v, vc, r = trace(p, 25.77)
    ax[0].plot(t, v, color=C[p], lw=1.2, label=f'{LBL[p]}  t={r.nav_time_s:.0f}s  {r.energy_Wh:.2f}Wh  {r.motor_Ah*1000:.1f}mAh')
ax[0].set(title='Real trips, payload 25.77 kg, same route (wheel odom speed)', xlabel='time from motion start [s]', ylabel='v [m/s]')
ax[0].legend(fontsize=8); ax[0].grid(alpha=.3)

for p in C:
    t, v, vc, _ = trace(p, 25.77)
    m = t < 8
    ax[1].plot(t[m], v[m], color=C[p], lw=1.5, label=f'{p} odom')
    ax[1].plot(t[m], vc[m], color=C[p], lw=1, ls='--', label=f'{p} cmd (smoothed)')
ax[1].set(title='Start ramp (accel 0.15 m/s^2): first 8 s', xlabel='s', ylabel='v [m/s]'); ax[1].legend(fontsize=7, ncol=3); ax[1].grid(alpha=.3)

# proposed online sequence (schematic): weigh at v2 until t_eval, then switch to selected profile
te = lin['t_eval_s']; t = np.arange(0, 60, 0.05)
def ramp(t, v_target, v_from, t0, a=0.15): return np.minimum(v_target, v_from + a * np.clip(t - t0, 0, None))
base = ramp(t, 0.2, 0.0, 0)
for vt, col, txt in ((0.4, C['a15_15_v4'], 'est+margin light -> v0.4'), (0.3, C['a15_15_v3'], 'est+margin heavy / motor objective -> v0.3')):
    v = np.where(t < te, base, ramp(t, vt, 0.2, te))
    ax[2].plot(t, v, color=col, lw=2, label=txt)
ax[2].axvspan(0, te, color='#eeeeee'); ax[2].axvline(te, color='k', ls=':')
ax[2].text(te / 2, 0.42, f'weighing at a15_15_v2\n(I_med, Ib_cru)', ha='center', fontsize=9)
ax[2].text(te + 0.5, 0.05, f't_eval={te:.0f}s: payload = {lin["intercept"]:.1f} + {lin["coef"]["I_med"]:.1f}*I_med + {lin["coef"]["Ib_cru"]:.1f}*Ib_cru (+{lin["margin_kg"]} kg)', fontsize=8)
ax[2].set(title='Proposed online sequence (schematic, straight segment, not yet run on the robot)', xlabel='time from motion start [s]',
          ylabel='v [m/s]', ylim=(0, 0.48)); ax[2].legend(fontsize=8, loc='lower right'); ax[2].grid(alpha=.3)
fig.tight_layout(); out = os.path.join(ROOT, 'figures', 'speed_profiles.png'); fig.savefig(out, dpi=110); print(out)
