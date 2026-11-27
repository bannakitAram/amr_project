#!/usr/bin/env python3
"""MLP payload estimator. Train only on profile a15_15_v2 (accel 0.15, vmax 0.2).
Features are cumulative from motion start to the evaluation time (same as the online node):
  I_mean = mean(|I_L|+|I_R|)  while v > 0.03
  I_med  = median(|I_L|+|I_R|) while v > 0.03
  Ib_cru = median(battery current) in cruise: |a| < 0.02, v > 0.15, |w| < 0.1
Training samples = each bag evaluated every EVAL_STEP s after motion start (works on partial trips).
CV: (1) leave-one-run-out (b01/b02/b03), (2) leave-one-payload-out (unseen weight)."""
import os, json, numpy as np, pandas as pd, joblib
from sklearn.neural_network import MLPRegressor
from sklearn.preprocessing import StandardScaler
from sklearn.pipeline import make_pipeline

ROOT = os.path.dirname(os.path.abspath(__file__))
PROFILE = 'a15_15_v2'; HZ = 20
T_MIN, EVAL_STEP = 15.0, 5.0          # first evaluation 15 s after motion start
FEATS = ['I_mean', 'I_med', 'Ib_cru']

def cumulative_features(d, t_eval=None):
    """Features from motion start up to t_eval seconds after it (None = whole trip)."""
    v, w, a = np.abs(d['v']), np.abs(d['w']), np.abs(d['acc'])
    mov = v > 0.03
    if not mov.any(): return None
    k0 = np.argmax(mov)
    k1 = len(v) if t_eval is None else min(len(v), k0 + int(t_eval * HZ))
    sl = slice(k0, k1)
    i_sum = (np.abs(d['i_l']) + np.abs(d['i_r']))[sl]
    m = mov[sl]; cru = (a[sl] < 0.02) & (v[sl] > 0.15) & (w[sl] < 0.1)
    if m.sum() < HZ or cru.sum() < HZ: return None
    return [i_sum[m].mean(), np.median(i_sum[m]), np.median(d['bat_i'][sl][cru])]

def bag_samples(d):
    """(t_eval, features) at every EVAL_STEP s, plus the whole trip (t_eval=inf)."""
    dur = len(d['t']) / HZ
    out = []
    for t in list(np.arange(T_MIN, dur, EVAL_STEP)) + [None]:
        f = cumulative_features(d, t)
        if f is not None: out.append((np.inf if t is None else t, f))
    return out

def make_model(seed=0):
    return make_pipeline(StandardScaler(), MLPRegressor(
        hidden_layer_sizes=(16, 8), alpha=1e-2, learning_rate_init=3e-3,
        max_iter=3000, early_stopping=True, validation_fraction=0.15, random_state=seed))

def load():
    runs = pd.read_csv(os.path.join(ROOT, 'data', 'runs.csv'))
    runs = runs[runs.profile == PROFILE].reset_index(drop=True)
    X, y, g, te = [], [], [], []
    for k, r in runs.iterrows():
        for t, f in bag_samples(dict(np.load(os.path.join(ROOT, 'data', 'ts', r.bag + '.npz')))):
            X.append(f); y.append(r.payload_kg); g.append(k); te.append(t)
    return runs, np.array(X), np.array(y), np.array(g), np.array(te)

def metrics(err):
    return dict(MAE_kg=float(np.mean(np.abs(err))), RMSE_kg=float(np.sqrt(np.mean(err ** 2))),
                max_abs_kg=float(np.max(np.abs(err))))

def cv(runs, X, y, g, te, key):
    pred = np.full(len(X), np.nan)
    for val in runs[key].unique():
        test = np.isin(g, runs.index[runs[key] == val])
        pred[test] = make_model().fit(X[~test], y[~test]).predict(X[test])
    return pred

def main():
    runs, X, y, g, te = load()
    print(f'{PROFILE}: {len(runs)} bags, {len(X)} samples, features {FEATS}')
    report = {}
    for key, name in (('run', 'leave-one-run-out'), ('payload_kg', 'leave-one-payload-out')):
        pred = cv(runs, X, y, g, te, key); err = pred - y
        report[name] = {'full_trip': metrics(err[np.isinf(te)])}
        line = f'{name:24s} full trip MAE {report[name]["full_trip"]["MAE_kg"]:.2f} kg (max {report[name]["full_trip"]["max_abs_kg"]:.2f})'
        for t in (15, 30, 45, 60):
            sel = te == t
            if sel.any():
                report[name][f't{t}s'] = metrics(err[sel]); line += f' | {t}s {report[name][f"t{t}s"]["MAE_kg"]:.2f}'
        print(line)
        runs['pred_' + key] = pred[np.isinf(te)].round(2)
    print(runs[['bag', 'payload_kg', 'pred_run', 'pred_payload_kg']].to_string(index=False))
    model = make_model().fit(X, y)
    os.makedirs(os.path.join(ROOT, 'models'), exist_ok=True)
    joblib.dump(dict(model=model, features=FEATS, profile=PROFILE, hz=HZ,
                     feature_def=__doc__),
                os.path.join(ROOT, 'models', 'weight_mlp.joblib'))
    report.update(profile=PROFILE, features=FEATS, n_bags=len(runs), n_samples=int(len(X)))
    json.dump(report, open(os.path.join(ROOT, 'models', 'weight_mlp_cv.json'), 'w'), indent=2)
    runs.to_csv(os.path.join(ROOT, 'models', 'weight_mlp_cv_pred.csv'), index=False)

if __name__ == '__main__':
    main()
