#!/usr/bin/env python3
"""Compare MLP vs linear regression for payload estimation (same features / samples / CV as train_weight_mlp.py)."""
import numpy as np
from sklearn.linear_model import LinearRegression
from sklearn.preprocessing import StandardScaler
from sklearn.pipeline import make_pipeline
import train_weight_mlp as W

MODELS = {'MLP (16,8)': W.make_model,
          'LinearRegression': lambda: make_pipeline(StandardScaler(), LinearRegression())}

def run(make, runs, X, y, g, key):
    pred = np.full(len(X), np.nan)
    for val in runs[key].unique():
        test = np.isin(g, runs.index[runs[key] == val])
        pred[test] = make().fit(X[~test], y[~test]).predict(X[test])
    return pred - y

runs, X0, y, g, te = W.load()
SETS = {'3 feat': [0, 1, 2], 'no I_mean': [1, 2]}
cols = ['full', 15, 30, 45, 60]
for key, name in (('run', 'leave-one-run-out'), ('payload_kg', 'leave-one-payload-out')):
    print(f'\n{name}  (MAE kg; max |err| full trip)')
    print(f'{"model":30s}' + ''.join(f'{str(c):>8s}' for c in cols) + f'{"max":>8s}')
    for sname, idx in SETS.items():
        X = X0[:, idx]
        for mname, make in MODELS.items():
            e = run(make, runs, X, y, g, key)
            sel = {c: np.isinf(te) if c == 'full' else te == c for c in cols}
            print(f'{mname + " / " + sname:30s}' + ''.join(f'{np.mean(np.abs(e[sel[c]])):8.2f}' for c in cols)
                  + f'{np.max(np.abs(e[sel["full"]])):8.2f}')
for sname, idx in SETS.items():
    lr = MODELS['LinearRegression']().fit(X0[:, idx], y)
    c = lr[-1].coef_ / lr[0].scale_
    print(f'\nLinear {sname}: payload = %.2f' % (lr[-1].intercept_ - np.sum(c * lr[0].mean_))
          + ''.join(f' {ci:+.2f}*{W.FEATS[i]}' for ci, i in zip(c, idx)))
