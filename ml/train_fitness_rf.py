#!/usr/bin/env python3
"""Random Forest surrogate: (accel, vmax, payload_kg) -> run KPIs, trained on all 147 runs.
Used as the fitness model of ga_speed_profile.py. decel == accel in every recorded profile."""
import os, json, numpy as np, pandas as pd, joblib
from sklearn.ensemble import RandomForestRegressor
from sklearn.model_selection import GroupKFold, cross_val_predict
from sklearn.pipeline import make_pipeline
from sklearn.preprocessing import PolynomialFeatures, StandardScaler
from sklearn.linear_model import Ridge

ROOT = os.path.dirname(os.path.abspath(__file__))
X_COLS = ['accel', 'vmax', 'payload_kg']
Y_COLS = ['nav_time_s', 'motor_Ah', 'energy_Wh', 'i_motor_peak', 'track_err_rms', 'acc_peak']

def main():
    d = pd.read_csv(os.path.join(ROOT, 'data', 'runs.csv')).dropna(subset=Y_COLS)
    X, Y = d[X_COLS].values, d[Y_COLS].values
    groups = d.profile + '_' + d.payload_kg.astype(str)
    bounds = dict(accel=[float(d.accel.min()), float(d.accel.max())], vmax=[float(d.vmax.min()), float(d.vmax.max())],
                  payload_kg=[float(d.payload_kg.min()), float(d.payload_kg.max())])
    tested = d[['accel', 'vmax']].drop_duplicates().values.tolist()
    # RF = step-wise surrogate; poly2 = smooth surrogate (quadratic + interactions) for fine-resolution GA
    models = {'rf': RandomForestRegressor(n_estimators=300, min_samples_leaf=2, random_state=0, n_jobs=-1),
              'poly': make_pipeline(StandardScaler(), PolynomialFeatures(2), Ridge(alpha=1e-3))}
    # higher degree: accel and vmax have only 3 tested levels -> a^3, v^3 are not identifiable
    # (exactly collinear with 1, x, x^2); the extra terms that carry information are the payload ones
    # (kg has 5-9 levels): v^2*kg, v*kg^2, a*v*kg, ...  Ridge keeps the collinear part bounded.
    for deg in (3, 4):
        models[f'poly{deg}'] = make_pipeline(StandardScaler(), PolynomialFeatures(deg), Ridge(alpha=1e-2))
    rep = {}
    os.makedirs(os.path.join(ROOT, 'models'), exist_ok=True)
    for name, m in models.items():
        P = cross_val_predict(m, X, Y, cv=GroupKFold(5), groups=groups); rep[name] = {}
        for j, c in enumerate(Y_COLS):
            y, p = Y[:, j], P[:, j]
            rep[name][c] = dict(MAE=float(np.mean(np.abs(p - y))), R2=float(1 - np.sum((p - y) ** 2) / np.sum((y - y.mean()) ** 2)))
            print(f'{name:4s} {c:15s} MAE {rep[name][c]["MAE"]:.4f}  R2 {rep[name][c]["R2"]:.2f}')
        m.fit(X, Y)
        fn = f'fitness_{name}.joblib'
        joblib.dump(dict(model=m, x_cols=X_COLS, y_cols=Y_COLS, bounds=bounds, tested=tested, kind=name),
                    os.path.join(ROOT, 'models', fn))
    json.dump(dict(cv=rep, bounds=bounds, n_runs=len(d)), open(os.path.join(ROOT, 'models', 'fitness_cv.json'), 'w'), indent=2)
    rf = models['rf']
    print('feature importance', dict(zip(X_COLS, rf.feature_importances_.round(3))))

if __name__ == '__main__':
    main()
