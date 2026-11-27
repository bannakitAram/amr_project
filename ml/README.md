# AMR payload / speed-profile ML

Env: `~/amr_ml_venv` (venv with system site-packages; numpy pinned 1.23.5 for ROS).
```
python3 -m venv --system-site-packages ~/amr_ml_venv
~/amr_ml_venv/bin/pip install numpy==1.23.5 "scikit-learn<1.6" "pandas<2.3" joblib
```
1. `source /opt/ros/humble/setup.bash && python3 extract_bags.py` -> data/runs.csv, data/ts/*.npz
2. `~/amr_ml_venv/bin/python train_weight_mlp.py` -> models/weight_mlp.joblib (profile a15_15_v2 only)
2b. `~/amr_ml_venv/bin/python train_weight_linear.py [--p95-max 2.5 --stable 5]` -> models/weight_linear.yaml (earliest accurate t_eval + coefficients + margin)
3. `~/amr_ml_venv/bin/python train_fitness_rf.py` -> models/fitness_rf.joblib
4. `~/amr_ml_venv/bin/python ga_speed_profile.py --payload 18 --objective battery --i-peak-max <datasheet A>`
   or `--bag ../bags/<a15_15_v2 bag>` (payload from weight_linear.yaml at t_eval + margin_kg)

Profile names: a15_15_v2 = accel 0.15 / decel 0.15 m/s^2, vmax 0.2 m/s; a5 = 0.5.
GA output is an offline recommendation only: velocity_smoother / speed limits are safety
config -> commit + new tag + real-robot test before line use.
