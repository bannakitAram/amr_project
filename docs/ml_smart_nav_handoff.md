# Payload estimation + GA speed profile + Smart (⚡) navigation - handoff

State on 2026-11-27 (commit 90c4b5e and earlier). Everything under `ml/` is offline analysis;
the only robot-side part is `smart_speed_node.py` + the goal_picker ⚡ button.

## Data
- `bags/` rosbags from the goal_picker power-log page, labels in `bags/labels.csv`
  (`bags/` is in .gitignore - add new bags with `git add -f`).
- Route for every labelled run: start -> Station_b, straight, ~14.3 m, ends in the station
  speed zone (0.08 then 0.04 m/s, from FR2_e1_e1_speed.pgm).
- Profile names in the note column: `bNN_(aXX_YY_vZ)`
  - **a5 = accel 0.5 m/s^2 (NOT 0.05)**, a15 = 0.15, a25 = 0.25 (decel = accel at record time),
    v2 / v3 / v4 = vmax 0.2 / 0.3 / 0.4 m/s. Verified from the cmd ramp slope (a5 -> 0.500).
  - 147 runs on 11-25/26: 9 profiles x payload 0 / 6.44 / 12.88 / 19.32 / 25.77 kg x 3;
    a15_15_v2 also 7.54 / 13.98 / 14.74 / 21.18 kg.
  - note `GA` = Smart (⚡) runs on 11-27 (0-35.17 kg, 1 run each; the first two 0 kg did not move).

## Environment
```
python3 -m venv --system-site-packages ~/amr_ml_venv
~/amr_ml_venv/bin/pip install numpy==1.23.5 "scikit-learn<1.6" "pandas<2.3" joblib
```
numpy pinned so ROS (rclpy, rosbag2_py) keeps working. `pip3` on PATH is PlatformIO's - don't use it.
Bag extraction needs ROS sourced (`source /opt/ros/humble/setup.bash`), system python3.

## Pipeline (ml/, see ml/README.md)
1. `extract_bags.py` -> `ml/data/` (20 Hz npz + runs.csv; ignored by git, regenerable)
2. `train_weight_linear.py` -> `models/weight_linear.yaml` (the payload model in use)
3. `train_fitness_rf.py` -> `models/fitness_{rf,poly,poly3,poly4}.joblib` (+ fitness_cv.json)
4. `ga_speed_profile.py` (whole trip), `segment_ga.py` (2 m segments), `station_ga.py`
   (segments + station approach, sample-level energy model)
5. `export_ga_table.py` -> `models/ga_speed_table.yaml` (read by smart_speed_node)
6. Real-robot check: `extract_ga_runs.py`, `plot_ga_real.py`, `plot_ga_real_after_switch.py`

## Payload estimate (what works)
- Features, cumulative from motion start: `I_med` = median(|I_L|+|I_R|) while |v|>0.03,
  `Ib_cru` = median(battery current) while |a|<0.02, |v|>0.15, |w|<0.1.
  (Vbatt tracks recording order, not weight - leave it out. I_mean adds nothing over I_med.)
- Linear regression beats MLP on unseen weights. Earliest time that is accurate and stays
  accurate (P95 <= 2.5 kg, both CVs, holds 5 s): **20 s** ->
  `payload = -192.85 + 110.68*I_med + 36.57*Ib_cru`, margin 2.18 kg. Trained on a15_15_v2 only.
- Real robot 11-27: |err| <= 1 kg at 6.44-35.17 kg (also beyond the 25.77 kg training range);
  0 kg read -4.8 kg (one run - collect more).
- A model valid for every profile also works offline: poly2 of (I_med, Ib_cru, vmax, accel),
  MAE ~0.9 kg at 20 s. Not exported yet - would remove the 0.20 m/s weighing phase.
- Limits: ACS712-20A on a 10-bit Mega ADC = 49 mA/count ~ 5 kg/count; that is why it needs ~20 s.
  ADS1115 / INA226 would allow faster estimates.

## GA / speed profile findings
- Base load ~51 W (Jetson + sensors) of ~72 W while driving -> trip time dominates battery Wh.
  Motors run at 12-22 % of the 10.42 A rating, so the current limit never binds.
- Measured: v0.40 vs v0.20 = -19..-29 % battery Wh; v0.30 vs v0.40 = -1..-3 % motor mAh.
  a0.5 is worse than a0.15 everywhere (tracking error x2, acc_peak 0.37 m/s^2).
- Payload does not change the best profile in 0-26 kg (all profiles scale ~equally).
- poly2 surrogate (motor_Ah R2 0.96 vs RF 0.67) gives a weight trend v 0.33 -> 0.37, but it is
  model-dependent (poly3/poly4 disagree) and **was not confirmed on the robot**: after the GA
  switch, same distance window, GA used +4..+8 % motor mAh and +1..+6 % Wh vs v0.40.
- Whole Smart trip on the robot: -15..-21 % Wh vs v0.20, +8..+11 % vs v0.40
  (20 s weighing at 0.20 m/s + v 0.33-0.37 < 0.40).
- Per-segment GA on this straight route: < 1 % (no significance). The station approach
  (25 % of trip time for 6 % of distance) is where time/energy can still be won - safety review first.

## Smart navigation (⚡) on the robot
- goal_picker: Station panel -> purple **⚡ <station>** button -> rosboard publishes
  `{"objective":"auto"}` on `/rosboard_smart_nav`, then the station goal 0.5 s later.
- `amr_navigation/scripts/smart_speed_node.py` (launched with navigation, idle until triggered):
  weighing profile a0.15 v0.20 -> estimate at 20 s -> velocity_smoother max_velocity[0] /
  max_accel[0] from the table (param `table`, default `motor_ga`; `battery_tested` = v0.40,
  which matches the real-robot result best) -> restore on arrived/failed/canceled/interrupted/
  timeout/exit. Only lowers limits, never touches max_decel; out-of-range -> stays at 0.20.
- State on `/smart_nav/state` (badge on the page); events `smart_*` on `/nav_event` (in bags).
- Gotcha: two rosboard instances -> the old one keeps :8888 and the page talks to old code
  (goal silently dropped). Check `ss -ltnp | grep 8888` and kill the old one.
- Rebuild after edits: `colcon build --symlink-install --packages-select amr_navigation rosboard`,
  then restart rosboard AND navigation.

## Open items / safety
- collision_monitor depth source disabled (commit abd4572, reason not recorded) - re-enable or
  document the risk assessment before line use.
- Safety-related commits since v1.1.0 (speed mask, executor, collision_monitor, smart nav) need
  a real-robot test (speed, stop distance at 0 and 25.77 kg, route) and a new tag before line use.
- GA speeds 0.33-0.37 have no stop-distance test yet.
