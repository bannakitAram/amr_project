#!/usr/bin/env python3
"""Extract rosbags in ../bags into per-bag 20 Hz time series (ml/data/ts/<bag>.npz)
and a per-run summary (ml/data/runs.csv). Run with ROS sourced (needs rosbag2_py)."""
import os, re, json, csv, sys
import numpy as np
import rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message

ROOT = os.path.dirname(os.path.abspath(__file__))
BAGS = os.path.join(ROOT, '..', 'bags')
OUT = os.path.join(ROOT, 'data'); os.makedirs(os.path.join(OUT, 'ts'), exist_ok=True)
HZ = 20.0
NOTE_RE = re.compile(r'b(\d+)_\(a(\d+)_(\d+)_v(\d+)\)')

def read(bag):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='sqlite3'), rosbag2_py.ConverterOptions('', ''))
    types = {t.name: get_message(t.type) for t in r.get_all_topics_and_types()}
    s = {k: [] for k in ('cmd', 'odom', 'il', 'ir', 'bat', 'imu')}
    events = []
    while r.has_next():
        topic, data, ts = r.read_next()
        if topic not in types: continue
        t = ts * 1e-9
        m = deserialize_message(data, types[topic])
        if topic == '/cmd_vel_smoothed': s['cmd'].append((t, m.linear.x, m.angular.z))
        elif topic == '/wheel/odom': s['odom'].append((t, m.twist.twist.linear.x, m.twist.twist.angular.z))
        elif topic == '/motor_current/left': s['il'].append((t, m.data))
        elif topic == '/motor_current/right': s['ir'].append((t, m.data))
        elif topic == '/battery/state': s['bat'].append((t, m.voltage, m.current, m.percentage))
        elif topic == '/imu/data': s['imu'].append((t, m.linear_acceleration.x, m.angular_velocity.z))
        elif topic == '/nav_event':
            try: events.append((t, json.loads(m.data)))
            except Exception: pass
    return {k: np.array(v, dtype=float) for k, v in s.items()}, events

def resample(s):
    t0 = max(s[k][0, 0] for k in ('odom', 'il', 'ir', 'bat'))
    t1 = min(s[k][-1, 0] for k in ('odom', 'il', 'ir', 'bat'))
    tg = np.arange(t0, t1, 1.0 / HZ)
    def hold(a, col):   # zero-order hold (cmd_vel is event-driven)
        if len(a) == 0: return np.zeros_like(tg)
        idx = np.clip(np.searchsorted(a[:, 0], tg, side='right') - 1, 0, len(a) - 1)
        v = a[idx, col]; v[tg < a[0, 0]] = 0.0; return v
    def interp(a, col): return np.interp(tg, a[:, 0], a[:, col])
    d = dict(t=tg - t0,
             v_cmd=hold(s['cmd'], 1), w_cmd=hold(s['cmd'], 2),
             v=interp(s['odom'], 1), w=interp(s['odom'], 2),
             i_l=interp(s['il'], 1), i_r=interp(s['ir'], 1),
             bat_v=interp(s['bat'], 1), bat_i=interp(s['bat'], 2), soc=interp(s['bat'], 3))
    if len(s['imu']): d['ax'] = interp(s['imu'], 1); d['gz'] = interp(s['imu'], 2)
    d['acc'] = np.gradient(np.convolve(d['v'], np.ones(5) / 5, 'same'), 1.0 / HZ)
    return d, t0

def summarize(d, events, t0):
    dt = 1.0 / HZ
    mov = np.abs(d['v']) > 0.02
    i_m = 0.5 * (np.abs(d['i_l']) + np.abs(d['i_r']))
    arr = [e for t, e in events if e.get('event') == 'arrived']
    return dict(
        nav_time_s=arr[-1].get('duration_s', np.nan) if arr else np.nan,
        arrived=int(bool(arr)),
        move_time_s=mov.sum() * dt,
        dist_m=np.abs(d['v']).sum() * dt,
        energy_Wh=np.sum(d['bat_v'] * np.clip(d['bat_i'], 0, None)) * dt / 3600,
        motor_Ah=i_m.sum() * dt / 3600,
        i_motor_peak=np.percentile(i_m, 99),
        i_motor_mean_mov=i_m[mov].mean() if mov.any() else np.nan,
        bat_i_peak=np.percentile(d['bat_i'], 99),
        track_err_rms=np.sqrt(np.mean((d['v_cmd'] - d['v'])[mov] ** 2)) if mov.any() else np.nan,
        acc_peak=np.percentile(np.abs(d['acc']), 99),
        v_mean_mov=d['v'][mov].mean() if mov.any() else np.nan,
        soc_start=d['soc'][0], bat_v_start=d['bat_v'][0])

def main():
    labels = list(csv.DictReader(open(os.path.join(BAGS, 'labels.csv'))))
    rows = []
    for L in labels:
        bag = os.path.join(BAGS, L['bag'])
        if not os.path.isdir(bag): print('skip (missing)', L['bag']); continue
        m = NOTE_RE.search(L['note'])
        if not m: print('skip (note)', L['bag'], L['note']); continue
        try:
            s, ev = read(bag); d, t0 = resample(s)
        except Exception as e:
            print('skip (read)', L['bag'], e); continue
        np.savez_compressed(os.path.join(OUT, 'ts', L['bag'] + '.npz'), **d)
        row = dict(bag=L['bag'], payload_kg=float(L['payload_kg']), run=int(m.group(1)),
                   accel=float('0.' + m.group(2)), decel=float('0.' + m.group(3)), vmax=float('0.' + m.group(4)),  # a5=0.5, a15=0.15, v2=0.2
                   profile=f"a{m.group(2)}_{m.group(3)}_v{m.group(4)}", start=L['start'])
        row.update(summarize(d, ev, t0)); rows.append(row)
        print(f"{L['bag']:32s} {row['profile']:12s} {row['payload_kg']:6.2f}kg t={row['nav_time_s']} E={row['energy_Wh']:.2f}Wh")
    with open(os.path.join(OUT, 'runs.csv'), 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0].keys())); w.writeheader(); w.writerows(rows)
    print(len(rows), 'runs ->', os.path.join(OUT, 'runs.csv'))

if __name__ == '__main__':
    main()
