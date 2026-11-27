#!/usr/bin/env python3
"""Extract the Smart/GA runs (labels.csv note 'GA') -> data/ts/<bag>.npz + data/ga_runs.csv,
with the smart_speed_node events from /nav_event (payload estimate, applied vmax). Run with ROS sourced."""
import os, csv, json, numpy as np, rosbag2_py
from rclpy.serialization import deserialize_message
from rosidl_runtime_py.utilities import get_message
import extract_bags as E
ROOT = os.path.dirname(os.path.abspath(__file__))

def events(bag):
    r = rosbag2_py.SequentialReader()
    r.open(rosbag2_py.StorageOptions(uri=bag, storage_id='sqlite3'), rosbag2_py.ConverterOptions('', ''))
    T = get_message('std_msgs/msg/String'); out = []
    while r.has_next():
        t, d, ts = r.read_next()
        if t == '/nav_event':
            try: out.append((ts * 1e-9, json.loads(deserialize_message(d, T).data)))
            except ValueError: pass
    return out

rows = []
for L in csv.DictReader(open(os.path.join(E.BAGS, 'labels.csv'))):
    if L['note'].strip() != 'GA': continue
    bag = os.path.join(E.BAGS, L['bag'])
    s, ev = E.read(bag); d, t0 = E.resample(s)
    np.savez_compressed(os.path.join(E.OUT, 'ts', L['bag'] + '.npz'), **d)
    allev = events(bag)
    w = next((e for _, e in allev if e.get('event') == 'smart_weight'), {})
    st = next((e for _, e in allev if e.get('event') == 'smart_start'), {})
    row = dict(bag=L['bag'], payload_kg=float(L['payload_kg']), start=L['start'],
               smart_started=bool(st), est_kg=w.get('payload_est'), used_kg=w.get('payload_used'),
               ga_vmax=w.get('vmax'), ga_applied=w.get('applied'),
               events=' '.join(e.get('event', '?') for _, e in allev))
    row.update(E.summarize(d, ev, t0)); rows.append(row)
    print(row['bag'], row['payload_kg'], 'est', row['est_kg'], 'v', row['ga_vmax'], '|', row['events'], '| t', row['nav_time_s'], 'E', round(row['energy_Wh'], 3))
with open(os.path.join(E.OUT, 'ga_runs.csv'), 'w', newline='') as f:
    wr = csv.DictWriter(f, fieldnames=list(rows[0])); wr.writeheader(); wr.writerows(rows)
