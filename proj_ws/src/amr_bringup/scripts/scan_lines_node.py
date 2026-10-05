#!/usr/bin/env python3
"""Finds straight walls in /scan and prints their angles - used by
amr_calib_panel.py's gyro section to measure the robot's REAL heading change
from a wall, independent of wheel slip and of the robot drifting off its
spot while turning (the wall's angle as seen by the robot depends on
heading only, not position).

Collects `num_scans` scans (robot must be still), then repeatedly RANSAC-
fits a line, refines it with a total-least-squares (PCA) fit on the
inliers, and removes those points. Prints one line on stdout:

  LINES [{"angle_deg": a, "length_m": l, "dist_m": d, "n": n, "rms_m": r}, ...]

angle_deg is the wall direction in the LiDAR frame, in [0, 180) (a line has
no direction, so it's only defined mod 180), longest wall first.

Usage: scan_lines_node.py [num_scans]
Exit codes: 0 ok (possibly empty list), 2 no /scan received.
"""
import json
import math
import sys

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan

RANGE_MIN = 0.3        # m - ignore the robot body / very close clutter
RANGE_MAX = 6.0        # m - beyond this the point spacing gets too coarse
INLIER_THRESH = 0.03   # m
MIN_INLIERS = 30
MIN_LENGTH = 0.8       # m - shorter segments are too short for a good angle
MAX_LINES = 4
RANSAC_ITERS = 400
WAIT_TIMEOUT_S = 5.0


def collect_points(num_scans):
    rclpy.init()
    node = Node('amr_scan_lines')
    pts = []
    count = {'n': 0}

    def on_scan(msg):
        if count['n'] >= num_scans:
            return
        r = np.asarray(msg.ranges, dtype=np.float64)
        a = msg.angle_min + np.arange(len(r)) * msg.angle_increment
        ok = np.isfinite(r) & (r > max(RANGE_MIN, msg.range_min)) & (r < min(RANGE_MAX, msg.range_max))
        pts.append(np.column_stack((r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok]))))
        count['n'] += 1

    node.create_subscription(LaserScan, '/scan', on_scan, qos_profile_sensor_data)
    start = node.get_clock().now().nanoseconds * 1e-9
    while count['n'] < num_scans:
        rclpy.spin_once(node, timeout_sec=0.1)
        if count['n'] == 0 and node.get_clock().now().nanoseconds * 1e-9 - start > WAIT_TIMEOUT_S:
            break
    node.destroy_node()
    rclpy.shutdown()
    return np.vstack(pts) if pts else None


def fit_tls(p):
    """Total-least-squares line through points: (centroid, unit direction, rms)."""
    c = p.mean(axis=0)
    _, s, vt = np.linalg.svd(p - c, full_matrices=False)
    d = vt[0]
    rms = s[1] / math.sqrt(len(p)) if len(s) > 1 else 0.0
    return c, d, rms


def find_lines(p, rng):
    lines = []
    remaining = p
    for _ in range(MAX_LINES):
        if len(remaining) < MIN_INLIERS:
            break
        best = None
        for _ in range(RANSAC_ITERS):
            i, j = rng.choice(len(remaining), 2, replace=False)
            d = remaining[j] - remaining[i]
            nrm = np.hypot(*d)
            if nrm < 0.1:
                continue
            n = np.array([-d[1], d[0]]) / nrm
            inl = np.abs((remaining - remaining[i]) @ n) < INLIER_THRESH
            if best is None or inl.sum() > best.sum():
                best = inl
        if best is None or best.sum() < MIN_INLIERS:
            break
        c, d, rms = fit_tls(remaining[best])
        n = np.array([-d[1], d[0]])
        inl = np.abs((remaining - c) @ n) < INLIER_THRESH  # re-collect with the refined line
        c, d, rms = fit_tls(remaining[inl])
        proj = (remaining[inl] - c) @ d
        length = float(proj.max() - proj.min())
        if length >= MIN_LENGTH:
            lines.append({
                'angle_deg': round(math.degrees(math.atan2(d[1], d[0])) % 180.0, 3),
                'length_m': round(length, 3),
                'dist_m': round(float(abs(c @ np.array([-d[1], d[0]]))), 3),
                'n': int(inl.sum()),
                'rms_m': round(float(rms), 4),
            })
        remaining = remaining[~inl]
    lines.sort(key=lambda l: -l['length_m'])
    return lines


def main():
    num_scans = int(sys.argv[1]) if len(sys.argv) > 1 else 10
    p = collect_points(num_scans)
    if p is None:
        print('no /scan data received', file=sys.stderr)
        sys.exit(2)
    lines = find_lines(p, np.random.default_rng(0))
    print('LINES ' + json.dumps(lines), flush=True)


if __name__ == '__main__':
    main()
