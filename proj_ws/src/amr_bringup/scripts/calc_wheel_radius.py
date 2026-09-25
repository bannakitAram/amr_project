#!/usr/bin/env python3
"""Offline calculator: new wheel_radius_l / wheel_radius_r from straight runs.

Pure math, no ROS needed. Feed it the numbers you measured with
move_forward_node.py / calib_straight_node.py and it prints the values to
paste into stm/src/motor_control_with_pid_optimized.cpp.

Each --run is: <dir> <real_dist_m> <odom_dist_m> <lateral_offset_m>
  dir            fwd | rev  (rev = driven backwards)
  real_dist_m    distance actually travelled (tape measure / laser)
  odom_dist_m    distance the robot reported (/wheel/raw_distance)
  lateral_offset perpendicular drift of the final position from the marked
                 straight line, metres, POSITIVE = to the RIGHT of the
                 direction of travel (fwd: right of robot; rev: right when
                 looking along the way it moved, i.e. behind the robot)

Model (first order, small angles):
  * Common scale  s = mean(real / odom): both wheels wrong by the same factor.
  * Heading drift phi = 2*y/D (rad, clockwise-positive for fwd; sign flipped
    for rev since the same wheel mismatch bends a reversing path the other
    way). The real left/right distance difference is  dL - dR = phi * sep.
  * Both wheels are PID-driven to the same encoder speed, so odom for each
    wheel is ~D_odom; per-wheel scale = dL_real / D_odom, dR_real / D_odom.
  * new_radius = old_radius * scale (encoder PPR is ground truth, never
    touched). The trim part is a small-angle estimate, so it is applied with
    --trim-gain (default 0.5 = go half way, re-flash, re-measure).

Assumes /wheel/raw_distance is the mean of both wheels' distance. Run a mix
of fwd and rev (drift that flips sign with direction is wheel mismatch;
drift that does not flip is floor slope/slip, and is NOT corrected here).

Example:
  calc_wheel_radius.py --run fwd 5.00 5.06 0.04 --run rev 5.00 5.05 0.03 \\
      --run fwd 5.01 5.06 0.05
"""
import argparse
import statistics
import sys

MIN_DISTANCE_M = 3.0


def parse_runs(raw):
    runs = []
    for d, real, odom, y in raw:
        if d not in ('fwd', 'rev'):
            sys.exit(f"error: direction must be fwd or rev, got '{d}'")
        real, odom, y = float(real), float(odom), float(y)
        if real <= 0 or odom <= 0:
            sys.exit('error: distances must be > 0')
        runs.append((d, real, odom, y))
    return runs


def compute(runs, radius_l, radius_r, sep, trim_gain):
    """runs: list of (dir, real_m, odom_m, offset_m). Returns dict with
    per-run rows, mean scale, mismatch, new radii and a warnings list."""
    warn = []
    rows, scales, trims = [], [], []
    for i, (d, real, odom, y) in enumerate(runs, 1):
        if real < MIN_DISTANCE_M:
            warn.append(f'run {i}: distance {real} m < {MIN_DISTANCE_M} m, '
                        'drift too small to measure reliably')
        sign = 1.0 if d == 'fwd' else -1.0
        phi = sign * 2.0 * y / real            # rad, real heading change
        scale = real / odom
        scales.append(scale)
        trims.append(phi * sep / real)         # (dL - dR) / D_real
        rows.append((i, d, real, odom, y, scale, phi))

    s = statistics.mean(scales)
    t = statistics.mean(trims)
    if len(runs) < 3:
        warn.append('fewer than 3 runs: averages are not trustworthy')
    if len(scales) > 1:
        sd = statistics.stdev(scales)
        if sd > 0.005:
            warn.append(f'scale varies between runs (stdev {sd:.4f} = '
                        f'{sd * 100:.2f}%): wheel slip / uneven floor, '
                        'repeat before trusting')
    if not {'fwd', 'rev'} <= {r[0] for r in runs}:
        warn.append('only one direction measured: cannot separate wheel '
                    'mismatch from floor slope; run fwd AND rev')
    fwd_t = [x for x, r in zip(trims, runs) if r[0] == 'fwd']
    rev_t = [x for x, r in zip(trims, runs) if r[0] == 'rev']
    if fwd_t and rev_t and statistics.mean(fwd_t) * statistics.mean(rev_t) < 0:
        warn.append('fwd and rev trims have OPPOSITE sign after direction '
                    'correction: drift is not from wheel mismatch (floor '
                    'slope / caster / slip) - do not apply the trim')

    # dL_real = D(1 + t/2), dR_real = D(1 - t/2) about mean scale s; the
    # trim gain scales only the left/right part.
    sl = s * (1.0 + trim_gain * t / 2.0)
    sr = s * (1.0 - trim_gain * t / 2.0)
    return {'rows': rows, 'scale': s, 'mismatch': t,
            'new_l': radius_l * sl, 'new_r': radius_r * sr, 'warnings': warn}


def main():
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument('--run', nargs=4, action='append', required=True,
                    metavar=('DIR', 'REAL_M', 'ODOM_M', 'OFFSET_M'))
    ap.add_argument('--radius-l', type=float, default=0.064,
                    help='current wheel_radius_l in firmware (m)')
    ap.add_argument('--radius-r', type=float, default=0.064,
                    help='current wheel_radius_r in firmware (m)')
    ap.add_argument('--sep', type=float, default=0.472,
                    help='wheel_sep in firmware (m)')
    ap.add_argument('--trim-gain', type=float, default=0.5,
                    help='fraction of the left/right correction to apply now')
    args = ap.parse_args()
    res = compute(parse_runs(args.run), args.radius_l, args.radius_r,
                  args.sep, args.trim_gain)

    print(f'{"#":>2} {"dir":>4} {"real":>7} {"odom":>7} {"y":>7} '
          f'{"scale":>7} {"phi[deg]":>9}')
    for i, d, real, odom, y, scale, phi in res['rows']:
        print(f'{i:>2} {d:>4} {real:7.3f} {odom:7.3f} {y:7.3f} '
              f'{scale:7.4f} {phi * 57.29578:9.2f}')
    s, t = res['scale'], res['mismatch']
    print(f'\nmean scale (both wheels)   : {s:.4f}  ({(s - 1) * 100:+.2f}%)')
    print(f'left-right mismatch (L-R)/D: {t * 100:+.3f}%  '
          f'(applied x{args.trim_gain} this iteration)')
    print(f'\nwheel_radius_l: {args.radius_l:.5f} -> {res["new_l"]:.5f} m')
    print(f'wheel_radius_r: {args.radius_r:.5f} -> {res["new_r"]:.5f} m')
    print('\nPaste into stm/src/motor_control_with_pid_optimized.cpp:')
    print(f'const float wheel_radius_l = {res["new_l"]:.5f}f;')
    print(f'const float wheel_radius_r = {res["new_r"]:.5f}f;')
    if res['warnings']:
        print('\nWARNINGS:')
        for w in res['warnings']:
            print(f'  - {w}')
    print('\nNext: flash, low speed first (check E-stop), re-run the '
          'straight test and repeat until drift < 2 cm / 5 m and '
          'distance error < 1%.')


if __name__ == '__main__':
    main()
