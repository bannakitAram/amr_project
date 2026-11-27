#!/usr/bin/env python3
"""Genetic algorithm: find (accel, vmax) for a payload, fitness = Random Forest surrogate.
  python ga_speed_profile.py --payload 18.5            # payload known
  python ga_speed_profile.py --bag ../bags/<bag>       # payload from linear model at t_eval (+margin), bag must be a15_15_v2
Search is bounded to the tested envelope (no extrapolation). Cost (minimise):
  objective (motor Ah | battery Wh | time) + small accel-peak tie-break,
  subject to payload-dependent constraints: motor peak current, tracking error, accel peak.
Result is snapped to the nearest tested profile (use --no-snap for the raw GA optimum)."""
import os, argparse, json, numpy as np, joblib

ROOT = os.path.dirname(os.path.abspath(__file__))

OBJ = {'motor': ('motor_Ah', 0.01), 'battery': ('energy_Wh', 1.0), 'time': ('nav_time_s', 60.0)}

def cost_fn(rf, payload, args):
    y_cols = rf['y_cols']
    obj, ref = OBJ[args.objective]
    def f(pop):
        X = np.column_stack([pop[:, 0], pop[:, 1], np.full(len(pop), payload)])
        Y = dict(zip(y_cols, rf['model'].predict(X).T))
        c = Y[obj] / ref + args.w_smooth * Y['acc_peak'] / 0.2
        # hard constraints (payload-dependent through the RF): big penalty + distance to feasibility
        viol = (np.clip(Y['i_motor_peak'] - args.i_peak_max, 0, None) / 0.1
                + np.clip(Y['track_err_rms'] - args.track_err_max, 0, None) / 0.005
                + np.clip(Y['acc_peak'] - args.acc_peak_max, 0, None) / 0.05)
        c = c + np.where(viol > 0, 100 + viol, 0)
        return c, Y
    return f

def ga(cost, lo, hi, pop_n=40, gens=60, pm=0.2, seed=0):
    rng = np.random.default_rng(seed)
    pop = rng.uniform(lo, hi, (pop_n, len(lo)))
    best, best_c = None, np.inf
    for _ in range(gens):
        c, _ = cost(pop)
        i = np.argmin(c)
        if c[i] < best_c: best, best_c = pop[i].copy(), c[i]
        # tournament selection
        a, b = rng.integers(0, pop_n, (2, pop_n))
        parents = pop[np.where(c[a] < c[b], a, b)]
        # BLX-0.3 crossover
        p2 = parents[rng.permutation(pop_n)]
        alpha = rng.uniform(-0.3, 1.3, parents.shape)
        child = parents + alpha * (p2 - parents)
        # gaussian mutation
        mask = rng.random(child.shape) < pm
        child += mask * rng.normal(0, 0.1 * (hi - lo), child.shape)
        child = np.clip(child, lo, hi)
        child[0] = best                                                     # elitism
        pop = child
    return best, best_c

def payload_from_bag(bag, t_eval=None, add_margin=True):
    """Early estimate as the online node would do it: linear model at its t_eval (+ safety margin)."""
    import sys, yaml; sys.path.insert(0, ROOT)
    from train_weight_mlp import cumulative_features
    name = os.path.basename(os.path.normpath(bag))
    npz = os.path.join(ROOT, 'data', 'ts', name + '.npz')
    if not os.path.exists(npz):
        raise SystemExit(f'{npz} missing: run extract_bags.py (ROS sourced) first')
    m = yaml.safe_load(open(os.path.join(ROOT, 'models', 'weight_linear.yaml')))
    t = m['t_eval_s'] if t_eval is None else t_eval
    f = cumulative_features(dict(np.load(npz)), t)
    if f is None: raise SystemExit(f'not enough moving/cruise data in the first {t} s')
    I_med, Ib_cru = f[1], f[2]
    est = m['intercept'] + m['coef']['I_med'] * I_med + m['coef']['Ib_cru'] * Ib_cru
    used = est + (m['margin_kg'] if add_margin else 0.0)
    print(f'payload estimate at t={t:g}s: {est:.2f} kg (+{m["margin_kg"] if add_margin else 0} margin -> {used:.2f} kg)')
    return float(used)

def main():
    ap = argparse.ArgumentParser()
    g = ap.add_mutually_exclusive_group(required=True)
    g.add_argument('--payload', type=float); g.add_argument('--bag')
    ap.add_argument('--objective', choices=list(OBJ), default='motor', help='motor Ah | battery Wh | trip time')
    ap.add_argument('--w-smooth', type=float, default=0.05, help='tie-break weight on measured accel peak')
    ap.add_argument('--i-peak-max', type=float, default=1.20, help='motor current p99 limit [A] - SET FROM MOTOR/DRIVER DATASHEET')
    ap.add_argument('--track-err-max', type=float, default=0.025, help='cmd vs odom RMS [m/s]')
    ap.add_argument('--acc-peak-max', type=float, default=0.30, help='measured |accel| p99 [m/s^2]')
    ap.add_argument('--t-eval', type=float, default=None, help='--bag: estimate time [s], default from weight_linear.yaml')
    ap.add_argument('--no-margin', action='store_true', help='--bag: do not add margin_kg to the estimate')
    ap.add_argument('--surrogate', choices=['rf', 'poly', 'poly3', 'poly4'], default='rf', help='rf = step-wise, poly = smooth (use with --no-snap for fine values)')
    ap.add_argument('--resolution', type=float, default=0.01, help='--no-snap: round accel/vmax to this step')
    ap.add_argument('--no-snap', action='store_true', help='report the continuous GA optimum (untested values)')
    ap.add_argument('--vmax-limit', type=float, default=None, help='site speed limit cap [m/s]')
    args = ap.parse_args()
    rf = joblib.load(os.path.join(ROOT, 'models', f'fitness_{args.surrogate}.joblib'))
    payload = args.payload if args.payload is not None else payload_from_bag(args.bag, args.t_eval, not args.no_margin)
    pb = rf['bounds']['payload_kg']
    if not pb[0] <= payload <= pb[1]:
        print(f'WARNING: payload {payload:.1f} kg outside trained range {pb} - clipped, result not validated')
        payload = float(np.clip(payload, *pb))
    lo = np.array([rf['bounds']['accel'][0], rf['bounds']['vmax'][0]])
    hi = np.array([rf['bounds']['accel'][1], rf['bounds']['vmax'][1]])
    if args.vmax_limit: hi[1] = min(hi[1], args.vmax_limit)
    cost = cost_fn(rf, payload, args)
    best, c = ga(cost, lo, hi)
    tested = np.array(rf['tested'])
    if not args.no_snap:   # evaluate all tested profiles, keep GA optimum only if a tested one is not better
        ct, _ = cost(tested)
        near = tested[np.argmin(np.sum(((tested - best) / (hi - lo)) ** 2, axis=1))]
        best = tested[np.argmin(ct)] if ct.min() <= cost(near[None])[0][0] else near
        c = cost(best[None])[0][0]
    else:
        best = np.round(best / args.resolution) * args.resolution; c = cost(best[None])[0][0]
    _, Y = cost(best[None])
    if c >= 100: print('WARNING: no profile satisfies the constraints - use the slowest tested profile / reduce payload')
    dist = float(np.min(np.sqrt(np.sum(((np.array(rf['tested']) - best) / (hi - lo)) ** 2, axis=1))))
    out = dict(surrogate=rf.get('kind', 'rf'), nearest_tested_dist=round(dist, 3), payload_kg=round(payload, 2), accel=round(best[0], 3), decel=round(best[0], 3), vmax=round(best[1], 3),
               cost=round(float(c), 4), feasible=bool(c < 100), snapped=not args.no_snap, predicted={k: round(float(v[0]), 4) for k, v in Y.items()})
    print(json.dumps(out, indent=2))

if __name__ == '__main__':
    main()
