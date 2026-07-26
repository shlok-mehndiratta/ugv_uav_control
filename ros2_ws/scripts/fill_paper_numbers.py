#!/usr/bin/env python3
"""Compute the co-design Table II numbers from the curated runs in
paper_data/table2_codesign/ (gcod_C1 = continuous baseline, gcod_C2 = event-triggered).

Reports, as mean +/- std over the R=3 repeats:
  - Aerial solves per run   : uav_solve_count            (summary)
  - Mean aerial solve time   : mean_uav_solve_ms          (summary)
  - Aerial CPU per run (ms)  : total_uav_cpu_ms           (summary)
  - Steady-state tr(Sigma)   : mean cov_trace for t>=8 s  (CSV, skips takeoff)
  - ArUco visibility (%)     : aruco_visible_pct          (summary)
  - UAV-UGV offset (m)       : mean horizontal ||uav - UGV_est|| for t>=8 s (CSV; uses the
                               EKF-estimated UGV position (x,y), matching the paper's Table II)
"""
import csv, glob, math, os
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
R = os.path.normpath(os.path.join(HERE, '..', 'paper_data', 'table2_codesign'))
T0 = 8.0


def summ(path):
    d = {}
    for line in open(path):
        if ':' in line and not line.startswith('-'):
            k, _, v = line.partition(':')
            try:
                d[k.strip()] = float(v.strip())
            except ValueError:
                pass
    return d


def steady(csv_path):
    rows = list(csv.DictReader(open(csv_path)))
    t0 = float(rows[0]['timestamp'])
    cov, off = [], []
    for r in rows:
        if float(r['timestamp']) - t0 < T0:
            continue
        cov.append(float(r['cov_trace']))
        off.append(math.hypot(float(r['uav_x']) - float(r['x']),
                              float(r['uav_y']) - float(r['y'])))   # x,y = EKF-estimated UGV pose
    return (np.mean(cov) if cov else np.nan, np.mean(off) if off else np.nan)


def ms(v):
    v = [x for x in v if x is not None and not (isinstance(x, float) and math.isnan(x))]
    return (np.mean(v), np.std(v)) if v else (float('nan'), float('nan'))


print(f'{"metric":24s} {"C1 (cont.)":>18s} {"C2 (event)":>18s}')
agg = {}
for M in ('C1', 'C2'):
    csvs = sorted(glob.glob(os.path.join(R, f'gcod_{M}_*.csv')))
    S = [summ(c[:-4] + '_summary.txt') for c in csvs]
    steadies = [steady(c) for c in csvs]
    agg[M] = {
        'Aerial solves (#/run)':  ms([s.get('uav_solve_count') for s in S]),
        'Mean solve (ms)':        ms([s.get('mean_uav_solve_ms') for s in S]),
        'Aerial CPU (ms/run)':    ms([s.get('total_uav_cpu_ms') for s in S]),
        'Steady tr(Sigma)':       ms([c for c, _ in steadies]),
        'ArUco vis (%)':          ms([s.get('aruco_visible_pct') for s in S]),
        'UAV-UGV offset (m)':     ms([o for _, o in steadies]),
    }
for k in ['Aerial solves (#/run)', 'Mean solve (ms)', 'Aerial CPU (ms/run)',
          'Steady tr(Sigma)', 'ArUco vis (%)', 'UAV-UGV offset (m)']:
    nd = 0 if 'solves' in k else (1 if ('ms' in k or '%' in k) else 4)
    c1 = agg['C1'][k]; c2 = agg['C2'][k]
    print(f'{k:24s} {f"{c1[0]:.{nd}f}+/-{c1[1]:.{nd}f}":>18s} {f"{c2[0]:.{nd}f}+/-{c2[1]:.{nd}f}":>18s}')
