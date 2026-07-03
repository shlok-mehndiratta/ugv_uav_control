#!/usr/bin/env python3
"""Compare the two determinism-probe runs (det_run_A2 vs det_run_B2) and report
whether the simulator is effectively deterministic under fixed configuration."""
import glob
import os
import sys
import numpy as np
import pandas as pd

R = os.path.normpath(os.path.join(os.path.dirname(__file__), '..', 'results'))


def latest(prefix):
    fs = glob.glob(os.path.join(R, prefix + '*.csv'))
    return max(fs, key=os.path.getctime) if fs else None


def main():
    a, b = latest('det_run_A2'), latest('det_run_B2')
    if not a or not b:
        print('Missing det_run_A2/B2 CSVs; run scripts/det_test.sh first.')
        return 1
    da, db = pd.read_csv(a), pd.read_csv(b)
    n = min(len(da), len(db))
    print(f'A={os.path.basename(a)} ({len(da)} rows)  B={os.path.basename(b)} ({len(db)} rows)')
    worst = 0.0
    for col in ['true_ugv_x', 'true_ugv_y', 'x', 'y', 'cov_trace', 'uav_x', 'uav_y', 'uav_z']:
        if col in da and col in db:
            d = np.abs(da[col].values[:n] - db[col].values[:n])
            worst = max(worst, d.max())
            print(f'  {col:12s} maxΔ={d.max():.6g}  meanΔ={d.mean():.6g}')
    verdict = 'DETERMINISTIC (use scenario sweep)' if worst < 1e-3 else 'STOCHASTIC (seed-based MC valid)'
    print(f'VERDICT: worst position/cov deviation = {worst:.6g} -> {verdict}')
    return 0


if __name__ == '__main__':
    sys.exit(main())
