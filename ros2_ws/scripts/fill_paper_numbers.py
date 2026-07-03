#!/usr/bin/env python3
"""Compute the exact numbers for the ICC12.tex results table from fresh logs and
print ready-to-paste \\renewcommand lines. Honest windowing:
  - tracking error / mean cov: computed on t>=8s (skip takeoff transient), all runs
  - peak cov (occlusion): max cov_trace in the injected window [20,28]s, occ runs
  - planner solves / UAV solve time: from per-run summaries
"""
import glob
import os
import numpy as np
import pandas as pd

R = os.path.normpath(os.path.join(os.path.dirname(__file__), '..', 'results'))
T0, OCC = 8.0, (20.0, 28.0)


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


def runs(method):
    out = []
    for c in sorted(glob.glob(os.path.join(R, f'{method}_*.csv'))):
        s = c[:-4] + '_summary.txt'
        if os.path.exists(s):
            out.append((os.path.basename(c), pd.read_csv(c), summ(s)))
    return out


def ms(vals):
    vals = [v for v in vals if v is not None and not np.isnan(v)]
    return (np.mean(vals), np.std(vals)) if vals else (float('nan'), float('nan'))


def fmt(m, s, nd=2):
    return f'{m:.{nd}f}$\\pm${s:.{nd}f}'


for M in ['C1', 'C3']:
    rs = runs(M)
    occ = [r for r in rs if '_occ_' in r[0]]
    solves = ms([r[2].get('total_replans') for r in rs])
    usolve = ms([r[2].get('mean_uav_solve_ms') for r in rs])
    err = ms([r[1][r[1].timestamp >= T0]['target_y'].abs().mean() for r in rs])
    covm = ms([r[1][r[1].timestamp >= T0]['cov_trace'].mean() for r in rs])
    peak = ms([r[1][(r[1].timestamp >= OCC[0]) & (r[1].timestamp <= OCC[1])]['cov_trace'].max()
               for r in occ]) if occ else (float('nan'), float('nan'))
    tag = 'Cone' if M == 'C1' else 'Cthree'
    print(f'% {M}: n={len(rs)} runs ({len(occ)} occ)')
    print(f'\\renewcommand{{\\RESsolves{tag}}}{{{fmt(*solves,0)}}}')
    print(f'\\renewcommand{{\\RESerr{tag}}}{{{fmt(*err,3)}}}')
    print(f'\\renewcommand{{\\REScov{tag}}}{{{fmt(*covm,4)}}}')
    print(f'\\renewcommand{{\\RESpeak{tag}}}{{{fmt(*peak,3)}}}')
    print(f'\\renewcommand{{\\RESaru{tag}}}{{{fmt(*usolve,1)}}}')
