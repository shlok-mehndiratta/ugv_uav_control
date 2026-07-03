#!/usr/bin/env python3
"""
Publication figures + summary table, generated ONLY from fresh logs in results/.
Groups runs by method prefix (C1_*, C2_*, C3_*), aggregates mean/std ACROSS the
scenario sweep (nominal + occlusion trials), and writes:
  results/Table_1_Summary.csv      (auto-filled; README/paper read from this)
  results/Figure_CovTrace.png      (tr(Sigma) vs time under occlusion, band + shaded window)
  results/Figure_Solves.png        (planner-solve count per method, mean +/- std)

Design rules (honesty + clarity): one fixed color per method across all figures;
mean +/- std band, not single-run spaghetti; the occlusion window is shaded from
the ACTUAL ArUco-loss region in the data; no cosmetic trajectory rotation-fit.
"""
import glob
import os
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
import pandas as pd

RESULTS = os.path.normpath(os.path.join(os.path.dirname(__file__), '..', 'results'))
METHODS = ['C1', 'C2', 'C3']
COLORS = {'C1': '#e74c3c', 'C2': '#f39c12', 'C3': '#2ecc71'}
LABELS = {'C1': 'C1 Continuous', 'C2': 'C2 Event-Trig.', 'C3': 'C3 Proposed'}


def parse_summary(path):
    d = {}
    with open(path) as f:
        for line in f:
            if ':' in line and not line.startswith('-'):
                k, _, v = line.partition(':')
                try:
                    d[k.strip()] = float(v.strip())
                except ValueError:
                    d[k.strip()] = v.strip()
    return d


def runs_for(method):
    """Return list of dicts {name, df, summary} for a method, newest per scenario."""
    out = {}
    for csv in sorted(glob.glob(os.path.join(RESULTS, f'{method}_*.csv'))):
        base = os.path.basename(csv)
        # strip trailing _YYYYMMDD_HHMMSS.csv -> scenario key
        scen = base.rsplit('_', 2)[0]
        summ = csv[:-4] + '_summary.txt'
        if not os.path.exists(summ):
            continue
        # keep newest per scenario
        if scen not in out or os.path.getctime(csv) > out[scen][0]:
            out[scen] = (os.path.getctime(csv), csv, summ)
    runs = []
    for scen, (_, csv, summ) in sorted(out.items()):
        runs.append({'name': scen, 'df': pd.read_csv(csv), 'summary': parse_summary(summ)})
    return runs


def agg(vals):
    vals = [v for v in vals if v is not None and not (isinstance(v, float) and np.isnan(v))]
    if not vals:
        return float('nan'), float('nan')
    return float(np.mean(vals)), float(np.std(vals))


def build_table(data):
    rows = []
    present = []
    for m in METHODS:
        rs = data[m]
        if not rs:
            continue
        present.append(m)
        # STEADY-STATE WINDOW (t>=8s): skip the takeoff/EKF-convergence transient, so
        # Table_1_Summary.csv matches the paper prose/table exactly (one window everywhere).
        T0, OCC = 8.0, (20.0, 28.0)
        solves = [r['summary'].get('total_replans') for r in rs]          # full-run count
        usolve = [r['summary'].get('mean_uav_solve_ms') for r in rs]      # full-run mean
        err = [r['df'][r['df'].timestamp >= T0]['target_y'].abs().mean() for r in rs]
        covm = [r['df'][r['df'].timestamp >= T0]['cov_trace'].mean() for r in rs]
        peak_full = [r['df']['cov_trace'].max() for r in rs]              # full-run peak (transient-dominated)
        occ = [r for r in rs if '_occ_' in r['name']]
        peak_occ = [r['df'][(r['df'].timestamp >= OCC[0]) & (r['df'].timestamp <= OCC[1])]['cov_trace'].max()
                    for r in occ]
        rows.append({
            'Method': LABELS[m],
            'n_runs': len(rs),
            'PlannerSolves_mean': agg(solves)[0], 'PlannerSolves_std': agg(solves)[1],
            'MeanTrackErr_m_mean_ss': agg(err)[0], 'MeanTrackErr_m_std_ss': agg(err)[1],
            'MeanCovTrace_mean_ss': agg(covm)[0], 'MeanCovTrace_std_ss': agg(covm)[1],
            'PeakCovTrace_fullrun_mean': agg(peak_full)[0], 'PeakCovTrace_fullrun_std': agg(peak_full)[1],
            'PeakCovTrace_occwin_mean': agg(peak_occ)[0], 'PeakCovTrace_occwin_std': agg(peak_occ)[1],
            'UAVsolve_ms_mean': agg(usolve)[0], 'UAVsolve_ms_std': agg(usolve)[1],
        })
    df = pd.DataFrame(rows)
    df.attrs['present'] = present
    out = os.path.join(RESULTS, 'Table_1_Summary.csv')
    df.to_csv(out, index=False)
    print('Wrote', out)
    print(df.to_string(index=False))
    return df


def interp_common(runs, col, tgrid):
    """Interpolate a column onto a common time grid; return stacked array."""
    stack = []
    for r in runs:
        df = r['df']
        if col not in df or 'timestamp' not in df or len(df) < 2:
            continue
        stack.append(np.interp(tgrid, df['timestamp'].values, df[col].values))
    return np.array(stack) if stack else None


# Injected EKF ArUco-block window (see scripts/run_batch.sh: block_aruco 20-28s,
# EKF-time). The vision node keeps publishing, so aruco_visible does NOT flag it;
# the effect is the EKF covariance growth, which we shade directly.
OCC_START, OCC_END = 20.0, 28.0
T_START = 8.0  # skip the takeoff/startup covariance transient


def plot_covtrace(data):
    """tr(Sigma) vs time for occlusion runs, mean+/-std band, shaded injected window."""
    occ = {m: [r for r in data[m] if 'occ' in r['name']] for m in METHODS}
    if not any(occ.values()):
        print('No occlusion runs; skipping cov-trace figure.')
        return
    tmax = min(max((r['df']['timestamp'].max() for rs in occ.values() for r in rs), default=40), 60)
    tgrid = np.linspace(T_START, tmax, 200)
    fig, ax = plt.subplots(figsize=(8, 4.5))
    ax.axvspan(OCC_START, OCC_END, color='gray', alpha=0.18,
               label='ArUco EKF updates suppressed')
    for m in METHODS:
        arr = interp_common(occ[m], 'cov_trace', tgrid)
        if arr is None:
            continue
        mu = arr.mean(axis=0); sd = arr.std(axis=0)
        ax.plot(tgrid, mu, color=COLORS[m], label=LABELS[m], linewidth=2)
        ax.fill_between(tgrid, mu - sd, mu + sd, color=COLORS[m], alpha=0.18)
    ax.set_xlabel('Mission time (s)'); ax.set_ylabel(r'Covariance trace  tr($\Sigma$)')
    ax.set_title('Estimation uncertainty under injected occlusion (mean$\\pm$std over sweep)')
    ax.legend(loc='upper right'); ax.grid(True, linestyle=':')
    plt.tight_layout()
    for ext, dpi in [('png', 300), ('pdf', None)]:
        out = os.path.join(RESULTS, f'Figure_CovTrace.{ext}')
        plt.savefig(out, dpi=dpi) if dpi else plt.savefig(out)
        print('Wrote', out)
    plt.close()


def plot_solves(table):
    present = table.attrs.get('present', METHODS[:len(table)])
    fig, ax = plt.subplots(figsize=(3.4, 2.8))          # single-column aspect
    xs = list(range(len(table)))
    means = list(table['PlannerSolves_mean']); stds = list(table['PlannerSolves_std'])
    bars = ax.bar(xs, means, yerr=stds, color=[COLORS[m] for m in present],
                  alpha=0.9, capsize=4, edgecolor='black', linewidth=0.6)
    for x, m, s in zip(xs, means, stds):               # value labels above bars
        ax.text(x, m + s + max(means) * 0.03, f'{m:.0f}$\\pm${s:.0f}',
                ha='center', va='bottom', fontsize=10, fontweight='bold')
    # headline reduction annotation on the C3 bar (if both present)
    if len(means) == 2 and means[0] > 0:
        red = 100 * (means[0] - means[1]) / means[0]
        ax.text(xs[1], means[1] + stds[1] + max(means) * 0.14, f'$-{red:.0f}\\%$',
                ha='center', va='bottom', fontsize=10, color=COLORS[present[1]])
    ax.set_xticks(xs); ax.set_xticklabels(table['Method'])
    ax.set_ylabel('Planner solves per run (#)')
    ax.set_ylim(0, max(means) * 1.28)
    ax.set_title('Online optimization load', fontsize=11)
    ax.grid(True, axis='y', linestyle=':', alpha=0.6)
    plt.tight_layout()
    for ext, dpi in [('png', 300), ('pdf', None)]:      # 300-DPI raster + vector
        out = os.path.join(RESULTS, f'Figure_Solves.{ext}')
        plt.savefig(out, dpi=dpi) if dpi else plt.savefig(out)
        print('Wrote', out)
    plt.close()


if __name__ == '__main__':
    data = {m: runs_for(m) for m in METHODS}
    for m in METHODS:
        print(f'{m}: {[r["name"] for r in data[m]]}')
    if not any(data.values()):
        print('ERROR: no C1/C2/C3 logs in results/. Run scripts/run_batch.sh first.')
        raise SystemExit(1)
    table = build_table(data)
    plot_covtrace(data)
    plot_solves(table)
    print('Done.')
