#!/usr/bin/env python3
"""Figures for the perception-constrained altitude policy (hairpin, headless).
  Fig_AltPolicy.png  : adaptive altitude z(t) tracks path curvature (demonstrates the policy actuates).
  Fig_AltCompare.png : adaptive vs fixed-low vs fixed-high -- each fixed policy fails on one axis.
Reads cga_adaptive*/cga_fixedlow*/cga_fixedhi* CSV+summary from results/."""
import csv, glob, os, math
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

RES = os.path.join(os.path.dirname(__file__), '..', 'results')
OUT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', 'paper', 'media'))
C = {'adaptive': '#1b7837', 'fixedlow': '#d6604d', 'fixedhi': '#4393c3'}
LAB = {'adaptive': 'Adaptive', 'fixedlow': 'Fixed low (2.5 m)', 'fixedhi': 'Fixed high (7 m)'}

def load_csv(path):
    rows = list(csv.DictReader(open(path)))
    return {k: np.array([float(r[k]) for r in rows]) for k in rows[0].keys()}

def summ_val(path, key):
    for line in open(path):
        if line.startswith(key + ':'):
            return float(line.split(':')[1])
    return float('nan')

def runs(cfg):
    return sorted(glob.glob(os.path.join(RES, f'cga_{cfg}_*_summary.txt')))

def agg(cfg, key):
    vals = [summ_val(p, key) for p in runs(cfg)]
    vals = [v for v in vals if not math.isnan(v)]
    return (np.mean(vals), np.std(vals), len(vals)) if vals else (float('nan'), 0, 0)

def curvature(x, y, th):
    th = np.unwrap(th)
    ds = np.hypot(np.diff(x), np.diff(y))
    dth = np.abs(np.diff(th))
    k = dth / np.maximum(ds, 1e-3)
    k = np.concatenate([[0], k])
    # smooth
    w = 9
    ker = np.ones(w) / w
    return np.convolve(np.clip(k, 0, 5), ker, mode='same')

# ---------- Combined figure: (a) policy actuates altitude; (b,c) dominates fixed baselines ----------
def first_csv(cfg):
    g = sorted(glob.glob(os.path.join(RES, f'cga_{cfg}_2*.csv')))
    return load_csv(g[0]) if g else None
da, dl, dh = first_csv('adaptive'), first_csv('fixedlow'), first_csv('fixedhi')

cfgs = ['adaptive', 'fixedlow', 'fixedhi']
mcov = {c: agg(c, 'mean_cov_trace') for c in cfgs}
xcov = {c: agg(c, 'max_cov_trace') for c in cfgs}
fov = {c: agg(c, 'fov_exit_pct') for c in cfgs}
print('mean_cov', {c: round(mcov[c][0], 4) for c in cfgs})
print('max_cov', {c: round(xcov[c][0], 4) for c in cfgs})
print('fov_exit', {c: round(fov[c][0], 3) for c in cfgs})

fig = plt.figure(figsize=(3.4, 3.6))
gs = fig.add_gridspec(2, 2, height_ratios=[1.25, 1.0], hspace=0.55, wspace=0.55)
# (a) altitude traces
axt = fig.add_subplot(gs[0, :])
axt.axhspan(2.5, 7.0, color='gray', alpha=0.06)
axt.plot(dh['timestamp'], dh['uav_z'], color=C['fixedhi'], lw=1.2, ls='--', label='Fixed high (7 m)')
axt.plot(dl['timestamp'], dl['uav_z'], color=C['fixedlow'], lw=1.2, ls='--', label='Fixed low (2.5 m)')
axt.plot(da['timestamp'], da['uav_z'], color=C['adaptive'], lw=1.7, label='Adaptive')
axt.set_ylabel('altitude $z$ (m)', fontsize=8.5)
axt.set_xlabel('time (s)', fontsize=8.5)
axt.set_ylim(2.0, 7.5)
axt.tick_params(labelsize=7.5)
axt.legend(fontsize=6.2, loc='upper left', ncol=1, framealpha=0.9)
axt.set_title('(a) Perception-driven altitude actuates over [2.5, 5.1] m', fontsize=8)
# (b) estimation cost, (c) coverage transient
xs = np.arange(len(cfgs)); cols = [C[c] for c in cfgs]
axb = fig.add_subplot(gs[1, 0])
axb.bar(xs, [mcov[c][0] for c in cfgs], yerr=[mcov[c][1] for c in cfgs], color=cols, capsize=2.5, edgecolor='k', lw=0.4, alpha=0.9)
axb.set_title('(b) Estimation cost', fontsize=7.8)
axb.set_ylabel(r'mean $\mathrm{tr}\,\Sigma$', fontsize=8)
axc = fig.add_subplot(gs[1, 1])
axc.bar(xs, [xcov[c][0] for c in cfgs], yerr=[xcov[c][1] for c in cfgs], color=cols, capsize=2.5, edgecolor='k', lw=0.4, alpha=0.9)
axc.set_yscale('log')
axc.set_title('(c) Coverage transient', fontsize=7.8)
axc.set_ylabel(r'max $\mathrm{tr}\,\Sigma$', fontsize=8)
for a in (axb, axc):
    a.set_xticks(xs); a.set_xticklabels(['Adapt.', 'Low', 'High'], fontsize=7)
    a.tick_params(axis='y', labelsize=7)
for ext, dpi in [('png', 300), ('pdf', None)]:
    p = os.path.join(OUT, f'Fig_AltPolicy.{ext}')
    plt.savefig(p, dpi=dpi, bbox_inches='tight') if dpi else plt.savefig(p, bbox_inches='tight')
print('wrote combined Fig_AltPolicy; adaptive z range %.2f-%.2f' % (da['uav_z'].min(), da['uav_z'].max()))
