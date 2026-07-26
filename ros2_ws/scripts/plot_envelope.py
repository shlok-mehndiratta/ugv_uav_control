#!/usr/bin/env python3
"""Regenerate Fig. 3 -- the "two jaws of the altitude tension" -- from the curated
coverage-resolution sweep in paper_data/fig3_envelope/.

Left  (a) Navigation cost : UGV steering effort std(omega) vs eta, one curve per track
        gentle = track1 (kappa=0.29), slalom = track_slalom (kappa=0.34), sharp = track4 (kappa=0.54).
        Rises at low altitude and with path curvature.
Right (b) Estimation cost : mean tr(Sigma) vs eta, averaged over tracks
        (GSD model, R proportional to z^2; nearly track-independent).

Geometry:  eta = r_fov / d_look = z * tan(phi/2) / d_look,  phi = 1.047 rad (60 deg FoV),
           d_look = 0.70 m nominal lookahead  (eta ranges ~1.65 at z=2 to ~4.95 at z=6).
Metric:    std(omega) is the population std of the w_cmd column.  WINDOW_S selects the
           averaging window (0 = full run; 8.0 = steady-state t>=8 s, skipping takeoff).

The canonical shipped figure is paper/media/Fig_Envelope.png (committed). This script
reproduces it from data; --out lets you write elsewhere for comparison.
"""
import argparse, csv, glob, math, os, re
from collections import defaultdict
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
RES = os.path.normpath(os.path.join(HERE, '..', 'paper_data', 'fig3_envelope'))
OUT = os.path.normpath(os.path.join(HERE, '..', '..', 'paper', 'media', 'Fig_Envelope.png'))

PHI, D_LOOK = 1.047, 0.70
TAN_HALF = math.tan(PHI / 2.0)
TRACKS = {                                    # track name -> (legend label, colour)
    'track1':       (r'gentle $\kappa$=0.29', '#1f77b4'),
    'track_slalom': (r'slalom $\kappa$=0.34', '#ff7f0e'),
    'track4':       (r'sharp $\kappa$=0.54',  '#d62728'),
}
ORDER = ['track1', 'track_slalom', 'track4']


def std_wcmd(csv_path, window_s):
    rows = list(csv.DictReader(open(csv_path)))
    t0 = float(rows[0]['timestamp'])
    w = [float(r['w_cmd']) for r in rows
         if r['w_cmd'] not in ('', 'nan') and (float(r['timestamp']) - t0) >= window_s]
    return float(np.std(w)) if w else float('nan')


def mean_cov(summary_path):
    for line in open(summary_path):
        if line.startswith('mean_cov_trace:'):
            return float(line.split(':')[1])
    return float('nan')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--window', type=float, default=0.0, help='std averaging window start (s); 0=full run')
    ap.add_argument('--out', default=OUT, help='output PNG path')
    args = ap.parse_args()

    data = defaultdict(dict)                   # track -> {eta: (std_w, mean_cov)}
    for c in sorted(glob.glob(os.path.join(RES, 'env_*_z*.csv'))):
        m = re.match(r'env_(.+)_z([\d.]+)_', os.path.basename(c))
        if not m or m.group(1) not in TRACKS:
            continue
        eta = float(m.group(2)) * TAN_HALF / D_LOOK
        s = c[:-4] + '_summary.txt'
        data[m.group(1)][eta] = (std_wcmd(c, args.window),
                                 mean_cov(s) if os.path.exists(s) else float('nan'))

    fig, (axL, axR) = plt.subplots(1, 2, figsize=(7.2, 3.0))
    for track in ORDER:
        if track not in data:
            continue
        lab, col = TRACKS[track]
        etas = sorted(data[track])
        axL.plot(etas, [data[track][e][0] for e in etas], marker='o', ms=5, lw=1.8, color=col, label=lab)
    axL.set_xlabel(r'$\eta = r_\mathrm{fov}/d_\mathrm{look}$  (altitude $\rightarrow$)', fontsize=10)
    axL.set_ylabel(r'UGV steering effort std($\omega$)', fontsize=10)
    axL.set_title('(a) Navigation cost', fontsize=11)
    axL.legend(fontsize=8)

    etas_all = sorted({e for t in ORDER if t in data for e in data[t]})
    covmean = [np.nanmean([data[t][e][1] for t in ORDER if t in data and e in data[t]]) for e in etas_all]
    axR.plot(etas_all, covmean, marker='s', ms=6, lw=1.8, color='#2ca02c')
    axR.set_xlabel(r'$\eta = r_\mathrm{fov}/d_\mathrm{look}$  (altitude $\rightarrow$)', fontsize=10)
    axR.set_ylabel(r'mean tr($\Sigma$)', fontsize=10)
    axR.set_title(r'(b) Estimation cost ($R\propto z^2$)', fontsize=11)

    plt.tight_layout()
    plt.savefig(args.out, dpi=300, bbox_inches='tight')
    print('wrote', args.out)


if __name__ == '__main__':
    main()
