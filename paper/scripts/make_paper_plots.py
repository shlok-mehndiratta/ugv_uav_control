#!/usr/bin/env python3
"""
Generate the data figures for the ACODS paper, one PNG per plot (not merged).

House style: figsize (4,3); FIXED margins so every plot has an identical plot area;
bold axis labels; bold, large tick numbers; bold Title-Case legend placed so it never
covers the data; no title (captions live in the LaTeX); no grid. Repo-relative paths.

Outputs (into New Paper/media/):
    fig_e0.png        ArUco error vs altitude              -> R prop z^2 (Assumption 1)
    fig_env_nav.png   navigation cost vs altitude          -> coverage side
    fig_env_cov.png   estimation cost tr(Sigma) vs altitude-> monotone, track-independent
    fig_alt_trace.png adaptive altitude vs time            -> climbs into turns
    fig_cov_bars.png  mean tr(Sigma): adaptive vs fixed    -> adaptive ties the lowest fixed
"""
import csv
import glob
import os

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import Patch
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.normpath(os.path.join(HERE, "..", "..", "ros2_ws", "paper_data"))
MEDIA = os.path.normpath(os.path.join(HERE, "..", "media"))
os.makedirs(MEDIA, exist_ok=True)

DPI = 220
SS_WARMUP = 8.0
LABEL_FS, TICK_FS, LEG_FS = 15, 13, 12
LW, MS = 2.4, 8.0
GREEN, BLUE = "#2ca02c", "#4c72b0"
# fixed margins -> identical plot area in every figure; left is wide enough that no y-label clips
MARG = dict(left=0.25, right=0.965, bottom=0.175)

TRACKS = [("track_lowk", r"Low-$\kappa$", "o", "#1f77b4"),
          ("track_midk", r"Mid-$\kappa$", "s", "#ff7f0e"),
          ("track_highk", r"High-$\kappa$", "^", "#2ca02c")]


def _style():
    ax = plt.gca()
    ax.tick_params(labelsize=TICK_FS, width=1.2, length=4.5)
    for t in ax.get_xticklabels() + ax.get_yticklabels():
        t.set_fontweight("bold")
    for s in ax.spines.values():
        s.set_linewidth(1.2)
    return ax


def _save(path, top):
    plt.subplots_adjust(top=top, **MARG)
    plt.savefig(os.path.join(MEDIA, path), dpi=DPI)
    plt.close()
    print("wrote", path)


def _legend_inside(loc):
    plt.legend(loc=loc, prop={"weight": "bold", "size": LEG_FS}, framealpha=0.95,
               handlelength=1.6, borderpad=0.3, labelspacing=0.3)


def _legend_above(ncol):
    plt.legend(loc="lower center", bbox_to_anchor=(0.5, 1.0), ncol=ncol, frameon=False,
               prop={"weight": "bold", "size": LEG_FS}, handlelength=1.5,
               columnspacing=1.1, handletextpad=0.4)


def _read(path):
    with open(path) as fh:
        return list(csv.DictReader(fh))


# --------------------------------------------------------------------- Fig 3: E0
def plot_e0():
    zs, errs = [], []
    for f in sorted(glob.glob(os.path.join(DATA, "fig3a_e0", "e0_z*.csv"))):
        rows = _read(f)
        z = round(float(rows[0]["uav_z"]), 1)
        if abs(z - 6.0) < 1e-6:
            continue
        errs.append(np.mean([float(r["err"]) for r in rows]) * 1000.0)
        zs.append(z)
    order = np.argsort(zs); zs = np.array(zs)[order]; errs = np.array(errs)[order]
    b, a = np.polyfit(zs, errs, 1)
    r2 = 1 - np.sum((errs - (a + b * zs)) ** 2) / np.sum((errs - errs.mean()) ** 2)

    plt.figure(figsize=(4, 3))
    zl = np.linspace(zs.min(), zs.max(), 100)
    plt.plot(zl, a + b * zl, "-", color="crimson", linewidth=LW,
             label=r"Fit ($R^2$=%.3f)" % r2, zorder=1)
    plt.scatter(zs, errs, s=70, color="navy", zorder=2, label="Measured")
    plt.ylim(errs.min() - 1.0, errs.max() + 2.6)     # headroom so the legend clears the line
    plt.xlabel("Altitude $z$ (m)", fontsize=LABEL_FS, fontweight="bold")
    plt.ylabel("ArUco error (mm)", fontsize=LABEL_FS, fontweight="bold")
    _style()
    _legend_inside("upper left")
    _save("fig_e0.png", top=0.95)


# ----------------------------------------------------------- Fig 4: two costs
def _env(field):
    rows = _read(os.path.join(DATA, "fig3_envelope", "envelope_results.csv"))
    out = {}
    for key, _, _, _ in TRACKS:
        pts = {}
        for r in rows:
            if r["track"] == key:
                pts.setdefault(float(r["z"]), []).append(float(r[field]))
        zs = sorted(pts)
        out[key] = (np.array(zs), np.array([np.mean(pts[z]) for z in zs]))
    return out


def plot_env_cov():
    d = _env("meanCov")
    plt.figure(figsize=(4, 3))
    for key, label, mk, col in TRACKS:
        zs, v = d[key]
        plt.plot(zs, v, marker=mk, linewidth=LW, markersize=MS, color=col, label=label)
    plt.xlabel("Altitude $z$ (m)", fontsize=LABEL_FS, fontweight="bold")
    plt.ylabel(r"Estimation cost  tr$\,\Sigma$", fontsize=LABEL_FS, fontweight="bold")
    _style()
    _legend_above(3)
    _save("fig_env_cov.png", top=0.86)


def plot_env_nav():
    d = _env("steer")
    plt.figure(figsize=(4, 3))
    for key, label, mk, col in TRACKS:
        zs, v = d[key]
        plt.plot(zs, v, marker=mk, linewidth=LW, markersize=MS, color=col, label=label)
    plt.xlabel("Altitude $z$ (m)", fontsize=LABEL_FS, fontweight="bold")
    plt.ylabel("Navigation cost", fontsize=LABEL_FS, fontweight="bold")
    _style()
    _legend_above(3)
    _save("fig_env_nav.png", top=0.86)


# ---------------------------------------------------------- Fig 5: adaptive
def _f4(pattern):
    for f in sorted(glob.glob(os.path.join(DATA, "fig4_adaptive", pattern))):
        rows = _read(f)
        t = np.array([float(r["timestamp"]) for r in rows])
        cov = np.array([float(r["cov_trace"]) for r in rows])
        z = np.array([float(r["uav_z"]) for r in rows])
        yield t - t[0], cov, z


def plot_alt_trace():
    t, _, z = next(_f4("f4_adaptive_r1_*.csv"))
    plt.figure(figsize=(4, 3))
    plt.axhline(2.5, linestyle="--", color="darkorange", linewidth=2.6, dashes=(6, 4), label="Fixed Low")
    plt.plot(t, z, "-", color=GREEN, linewidth=2.0, label="Adaptive")
    plt.ylim(2.35, z.max() * 1.16)
    plt.xlabel("Time (s)", fontsize=LABEL_FS, fontweight="bold")
    plt.ylabel("Altitude $z$ (m)", fontsize=LABEL_FS, fontweight="bold")
    _style()
    _legend_inside("upper left")
    _save("fig_alt_trace.png", top=0.95)


def plot_cov_bars():
    conds = [("f4_adaptive_r*.csv", "",      GREEN),
             ("f4_fixed2p5_r*.csv", "2.5 m", BLUE),
             ("f4_fixed4p0_r*.csv", "4.0 m", BLUE),
             ("f4_fixed7p0_r*.csv", "7.0 m", BLUE)]
    labels, means, stds, cols = [], [], [], []
    for pat, lab, col in conds:
        per = [cov[(t >= SS_WARMUP) & (cov > 0)].mean()
               for t, cov, _ in _f4(pat) if cov[(t >= SS_WARMUP) & (cov > 0)].size]
        labels.append(lab); means.append(np.mean(per)); stds.append(np.std(per)); cols.append(col)

    plt.figure(figsize=(4, 3))
    x = np.arange(len(labels))
    plt.bar(x, means, yerr=stds, capsize=5, color=cols, edgecolor="black", linewidth=0.8, width=0.66)
    ax = plt.gca()
    top = max(m + s for m, s in zip(means, stds))
    ax.set_ylim(0, top * 1.42)
    for xi, m, s, lab in zip(x, means, stds, labels):
        if lab:
            ax.text(xi, m + s + top * 0.03, lab, ha="center", va="bottom",
                    fontsize=12, fontweight="bold")
    ax.set_xticks([])
    plt.ylabel(r"Mean  tr$\,\Sigma$", fontsize=LABEL_FS, fontweight="bold")
    _style()
    ax.legend([Patch(fc=GREEN, ec="black"), Patch(fc=BLUE, ec="black")],
              ["Adaptive", "Fixed Altitude"], loc="upper left",
              prop={"weight": "bold", "size": LEG_FS}, framealpha=0.95, handlelength=1.3)
    _save("fig_cov_bars.png", top=0.95)


if __name__ == "__main__":
    plot_e0()
    plot_env_nav()
    plot_env_cov()
    plot_alt_trace()
    plot_cov_bars()
    print("done ->", MEDIA)
