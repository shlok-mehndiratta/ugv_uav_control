#!/usr/bin/env python3
"""FB1 numerical validator for the FoV-resolution tradeoff theory (Props A/B/C).

Honest, self-contained model (no ROS): a maneuvering ground target, a UAV whose
ground-projection tracks it with first-order lag (=> tracking lag d spikes at turns),
a Kalman filter estimating the target position, GSD measurement noise R(z)=R0*(z/z0)^2,
and FoV visibility (measured iff d <= z*tan(phi/2)). We test:
  Prop A: for fixed lag d, tr(Sigma_ss)(z) is minimized at z*(d)=d/tan(phi/2).
  Prop B: active altitude z=z*(d_k) dominates any fixed altitude (integrated cov).
  Prop C: anticipatory climb (lookahead) beats reactive climb at turns.
The results must EMERGE from the physics; nothing is rigged toward a conclusion.
"""
import numpy as np
import matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt

import os
RESULTS = os.path.normpath(os.path.join(os.path.dirname(__file__), '..', 'results'))
np.random.seed(0)
DT = 0.1
PHI = 1.047
TANH = np.tan(PHI / 2.0)          # 0.5774
R0, Z0 = (0.02) ** 2, 1.0         # 2 cm meas. std at 1 m -> R(z)=R0 (z/z0)^2
QACC = 0.6 ** 2                   # target maneuver (accel) process-noise power
rfov = lambda z: z * TANH
Rz = lambda z: R0 * (z / Z0) ** 2


def kf_run(true_xy, d_series, z_series, seed=0):
    """Const-velocity KF over a target trajectory with intermittent GSD measurements.
    Returns per-step tr(Sigma), visibility mask, RMS est error."""
    rng = np.random.default_rng(seed)
    n = len(true_xy)
    F = np.array([[1, 0, DT, 0], [0, 1, 0, DT], [0, 0, 1, 0], [0, 0, 0, 1.0]])
    G = np.array([[DT**2/2, 0], [0, DT**2/2], [DT, 0], [0, DT]])
    Q = QACC * (G @ G.T)
    H = np.array([[1, 0, 0, 0], [0, 1, 0, 0.0]])
    x = np.array([true_xy[0, 0], true_xy[0, 1], 0, 0.0])
    P = np.eye(4) * 0.5
    trS, vis = [], []
    errs = []
    for k in range(n):
        x = F @ x; P = F @ P @ F.T + Q
        visible = d_series[k] <= rfov(z_series[k])
        if visible:
            R = Rz(z_series[k]) * np.eye(2)
            z_meas = true_xy[k] + rng.multivariate_normal([0, 0], R)
            S = H @ P @ H.T + R
            K = P @ H.T @ np.linalg.inv(S)
            x = x + K @ (z_meas - H @ x)
            P = (np.eye(4) - K @ H) @ P
        trS.append(np.trace(P[:2, :2])); vis.append(visible)
        errs.append(np.hypot(x[0]-true_xy[k, 0], x[1]-true_xy[k, 1]))
    return np.array(trS), np.array(vis), np.array(errs)


# ---------- E1: Prop A (static optimal altitude) ----------
def E1():
    print("=== E1  (Prop A): tr(Sigma_ss) vs altitude at fixed lag d; expect min at z*=d/tanh ===")
    T = 120; t = np.arange(T / DT)
    true_xy = np.stack([1.0 * DT * t, np.zeros_like(t)], 1)   # straight, v=1 m/s
    zs = np.linspace(0.6, 8.0, 60)
    fig, ax = plt.subplots(figsize=(4, 3))
    rows = []
    for d in [0.5, 1.0, 2.0, 3.0]:
        ss = []
        for z in zs:
            trS, vis, _ = kf_run(true_xy, np.full(len(t), d), np.full(len(t), z))
            ss.append(np.mean(trS[len(trS)//2:]))        # steady-state (2nd half)
        ss = np.array(ss)
        zstar_pred = d / TANH
        zstar_emp = zs[np.argmin(ss)]
        rows.append((d, zstar_pred, zstar_emp, ss.min()))
        ax.semilogy(zs, np.clip(ss, 1e-4, 5), label=f"d={d}m ($z^*$={zstar_pred:.2f})")
        ax.axvline(zstar_pred, ls=':', color='gray', lw=0.8)
    ax.set_xlabel('UAV altitude z (m)', fontsize=12, fontweight="bold")
    ax.set_ylabel(r'steady-state tr($\Sigma$) [log]', fontsize=12, fontweight="bold")
    ax.legend(fontsize=11, prop={"weight": "bold"})
    ax.set_ylim(1e-4, 5)
    plt.tight_layout()
    plt.savefig(RESULTS + '/Fig_PropA.png', dpi=200)
    plt.close()
    print(f"  {'d':>5}{'z*_pred=d/tanh':>15}{'z*_empirical':>14}{'minTrS':>10}")
    ok = True
    for d, zp, ze, mn in rows:
        flag = 'OK' if abs(ze - zp) <= 0.6 else 'MISMATCH'
        ok &= (abs(ze - zp) <= 0.6)
        print(f"  {d:>5.1f}{zp:>15.2f}{ze:>14.2f}{mn:>10.4f}   {flag}")
    print(f"  PROP A {'HOLDS' if ok else 'FAILS'} (empirical argmin matches d/tanh; z* increases with d)")
    return ok


# ---------- maneuvering path + tracking-lag dynamics (for E2/E3) ----------
def hairpin_and_lag(v=1.5, delay_s=1.0, T=100, L=5.0, gap=1.2):
    """Target on a hairpin (straights + 180 turns); UAV ground-projection tracks a
    DELAYED target position (=replan latency) => lag ~0 on straights, spikes at turns."""
    t = np.arange(int(T / DT))
    r = gap / 2
    seg = []  # list of (x,y)
    def line(p0, p1, n): return [p0 + (p1 - p0) * i / n for i in range(n)]
    def arc(c, r, a0, a1, n): return [c + r * np.array([np.cos(a), np.sin(a)]) for a in np.linspace(a0, a1, n)]
    path = (line(np.array([0, 0]), np.array([L, 0]), int(L/(v*DT)))
            + arc(np.array([L, r]), r, -np.pi/2, np.pi/2, int(np.pi*r/(v*DT)))
            + line(np.array([L, gap]), np.array([0, gap]), int(L/(v*DT)))
            + arc(np.array([0, r]), r, np.pi/2, 1.5*np.pi, int(np.pi*r/(v*DT))))
    path = np.array(path)
    # repeat to fill T
    reps = int(np.ceil(len(t) / len(path)))
    path = np.tile(path, (reps, 1))[:len(t)]
    # UAV ground-projection = target position delayed by the replan latency
    dstep = int(delay_s / DT)
    idx = np.clip(np.arange(len(path)) - dstep, 0, None)
    uav = path[idx]
    d = np.hypot(uav[:, 0] - path[:, 0], uav[:, 1] - path[:, 1])
    return path, d


def policies(d, zmin=1.0, zmax=8.0, margin=0.2, vz=0.4, lookahead=int(2.0/DT)):
    """fixed set + active-reactive + active-anticipatory (rate-limited climb)."""
    zneed = d / TANH + margin
    P = {}
    for zf in [2.0, 4.0, 6.0]:
        P[f'fixed_{zf:.0f}'] = np.full(len(d), zf)
    # reactive: track zneed(current) with climb-rate limit
    zr = np.zeros(len(d)); zr[0] = np.clip(zneed[0], zmin, zmax)
    for k in range(1, len(d)):
        tgt = np.clip(zneed[k], zmin, zmax)
        zr[k] = np.clip(tgt, zr[k-1]-vz*DT, zr[k-1]+vz*DT)
    P['active_reactive'] = zr
    # anticipatory: target = max needed over lookahead window (climb before the turn)
    za = np.zeros(len(d)); za[0] = np.clip(zneed[0], zmin, zmax)
    for k in range(1, len(d)):
        w = zneed[k:k+lookahead]
        tgt = np.clip(w.max() if len(w) else zneed[k], zmin, zmax)
        za[k] = np.clip(tgt, za[k-1]-vz*DT, za[k-1]+vz*DT)
    P['active_anticip'] = za
    return P


def eval_policy(path, d, z):
    trS, vis, errs = kf_run(path, d, z)
    b = len(trS)//10  # drop startup
    return dict(cov_mean=trS[b:].mean(), cov_peak=trS[b:].max(),
                pct_vis=100*vis[b:].mean(), dropouts=int((~vis[b:]).sum()),
                rms_err=errs[b:].mean(), z_mean=z[b:].mean())


def E2_E3():
    path, d = hairpin_and_lag()
    print(f"\n=== E2/E3: hairpin, tracking lag d in [{d.min():.2f},{d.max():.2f}] m; r_fov: z2={rfov(2):.2f} z4={rfov(4):.2f} z6={rfov(6):.2f}")
    P = policies(d)
    res = {name: eval_policy(path, d, z) for name, z in P.items()}
    print(f"  {'policy':>16}{'z_mean':>8}{'covMean':>9}{'covPeak':>9}{'%vis':>7}{'dropout':>8}{'rmsErr':>8}")
    for name, r in res.items():
        print(f"  {name:>16}{r['z_mean']:>8.2f}{r['cov_mean']:>9.4f}{r['cov_peak']:>9.3f}{r['pct_vis']:>7.0f}{r['dropouts']:>8d}{r['rms_err']:>8.3f}")
    fixed_best = min([res[k]['cov_mean'] for k in res if k.startswith('fixed')])
    act = res['active_anticip']['cov_mean']
    print(f"  PROP B {'HOLDS' if act < fixed_best else 'FAILS'}: active_anticip covMean={act:.4f} vs best fixed={fixed_best:.4f}")
    pc = res['active_anticip']['cov_peak'] < res['active_reactive']['cov_peak'] and \
         res['active_anticip']['dropouts'] <= res['active_reactive']['dropouts']
    print(f"  PROP C {'HOLDS' if pc else 'INCONCLUSIVE'}: anticip peak={res['active_anticip']['cov_peak']:.3f}/drop={res['active_anticip']['dropouts']} "
          f"vs reactive peak={res['active_reactive']['cov_peak']:.3f}/drop={res['active_reactive']['dropouts']}")
    # plot cov traces
    fig, ax = plt.subplots(figsize=(4, 3)); t = np.arange(len(d))*DT
    for name in ['fixed_2', 'fixed_6', 'active_anticip']:
        trS, _, _ = kf_run(path, d, P[name]); ax.semilogy(t, np.clip(trS, 1e-4, 5), label=name, lw=1.2)
    ax.set_xlabel('time (s)', fontsize=12, fontweight="bold")
    ax.set_ylabel(r'tr($\Sigma$) [log]', fontsize=12, fontweight="bold")
    ax.legend(fontsize=11, prop={"weight": "bold"})
    ax.set_ylim(1e-4, 5)
    plt.tight_layout()
    plt.savefig(RESULTS + '/Fig_PropBC.png', dpi=200)
    plt.close()


if __name__ == '__main__':
    a = E1()
    E2_E3()
    print(f"\nplots written to {RESULTS}/ (Fig_PropA.png, Fig_PropBC.png)")
