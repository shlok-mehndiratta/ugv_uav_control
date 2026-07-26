#!/usr/bin/env python3
"""Standard track family for the ACODS paper — ONE reproducible set, used across every
figure/table so all comparisons are apples-to-apples.

Design (addresses: differentiate by CURVATURE not scale; wide lane; same footprint):
  * SAME outer footprint for all tracks: bottom straight of width W=9 m, serpentine top
    oscillating about y0, gentle semicircular ends -> identical bounding box.
  * Differentiation = NUMBER + SHARPNESS of turns (winding), NOT overall size:
      P1 gentle   : 2 bumps (radius 2.25 m)   -> few, wide bends       (kappa_max~0.44)
      P2 moderate : 4 bumps (radius 1.12 m)   -> more, tighter bends    (kappa_max~0.89)
      P3 sharp    : 6 bumps (radius 0.75 m)   -> many, sharp bends      (kappa_max~1.33)
      P4 complex  : mixed radii [2.0,0.7,0.7,1.1] -> one gentle + sharp bends + straights
                    (VARYING curvature along the path) -> adaptive-altitude showcase + co-design
  * WIDE lane: corridor half-offset O=0.6 m -> ~1.2 m drivable lane (>2x the old ~0.5 m),
    and every turn radius > O so the corridor never self-intersects (UGV can follow).
Prints curvature/navigability stats and writes a preview.
"""
import os, numpy as np
from PIL import Image, ImageDraw

# Repo-relative: this script lives in ros2_ws/scripts/, so ROOT = ros2_ws/
ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..')) + '/'
TEX  = ROOT + 'src/ugv_uav_control/models/path_ground_plane_v2/materials/textures/'
INST = ROOT + 'install/ugv_uav_control/share/ugv_uav_control/models/path_ground_plane_v2/materials/textures/'
PREVIEW = os.environ.get('PREVIEW_DIR', '/tmp') + '/standard_tracks_preview.png'
RES = 1332; S = RES / 12.0; C = RES / 2.0           # 111 px/m, world (0,0) at image centre

W = 10.0; YB = -3.0; Y0 = 1.5; CR = 0.75             # rounded-rect base: bottom y=-3, top baseline y=1.5
XL, XR = -W/2, W/2                                    # vertical sides at x=+-5 -> bbox ~11 m wide
CORRIDOR = 0.55                                      # -> ~1.1 m lane (>2x the old ~0.5 m)
LW = 8
SMOOTH_SIGMA = 10                                    # periodic Gaussian smoothing (points) -> no kinks

def _arc(xc, yc, r, a0, a1, n=60):
    return [(xc + r*np.cos(t), yc + r*np.sin(t)) for t in np.linspace(a0, a1, n)]
def _line(p0, p1, n=80):
    return [(p0[0] + (p1[0]-p0[0])*i/n, p0[1] + (p1[1]-p0[1])*i/n) for i in range(1, n+1)]

def serp_RtoL(radii):
    """Wavy top traversed RIGHT->LEFT, ALTERNATING up/down semicircular bends about y=Y0.
    First (rightmost) bend is UP so it continues the right side (which arrives going +y);
    with an ODD bend count the leftmost bend is also UP, so the left side (going -y) also
    joins smoothly. All internal joins share a vertical tangent -> NO cusps, NO offset loop.
    radii ordered right-to-left; sum(2*radii)=W."""
    pts = []; x = XR
    for i, rb in enumerate(radii):
        xc = x - rb
        a0, a1 = (0.0, np.pi) if i % 2 == 0 else (0.0, -np.pi)      # up : down (over : under)
        pts += _arc(xc, Y0, rb, a0, a1, max(24, int(rb*50)))
        x -= 2*rb
    return pts

def smooth_closed(p, sigma):
    """Periodic Gaussian smoothing of a closed polyline -> rounds any residual joins."""
    if sigma <= 0: return p
    r = int(3*sigma); k = np.exp(-np.arange(-r, r+1)**2 / (2.0*sigma**2)); k /= k.sum()
    out = np.empty_like(p)
    for j in (0, 1):
        pad = np.concatenate([p[-r:, j], p[:, j], p[:r, j]])
        out[:, j] = np.convolve(pad, k, 'valid')
    return out

def loop(radii):
    assert abs(sum(2*r for r in radii) - W) < 1e-2, f"radii must sum to W/2={W/2}"
    p  = _line((XL+CR, YB), (XR-CR, YB))                            # bottom straight -> (spawn here)
    p += _arc(XR-CR, YB+CR, CR, -np.pi/2, 0.0, 30)                  # bottom-right rounded corner
    p += _line((XR, YB+CR), (XR, Y0))                              # right side (vertical, up)
    p += serp_RtoL(radii)[1:]                                      # wavy top (R->L), vertical-tangent ends
    p += _line((XL, Y0), (XL, YB+CR))                              # left side (vertical, down)
    p += _arc(XL+CR, YB+CR, CR, np.pi, 1.5*np.pi, 30)              # bottom-left rounded corner
    return smooth_closed(resample(np.array(p), 1200), SMOOTH_SIGMA)

def offset(p, w):
    t = np.gradient(p, axis=0); t /= (np.linalg.norm(t, axis=1, keepdims=True) + 1e-9)
    n = np.stack([-t[:, 1], t[:, 0]], 1); return p + w*n, p - w*n

def resample(p, n=1600):
    d = np.r_[0, np.cumsum(np.linalg.norm(np.diff(p, axis=0), axis=1))]
    u = np.linspace(0, d[-1], n)
    return np.stack([np.interp(u, d, p[:, 0]), np.interp(u, d, p[:, 1])], 1)

def curv(p):
    dx = np.gradient(p[:, 0]); dy = np.gradient(p[:, 1])
    ddx = np.gradient(dx); ddy = np.gradient(dy); ds = np.hypot(dx, dy)
    return np.clip(np.abs(dx*ddy - dy*ddx) / (ds**3 + 1e-9), 0, 20), ds

def stats(p):
    k, ds = curv(p); perim = ds.sum()
    return dict(mean_kappa=(k*ds).sum()/perim, kmax=np.percentile(k, 99.5),
                min_turn_R=1.0/max(np.percentile(k, 99.5), 1e-6),
                straight=100*(k < 0.2).mean(), perim=perim,
                bbox=(p[:,0].min(), p[:,0].max(), p[:,1].min(), p[:,1].max()))

def render(name, center):
    img = Image.new('RGB', (RES, RES), 'white'); dr = ImageDraw.Draw(img)
    inn, out = offset(center, CORRIDOR)
    for poly in (inn, out):
        px = [(C + x*S, C + y*S) for x, y in poly]
        dr.line(px + [px[0]], fill=(20, 20, 160), width=LW, joint='curve')
    for d in (TEX, INST):
        try: os.makedirs(d, exist_ok=True); img.save(d + name + '.png')
        except Exception as e: print('  (skip', d, e, ')')

# Names: Low-κ / Mid-κ / High-κ / Var-κ (graded by mean curvature; Var-κ = variable curvature).
# Filesystem-safe ids lowk/midk/highk/vark; display names carry the κ.
TRACKS = [('track_lowk',  loop([5/3]*3),                     'Low-κ  (3 bends, R=1.67 m)'),
          ('track_midk',  loop([1.0]*5),                     'Mid-κ  (5 bends, R=1.0 m)'),
          ('track_highk', loop([5/7]*7),                     'High-κ (7 bends, R=0.71 m)'),
          ('track_vark',  loop([1.45, 1.45, 0.7, 0.7, 0.7]), 'Var-κ  (2 gentle + 3 sharp bends)')]

if __name__ == '__main__':
    import matplotlib; matplotlib.use('Agg'); import matplotlib.pyplot as plt
    from matplotlib.collections import LineCollection
    fig, axs = plt.subplots(2, 2, figsize=(11, 8)); axs = axs.ravel()
    print(f"{'track':16s}{'design':>20}{'meanK':>7}{'lane_m':>8}{'straight%':>10}{'perim_m':>9}{'bbox_wxh':>14}")
    for i, (name, cd, label) in enumerate(TRACKS):
        render(name, cd); c = resample(cd); st = stats(c); k, _ = curv(c)
        bw = st['bbox'][1]-st['bbox'][0]; bh = st['bbox'][3]-st['bbox'][2]
        print(f"{name:16s}{label:>20}{st['mean_kappa']:7.3f}{2*CORRIDOR:8.2f}"
              f"{st['straight']:10.0f}{st['perim']:9.1f}{f'{bw:.1f}x{bh:.1f}':>14}")
        ax = axs[i]
        segs = np.concatenate([c[:-1, None], c[1:, None]], axis=1)
        lc = LineCollection(segs, cmap='turbo', norm=plt.Normalize(0, 1.6)); lc.set_array(k[:-1]); lc.set_linewidth(5)
        ax.add_collection(lc)
        ax.set_title(f"{name}   {label}\nmeanκ={st['mean_kappa']:.2f}  footprint {bw:.1f}×{bh:.1f} m  lane {2*CORRIDOR:.1f} m",
                     fontsize=9, fontweight='bold')
        ax.set_aspect('equal'); ax.set_xlim(-6, 6); ax.set_ylim(-4, 5); ax.grid(alpha=0.3)
        ax.set_xlabel('x (m)'); ax.set_ylabel('y (m)')
    sm = plt.cm.ScalarMappable(cmap='turbo', norm=plt.Normalize(0, 1.6))
    fig.colorbar(sm, ax=axs, shrink=0.6, label='local curvature κ (rad/m)')
    fig.suptitle(f'Standard track family — SAME {W:.0f} m footprint, ~{2*CORRIDOR:.1f} m lane, graded by number/sharpness of bends',
                 fontweight='bold')
    plt.savefig(PREVIEW, dpi=140, bbox_inches='tight'); print('preview ->', PREVIEW)
