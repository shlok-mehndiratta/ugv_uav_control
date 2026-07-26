#!/usr/bin/env python3
"""Generate candidate track textures (1332x1332, boundary padding, lane through the
UGV spawn (0.5,0) which the UGV enters facing +x). Shapes chosen by the anticipation
physics: long straights (B=C3 parity) + few very sharp large-angle turns (max stale-plan
overshoot, where C3's curvature-anticipation can reduce peak tracking error).

Shapes: circuit (rounded rect), hairpin (paperclip: long straights + 180deg ends),
slalom (straight bottom + wavy top = several alternating sharp turns)."""
import glob, os
import numpy as np
import pandas as pd
from PIL import Image, ImageDraw

# Repo-relative: this script lives in ros2_ws/scripts/, so ROOT = ros2_ws/
ROOT = os.path.abspath(os.path.join(os.path.dirname(__file__), '..')) + '/'
TEX = ROOT + 'src/ugv_uav_control/models/path_ground_plane_v2/materials/textures/'
INST = ROOT + 'install/ugv_uav_control/share/ugv_uav_control/models/path_ground_plane_v2/materials/textures/'
RES = 1332; S = RES / 12.0; C = RES / 2.0
ORI = [lambda x, y: (x, y), lambda x, y: (-x, y), lambda x, y: (x, -y), lambda x, y: (-x, -y),
       lambda x, y: (y, x), lambda x, y: (-y, x), lambda x, y: (y, -x), lambda x, y: (-y, -x)]


def derive_orient():
    img = np.array(Image.open(TEX + 'track2.png').convert('RGB')); H, W = img.shape[:2]
    blue = (img[:, :, 2] > 100) & (img[:, :, 0] < 100) & (img[:, :, 1] < 100)
    s = W / 12.0; c = W / 2.0
    d = pd.read_csv(sorted(glob.glob(ROOT + 'results/C3_nominal_r1_*.csv'))[0]); d = d[d.timestamp >= 8]
    wx, wy = d.true_ugv_x.values, d.true_ugv_y.values; best = (5, -1)
    for m, f in enumerate(ORI):
        u, v = f(wx, wy); a = (c + u * s).astype(int); b = (c + v * s).astype(int)
        ok = (a >= 0) & (a < W) & (b >= 0) & (b < H); a, b = a[ok], b[ok]
        if not len(a): continue
        h = sum(blue[max(0, j - 30):j + 30, max(0, i - 30):i + 30].any() for i, j in zip(a[::3], b[::3]))
        if h / len(a[::3]) > best[1]: best = (m, h / len(a[::3]))
    return best[0]


def circuit():
    cx, cy, a, b, r = 0.5, 1.3, 1.6, 1.3, 0.6
    pts = []
    xs = np.linspace(cx - a + r, cx + a - r, 60); pts += [(x, cy - b) for x in xs]
    for ox, oy, t0, t1, edge in [(cx + a - r, cy - b + r, -np.pi / 2, 0, ('R',)),
                                 (cx + a - r, cy + b - r, 0, np.pi / 2, ('T',)),
                                 (cx - a + r, cy + b - r, np.pi / 2, np.pi, ('L',)),
                                 (cx - a + r, cy - b + r, np.pi, 1.5 * np.pi, ())]:
        pts += [(ox + r * np.cos(t), oy + r * np.sin(t)) for t in np.linspace(t0, t1, 40)]
        if edge == ('R',): pts += [(cx + a, y) for y in np.linspace(cy - b + r, cy + b - r, 50)]
        elif edge == ('T',): pts += [(x, cy + b) for x in np.linspace(cx + a - r, cx - a + r, 50)]
        elif edge == ('L',): pts += [(cx - a, y) for y in np.linspace(cy + b - r, cy - b + r, 50)]
    return np.array(pts)


def hairpin(cx=0.2, gap=0.9, L=2.4):
    """Paperclip: bottom straight y=0 (through spawn), sharp 180 semicircle ends."""
    cy = gap / 2.0; r = gap / 2.0; xl, xr = cx - L / 2, cx + L / 2; p = []
    p += [(x, cy - r) for x in np.linspace(xl, xr, 90)]                       # bottom straight ->
    p += [(xr + r * np.cos(t), cy + r * np.sin(t)) for t in np.linspace(-np.pi / 2, np.pi / 2, 60)]  # right hairpin
    p += [(x, cy + r) for x in np.linspace(xr, xl, 90)]                       # top straight <-
    p += [(xl + r * np.cos(t), cy + r * np.sin(t)) for t in np.linspace(np.pi / 2, 1.5 * np.pi, 60)]  # left hairpin
    return np.array(p)


def slalom(cx=0.5, w=2.6, ytop=2.2, nb=3, amp=0.5):
    """Straight bottom (through spawn) + wavy top (nb sharp bumps) + semicircle ends."""
    xl, xr = cx - w / 2, cx + w / 2; r = ytop / 2.0; cyr = ytop / 2.0; p = []
    p += [(x, 0.0) for x in np.linspace(xl, xr, 100)]                         # bottom straight ->
    p += [(xr + r * np.cos(t), cyr + r * np.sin(t)) for t in np.linspace(-np.pi / 2, np.pi / 2, 50)]  # right end up
    xs = np.linspace(xr, xl, 220)
    p += [(x, ytop + amp * np.sin(2 * np.pi * nb * (x - xr) / (xl - xr))) for x in xs]  # wavy top <-
    p += [(xl + r * np.cos(t), cyr + r * np.sin(t)) for t in np.linspace(np.pi / 2, 1.5 * np.pi, 50)]  # left end down
    return np.array(p)


def offset(p, w):
    t = np.gradient(p, axis=0); t /= (np.linalg.norm(t, axis=1, keepdims=True) + 1e-9)
    n = np.stack([-t[:, 1], t[:, 0]], 1); return p + w * n, p - w * n


def curv_stats(p):
    t = np.gradient(p, axis=0); ds = np.linalg.norm(t, axis=1)
    th = np.unwrap(np.arctan2(t[:, 1], t[:, 0])); k = np.abs(np.gradient(th)) / (ds + 1e-9)
    kk = np.clip(k, 0, 20)
    return dict(straight=100 * (kk < 0.2).mean(), corner=100 * (kk > 0.4).mean(),
                kmax=kk.max(), perim=ds.sum())


def render(name, center, orient, corridor=0.30, lw=11):
    img = Image.new('RGB', (RES, RES), 'white'); dr = ImageDraw.Draw(img)
    inn, out = offset(center, corridor)
    for poly in (inn, out):
        f = ORI[orient]; px = [(C + f(x, y)[0] * S, C + f(x, y)[1] * S) for x, y in poly]
        dr.line(px + [px[0]], fill=(20, 20, 160), width=lw, joint='curve')
    allpx = np.array([(C + ORI[orient](x, y)[0] * S, C + ORI[orient](x, y)[1] * S)
                      for x, y in np.vstack([inn, out])])
    pad = min(allpx[:, 0].min(), allpx[:, 1].min(), RES - allpx[:, 0].max(), RES - allpx[:, 1].max())
    cs = curv_stats(center)
    print(f"{name:9s}: straight={cs['straight']:.0f}% corner={cs['corner']:.0f}% "
          f"kmax={cs['kmax']:.1f} perim={cs['perim']:.1f}m padding={pad:.0f}px({pad/S:.2f}m)")
    for d in (TEX, INST):
        try: img.save(d + name + '.png')
        except Exception: pass


if __name__ == '__main__':
    o = derive_orient(); print(f"orientation={o}")
    render('track_circuit', circuit(), o)
    render('track_hairpin', hairpin(), o)
    render('track_slalom', slalom(), o)
    print('saved to', TEX)
