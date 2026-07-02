#!/usr/bin/env python3
"""Generate a 'circuit' track texture: long straights + a few discrete rounded-sharp
corners, so the curvature trigger (kappa>0.4) fires ONLY at corners (clean B-vs-C3
comparison). 1332x1332, boundary padding, lane passes through the UGV spawn (0.5,0).

The world->texture-pixel orientation is derived empirically by overlaying a known
driven trajectory on track2 (so Gazebo renders our lane at the intended world coords).
"""
import glob
import numpy as np
import pandas as pd
from PIL import Image, ImageDraw

ROOT = '/home/omen/CRAZYFLY_SITL/ugv_uav_control/03_System_Integration_ROS2/'
TEX = ROOT + 'src/ugv_uav_control/models/path_ground_plane_v2/materials/textures/'
PLANE_M = 12.0          # ground plane size (m)
OUT_RES = 1332          # requested resolution
SCALE = OUT_RES / PLANE_M   # px per metre for the NEW texture
C = OUT_RES / 2.0           # world (0,0) -> image centre

ORIENTS = [lambda x, y: (x, y), lambda x, y: (-x, y), lambda x, y: (x, -y),
           lambda x, y: (-x, -y), lambda x, y: (y, x), lambda x, y: (-y, x),
           lambda x, y: (y, -x), lambda x, y: (-y, -x)]


def derive_orientation():
    img = np.array(Image.open(TEX + 'track2.png').convert('RGB'))
    H, W = img.shape[:2]
    blue = (img[:, :, 2] > 100) & (img[:, :, 0] < 100) & (img[:, :, 1] < 100)
    s = W / PLANE_M
    c = W / 2.0
    d = pd.read_csv(sorted(glob.glob(ROOT + 'results/C3_nominal_r1_*.csv'))[0])
    d = d[d.timestamp >= 8]
    wx, wy = d.true_ugv_x.values, d.true_ugv_y.values
    best = (0, -1)
    for m, f in enumerate(ORIENTS):
        u, v = f(wx, wy)
        a = (c + u * s).astype(int); b = (c + v * s).astype(int)
        ok = (a >= 0) & (a < W) & (b >= 0) & (b < H); a, b = a[ok], b[ok]
        if not len(a):
            continue
        hit = sum(blue[max(0, j - 30):j + 30, max(0, i - 30):i + 30].any()
                  for i, j in zip(a[::3], b[::3]))
        frac = hit / len(a[::3])
        if frac > best[1]:
            best = (m, frac)
    print(f"derived world->pixel orientation = {best[0]} (track2 overlap {best[1]:.2f})")
    return best[0]


def rounded_rect_centerline(cx, cy, a, b, r, n_straight=60, n_arc=40):
    """Closed rounded-rectangle centreline in world coords (CCW)."""
    pts = []
    # centres of the four corner arcs
    corners = [(cx + a - r, cy - b + r, -np.pi / 2, 0.0),      # bottom-right
               (cx + a - r, cy + b - r, 0.0, np.pi / 2),       # top-right
               (cx - a + r, cy + b - r, np.pi / 2, np.pi),     # top-left
               (cx - a + r, cy - b + r, np.pi, 1.5 * np.pi)]   # bottom-left
    # bottom straight (left->right) at y = cy-b
    xs = np.linspace(cx - a + r, cx + a - r, n_straight)
    pts += [(x, cy - b) for x in xs]
    for (ox, oy, t0, t1) in corners:
        ts = np.linspace(t0, t1, n_arc)
        pts += [(ox + r * np.cos(t), oy + r * np.sin(t)) for t in ts]
        # straight after each corner handled implicitly by next corner's start;
        # add explicit straights for right/top/left edges:
        if (ox, oy) == corners[0][:2]:   # after bottom-right -> right straight up
            pts += [(cx + a, y) for y in np.linspace(cy - b + r, cy + b - r, n_straight)]
        elif (ox, oy) == corners[1][:2]:  # after top-right -> top straight (right->left)
            pts += [(x, cy + b) for x in np.linspace(cx + a - r, cx - a + r, n_straight)]
        elif (ox, oy) == corners[2][:2]:  # after top-left -> left straight (top->bottom)
            pts += [(cx - a, y) for y in np.linspace(cy + b - r, cy - b + r, n_straight)]
    return np.array(pts)


def offset_polyline(pts, w):
    """Return inner/outer offset closed polylines by +/- w along local normal."""
    p = pts
    tang = np.gradient(p, axis=0)
    tang /= (np.linalg.norm(tang, axis=1, keepdims=True) + 1e-9)
    nrm = np.stack([-tang[:, 1], tang[:, 0]], axis=1)   # left normal
    return p + w * nrm, p - w * nrm


def to_px(world_pts, orient):
    f = ORIENTS[orient]
    out = []
    for x, y in world_pts:
        u, v = f(x, y)
        out.append((C + u * SCALE, C + v * SCALE))
    return out


def main():
    orient = derive_orientation()
    # --- circuit geometry (world m): centred so bottom straight (y=0) hits spawn (0.5,0)
    cx, cy, a, b, r = 0.5, 1.3, 1.6, 1.3, 0.6      # half-width, half-height, corner radius
    corridor_half = 0.30                            # lane half-width (m)
    line_px = 11                                    # blue line thickness (px)
    center = rounded_rect_centerline(cx, cy, a, b, r)
    inner, outer = offset_polyline(center, corridor_half)

    img = Image.new('RGB', (OUT_RES, OUT_RES), 'white')
    dr = ImageDraw.Draw(img)
    for poly in (inner, outer):
        px = to_px(poly, orient)
        dr.line(px + [px[0]], fill=(20, 20, 160), width=line_px, joint='curve')

    # padding check
    allpx = np.array(to_px(np.vstack([inner, outer]), orient))
    pad = min(allpx[:, 0].min(), allpx[:, 1].min(),
              OUT_RES - allpx[:, 0].max(), OUT_RES - allpx[:, 1].max())
    print(f"lane pixel bbox x[{allpx[:,0].min():.0f},{allpx[:,0].max():.0f}] "
          f"y[{allpx[:,1].min():.0f},{allpx[:,1].max():.0f}]  min boundary padding = {pad:.0f}px "
          f"({pad/SCALE:.2f} m)")
    # curvature of centreline (world) for the report
    t = np.gradient(center, axis=0); ds = np.linalg.norm(t, axis=1)
    th = np.unwrap(np.arctan2(t[:, 1], t[:, 0])); k = np.abs(np.gradient(th)) / (ds + 1e-9)
    print(f"centreline curvature: straight frac(k<0.2)={100*(k<0.2).mean():.0f}%  "
          f"corner frac(k>0.4)={100*(k>0.4).mean():.0f}%  max k={k.max():.2f}  "
          f"perimeter={ds.sum():.1f} m")

    for d in (TEX, ROOT + 'install/ugv_uav_control/share/ugv_uav_control/models/'
              'path_ground_plane_v2/materials/textures/'):
        try:
            img.save(d + 'track_circuit.png'); print('wrote', d + 'track_circuit.png')
        except Exception as e:
            print('skip', d, e)


if __name__ == '__main__':
    main()
