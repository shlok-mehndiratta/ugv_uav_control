#!/usr/bin/env python3
"""
Rebuild the two testbed panels from the real captured frame, as SEPARATE images
with large, clean annotations (the old panel (b) text was too small):
    Fig_Testbed1.png  (a) raw downward view
    Fig_Testbed2.png  (b) same frame -> lane -> waypoint (navigation) + ArUco -> pose (estimation)
Repo-relative paths; output into New Paper/media/.

Note: cv2.aruco.detectMarkers segfaults in this environment, so the marker is located
geometrically (the dark, marker-sized blob inside the corridor that does not touch the
image border, unlike the track edges). This only positions the green box, not the pose.
"""
import os
import cv2
import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

HERE = os.path.dirname(os.path.abspath(__file__))
FR = os.path.join(HERE, "..", "..", "ros2_ws", "paper_data", "fig1_testbed", "testbed_frame.png")
MEDIA = os.path.normpath(os.path.join(HERE, "..", "media"))

img = cv2.imread(FR)
crop = img[:, 95:545]
raw = cv2.cvtColor(crop, cv2.COLOR_BGR2RGB)
H, W = crop.shape[:2]
gray = cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
_, thresh = cv2.threshold(gray, 100, 255, cv2.THRESH_BINARY)      # white = drivable ground
h, w = thresh.shape[:2]

# ---- marker (aruco-free): dark, marker-sized blob not touching the border ----
_, dark = cv2.threshold(gray, 90, 255, cv2.THRESH_BINARY_INV)
dark = cv2.dilate(dark, np.ones((11, 11), np.uint8))
n, lbl, st, cen = cv2.connectedComponentsWithStats(dark, 8)
best = None
for i in range(1, n):
    x, y, ww, hh, area = st[i]
    touches = x <= 1 or y <= 1 or x + ww >= W - 1 or y + hh >= H - 1
    ar = ww / float(hh) if hh else 9
    if touches or area < 1500 or area > 40000 or ar < 0.5 or ar > 2.0:
        continue
    dc = (cen[i][0] - W / 2) ** 2 + (cen[i][1] - H / 2) ** 2
    if best is None or dc < best[0]:
        best = (dc, (x, y, ww, hh), cen[i])
if best is None:
    raise SystemExit("marker blob not found")
(mx0, my0, mw, mh), mc = best[1], best[2]
cx, cy = int(mc[0]), int(mc[1])

# ---- corridor flood-fill + distance transform (navigation heatmap) ----
seed = None
for r in range(80, 240, 6):
    for ang in np.linspace(0, 2 * np.pi, 24, endpoint=False):
        sx, sy = int(cx + r * np.cos(ang)), int(cy + r * np.sin(ang))
        if 0 <= sx < w and 0 <= sy < h and thresh[sy, sx] == 255:
            seed = (sx, sy); break
    if seed:
        break
lane_mask = np.zeros_like(thresh)
if seed is not None:
    fm = np.zeros((h + 2, w + 2), np.uint8); li = thresh.copy()
    cv2.floodFill(li, fm, seed, 128); lane_mask[li == 128] = 255
else:
    lane_mask = thresh.copy()
dist = cv2.distanceTransform(thresh, cv2.DIST_L2, 5); dist[lane_mask == 0] = 0
heat = cv2.applyColorMap((((dist / (dist.max() or 1.0)) ** 0.5) * 255).astype(np.uint8), cv2.COLORMAP_JET)
ann = cv2.addWeighted(crop, 0.55, heat, 0.45, 0)

# ---- waypoint = corridor-center peak in a lookahead annulus around the marker ----
dpk = dist.copy()
yy, xx = np.ogrid[:h, :w]; rr = np.hypot(xx - cx, yy - cy)
dpk[(rr < 70) | (rr > 190)] = 0
_, _, _, peak = cv2.minMaxLoc(dpk)

# ---- draw markers ----
pad = 8
cv2.rectangle(ann, (mx0 - pad, my0 - pad), (mx0 + mw + pad, my0 + mh + pad), (0, 255, 0), 4)
cv2.circle(ann, peak, 12, (0, 0, 0), -1); cv2.circle(ann, peak, 12, (255, 255, 255), 3)
ann = cv2.cvtColor(ann, cv2.COLOR_BGR2RGB)
print("marker center:", (cx, cy), "waypoint:", peak)

BOX = dict(boxstyle="round,pad=0.32", fc="black", alpha=0.72)


def save_panel(image, out, annotate=False):
    fig, ax = plt.subplots(figsize=(3.6, 3.6 * H / W))
    ax.imshow(image); ax.set_xticks([]); ax.set_yticks([])
    for s in ax.spines.values():
        s.set_visible(False)
    if annotate:
        ax.annotate("ArUco $\\rightarrow$ pose", xy=(cx, my0 - pad),
                    xytext=(0.28 * W, 0.09 * H), fontsize=14, color="white", weight="bold",
                    ha="left", va="center",
                    arrowprops=dict(arrowstyle="->", color="lime", lw=2.6),
                    bbox=dict(ec="lime", **BOX))
        ax.annotate("lane $\\rightarrow$ waypoint", xy=peak,
                    xytext=(peak[0], 0.95 * H), fontsize=13.5, color="white", weight="bold",
                    ha="center", va="top",
                    arrowprops=dict(arrowstyle="-|>", color="black", lw=2.0,
                                    mutation_scale=12, shrinkA=3, shrinkB=7),
                    bbox=dict(ec="white", **BOX))
    fig.savefig(os.path.join(MEDIA, out), dpi=300, bbox_inches="tight", pad_inches=0.02)
    plt.close(fig)
    print("wrote", os.path.join(MEDIA, out))


save_panel(raw, "Fig_Testbed1.png", annotate=False)
save_panel(ann, "Fig_Testbed2.png", annotate=True)
