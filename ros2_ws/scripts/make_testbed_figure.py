#!/usr/bin/env python3
"""Build the 'single-camera dual-role' figure from a real captured frame:
(a) raw UAV downward view, (b) same frame -> lane segmentation (navigation) + ArUco (estimation)."""
import cv2, numpy as np, matplotlib
matplotlib.use('Agg')
import matplotlib.pyplot as plt
from matplotlib.patches import FancyArrowPatch

SP="/tmp/claude-1000/-home-omen-CRAZYFLY-SITL-ugv-uav-control/a35db6a5-7828-43aa-a1f3-f7f81b0184ca/scratchpad"
FR=f"{SP}/frames/live_251.png"
OUT=f"{SP}/Fig_Testbed.png"

img=cv2.imread(FR)
crop=img[:, 95:545]           # trim the peripheral edge geometry, keep curve+marker
raw=cv2.cvtColor(crop, cv2.COLOR_BGR2RGB)
H,W=crop.shape[:2]

# --- annotated copy: pipeline distance-transform heatmap (navigation) + ArUco (estimation) ---
gray=cv2.cvtColor(crop, cv2.COLOR_BGR2GRAY)
_,thresh=cv2.threshold(gray,100,255,cv2.THRESH_BINARY)   # white = drivable ground (pipeline)
h,w=thresh.shape[:2]

# (1) ArUco detection first (real, same dict/params as the pipeline)
d=cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_250)
try: p=cv2.aruco.DetectorParameters()
except AttributeError: p=cv2.aruco.DetectorParameters_create()
p.minMarkerPerimeterRate=0.01
if hasattr(cv2.aruco,'ArucoDetector'):
    corners,ids,_=cv2.aruco.ArucoDetector(d,p).detectMarkers(crop)
else:
    corners,ids,_=cv2.aruco.detectMarkers(crop,d,parameters=p)
cx=cy=None; mrad=60
if ids is not None:
    c=corners[0][0].astype(int); cx,cy=int(c[:,0].mean()),int(c[:,1].mean())

# (2) corridor flood-fill from a white GROUND seed (offset beyond the marker), distance transform
scx,scy=(cx,cy) if cx is not None else (W//2,H//2)
seed=None
for r in range(80,240,6):           # start beyond the marker (~90px) so we seed ground, not the tag
    for ang in np.linspace(0,2*np.pi,24,endpoint=False):
        sx=int(scx+r*np.cos(ang)); sy=int(scy+r*np.sin(ang))
        if 0<=sx<w and 0<=sy<h and thresh[sy,sx]==255: seed=(sx,sy); break
    if seed: break
lane_mask=np.zeros_like(thresh)
if seed is not None:
    fm=np.zeros((h+2,w+2),np.uint8); li=thresh.copy()
    cv2.floodFill(li,fm,seed,128); lane_mask[li==128]=255
else:
    lane_mask=thresh.copy()
dist=cv2.distanceTransform(thresh,cv2.DIST_L2,5); dist[lane_mask==0]=0
mx=dist.max() or 1.0
dnorm=(dist/mx)**0.5                                       # gamma -> legible gradient
heat=cv2.applyColorMap((dnorm*255).astype(np.uint8),cv2.COLORMAP_JET)
ann=cv2.addWeighted(crop,0.55,heat,0.45,0)               # pipeline-style final_img blend

# (3) waypoint = corridor-center (distance) peak in a LOOKAHEAD annulus around the robot
#     (the pipeline masks to a lookahead region ahead of the robot; we use a local annulus)
dpk=dist.copy()
if cx is not None:
    yy,xx=np.ogrid[:h,:w]; rr=np.hypot(xx-cx,yy-cy)
    dpk[(rr<70)|(rr>190)]=0          # exclude the tag patch (<70) and far-field (>190)
_,_,_,peak=cv2.minMaxLoc(dpk)

# (4) draw: green ArUco quad+box, yellow waypoint dot
if cx is not None:
    cv2.polylines(ann,[c.reshape(-1,1,2)],True,(0,255,0),3)
    cv2.rectangle(ann,(cx-52,cy-52),(cx+52,cy+52),(0,255,0),2)
cv2.circle(ann,peak,8,(0,255,255),-1); cv2.circle(ann,peak,8,(0,0,0),2)
ann=cv2.cvtColor(ann, cv2.COLOR_BGR2RGB)
print("aruco:", ids is not None, "center:", (cx,cy), "waypoint peak:", peak)

fig,ax=plt.subplots(1,2,figsize=(7.0,2.9))
ax[0].imshow(raw); ax[0].set_title("(a) UAV downward camera (the sole sensor)", fontsize=9)
ax[1].imshow(ann); ax[1].set_title("(b) one frame, dual role", fontsize=9)
for a in ax: a.set_xticks([]); a.set_yticks([])
# annotate dual role on panel b
if cx is not None:
    ax[1].annotate("ArUco  →  EKF (pose)", xy=(cx,cy-46), xytext=(W*0.30,H*0.10),
                   fontsize=8, color='white', weight='bold',
                   arrowprops=dict(arrowstyle='->',color='lime',lw=1.6),
                   bbox=dict(boxstyle='round,pad=0.2',fc='black',ec='lime',alpha=0.8))
ax[1].annotate("lane heatmap  →  waypoint", xy=(peak[0],peak[1]), xytext=(W*0.04,H*0.955),
               fontsize=8, color='white', weight='bold',
               arrowprops=dict(arrowstyle='->',color='yellow',lw=1.6),
               bbox=dict(boxstyle='round,pad=0.2',fc='black',ec='yellow',alpha=0.85))
plt.tight_layout(pad=0.4)
plt.savefig(OUT, dpi=300, bbox_inches='tight')
print("wrote", OUT)
