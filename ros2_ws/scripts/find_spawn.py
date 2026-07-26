#!/usr/bin/env python3
"""Find an on-lane spawn (world x,y,yaw) for a track texture on the 12x12 m ground plane.
Corridor = the white RING enclosed between the two dark boundary lines. Spawn = widest
centerline point; heading = along the corridor (perpendicular to the dist-transform gradient)."""
import sys, numpy as np, cv2, os

# Repo-relative: this script lives in ros2_ws/scripts/, textures live under ros2_ws/src/...
_HERE = os.path.dirname(os.path.abspath(__file__))
TEX = os.path.abspath(os.path.join(
    _HERE, "..", "src", "ugv_uav_control", "models",
    "path_ground_plane_v2", "materials", "textures"))
OUT=os.environ.get("SPAWN_OUT", "/tmp")
PLANE=12.0

def find_spawn(track):
    im=cv2.imread(f"{TEX}/{track}", cv2.IMREAD_GRAYSCALE); H,W=im.shape
    scale=PLANE/W
    white=(im>100).astype(np.uint8)
    # connected components of white; background = component touching the border
    n,lab=cv2.connectedComponents(white)
    border=set(lab[0,:]) | set(lab[-1,:]) | set(lab[:,0]) | set(lab[:,-1])
    best=None
    for l in range(1,n):
        if l in border: continue
        comp=(lab==l)
        area=comp.sum()
        if area < 0.002*H*W: continue           # drop tiny holes
        ys,xs=np.where(comp); cy,cx=int(ys.mean()),int(xs.mean())
        is_ring = not comp[cy,cx]                # ring: centroid not inside -> corridor
        # The drivable LANE is always the corridor RING (white annulus between the two
        # boundary lines); the big filled region is the enclosed hole -> skip non-rings.
        if not is_ring: continue
        score=area
        if best is None or score>best[0]: best=(score,comp,is_ring,area)
    comp=best[1]; is_ring=best[2]
    # distance transform within corridor -> widest centerline point = spawn
    dist=cv2.distanceTransform(comp.astype(np.uint8)*255, cv2.DIST_L2,5)
    maxd=dist.max(); win=int(1.2/scale)           # ~1.2 m PCA window
    # candidates: good-margin centerline pixels; pick the STRAIGHTEST (max PCA anisotropy)
    cand=np.argwhere(dist > 0.55*maxd)
    cand=cand[::max(1,len(cand)//400)]            # subsample for speed
    best=(-1,None,None)
    for (rr,cc) in cand:
        ys,xs=np.where(comp[max(0,rr-win):rr+win, max(0,cc-win):cc+win])
        if len(xs)<20: continue
        pts=np.c_[xs,ys].astype(float); pts-=pts.mean(0)
        s=np.linalg.svd(pts, full_matrices=False, compute_uv=False)
        aniso=s[0]/(s[1]+1e-6)                     # high => locally straight corridor
        if aniso>best[0]: best=(aniso,rr,cc)
    _,r,c=best; halfw=dist[r,c]*scale
    ys,xs=np.where(comp[max(0,r-win):r+win, max(0,c-win):c+win])
    pts=np.c_[xs,ys].astype(float); pts-=pts.mean(0)
    _,_,V=np.linalg.svd(pts, full_matrices=False)
    dcol,drow=V[0]                                # principal axis = along corridor (image frame)
    # world mapping (convention C, matches camera dx/dy_world in code):
    #   wx = -(row-H/2)*scale ; wy = -(col-W/2)*scale
    wx=-(r-H/2)*scale; wy=-(c-W/2)*scale
    yaw=np.arctan2(-dcol, -drow)                  # dwy=-dcol*scale, dwx=-drow*scale
    # overlay
    vis=cv2.cvtColor(im,cv2.COLOR_GRAY2BGR)
    vis[comp]=(0,180,0)                           # corridor green
    cv2.circle(vis,(c,r),12,(0,0,255),-1)         # spawn red
    tip=(int(c+120*dcol),int(r+120*drow))
    cv2.arrowedLine(vis,(c,r),tip,(255,0,0),6)    # heading blue
    cv2.imwrite(f"{OUT}/spawn_{track}", cv2.resize(vis,(700,700)))
    return dict(track=track,x=round(wx,2),y=round(wy,2),yaw=round(float(yaw),2),
                lane_halfwidth_m=round(halfw,2),ring=is_ring)

if __name__=="__main__":
    tracks=sys.argv[1:] or ["track2.png","track1.png","track3.png","track4.png","track_hairpin.png","track_slalom.png"]
    for t in tracks:
        try: print(find_spawn(t))
        except Exception as e: print(t,"ERROR",e)
