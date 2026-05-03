"""
pontryagin_visualizer.py
========================

Interactive 3-D visualisation of the Pontryagin-difference constraint
tightening used by the offline Tube MPC pipeline.

Mathematical mapping (defend this in the viva)
----------------------------------------------
The full state is 6-dimensional, x = [p_x, p_y, p_z, v_x, v_y, v_z]^T, but the
*physical* arena is a 3-D box.  All three sets we want to visualise live in
R^6 and we project them onto the position subspace R^3 for display.

    X       := { x in R^6 :  Hx        x  <=  hx        }        Physical arena
    Omega   := { e in R^6 :  Omega_H   e  <=  Omega_h   }        mRPI tube  (the
                                                                  worst-case
                                                                  closed-loop
                                                                  tracking error
                                                                  under
                                                                  A_k = A - B K)
    X_bar   := X (-) Omega
            =  { z in R^6 : z + e  in X     for every  e in Omega }
            =  { z in R^6 : Hx z  <=  hx  -  h_Omega(Hx_i^T) }

where  h_Omega(d) = sup_{e in Omega} d^T e   is the support function of the
mRPI set.  Each row of  Hx  is shifted inwards by the maximum extent of
Omega in that row's normal direction, so the nominal trajectory  z  has
exactly enough margin for the actuator-bounded tracking error  e  to live
inside  Omega  without ever pushing the *true* state  x = z + e  outside
the physical arena  X.  This is what makes the Tube MPC robustly safe.

Geometric projection from R^6 to R^3
------------------------------------
Projection of a convex polytope onto a coordinate subspace is the convex
hull of the projected vertices, so the recipe is:

    1. enumerate the vertices V in R^6 with pypoman.compute_polytope_vertices,
    2. drop the velocity coordinates: V_pos = V[:, :3],
    3. take the 3-D convex hull and hand it to Plotly.

For axis-aligned boxes (X, X_bar) the projection is trivial; we still run
it through the same routine so that the script remains correct if the
state polytope ever becomes non-box.

Run
---
    python pontryagin_visualizer.py            # writes pontryagin_visualization.html
    python pontryagin_visualizer.py --show     # also opens it in the browser
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import plotly.graph_objects as go
import pypoman
from scipy.spatial import ConvexHull

POSITION_DIMS = 3  # we keep p_x, p_y, p_z and discard v_x, v_y, v_z


def project_to_position(H: np.ndarray, h: np.ndarray, dims: int = POSITION_DIMS) -> np.ndarray:
    """
    Project { y in R^n : H y <= h } onto its first `dims` coordinates.
    Returns only the extreme points of the projected hull (deduplicated),
    which is all Plotly's Mesh3d needs when alphahull=0.
    """
    V_full = np.asarray(pypoman.compute_polytope_vertices(H, h))
    V_proj = V_full[:, :dims]
    hull = ConvexHull(V_proj)
    return V_proj[hull.vertices]


def mesh3d_from_points(points: np.ndarray, color: str, opacity: float, name: str) -> go.Mesh3d:
    """
    Build a Mesh3d trace from an unordered point cloud.  alphahull=0 tells
    Plotly to triangulate the convex hull internally, so we don't have to
    supply simplex indices.
    """
    return go.Mesh3d(
        x=points[:, 0],
        y=points[:, 1],
        z=points[:, 2],
        color=color,
        opacity=opacity,
        alphahull=0,
        flatshading=True,
        name=name,
        showlegend=True,
        hoverinfo="name",
    )


def axis_extents(V: np.ndarray) -> str:
    lo, hi = V.min(axis=0), V.max(axis=0)
    return (f"x in [{lo[0]:+.3f}, {hi[0]:+.3f}]  "
            f"y in [{lo[1]:+.3f}, {hi[1]:+.3f}]  "
            f"z in [{lo[2]:+.3f}, {hi[2]:+.3f}]")


def main() -> None:
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument(
        "--data",
        type=Path,
        default=Path(__file__).resolve().parent / "tube_data.npz",
        help="Path to tube_data.npz (default: alongside this script).",
    )
    parser.add_argument(
        "--out",
        type=Path,
        default=Path(__file__).resolve().parent / "pontryagin_visualization.html",
        help="Output HTML path.",
    )
    parser.add_argument("--show", action="store_true", help="Open the figure in the default browser after writing.")
    args = parser.parse_args()

    if not args.data.exists():
        raise FileNotFoundError(f"tube data not found at {args.data}")

    data = np.load(args.data)
    Hx, hx = data["Hx"], data["hx"]                            # 12 x 6,  12
    Omega_H, Omega_h = data["Omega_H"], data["Omega_h"]        # 72 x 6,  72
    Hx_tight, hx_tight = data["Hx_tight"], data["hx_tight"]    # 12 x 6,  12

    print("Loaded H-representations:")
    print(f"  X       (physical arena)   Hx       : {Hx.shape},  hx       : {hx.shape}")
    print(f"  Omega   (mRPI tube)        Omega_H  : {Omega_H.shape},  Omega_h  : {Omega_h.shape}")
    print(f"  X_bar   (X (-) Omega)      Hx_tight : {Hx_tight.shape},  hx_tight : {hx_tight.shape}")

    # 6-D -> 3-D position-space projection
    V_room  = project_to_position(Hx,       hx)
    V_omega = project_to_position(Omega_H,  Omega_h)
    V_tight = project_to_position(Hx_tight, hx_tight)

    print("\nPosition-space extents:")
    print(f"  X     : {axis_extents(V_room)}")
    print(f"  Omega : {axis_extents(V_omega)}")
    print(f"  X_bar : {axis_extents(V_tight)}")

    # Plotly traces.  Order matters: index 0 = room, 1 = tube, 2 = tightened.
    trace_room = mesh3d_from_points(
        V_room,  color="royalblue", opacity=0.08,
        name="Physical Arena Boundaries (X)",
    )
    trace_omega = mesh3d_from_points(
        V_omega, color="crimson",   opacity=0.95,
        name="mRPI Set / Worst-Case Error (Ω)",
    )
    trace_tight = mesh3d_from_points(
        V_tight, color="seagreen",  opacity=0.45,
        name="Tightened MPC Constraints (X̄ = X ⊖ Ω)",
    )

    fig = go.Figure(data=[trace_room, trace_omega, trace_tight])

    # ----- Interactive view toggles -------------------------------------------
    # Each visibility vector is in trace order [room, omega, tight].
    view_room_only      = [True,  False, False]
    view_room_plus_tube = [True,  True,  False]
    view_pontryagin     = [True,  False, True]
    view_all            = [True,  True,  True]

    base_title = "Pontryagin Difference  X̄ = X ⊖ Ω  (6-D state → 3-D position projection)"

    fig.update_layout(
        title=dict(text=base_title, x=0.5),
        scene=dict(
            xaxis_title="p_x  [m]",
            yaxis_title="p_y  [m]",
            zaxis_title="p_z  [m]",
            aspectmode="data",
            camera=dict(eye=dict(x=1.6, y=1.6, z=1.0)),
        ),
        legend=dict(itemsizing="constant"),
        margin=dict(l=0, r=0, t=80, b=0),
        updatemenus=[
            dict(
                type="buttons",
                direction="right",
                x=0.0, y=1.10, xanchor="left", yanchor="top",
                pad=dict(r=6, t=4),
                showactive=True,
                buttons=[
                    dict(
                        label="1 · Physical Room",
                        method="update",
                        args=[{"visible": view_room_only},
                              {"title": "View 1 · Physical arena X only"}],
                    ),
                    dict(
                        label="2 · Room + Tube",
                        method="update",
                        args=[{"visible": view_room_plus_tube},
                              {"title": "View 2 · Arena X with mRPI tube Ω at the origin"}],
                    ),
                    dict(
                        label="3 · Pontryagin Result",
                        method="update",
                        args=[{"visible": view_pontryagin},
                              {"title": "View 3 · Tightened safe set  X̄ = X ⊖ Ω"}],
                    ),
                    dict(
                        label="All",
                        method="update",
                        args=[{"visible": view_all},
                              {"title": base_title}],
                    ),
                ],
            )
        ],
    )

    fig.write_html(str(args.out), include_plotlyjs="cdn", full_html=True)
    print(f"\nWrote interactive figure -> {args.out}")
    if args.show:
        fig.show()


if __name__ == "__main__":
    main()
