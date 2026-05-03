"""
tube_path_visualizer.py
=======================

Renders an interactive 3-D Plotly figure that shows, for every
(scenario × controller) pair recorded by sim_stress_tests_logged.py:

    - the physical arena  X       (faint blue, for context)
    - the tightened set   X_bar   (faint green, for context)
    - the actual drone trajectory      x_hist[:3]
    - the nominal MPC trajectory       x_bar_hist[:3]   (centre of the tube)
    - the mRPI tube  Omega  stamped at every `stride`-th point of the
      nominal trajectory ("tube garden") — this is the safety envelope
      that the closed loop must stay inside, by construction of the
      ancillary controller  u = u_bar - K (x - x_bar).

The viva-relevant claim is:  x_hist[k]  -  x_bar_hist[k]  in  Omega  for all k.
Visually that means the actual drone polyline never leaves the union of the
stamped tubes.

A dropdown menu switches between the 12 runs.  The static-garden choice
(stamped tubes) was made over a Plotly frame animation so the entire safety
envelope is visible at once during a presentation — no scrubbing needed.

Run
---
    python tube_path_visualizer.py [--stride 10] [--show]
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import plotly.graph_objects as go
import pypoman
from scipy.spatial import ConvexHull

POSITION_DIMS = 3

SCENARIO_LABELS = {
    'A_wind':    'A · Persistent Crosswind',
    'B_payload': 'B · Payload Drop (100%)',
    'C_wall':    'C · High-Speed Catch + Gust',
    'D_glitch':  'D · UWB Sensor Glitch',
}
CONTROLLER_LABELS = {
    'standard':  'Standard MPC',
    'full_tube': 'Full Tube MPC',
    'et_tube':   'ET Tube MPC',
}
CONTROLLER_COLORS = {
    'standard':  '#e74c3c',
    'full_tube': '#8e44ad',
    'et_tube':   '#27ae60',
}


def project_to_position(H: np.ndarray, h: np.ndarray, dims: int = POSITION_DIMS) -> np.ndarray:
    """6-D polytope -> 3-D position-space hull, returning extreme points only."""
    V_full = np.asarray(pypoman.compute_polytope_vertices(H, h))
    V_proj = V_full[:, :dims]
    hull = ConvexHull(V_proj)
    return V_proj[hull.vertices]


def context_box_mesh(points: np.ndarray, color: str, opacity: float, name: str) -> go.Mesh3d:
    """A static convex-hull mesh used for arena and tightened-set context."""
    return go.Mesh3d(
        x=points[:, 0], y=points[:, 1], z=points[:, 2],
        color=color, opacity=opacity, alphahull=0, flatshading=True,
        name=name, showlegend=True, hoverinfo='name',
    )


def build_tube_garden(omega_pts: np.ndarray,
                      nominal_xyz: np.ndarray,
                      stride: int,
                      color: str,
                      opacity: float,
                      name: str) -> go.Mesh3d:
    """
    Concatenate copies of the mRPI hull translated to every `stride`-th point
    of the nominal trajectory into a SINGLE Mesh3d.  Plotly's alphahull builds
    one hull per trace, which would merge all stamps into a sausage; we
    therefore precompute the hull's triangle indices and offset them per stamp.
    """
    base_hull = ConvexHull(omega_pts)
    base_V = omega_pts                      # already only extreme points
    base_F = base_hull.simplices            # (M, 3) triangle indices

    finite = nominal_xyz[np.isfinite(nominal_xyz).all(axis=1)]
    stamps = finite[::stride]
    if len(stamps) == 0:
        stamps = finite[:1]                 # empty traj guard

    n_v = base_V.shape[0]
    Vs, Fs = [], []
    for s_idx, p in enumerate(stamps):
        Vs.append(base_V + p)
        Fs.append(base_F + s_idx * n_v)
    V = np.vstack(Vs)
    F = np.vstack(Fs)
    return go.Mesh3d(
        x=V[:, 0], y=V[:, 1], z=V[:, 2],
        i=F[:, 0], j=F[:, 1], k=F[:, 2],
        color=color, opacity=opacity, flatshading=True,
        name=name, showlegend=True, hoverinfo='name',
    )


def line_trace(xyz: np.ndarray, color: str, dash: str, width: float, name: str) -> go.Scatter3d:
    finite = np.isfinite(xyz).all(axis=1)
    return go.Scatter3d(
        x=xyz[finite, 0], y=xyz[finite, 1], z=xyz[finite, 2],
        mode='lines', line=dict(color=color, width=width, dash=dash),
        name=name, showlegend=True, hoverinfo='name',
    )


def waypoint_trace(waypoints: np.ndarray) -> go.Scatter3d:
    return go.Scatter3d(
        x=waypoints[:, 0], y=waypoints[:, 1], z=waypoints[:, 2],
        mode='markers+text',
        marker=dict(size=6, color='purple', symbol='x'),
        text=[f"wp{i}" for i in range(len(waypoints))],
        textposition='top center',
        name='Waypoints',
        showlegend=True,
        hoverinfo='name+text',
    )


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    here = Path(__file__).resolve().parent
    p.add_argument('--runs',      type=Path, default=here / 'tube_runs.npz')
    p.add_argument('--tube-data', type=Path, default=here / 'tube_data.npz')
    p.add_argument('--out',       type=Path, default=here / 'tube_path_visualization.html')
    p.add_argument('--stride',    type=int,  default=10,
                   help="Stamp the tube every N-th nominal point (default 10).")
    p.add_argument('--show',      action='store_true')
    args = p.parse_args()

    if not args.runs.exists():
        raise FileNotFoundError(f"{args.runs} not found — run sim_stress_tests_logged.py first.")
    if not args.tube_data.exists():
        raise FileNotFoundError(f"{args.tube_data} not found.")

    runs = np.load(args.runs, allow_pickle=True)
    tube = np.load(args.tube_data, allow_pickle=True)

    # ---- context: arena + tightened set, projected 6-D -> 3-D --------------
    V_room  = project_to_position(tube['Hx'],       tube['hx'])
    V_tight = project_to_position(tube['Hx_tight'], tube['hx_tight'])
    V_omega = project_to_position(tube['Omega_H'],  tube['Omega_h'])

    print("Loaded tube_data.npz; position-space extents:")
    for label, V in [('X', V_room), ('Omega', V_omega), ('X_bar', V_tight)]:
        lo, hi = V.min(0), V.max(0)
        print(f"  {label:<6}: x∈[{lo[0]:+.2f},{hi[0]:+.2f}]  "
              f"y∈[{lo[1]:+.2f},{hi[1]:+.2f}]  "
              f"z∈[{lo[2]:+.2f},{hi[2]:+.2f}]")

    waypoints = np.asarray(runs['_waypoints'])[:, :3]

    # ---- build traces ------------------------------------------------------
    traces: list = []
    traces.append(context_box_mesh(V_room,  'royalblue', 0.06,
                                   'Physical Arena (X)'))
    traces.append(context_box_mesh(V_tight, 'seagreen',  0.10,
                                   'Tightened Set (X̄ = X ⊖ Ω)'))
    traces.append(waypoint_trace(waypoints))
    n_context = len(traces)

    # Per-run traces, in canonical order (matches sim_stress_tests_logged.py)
    run_keys = [k for k in runs['_runs']]   # already strings
    print(f"\n{len(run_keys)} runs in {args.runs.name}:")
    for k in run_keys: print(f"  {k}")

    # For each run we add 3 traces: drone path, nominal path, tube garden.
    per_run_count = 3
    for key in run_keys:
        scenario, controller = key.split('__')
        ctrl_color = CONTROLLER_COLORS[controller]
        x_hist     = runs[f"{key}__x_hist"]
        x_bar_hist = runs[f"{key}__x_bar_hist"]
        drone_xyz  = x_hist[:3, :].T
        nom_xyz    = x_bar_hist[:3, :].T

        traces.append(line_trace(
            drone_xyz, color=ctrl_color, dash='solid', width=6,
            name=f"{scenario} · {CONTROLLER_LABELS[controller]} — drone",
        ))
        traces.append(line_trace(
            nom_xyz, color=ctrl_color, dash='dash', width=3,
            name=f"{scenario} · {CONTROLLER_LABELS[controller]} — nominal x̄",
        ))
        traces.append(build_tube_garden(
            V_omega, nom_xyz, stride=args.stride,
            color='crimson', opacity=0.18,
            name=f"{scenario} · {CONTROLLER_LABELS[controller]} — Ω stamps",
        ))

    fig = go.Figure(data=traces)

    # ---- visibility per run ------------------------------------------------
    n_total = len(traces)

    def visibility_for(run_idx: int) -> list:
        v = [False] * n_total
        v[0] = v[1] = v[2] = True            # arena, tightened, waypoints
        base = n_context + run_idx * per_run_count
        for j in range(per_run_count):
            v[base + j] = True
        return v

    # initial selection: et_tube on A_wind if present, else first run
    initial_idx = next((i for i, k in enumerate(run_keys)
                        if k.endswith('et_tube') and k.startswith('A_wind')),
                       0)
    for i, t in enumerate(traces):
        t.visible = visibility_for(initial_idx)[i]

    # Dropdown buttons: scenario × controller
    buttons = []
    for i, key in enumerate(run_keys):
        scenario, controller = key.split('__')
        label = f"{SCENARIO_LABELS[scenario]}  ·  {CONTROLLER_LABELS[controller]}"
        buttons.append(dict(
            label=label,
            method='update',
            args=[{'visible': visibility_for(i)},
                  {'title': f"Tube + Path · {label}"}],
        ))

    initial_key = run_keys[initial_idx]
    init_scn, init_ctrl = initial_key.split('__')
    initial_title = (f"Tube + Path · {SCENARIO_LABELS[init_scn]}  ·  "
                     f"{CONTROLLER_LABELS[init_ctrl]}")

    fig.update_layout(
        title=dict(text=initial_title, x=0.5),
        scene=dict(
            xaxis_title='p_x  [m]',
            yaxis_title='p_y  [m]',
            zaxis_title='p_z  [m]',
            aspectmode='data',
            camera=dict(eye=dict(x=1.6, y=1.6, z=1.0)),
        ),
        legend=dict(itemsizing='constant'),
        margin=dict(l=0, r=0, t=80, b=0),
        updatemenus=[
            dict(
                type='dropdown',
                direction='down',
                x=0.0, y=1.10, xanchor='left', yanchor='top',
                showactive=True,
                buttons=buttons,
                active=initial_idx,
            ),
        ],
    )

    fig.write_html(str(args.out), include_plotlyjs='cdn', full_html=True)
    print(f"\nWrote {args.out}")
    if args.show:
        fig.show()


if __name__ == '__main__':
    main()
