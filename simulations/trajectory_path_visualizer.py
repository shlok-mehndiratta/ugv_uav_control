"""
trajectory_path_visualizer.py
=============================

Phase-2-trajectory variant of tube_path_visualizer.py.  Loads
trajectory_runs.npz (24 runs: M-shape & S-shape × 4 stress scenarios × 3
controllers, produced by sim_trajectory_logged.py) and tube_data_sitl.npz,
then renders a single interactive Plotly figure with a 24-entry dropdown.

For each (trajectory, scenario, controller) selection we draw:
    - the physical arena  X       (faint blue, context)
    - the tightened set   X̄ = X ⊖ Ω   (faint green, context)
    - the waypoint X markers for the SELECTED trajectory
    - the actual drone trajectory  x_hist[:3]  (solid polyline)
    - the nominal MPC trajectory   x_bar_hist[:3]  (dashed polyline)
    - the mRPI tube Ω stamped at every `stride`-th nominal point

The viva claim — x_hist[k] - x_bar_hist[k] ∈ Ω — is visually evident as the
drone polyline staying inside the union of red tube stamps for the Tube/ET-
Tube controllers, and visibly leaving / truncating for Standard MPC under
disturbance.

This script does NOT touch tube_path_visualizer.py, tube_runs.npz, or the
existing stress-test rendering pipeline; both pipelines coexist.

Run
---
    python trajectory_path_visualizer.py [--stride 10] [--show]
"""

from __future__ import annotations

import argparse
from pathlib import Path

import numpy as np
import plotly.graph_objects as go
import pypoman
from scipy.spatial import ConvexHull

POSITION_DIMS = 3

TRAJECTORY_LABELS = {
    'M_shape': 'M-shape Trajectory',
    'S_shape': 'S-shape Trajectory',
}
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
    V_full = np.asarray(pypoman.compute_polytope_vertices(H, h))
    V_proj = V_full[:, :dims]
    hull = ConvexHull(V_proj)
    return V_proj[hull.vertices]


def context_box_mesh(points: np.ndarray, color: str, opacity: float, name: str) -> go.Mesh3d:
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
    Stamp the convex hull of the mRPI tube at every `stride`-th nominal point
    and pack into a SINGLE Mesh3d with explicit triangulation.  alphahull=0
    can't do this — it'd hull all stamps together — so we precompute the
    triangle indices once and offset them per stamp.
    """
    base_hull = ConvexHull(omega_pts)
    base_V = omega_pts
    base_F = base_hull.simplices

    finite = nominal_xyz[np.isfinite(nominal_xyz).all(axis=1)]
    stamps = finite[::stride] if len(finite) > 0 else np.zeros((1, 3))
    if len(stamps) == 0:
        stamps = finite[:1]

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


def waypoint_trace(waypoints: np.ndarray, name: str) -> go.Scatter3d:
    return go.Scatter3d(
        x=waypoints[:, 0], y=waypoints[:, 1], z=waypoints[:, 2],
        mode='markers+text',
        marker=dict(size=6, color='purple', symbol='x'),
        text=[f"wp{i}" for i in range(len(waypoints))],
        textposition='top center',
        name=name,
        showlegend=True,
        hoverinfo='name+text',
    )


def main() -> None:
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    here = Path(__file__).resolve().parent
    p.add_argument('--runs',      type=Path, default=here / 'trajectory_runs.npz')
    p.add_argument('--tube-data', type=Path, default=here / 'tube_data_sitl.npz')
    p.add_argument('--out',       type=Path, default=here / 'trajectory_path_visualization.html')
    p.add_argument('--stride',    type=int,  default=10)
    p.add_argument('--show',      action='store_true')
    args = p.parse_args()

    if not args.runs.exists():
        raise FileNotFoundError(f"{args.runs} not found — run sim_trajectory_logged.py first.")
    if not args.tube_data.exists():
        raise FileNotFoundError(f"{args.tube_data} not found.")

    runs = np.load(args.runs, allow_pickle=True)
    tube = np.load(args.tube_data, allow_pickle=True)

    # ---- context: arena + tightened set, projected 6-D -> 3-D --------------
    V_room  = project_to_position(tube['Hx'],       tube['hx'])
    V_tight = project_to_position(tube['Hx_tight'], tube['hx_tight'])
    V_omega = project_to_position(tube['Omega_H'],  tube['Omega_h'])

    print(f"Loaded {args.tube_data.name}; position-space extents:")
    for label, V in [('X', V_room), ('Omega', V_omega), ('X_bar', V_tight)]:
        lo, hi = V.min(0), V.max(0)
        print(f"  {label:<6}: x∈[{lo[0]:+.2f},{hi[0]:+.2f}]  "
              f"y∈[{lo[1]:+.2f},{hi[1]:+.2f}]  "
              f"z∈[{lo[2]:+.2f},{hi[2]:+.2f}]")

    # ---- build context traces ---------------------------------------------
    traces: list = []
    traces.append(context_box_mesh(V_room,  'royalblue', 0.06,
                                   'Physical Arena (X)'))
    traces.append(context_box_mesh(V_tight, 'seagreen',  0.10,
                                   'Tightened Set (X̄ = X ⊖ Ω)'))
    n_context = len(traces)

    # ---- per-run traces (4 each: drone, nominal, garden, waypoints) -------
    run_keys = [str(k) for k in runs['_runs']]
    print(f"\n{len(run_keys)} runs in {args.runs.name}:")
    for k in run_keys: print(f"  {k}")

    per_run_count = 4
    for key in run_keys:
        traj, scenario, controller = key.split('__')
        ctrl_color = CONTROLLER_COLORS[controller]
        x_hist     = runs[f"{key}__x_hist"]
        x_bar_hist = runs[f"{key}__x_bar_hist"]
        drone_xyz  = x_hist[:3, :].T
        nom_xyz    = x_bar_hist[:3, :].T
        run_wps    = np.asarray(runs[f"{key}__waypoints"])[:, :3]

        traces.append(line_trace(
            drone_xyz, color=ctrl_color, dash='solid', width=6,
            name=f"{traj} · {scenario} · {CONTROLLER_LABELS[controller]} — drone",
        ))
        traces.append(line_trace(
            nom_xyz, color=ctrl_color, dash='dash', width=3,
            name=f"{traj} · {scenario} · {CONTROLLER_LABELS[controller]} — nominal x̄",
        ))
        traces.append(build_tube_garden(
            V_omega, nom_xyz, stride=args.stride,
            color='crimson', opacity=0.18,
            name=f"{traj} · {scenario} · {CONTROLLER_LABELS[controller]} — Ω stamps",
        ))
        traces.append(waypoint_trace(
            run_wps, name=f"{TRAJECTORY_LABELS.get(traj, traj)} waypoints",
        ))

    fig = go.Figure(data=traces)

    # ---- visibility helper ------------------------------------------------
    n_total = len(traces)

    def visibility_for(run_idx: int) -> list:
        v = [False] * n_total
        v[0] = v[1] = True             # arena, tightened
        base = n_context + run_idx * per_run_count
        for j in range(per_run_count):
            v[base + j] = True
        return v

    # initial: M_shape · A_wind · et_tube if present, else first run
    initial_idx = next(
        (i for i, k in enumerate(run_keys)
         if k.startswith('M_shape__A_wind') and k.endswith('et_tube')),
        0,
    )
    for i, t in enumerate(traces):
        t.visible = visibility_for(initial_idx)[i]

    # ---- dropdown ---------------------------------------------------------
    buttons = []
    for i, key in enumerate(run_keys):
        traj, scenario, controller = key.split('__')
        label = (f"{TRAJECTORY_LABELS.get(traj, traj)}  ·  "
                 f"{SCENARIO_LABELS.get(scenario, scenario)}  ·  "
                 f"{CONTROLLER_LABELS[controller]}")
        buttons.append(dict(
            label=label,
            method='update',
            args=[{'visible': visibility_for(i)},
                  {'title': f"Phase 2 Tube + Path · {label}"}],
        ))

    initial_key = run_keys[initial_idx]
    init_traj, init_scn, init_ctrl = initial_key.split('__')
    initial_title = (f"Phase 2 Tube + Path · "
                     f"{TRAJECTORY_LABELS.get(init_traj, init_traj)}  ·  "
                     f"{SCENARIO_LABELS.get(init_scn, init_scn)}  ·  "
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
