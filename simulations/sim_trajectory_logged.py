"""
sim_trajectory_logged.py

Phase 2 trajectory variant of sim_stress_tests_logged.py.  Same MPC pipeline,
same scenarios, same controllers, same logging — but the 3-waypoint stress-
test reference is replaced with the M-shape and S-shape Phase 2 trajectories
(github_repo/mpc_ws/phase2/trajectories.py).  Each (trajectory, scenario,
controller) triple is simulated and saved to trajectory_runs.npz, which the
sibling trajectory_path_visualizer.py renders.

This file does NOT touch sim_stress_tests*.py, tube_runs.npz, or
tube_path_visualizer.py — the working stress-test pipeline is preserved.

Notes
-----
Tube data: loads tube_data_sitl.npz (wide arena ±2.5 / ±2.5 / [0, 3]).  The
M-shape waypoint (2.0, 2.0, 1.0) and S-shape waypoint (-1.0, 1.5, 1.0) are
outside the tight tube_data.npz arena, so the SITL bounds are required for
the trajectories to be feasible at all.

x_bar_hist convention is identical to sim_stress_tests_logged.py: at the end
of step k we log X_nom[:, 0] post-roll, i.e. the planner's prediction for
time k+1.
"""

import numpy as np
import cvxpy as cp
import scipy.linalg as la
from pathlib import Path

# ----------------------------------------------------------------------
# Offline data
# ----------------------------------------------------------------------
base_dir = Path(__file__).parent
data = np.load(base_dir / 'tube_data_sitl.npz', allow_pickle=True)
A_d = data['A_d']
B_d = data['B_d']
K   = data['K']
Hx_tight = data['Hx_tight'];  hx_tight = data['hx_tight']
Hu_tight = data['Hu_tight'];  hu_tight = data['hu_tight']
Hx = data['Hx'];  hx = data['hx']
Hu = data['Hu'];  hu = data['hu']
Omega_H = data['Omega_H'];  Omega_h = data['Omega_h']
x_lb = data['x_lb'];  x_ub = data['x_ub']
u_lb = data['u_lb'];  u_ub = data['u_ub']

# ----------------------------------------------------------------------
# Shared constants
# ----------------------------------------------------------------------
N  = 20
dt = 0.05
T_SIM = 400           # 20 seconds — five waypoints need ~80 steps per segment
Q_MPC = np.diag([10.0, 10.0, 10.0, 1.0, 1.0, 1.0])
R_MPC = np.diag([0.1, 0.1, 0.1])
P_TERMINAL = la.solve_discrete_are(A_d, B_d, Q_MPC, R_MPC)
P_TERMINAL = (P_TERMINAL + P_TERMINAL.T) / 2.0


# ----------------------------------------------------------------------
# Phase 2 trajectories — inlined from github_repo/mpc_ws/phase2/trajectories.py
# (constant-altitude xy paths; the velocity components are zero by convention)
# ----------------------------------------------------------------------
M_SHAPE = [
    [0.0, 0.0, 1.0],   # bottom-left start
    [0.0, 2.0, 1.0],   # top-left
    [1.0, 1.0, 1.0],   # middle dip
    [2.0, 2.0, 1.0],   # top-right
    [2.0, 0.0, 1.0],   # bottom-right end
]

S_SHAPE = [
    [0.0,  0.0, 1.0],
    [1.0,  0.5, 1.0],
    [0.0,  1.0, 1.0],
    [-1.0, 1.5, 1.0],
    [0.0,  2.0, 1.0],
]

TRAJECTORIES = {
    'M_shape': [np.array(wp + [0.0, 0.0, 0.0]) for wp in M_SHAPE],
    'S_shape': [np.array(wp + [0.0, 0.0, 0.0]) for wp in S_SHAPE],
}


# ----------------------------------------------------------------------
# Scenario configurations (no x0 — set per-trajectory inside run_simulation)
# ----------------------------------------------------------------------
SCENARIOS = {
    'A_wind': {
        'label': 'A: Persistent Crosswind',
        'w_wind': lambda k, x: B_d @ np.array([1.0, 0.0, 0.0]),
        'thrust_scale': 1.0,
        'sensor_glitch_step': None,
    },
    'B_payload': {
        'label': 'B: Payload Drop (100%)',
        'w_wind': lambda k, x: np.zeros(6),
        'thrust_scale': 0.35,
        'sensor_glitch_step': None,
    },
    'C_wall': {
        'label': 'C: High-Speed Catch + Gust',
        'w_wind': lambda k, x: B_d @ np.array([1.0, 0.0, 0.0]) if x[0] > 0.2 and k < 80 else np.zeros(6),
        'thrust_scale': 1.0,
        'sensor_glitch_step': None,
    },
    'D_glitch': {
        'label': 'D: UWB Sensor Glitch',
        'w_wind': lambda k, x: np.zeros(6),
        'thrust_scale': 1.0,
        'sensor_glitch_step': int(1.5 / dt),
    },
}


# ----------------------------------------------------------------------
# Build CVXPY problems once (parametric – reused across all 24 runs)
# ----------------------------------------------------------------------
def _build_problem(controller_type):
    X_bar = cp.Variable((6, N + 1))
    U_bar = cp.Variable((3, N))
    x_curr = cp.Parameter(6)
    x_ref  = cp.Parameter(6)

    cost = 0
    for k in range(N):
        cost += cp.quad_form(X_bar[:, k] - x_ref, Q_MPC)
        cost += cp.quad_form(U_bar[:, k], R_MPC)
    cost += cp.quad_form(X_bar[:, N] - x_ref, P_TERMINAL)

    cons = []
    if controller_type == 'et_tube':
        cons.append(Omega_H @ (x_curr - X_bar[:, 0]) <= Omega_h)
        for k in range(N):
            cons.append(X_bar[:, k+1] == A_d @ X_bar[:, k] + B_d @ U_bar[:, k])
        for k in range(N + 1):
            cons.append(Hx_tight @ X_bar[:, k] <= hx_tight)
        for k in range(N):
            cons.append(Hu_tight @ U_bar[:, k] <= hu_tight)
    else:
        cons.append(X_bar[:, 0] == x_curr)
        for k in range(N):
            cons.append(X_bar[:, k+1] == A_d @ X_bar[:, k] + B_d @ U_bar[:, k])
        for k in range(N + 1):
            cons.append(Hx @ X_bar[:, k] <= hx)
        for k in range(N):
            cons.append(Hu @ U_bar[:, k] <= hu)

    prob = cp.Problem(cp.Minimize(cost), cons)
    return prob, X_bar, U_bar, x_curr, x_ref


prob_tube,     Xb_tube,     Ub_tube,     xp_tube,     rp_tube     = _build_problem('et_tube')
prob_full,     Xb_full,     Ub_full,     xp_full,     rp_full     = _build_problem('et_tube')
prob_standard, Xb_standard, Ub_standard, xp_standard, rp_standard = _build_problem('standard')


# ----------------------------------------------------------------------
# Single-run simulation
# ----------------------------------------------------------------------
def run_simulation(traj_name, scenario_name, controller_type, seed=43):
    """
    Run one (trajectory, scenario, controller) simulation.

    Returns a dict with full trajectory history and metrics, in the same
    shape as sim_stress_tests_logged.py's run_simulation.
    """
    np.random.seed(seed)
    cfg = SCENARIOS[scenario_name]
    waypoints = TRAJECTORIES[traj_name]

    # Pick the correct CVXPY objects
    if controller_type == 'et_tube':
        prob, X_var, U_var, xp, rp = prob_tube, Xb_tube, Ub_tube, xp_tube, rp_tube
        ctrl_name = 'ET Tube MPC'
        use_aux_loop = True
    elif controller_type == 'full_tube':
        prob, X_var, U_var, xp, rp = prob_full, Xb_full, Ub_full, xp_full, rp_full
        ctrl_name = 'Full Tube MPC'
        use_aux_loop = True
    elif controller_type == 'standard':
        prob, X_var, U_var, xp, rp = prob_standard, Xb_standard, Ub_standard, xp_standard, rp_standard
        ctrl_name = 'Standard MPC'
        use_aux_loop = False
    else:
        raise ValueError(controller_type)

    def solve_mpc(x_current, prev_X, prev_U, current_x_ref):
        xp.value = x_current
        rp.value = current_x_ref
        X_var.value = prev_X
        U_var.value = prev_U
        prob.solve(solver=cp.OSQP, warm_start=True)
        if prob.status not in ['optimal', 'optimal_inaccurate'] or X_var.value is None:
            Xf = np.roll(prev_X, -1, axis=1)
            Uf = np.roll(prev_U, -1, axis=1)
            if use_aux_loop:
                u_pad = -K @ (Xf[:, -2] - current_x_ref)
            else:
                u_pad = np.zeros(3)
            Uf[:, -1] = u_pad
            Xf[:, -1] = A_d @ Xf[:, -2] + B_d @ u_pad
            return Xf, Uf, False
        return X_var.value.copy(), U_var.value.copy(), True

    # Per-trajectory initial state: first waypoint with zero velocity
    x_true = waypoints[0].copy()
    X_nom  = np.tile(x_true, (N + 1, 1)).T
    U_nom  = np.zeros((3, N))

    # Trigger config
    consecutive_threshold = 3
    trigger_counter = 0
    trigger_events = []
    steps_since_last_solve = 0
    crashed = False

    # Logging
    x_hist = np.zeros((6, T_SIM + 1))
    u_hist = np.zeros((3, T_SIM))
    x_ref_hist = np.zeros((6, T_SIM + 1))
    x_bar_hist = np.zeros((6, T_SIM + 1))   # nominal trajectory = tube centre per step
    x_hist[:, 0] = x_true

    current_wp_idx = 0
    current_x_ref = waypoints[current_wp_idx]
    x_ref_hist[:, 0] = current_x_ref

    # Initial solve
    X_nom, U_nom, ok = solve_mpc(x_true, X_nom, U_nom, current_x_ref)
    if not ok:
        crashed = True
    x_bar_hist[:, 0] = X_nom[:, 0]

    for k in range(T_SIM):
        if crashed:
            x_hist[:, k + 1:] = np.nan
            x_bar_hist[:, k + 1:] = np.nan
            break

        # Waypoint advance — measured against the planner's NOMINAL position,
        # not x_true.  Under persistent disturbance (e.g. crosswind) the tube
        # auxiliary controller holds x_true within Ω of x_bar, so x_true never
        # crosses the small 0.2 m radius around the waypoint even though x_bar
        # does.  Standard MPC has x_bar == x_true so this is equivalent.
        x_bar_k = X_nom[:, 0]
        dist_to_wp = np.linalg.norm(x_bar_k[0:3] - current_x_ref[0:3])
        if dist_to_wp < 0.2 and current_wp_idx < len(waypoints) - 1:
            current_wp_idx += 1
            current_x_ref = waypoints[current_wp_idx]

        # --- Scenario D: sensor glitch on measured state ---
        measured_x = x_true.copy()
        if cfg['sensor_glitch_step'] is not None and k == cfg['sensor_glitch_step']:
            measured_x[0] -= 0.20

        e_k = measured_x - x_bar_k

        # --- Event trigger logic ---
        if controller_type in ['standard', 'full_tube']:
            trigger_counter = consecutive_threshold
        elif use_aux_loop:
            omega_margin = Omega_H @ e_k - Omega_h
            if np.max(omega_margin) > 0:
                trigger_counter += 1
            else:
                trigger_counter = 0
        else:
            pos_error = np.linalg.norm(e_k[0:3])
            if pos_error > 0.15:
                trigger_counter += 1
            else:
                trigger_counter = 0

        if trigger_counter >= consecutive_threshold or steps_since_last_solve >= N - 2:
            trigger_events.append((k, x_true[0:3].copy()))
            X_nom, U_nom, ok = solve_mpc(measured_x, X_nom, U_nom, current_x_ref)
            if not ok and not use_aux_loop:
                crashed = True
            trigger_counter = 0
            steps_since_last_solve = 0
        else:
            steps_since_last_solve += 1

        # --- Control law ---
        u_bar_k = U_nom[:, 0]
        x_bar_k = X_nom[:, 0]
        if use_aux_loop:
            u_true = u_bar_k - K @ (x_true - x_bar_k)
        else:
            u_true = u_bar_k
        u_true = np.clip(u_true, u_lb, u_ub)

        # --- True plant dynamics with scenario perturbations ---
        u_applied = u_true * cfg['thrust_scale']
        w_wind_val = cfg['w_wind'](k, x_true) if callable(cfg['w_wind']) else cfg['w_wind']
        x_true = A_d @ x_true + B_d @ u_applied + w_wind_val

        # Standard process noise (matches mRPI design)
        d = np.array([np.random.uniform(-1, 1), np.random.uniform(-1, 1),
                      np.random.uniform(-1.5, 1.5)])
        x_true += B_d @ d

        # Log
        x_hist[:, k + 1] = x_true
        u_hist[:, k] = u_true
        x_ref_hist[:, k + 1] = current_x_ref

        # Shift trajectory for next step
        X_nom = np.roll(X_nom, -1, axis=1)
        U_nom = np.roll(U_nom, -1, axis=1)
        if use_aux_loop:
            u_pad = -K @ (X_nom[:, -2] - current_x_ref)
        else:
            u_pad = np.zeros(3)
        U_nom[:, -1] = u_pad
        X_nom[:, -1] = A_d @ X_nom[:, -2] + B_d @ u_pad

        # Planner's prediction for time k+1 (post-roll)
        x_bar_hist[:, k + 1] = X_nom[:, 0]

    # --- Metrics ---
    valid = ~np.isnan(x_hist[0, :])
    valid_hist = x_hist[:, valid]
    valid_ref_hist = x_ref_hist[:, valid]
    if valid_hist.shape[1] > 0:
        pos_errors = np.linalg.norm(
            valid_hist[0:3, :] - valid_ref_hist[0:3, :], axis=0)
        rmse = np.sqrt(np.mean(pos_errors ** 2))
        final_err = np.linalg.norm(valid_hist[0:3, -1] - valid_ref_hist[0:3, -1])

        below = valid_hist < x_lb.reshape(6, 1)
        above = valid_hist > x_ub.reshape(6, 1)
        n_violations = int(np.sum(np.any(below | above, axis=0)))
    else:
        rmse = np.nan; final_err = np.nan; n_violations = 0

    status = 'Crashed/Infeasible' if crashed else 'Success'

    return {
        'trajectory': traj_name,
        'scenario': scenario_name,
        'ctrl_name': ctrl_name,
        'label': cfg['label'],
        'x_hist': x_hist,
        'u_hist': u_hist,
        'x_bar_hist': x_bar_hist,
        'x_ref_hist': x_ref_hist,
        'triggers': trigger_events,
        'rmse': rmse,
        'final_err': final_err,
        'n_violations': n_violations,
        'status': status,
    }


# ----------------------------------------------------------------------
# Drive the whole 24-run matrix and save trajectory_runs.npz
# ----------------------------------------------------------------------
if __name__ == '__main__':
    all_results = {}
    for traj_name in TRAJECTORIES:
        for sname in SCENARIOS:
            print(f"Running {traj_name} · {SCENARIOS[sname]['label']}...")
            standard  = run_simulation(traj_name, sname, controller_type='standard')
            full_tube = run_simulation(traj_name, sname, controller_type='full_tube')
            tube      = run_simulation(traj_name, sname, controller_type='et_tube')
            all_results[(traj_name, sname)] = (standard, full_tube, tube)

    print("\nTRAJECTORY RUN RESULTS")
    print(f"{'Trajectory':<10} | {'Scenario':<28} | {'Controller':<18} | {'Triggers':<9} | "
          f"{'RMSE (m)':<10} | {'Final Err':<10} | {'Bound Viols':<12} | {'Status'}")
    print("-" * 120)
    for (traj_name, sname), trio in all_results.items():
        for res in trio:
            viol = f"{res['n_violations']} steps" if res['n_violations'] > 0 else "NONE"
            print(f"{traj_name:<10} | {res['label']:<28} | {res['ctrl_name']:<18} | "
                  f"{len(res['triggers']):<9} | {res['rmse']:<10.4f} | "
                  f"{res['final_err']:<10.4f} | {viol:<12} | {res['status']}")
    print("=" * 120)

    # ----------------------------------------------------------------------
    # Save trajectory_runs.npz with 3-level run keys
    # ----------------------------------------------------------------------
    CTRL_KEY = {'Standard MPC': 'standard',
                'Full Tube MPC': 'full_tube',
                'ET Tube MPC':   'et_tube'}

    run_prefixes = []
    save_payload = {
        '_dt':                np.float64(dt),
        '_T_sim':             np.int32(T_SIM),
        '_tube_data_source':  np.array('tube_data_sitl.npz'),
    }

    for (traj_name, sname), trio in all_results.items():
        for res in trio:
            ck = CTRL_KEY[res['ctrl_name']]
            prefix = f"{traj_name}__{sname}__{ck}"
            run_prefixes.append(prefix)
            save_payload[f"{prefix}__x_hist"]     = res['x_hist']
            save_payload[f"{prefix}__x_bar_hist"] = res['x_bar_hist']
            save_payload[f"{prefix}__u_hist"]     = res['u_hist']
            save_payload[f"{prefix}__x_ref_hist"] = res['x_ref_hist']
            if res['triggers']:
                tk = np.array([t[0] for t in res['triggers']], dtype=np.int32)
                tp = np.array([t[1] for t in res['triggers']], dtype=np.float64)
            else:
                tk = np.zeros((0,), dtype=np.int32)
                tp = np.zeros((0, 3), dtype=np.float64)
            save_payload[f"{prefix}__trigger_steps"]     = tk
            save_payload[f"{prefix}__trigger_positions"] = tp
            save_payload[f"{prefix}__label"]   = np.array(res['label'])
            save_payload[f"{prefix}__rmse"]    = np.float64(res['rmse'])
            save_payload[f"{prefix}__status"]  = np.array(res['status'])
            save_payload[f"{prefix}__waypoints"] = np.array(TRAJECTORIES[traj_name])

    save_payload['_runs'] = np.array(run_prefixes, dtype=object)

    out_path = base_dir / 'trajectory_runs.npz'
    np.savez(out_path, **save_payload)
    print(f"\nSaved {len(run_prefixes)} runs to {out_path}")
    print("Run trajectory_path_visualizer.py to render.")
