"""
sim_stress_tests_logged.py

Logging variant of sim_stress_tests.py.  Same control logic, same scenarios,
same controllers — but additionally records the nominal trajectory
x_bar(k) = X_nom[:, 0] at every step (the planner's idea of "where the drone
should be"), so the downstream visualiser can draw the mRPI tube anchored to
it.  Writes a single tube_runs.npz at the end instead of the matplotlib
figures fig1..fig6.png; the original sim_stress_tests.py is untouched and
continues to be the source of those figures.
"""

import numpy as np
import cvxpy as cp
import scipy.linalg as la
from pathlib import Path

# Load offline data
base_dir = Path(__file__).parent
data = np.load(base_dir / 'tube_data.npz', allow_pickle=True)
#data = np.load('tube_data.npz', allow_pickle=True) #won't work with different path
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

# Shared constants
N  = 20
dt = 0.05
T_SIM = 200           # 10 seconds
Q_MPC = np.diag([10.0, 10.0, 10.0, 1.0, 1.0, 1.0])
R_MPC = np.diag([0.1, 0.1, 0.1])
P_TERMINAL = la.solve_discrete_are(A_d, B_d, Q_MPC, R_MPC)
P_TERMINAL = (P_TERMINAL + P_TERMINAL.T) / 2.0
WAYPOINTS = [
    np.array([0.5, 0.5, 1.0, 0.0, 0.0, 0.0]),
    np.array([-0.5, 1.0, 1.5, 0.0, 0.0, 0.0]),
    np.array([0.5, -0.5, 2.0, 0.0, 0.0, 0.0])
]

# Scenario configurations
SCENARIOS = {
    'A_wind': {
        'label': 'A: Persistent Crosswind',
        'x0': np.array([-0.95, 0.0, 0.5, 0.0, 0.0, 0.0]),
        'w_wind': lambda k, x: B_d @ np.array([1.0, 0.0, 0.0]),
        'thrust_scale': 1.0,
        'sensor_glitch_step': None,
    },
    'B_payload': {
        'label': 'B: Payload Drop (100%)',
        'x0': np.array([-0.95, 0.0, 0.5, 0.0, 0.0, 0.0]),
        'w_wind': lambda k, x: np.zeros(6),
        'thrust_scale': 0.35,
        'sensor_glitch_step': None,
    },
    'C_wall': {
        'label': 'C: High-Speed Catch + Gust',
        'x0': np.array([-0.95, 0.0, 0.5, 0.0, 0.0, 0.0]),
        'w_wind': lambda k, x: B_d @ np.array([1.0, 0.0, 0.0]) if x[0] > 0.2 and k < 80 else np.zeros(6),
        'thrust_scale': 1.0,
        'sensor_glitch_step': None,
    },
    'D_glitch': {
        'label': 'D: UWB Sensor Glitch',
        'x0': np.array([-0.95, 0.0, 0.5, 0.0, 0.0, 0.0]),
        'w_wind': lambda k, x: np.zeros(6),
        'thrust_scale': 1.0,
        'sensor_glitch_step': int(1.5 / dt),
    },
}


# ==============================================================================
# Build CVXPY problems once (parametric – reused across all runs)
# ==============================================================================
def _build_problem(controller_type):
    """Build and return (problem, X_bar, U_bar, x_curr_param, x_ref_param)."""
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

# Pre-build problem templates
prob_tube, Xb_tube, Ub_tube, xp_tube, rp_tube = _build_problem('et_tube')
prob_full, Xb_full, Ub_full, xp_full, rp_full = _build_problem('et_tube')
prob_standard, Xb_standard, Ub_standard, xp_standard, rp_standard = _build_problem('standard')


def run_simulation(scenario_name, controller_type, seed=43):
    """
    Run one simulation.

    Parameters
    ----------
    scenario_name : str   – key into SCENARIOS dict
    controller_type: str  – 'et_tube', 'full_tube', 'no_aux', or 'standard'
    seed          : int

    Returns
    -------
    dict with trajectory history and metrics
    """
    np.random.seed(seed)
    cfg = SCENARIOS[scenario_name]

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

    def solve_mpc(x_current, prev_X, prev_U, current_x_ref):
        xp.value = x_current
        rp.value = current_x_ref
        X_var.value = prev_X
        U_var.value = prev_U
        prob.solve(solver=cp.OSQP, warm_start=True)
        if prob.status not in ['optimal', 'optimal_inaccurate'] or X_var.value is None:
            # Safety fallback
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

    # State initialisation
    x_true = cfg['x0'].copy()
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
    current_x_ref = WAYPOINTS[current_wp_idx]
    x_ref_hist[:, 0] = current_x_ref

    # Initial solve
    X_nom, U_nom, ok = solve_mpc(x_true, X_nom, U_nom, current_x_ref)
    if not ok:
        crashed = True
    x_bar_hist[:, 0] = X_nom[:, 0]   # planner's prediction for time 0

    for k in range(T_SIM):
        if crashed:
            x_hist[:, k + 1:] = np.nan
            x_bar_hist[:, k + 1:] = np.nan
            break

        # Check waypoint switching
        dist_to_wp = np.linalg.norm(x_true[0:3] - current_x_ref[0:3])
        if dist_to_wp < 0.2 and current_wp_idx < len(WAYPOINTS) - 1:
            current_wp_idx += 1
            current_x_ref = WAYPOINTS[current_wp_idx]

        x_bar_k = X_nom[:, 0]

        # --- Scenario D: sensor glitch on measured state ---
        measured_x = x_true.copy()
        if cfg['sensor_glitch_step'] is not None and k == cfg['sensor_glitch_step']:
            measured_x[0] -= 0.20   # Fake 20 cm jump away from target for 1 step

        e_k = measured_x - x_bar_k

        # --- Event trigger logic ---
        if controller_type in ['standard', 'full_tube']:
            trigger_counter = consecutive_threshold  # Force trigger every step
        elif use_aux_loop:
            # Formal RPI set membership check
            omega_margin = Omega_H @ e_k - Omega_h
            if np.max(omega_margin) > 0:
                trigger_counter += 1
            else:
                trigger_counter = 0
        else:
            # Heuristic Euclidean trigger (no RPI available)
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

        # --- True plant dynamics (with scenario-specific perturbation) ---
        u_applied = u_true * cfg['thrust_scale']    # Scenario B: mass mismatch
        w_wind_val = cfg['w_wind'](k, x_true) if callable(cfg['w_wind']) else cfg['w_wind']
        x_true = A_d @ x_true + B_d @ u_applied + w_wind_val

        # Standard process noise
        d = np.array([np.random.uniform(-1,1), np.random.uniform(-1,1),
                       np.random.uniform(-1.5,1.5)])
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

        # After the roll, X_nom[:, 0] is the planner's prediction for time k+1
        x_bar_hist[:, k + 1] = X_nom[:, 0]

    # --- Compute metrics ---
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
        excess_above = np.max(valid_hist - x_ub.reshape(6, 1))
        excess_below = np.max(x_lb.reshape(6, 1) - valid_hist)
        max_excess = max(excess_above, excess_below, 0.0)
    else:
        rmse = np.nan; final_err = np.nan; n_violations = 0; max_excess = 0.0

    status = 'Crashed/Infeasible' if crashed else 'Success'

    return {
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
        'max_excess': max_excess,
        'status': status,
    }


if __name__ == '__main__':
    all_results = {}
    for sname in SCENARIOS:
        print(f"Running {SCENARIOS[sname]['label']}...")
        standard = run_simulation(sname, controller_type='standard')
        full_tube = run_simulation(sname, controller_type='full_tube')
        tube = run_simulation(sname, controller_type='et_tube')
        all_results[sname] = (standard, full_tube, tube)

    print("TEST RESULTS")
    print(f"{'Scenario':<28} | {'Controller':<18} | {'Triggers':<9} | "
          f"{'RMSE (m)':<10} | {'Final Err':<10} | {'Bound Viols':<12} | {'Status'}")
    print("-" * 100)
    for sname in SCENARIOS:
        for res in all_results[sname]:
            viol_str = f"{res['n_violations']} steps" if res['n_violations'] > 0 else "NONE ✓"
            print(f"{res['label']:<28} | {res['ctrl_name']:<18} | "
                  f"{len(res['triggers']):<9} | {res['rmse']:<10.4f} | "
                  f"{res['final_err']:<10.4f} | {viol_str:<12} | {res['status']}")
    print("=" * 100)

    # ----------------------------------------------------------------------
    # Save every (scenario, controller) trajectory to a single npz so the
    # downstream visualiser can mount whichever pair the user picks at viva
    # time.  Keys are flattened as "<scenario>__<controller>__<field>".
    # ----------------------------------------------------------------------
    CTRL_KEY = {'Standard MPC': 'standard',
                'Full Tube MPC': 'full_tube',
                'ET Tube MPC':   'et_tube'}

    save_payload = {
        '_dt':        np.float64(dt),
        '_T_sim':     np.int32(T_SIM),
        '_waypoints': np.array(WAYPOINTS),
        '_runs':      np.array([f"{s}__{CTRL_KEY[r['ctrl_name']]}"
                                for s in SCENARIOS
                                for r in all_results[s]], dtype=object),
        '_tube_data_source': np.array('tube_data.npz'),
    }
    for sname in SCENARIOS:
        for res in all_results[sname]:
            ck = CTRL_KEY[res['ctrl_name']]
            prefix = f"{sname}__{ck}"
            save_payload[f"{prefix}__x_hist"]     = res['x_hist']
            save_payload[f"{prefix}__x_bar_hist"] = res['x_bar_hist']
            save_payload[f"{prefix}__u_hist"]     = res['u_hist']
            save_payload[f"{prefix}__x_ref_hist"] = res['x_ref_hist']
            # triggers is a list of (k, [px,py,pz]); split into two arrays
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

    out_path = base_dir / 'tube_runs.npz'
    np.savez(out_path, **save_payload)
    print(f"\nSaved {len(save_payload['_runs'])} runs to {out_path}")
    print("Run pontryagin / tube_path_visualizer.py to render.")
