#!/usr/bin/env python3
"""
Numerical Validation of Active-Altitude Perception Control
===========================================================
This script instantiates the linear estimation surrogate and validates:
- Proposition 1: Static optimal altitude z*(d) and the covariance trade-off.
- Proposition 2: Active altitude tracking dominance.
- Proposition 3: Anticipatory vs. reactive policies under rate-limited climb.

Saves figures to the results directory.
"""

import os
import numpy as np
import scipy.linalg
import matplotlib.pyplot as plt

# === CONSTANTS & PARAMETERS ===
FOV = 1.047  # 60 degrees in radians
TAN_HALF_FOV = np.tan(FOV / 2.0)  # ~0.57735
Z0 = 5.0
R0_VAL = 0.0004  # Lower measurement noise for high-fidelity sitl reference

# Climb rate limit (Assumption 4)
VZ_MAX = 0.4  # m/s
DT = 0.05  # 20 Hz prediction/update rate

# State space matrices (2D double integrator target model)
F = np.array([
    [1.0, 0.0,  DT, 0.0],
    [0.0, 1.0, 0.0,  DT],
    [0.0, 0.0, 1.0, 0.0],
    [0.0, 0.0, 0.0, 1.0]
])
H = np.array([
    [1.0, 0.0, 0.0, 0.0],
    [0.0, 1.0, 0.0, 0.0]
])

# Process noise covariance (Q)
Q = np.diag([1e-6, 1e-6, 1e-5, 1e-5])

# Altitude Bounds
Z_MIN = 2.0
Z_MAX = 8.0
SAFETY_BUFFER = 1.20  # 20% safety buffer above visibility boundary

def get_R(z):
    """Measurement noise scaling quadratically with altitude (Assumption 2)."""
    return R0_VAL * (z / Z0)**2 * np.eye(2)

def solve_dare_ss(z):
    """Solve Discrete Algebraic Riccati Equation for steady-state covariance."""
    R = get_R(z)
    P_ss = scipy.linalg.solve_discrete_are(F.T, H.T, Q, R)
    return P_ss

def run_prop1_sweep():
    """Verify Proposition 1: Steady-state trace(Sigma) vs altitude for different lags."""
    lags = [0.5, 1.0, 2.0, 3.0]
    altitudes = np.linspace(0.5, 8.5, 300)
    
    plt.figure(figsize=(8, 5))
    
    for d in lags:
        z_star = d / TAN_HALF_FOV
        cov_traces = []
        for z in altitudes:
            if z < z_star:
                # Dropout region: covariance grows without bound
                cov_traces.append(np.nan)
            else:
                try:
                    P_ss = solve_dare_ss(z)
                    cov_traces.append(np.trace(P_ss))
                except:
                    cov_traces.append(np.nan)
                    
        # Plot with cliff
        line, = plt.plot(altitudes, cov_traces, label=f"d={d}m (z*={z_star:.2f})", linewidth=2)
        # Vertical cliff line
        plt.axvline(z_star, color=line.get_color(), linestyle=':', alpha=0.5)
        
    plt.yscale('log')
    plt.ylim(1e-5, 1.0)
    plt.xlim(0.5, 8.5)
    plt.xlabel('UAV altitude z (m)')
    plt.ylabel('steady-state tr($\Sigma$) [log]')
    plt.title('Prop 1: steady-state cov vs altitude')
    plt.legend()
    plt.grid(True, which="both", ls="-", alpha=0.2)
    
    results_dir = '/home/shlok-mehndiratta/Desktop/ugv_uav_control/03_System_Integration_ROS2/results'
    os.makedirs(results_dir, exist_ok=True)
    plt.savefig(os.path.join(results_dir, 'theory_figure1_prop1.png'), dpi=150)
    print(f"Saved {os.path.join(results_dir, 'theory_figure1_prop1.png')}")

def run_prop2_3_simulation():
    """Verify Propositions 2 & 3: Simulation on a periodic hairpin track."""
    t_total = 100.0  # seconds
    steps = int(t_total / DT)
    time_grid = np.arange(steps) * DT
    
    # Generate a realistic lag profile d_k representing a hairpin track:
    # Straight lines (lag = 0.5m) and sharp corners (lag = 1.5m) repeating every 5 seconds.
    lags = np.zeros(steps)
    for k in range(steps):
        t_mod = time_grid[k] % 5.0
        if t_mod < 3.0:
            lags[k] = 0.5  # Straight line
        elif t_mod < 3.5:
            lags[k] = 0.5 + 2.0 * (t_mod - 3.0)  # Rise
        elif t_mod < 4.5:
            lags[k] = 1.5  # Corner
        else:
            lags[k] = 1.5 - 2.0 * (t_mod - 4.5)  # Fall
            
    # Altitude trajectories
    z_fixed_2 = 2.0 * np.ones(steps)
    z_fixed_6 = 6.0 * np.ones(steps)
    
    # 3. Active-reactive policy: targets max(z_min, SAFETY_BUFFER * z*(d_k)) reactive to current lag
    z_reactive = np.zeros(steps)
    z_reactive[0] = max(Z_MIN, SAFETY_BUFFER * lags[0] / TAN_HALF_FOV)
    for k in range(1, steps):
        z_target = max(Z_MIN, SAFETY_BUFFER * lags[k] / TAN_HALF_FOV)
        # Rate-limited climb/descend
        dz = np.clip(z_target - z_reactive[k-1], -VZ_MAX * DT, VZ_MAX * DT)
        z_reactive[k] = np.clip(z_reactive[k-1] + dz, Z_MIN, Z_MAX)
        
    # 4. Active-anticipatory policy: targets max_{i in [k, k+horizon]} (max(z_min, SAFETY_BUFFER * z*(d_i)))
    # Lead time to climb from 2.0 to 3.12m is (3.12 - 2.0) / 0.4 = 2.8s (56 steps)
    lead_steps = int(2.8 / DT)
    z_target_anticipatory = np.zeros(steps)
    for k in range(steps):
        window_end = min(steps - 1, k + lead_steps)
        lags_window = lags[k : window_end + 1]
        z_target_anticipatory[k] = np.max(np.maximum(Z_MIN, SAFETY_BUFFER * lags_window / TAN_HALF_FOV))
    
    z_anticipatory = np.zeros(steps)
    z_anticipatory[0] = z_target_anticipatory[0]
    for k in range(1, steps):
        dz = np.clip(z_target_anticipatory[k] - z_anticipatory[k-1], -VZ_MAX * DT, VZ_MAX * DT)
        z_anticipatory[k] = np.clip(z_anticipatory[k-1] + dz, Z_MIN, Z_MAX)

    # Covariance simulations
    def simulate_covariance(z_trajectory):
        Sigma = np.diag([0.01, 0.01, 0.001, 0.001])
        trace_history = []
        visibility_history = []
        
        for k in range(steps):
            z = z_trajectory[k]
            d = lags[k]
            r_fov = z * TAN_HALF_FOV
            
            # Predict step
            Sigma_pred = F @ Sigma @ F.T + Q
            
            # Update step
            if d <= r_fov:
                # Target visible, update
                R = get_R(z)
                # Kalman gain
                S = H @ Sigma_pred @ H.T + R
                K = Sigma_pred @ H.T @ np.linalg.inv(S)
                Sigma = (np.eye(4) - K @ H) @ Sigma_pred
                visibility_history.append(1)
            else:
                # Target invisible, dropout
                Sigma = Sigma_pred
                visibility_history.append(0)
                
            trace_history.append(np.trace(Sigma))
            
        return np.array(trace_history), np.array(visibility_history)

    cov_fixed_2, vis_fixed_2 = simulate_covariance(z_fixed_2)
    cov_fixed_6, vis_fixed_6 = simulate_covariance(z_fixed_6)
    cov_reactive, vis_reactive = simulate_covariance(z_reactive)
    cov_anticipatory, vis_anticipatory = simulate_covariance(z_anticipatory)
    
    # Print statistics
    print("\n--- Telemetry Metrics Summary ---")
    print(f"Fixed Low (z=2): Mean tr(Sigma) = {np.mean(cov_fixed_2):.6f} | Visibility = {np.mean(vis_fixed_2)*100:.1f}%")
    print(f"Fixed High (z=6): Mean tr(Sigma) = {np.mean(cov_fixed_6):.6f} | Visibility = {np.mean(vis_fixed_6)*100:.1f}%")
    print(f"Active Reactive: Mean tr(Sigma) = {np.mean(cov_reactive):.6f} | Visibility = {np.mean(vis_reactive)*100:.1f}%")
    print(f"Active Anticipatory: Mean tr(Sigma) = {np.mean(cov_anticipatory):.6f} | Visibility = {np.mean(vis_anticipatory)*100:.1f}%")
    
    # Plot results (Figure 2)
    plt.figure(figsize=(8, 5))
    plt.plot(time_grid, cov_fixed_2, label='fixed_2 (low)', linewidth=1.5)
    plt.plot(time_grid, cov_fixed_6, label='fixed_6 (high)', linewidth=1.5)
    plt.plot(time_grid, cov_anticipatory, label='active_anticip', linewidth=1.5)
    
    plt.yscale('log')
    plt.ylim(1e-5, 1.0)
    plt.xlim(0, 100)
    plt.xlabel('time (s)')
    plt.ylabel('tr($\Sigma$) [log]')
    plt.title('Prop B/C: covariance over the hairpin')
    plt.legend()
    plt.grid(True, which="both", ls="-", alpha=0.2)
    
    results_dir = '/home/shlok-mehndiratta/Desktop/ugv_uav_control/03_System_Integration_ROS2/results'
    plt.savefig(os.path.join(results_dir, 'theory_figure2_prop2_3.png'), dpi=150)
    print(f"Saved {os.path.join(results_dir, 'theory_figure2_prop2_3.png')}")

if __name__ == "__main__":
    run_prop1_sweep()
    run_prop2_3_simulation()
