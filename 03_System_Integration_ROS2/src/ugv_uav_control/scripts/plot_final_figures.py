"""
Publication-Quality Plotting Script for Belief-MPC Paper.
Generates Figures 1-3 and Table 1 from experiment CSVs.

Experiment Naming:
  C1 = Continuous MPC (baseline, solves every timestep)
  C2 = Reactive Tube-MPC (event-triggered, no curvature anticipation)
  C3 = Proposed Belief-MPC (full system)
"""
import pandas as pd
import matplotlib.pyplot as plt
import matplotlib.image as mpimg
import numpy as np
import os
import glob

# ──────────────────── STYLE ────────────────────
plt.style.use('seaborn-v0_8-paper')
plt.rcParams.update({
    'font.family': 'serif',
    'font.size': 12,
    'axes.labelsize': 14,
    'axes.titlesize': 14,
    'legend.fontsize': 11,
    'xtick.labelsize': 11,
    'ytick.labelsize': 11,
    'lines.linewidth': 2.0,
    'grid.alpha': 0.5
})

# ──────────────────── PATHS ────────────────────
BASE_DIR = '/home/shlok-mehndiratta/Desktop/LCS-334-434-684/Project/Shlok_23309, Divyam_23112, Prashant_23237/03_System_Integration_ROS2'
RESULTS_DIR = os.path.join(BASE_DIR, 'results')
TRACK_IMG = os.path.join(BASE_DIR, 'src/ugv_uav_control/models/path_ground_plane_v2/materials/textures/track2.png')

# UGV spawn offset in the Gazebo world (odom resets to 0,0 at spawn)
SPAWN_OFFSET_X = 0.85
SPAWN_OFFSET_Y = -0.40

# ──────────────────── LOAD DATA ────────────────────
def get_latest_csv(prefix):
    files = glob.glob(os.path.join(RESULTS_DIR, f'{prefix}*.csv'))
    return max(files, key=os.path.getctime) if files else None

def load_all():
    data = {}
    for key, prefix in [('C1', 'C1_Continuous'), ('C2', 'C2_Reactive'), ('C3', 'C3_Proposed')]:
        path = get_latest_csv(prefix)
        if path:
            data[key] = pd.read_csv(path)
            print(f"  Loaded {key}: {os.path.basename(path)} ({len(data[key])} samples)")
        else:
            print(f"  {key}: No CSV found for prefix '{prefix}'")
    return data

# ──────────────────── FIGURE 1: Spatial Overlay ────────────────────
def plot_figure_1(data):
    fig, ax = plt.subplots(figsize=(10, 10))

    # Load track background
    try:
        # Load track background
        # standard images have top-left as origin, setting origin='upper' prevents vertical mirroring
        img = mpimg.imread(TRACK_IMG)
        ax.imshow(img, extent=[-6, 6, -6, 6], alpha=0.7, origin='upper')
    except Exception as e:
        print(f"  Warning: Could not load track image: {e}")

    # Pick best available data (prefer C3, then C2, then C1)
    df = data.get('C3')
    if df is None:
        df = data.get('C2')
    if df is None:
        df = data.get('C1')
    if df is not None:
        # Filter out the initial startup transient (first 7.5 seconds)
        df_filtered = df[df['timestamp'] > 7.5]
        
        if 'true_ugv_x' in df.columns:
            # Filter out any link pose contamination (jumps near origin or caster link)
            clean_mask = (df_filtered['true_ugv_x']**2 + df_filtered['true_ugv_y']**2 > 0.01) & \
                         (np.abs(df_filtered['true_ugv_x'] + 0.081) > 0.005) & \
                         (df_filtered['uav_x']**2 + df_filtered['uav_y']**2 > 0.01)
            df_plot = df_filtered[clean_mask]
            
            # Direct physically-correct mapping from Gazebo world coordinates:
            # - Matplotlib horizontal axis (Plot X) maps to world -Y
            # - Matplotlib vertical axis (Plot Y) maps to world +X
            sx = -df_plot['true_ugv_y'].values
            sy = df_plot['true_ugv_x'].values
            sux = -df_plot['uav_y'].values
            suy = df_plot['uav_x'].values
            
            # Dynamically align UAV trajectory rotation to UGV to compensate for camera/ArUco mounting alignment
            from scipy.optimize import minimize
            def loss(theta):
                R = np.array([[np.cos(theta[0]), -np.sin(theta[0])], [np.sin(theta[0]), np.cos(theta[0])]])
                rot_pts = R @ np.vstack([sux, suy])
                return np.mean(np.sum((rot_pts - np.vstack([sx, sy]))**2, axis=0))
            
            res = minimize(loss, [0.0])
            theta_opt = res.x[0]
            R = np.array([[np.cos(theta_opt), -np.sin(theta_opt)], [np.sin(theta_opt), np.cos(theta_opt)]])
            aligned_uav = R @ np.vstack([sux, suy])
            sux, suy = aligned_uav[0], aligned_uav[1]
        else:
            # Fallback to EKF coordinates
            sx = -df_filtered['y'] + 0.40
            sy = df_filtered['x'] + 0.85
            sux = -df_filtered['uav_y'] + 0.40
            suy = df_filtered['uav_x'] + 0.85

        ax.plot(sux, suy, label='UAV Trajectory', color='#e67e22',
                linestyle='-', linewidth=2.5, alpha=0.9)
        ax.plot(sx, sy, label='UGV Trajectory', color='#2980b9',
                linestyle='--', linewidth=2.5)

    ax.set_title('Figure 1: Spatial Trajectory Tracking', fontsize=16, fontweight='bold')
    ax.set_xlabel('Global X (m)')
    ax.set_ylabel('Global Y (m)')
    ax.set_xlim([-6, 6])
    ax.set_ylim([-6, 6])
    ax.set_aspect('equal')
    ax.legend(loc='upper right', frameon=True, shadow=True)

    out_png = os.path.join(RESULTS_DIR, 'Figure_1_Spatial.png')
    out_eps = os.path.join(RESULTS_DIR, 'Figure_1_Spatial.eps')
    plt.savefig(out_png, dpi=300, bbox_inches='tight')
    plt.savefig(out_eps, format='eps', bbox_inches='tight')
    plt.close()
    print(f"  Saved {out_png}")
    print(f"  Saved {out_eps}")

# ──────────────────── FIGURE 2: Time-Series (3-Panel) ────────────────────
def plot_figure_2(data):
    fig, (ax1, ax2, ax3) = plt.subplots(3, 1, figsize=(14, 10), sharex=True,
                                         gridspec_kw={'height_ratios': [2, 1.2, 1]})

    colours = {'C1': '#e74c3c', 'C2': '#f39c12', 'C3': '#2ecc71'}
    labels  = {'C1': 'Continuous MPC (C1)', 'C2': 'Reactive Tube-MPC (C2)', 'C3': 'Proposed Belief-MPC (C3)'}
    styles  = {'C1': '--', 'C2': '-', 'C3': '-'}
    alphas  = {'C1': 0.5, 'C2': 0.8, 'C3': 1.0}

    # Panel A — Tracking Error
    for key in ['C1', 'C2', 'C3']:
        df = data.get(key)
        if df is None:
            continue
        t = np.arange(len(df)) * 0.1
        err = df['target_y'].abs()
        ax1.plot(t, err, label=labels[key], color=colours[key],
                 linestyle=styles[key], alpha=alphas[key])

    ax1.set_ylabel('Tracking Error $e(t)$ (m)', fontweight='bold')
    ax1.legend(loc='upper right')
    ax1.grid(True, linestyle=':')
    ax1.set_title('Figure 2: Temporal Performance Comparison', fontsize=16, fontweight='bold')

    # Panel B — Covariance Trace
    for key in ['C1', 'C2', 'C3']:
        df = data.get(key)
        if df is None:
            continue
        t = np.arange(len(df)) * 0.1
        ax2.plot(t, df['cov_trace'], label=labels[key], color=colours[key],
                 linestyle=styles[key], alpha=alphas[key])

    ax2.set_ylabel('$\\mathrm{tr}(\\Sigma)$', fontweight='bold')
    ax2.legend(loc='upper right')
    ax2.grid(True, linestyle=':')

    # Panel C — Trigger Raster
    raster_positions = {'C1': 0.75, 'C2': 0.45, 'C3': 0.15}
    for key in ['C1', 'C2', 'C3']:
        df = data.get(key)
        if df is None:
            continue
        t = np.arange(len(df)) * 0.1
        trigger_t = t[df['replan_trigger'] > 0]
        y_pos = raster_positions[key]
        ax3.vlines(trigger_t, ymin=y_pos - 0.1, ymax=y_pos + 0.1,
                   color=colours[key], alpha=0.7, linewidth=1.5, label=labels[key])

    ax3.set_yticks([0.15, 0.45, 0.75])
    ax3.set_yticklabels(['C3', 'C2', 'C1'])
    ax3.set_ylabel('Trigger Events', fontweight='bold')
    ax3.set_xlabel('Mission Time (s)', fontweight='bold')
    ax3.set_ylim([0, 1])
    ax3.grid(True, axis='x', linestyle=':')

    plt.tight_layout()
    out_png = os.path.join(RESULTS_DIR, 'Figure_2_TimeSeries.png')
    out_eps = os.path.join(RESULTS_DIR, 'Figure_2_TimeSeries.eps')
    plt.savefig(out_png, dpi=300, bbox_inches='tight')
    plt.savefig(out_eps, format='eps', bbox_inches='tight')
    plt.close()
    print(f"  Saved {out_png}")
    print(f"  Saved {out_eps}")

# ──────────────────── FIGURE 3: Statistical Violin Plots ────────────────────
def plot_figure_3(data):
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(12, 5))

    colours = ['#e74c3c', '#f39c12', '#2ecc71']
    box_data_err = []
    box_data_jerk = []
    box_labels = []

    for i, (key, label) in enumerate([('C1', 'C1\nContinuous'), ('C2', 'C2\nReactive'), ('C3', 'C3\nProposed')]):
        df = data.get(key)
        if df is None:
            continue
        err = df['target_y'].abs()
        jerk = np.sqrt(np.diff(df['v_cmd'].values)**2 + np.diff(df['w_cmd'].values)**2)
        box_data_err.append(err.values)
        box_data_jerk.append(jerk)
        box_labels.append(label)

    def style_violins(vp, ax):
        for i, body in enumerate(vp['bodies']):
            body.set_facecolor(colours[i])
            body.set_alpha(0.6)
            body.set_edgecolor('black')
            body.set_linewidth(1.2)
        vp['cmeans'].set_color('black')
        vp['cmaxes'].set_color('black')
        vp['cmins'].set_color('black')
        vp['cbars'].set_color('black')

    if box_data_err:
        vp1 = ax1.violinplot(box_data_err, showmeans=True, showextrema=True)
        style_violins(vp1, ax1)
        ax1.set_xticks(range(1, len(box_labels) + 1))
        ax1.set_xticklabels(box_labels)
        ax1.set_ylabel('Tracking Error (m)', fontweight='bold')
        ax1.set_title('(a) Error Distribution', fontweight='bold')
        ax1.grid(True, axis='y', linestyle=':')

    if box_data_jerk:
        vp2 = ax2.violinplot(box_data_jerk, showmeans=True, showextrema=True)
        style_violins(vp2, ax2)
        ax2.set_xticks(range(1, len(box_labels) + 1))
        ax2.set_xticklabels(box_labels)
        ax2.set_ylabel('Control Jerk', fontweight='bold')
        ax2.set_title('(b) Control Smoothness', fontweight='bold')
        ax2.grid(True, axis='y', linestyle=':')

    fig.suptitle('Figure 3: Statistical Performance Comparison', fontsize=16, fontweight='bold')
    plt.tight_layout()
    out_png = os.path.join(RESULTS_DIR, 'Figure_3_ViolinPlots.png')
    out_eps = os.path.join(RESULTS_DIR, 'Figure_3_ViolinPlots.eps')
    plt.savefig(out_png, dpi=300, bbox_inches='tight')
    plt.savefig(out_eps, format='eps', bbox_inches='tight')
    plt.close()
    print(f"  Saved {out_png}")
    print(f"  Saved {out_eps}")

# ──────────────────── TABLE 1: Summary ────────────────────
def generate_table(data):
    rows = []
    for key, label in [('C1', 'Continuous MPC'), ('C2', 'Reactive Tube-MPC'), ('C3', 'Proposed Belief-MPC')]:
        df = data.get(key)
        if df is None:
            continue
        err = df['target_y'].abs()
        jerk = np.sqrt(np.diff(df['v_cmd'].values)**2 + np.diff(df['w_cmd'].values)**2)
        rows.append({
            'Condition': f'{key} ({label})',
            'Replans': int(df['replan_trigger'].sum()),
            'Mean Error (m)': f"{err.mean():.4f}",
            'Max Error (m)': f"{err.max():.4f}",
            'Control Jerk': f"{jerk.mean():.4f}",
            'Mean tr(Σ)': f"{df['cov_trace'].mean():.4f}",
            'ArUco (%)': f"{df['aruco_visible'].mean() * 100:.1f}",
        })

    if rows:
        table_df = pd.DataFrame(rows)
        out = os.path.join(RESULTS_DIR, 'Table_1_Summary.csv')
        table_df.to_csv(out, index=False)
        print(f"  Saved {out}")
        print("\n  === TABLE 1 ===")
        print(table_df.to_string(index=False))

# ──────────────────── MAIN ────────────────────
if __name__ == '__main__':
    print("Loading experiment data...")
    data = load_all()

    if not data:
        print("ERROR: No CSV files found! Run experiments first.")
        exit(1)

    print("\nGenerating Figure 1 (Spatial)...")
    plot_figure_1(data)

    print("\nGenerating Figure 2 (Time-Series)...")
    plot_figure_2(data)

    print("\nGenerating Figure 3 (Violin Plots)...")
    plot_figure_3(data)

    print("\nGenerating Table 1 (Summary)...")
    generate_table(data)

    print("\nDone! All figures saved to results/")
