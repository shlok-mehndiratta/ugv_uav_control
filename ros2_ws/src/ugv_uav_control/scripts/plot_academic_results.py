import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import os
import sys

# Set academic plotting style
plt.style.use('seaborn-v0_8-paper')
plt.rcParams.update({
    'font.family': 'serif',
    'font.size': 12,
    'axes.labelsize': 14,
    'axes.titlesize': 14,
    'legend.fontsize': 12,
    'xtick.labelsize': 11,
    'ytick.labelsize': 11,
    'lines.linewidth': 2.0,
    'grid.alpha': 0.6
})

def generate_academic_plots():
    results_dir = '/home/shlok-mehndiratta/Desktop/LCS-334-434-684/Project/Shlok_23309, Divyam_23112, Prashant_23237/ros2_ws/results'
    
    # Update these filenames if they change
    c1_csv = os.path.join(results_dir, 'C1_Baseline_20260627_001155.csv')
    c3_csv = os.path.join(results_dir, 'C3_NoCurve_20260628_010847.csv')
    c4_csv = os.path.join(results_dir, 'C4_Nominal_20260628_011726.csv')
    
    if not (os.path.exists(c1_csv) and os.path.exists(c3_csv) and os.path.exists(c4_csv)):
        print("Error: Could not find all CSV files. Please check the filenames.")
        return

    # Load Data
    df1 = pd.read_csv(c1_csv)
    df3 = pd.read_csv(c3_csv)
    df4 = pd.read_csv(c4_csv)
    
    # Calculate time array (assuming 10Hz sampling as per experiment logger)
    t1 = np.arange(len(df1)) * 0.1
    t3 = np.arange(len(df3)) * 0.1
    t4 = np.arange(len(df4)) * 0.1

    fig = plt.figure(figsize=(12, 10))
    
    # --- Subplot 1: Cumulative Computational Load (Step Plot) ---
    ax1 = fig.add_subplot(2, 1, 1)
    
    # C1 is continuous (1 solve per sample)
    ax1.plot(t1, np.arange(len(t1)), label='Continuous MPC (C1)', color='#e74c3c', linestyle='--')
    
    # C3 and C4 are event-triggered
    ax1.step(t3, df3['replan_trigger'].cumsum(), label='Event-Triggered w/o Curvature (C3)', color='#f39c12')
    ax1.step(t4, df4['replan_trigger'].cumsum(), label='Proposed Belief-MPC (C4)', color='#2ecc71')
    
    ax1.set_title('Cumulative Optimization Load Over Time')
    ax1.set_ylabel('Total Number of Optimizations ($N_{opt}$)')
    ax1.set_xlabel('Mission Time (s)')
    ax1.grid(True, linestyle=':')
    ax1.legend(loc='upper left')

    # --- Subplot 2: Tracking Error Distribution (Box Plot) ---
    ax2 = fig.add_subplot(2, 2, 3)
    
    err1 = np.sqrt((df1['x'] - df1['target_x'])**2 + (df1['y'] - df1['target_y'])**2)
    err3 = np.sqrt((df3['x'] - df3['target_x'])**2 + (df3['y'] - df3['target_y'])**2)
    err4 = np.sqrt((df4['x'] - df4['target_x'])**2 + (df4['y'] - df4['target_y'])**2)
    
    box = ax2.boxplot([err1, err3, err4], patch_artist=True, labels=['C1\n(Continuous)', 'C3\n(Reactive)', 'C4\n(Proposed)'])
    
    colors = ['#e74c3c', '#f39c12', '#2ecc71']
    for patch, color in zip(box['boxes'], colors):
        patch.set_facecolor(color)
        patch.set_alpha(0.6)
        
    for median in box['medians']:
        median.set(color='black', linewidth=1.5)
        
    ax2.set_title('Tracking Error Distribution')
    ax2.set_ylabel('Tracking Error $e(t)$ (m)')
    ax2.grid(True, axis='y', linestyle=':')
    
    # --- Subplot 3: Control Effort Smoothness (Box Plot) ---
    ax3 = fig.add_subplot(2, 2, 4)
    
    # Jerk is the derivative of command velocity
    jerk1 = np.sqrt(np.diff(df1['v_cmd'])**2 + np.diff(df1['w_cmd'])**2)
    jerk3 = np.sqrt(np.diff(df3['v_cmd'])**2 + np.diff(df3['w_cmd'])**2)
    jerk4 = np.sqrt(np.diff(df4['v_cmd'])**2 + np.diff(df4['w_cmd'])**2)
    
    box2 = ax3.boxplot([jerk1, jerk3, jerk4], patch_artist=True, labels=['C1', 'C3', 'C4'], showfliers=False)
    
    for patch, color in zip(box2['boxes'], colors):
        patch.set_facecolor(color)
        patch.set_alpha(0.6)
        
    for median in box2['medians']:
        median.set(color='black', linewidth=1.5)
        
    ax3.set_title('Control Smoothness (Derivatives)')
    ax3.set_ylabel('Control Jerk $\Delta u$')
    ax3.grid(True, axis='y', linestyle=':')

    plt.tight_layout()
    out_path = os.path.join(results_dir, 'academic_results_plot.png')
    plt.savefig(out_path, dpi=300, bbox_inches='tight')
    print(f"Successfully generated '{out_path}'")

if __name__ == '__main__':
    generate_academic_plots()
