#!/usr/bin/env python3
import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import sys
import os
def load_experiment(filepath):
    return pd.read_csv(filepath)
def plot_comparison(exp_files, labels):
    fig, axes = plt.subplots(2, 3, figsize=(15, 10))
    
    colors = ['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728']
    
    for i, (filepath, label) in enumerate(zip(exp_files, labels)):
        df = load_experiment(filepath)
        c = colors[i % len(colors)]
        
        # 1. Trajectory
        axes[0, 0].plot(df['x'], df['y'], label=label, color=c, alpha=0.7)
        
        # 2. Tracking Error
        error = np.sqrt((df['x'] - df['target_x'])**2 + (df['y'] - df['target_y'])**2)
        axes[0, 1].plot(df['timestamp'], error, label=label, color=c, alpha=0.7)
        
        # 3. Covariance Trace
        axes[0, 2].plot(df['timestamp'], df['cov_trace'], label=label, color=c, alpha=0.7)
        
        # 4. Control Commands
        axes[1, 0].plot(df['timestamp'], df['v_cmd'], label=f'{label} v', color=c, alpha=0.7)
        
        # 5. Angular Velocity
        axes[1, 1].plot(df['timestamp'], df['w_cmd'], label=f'{label} ω', color=c, alpha=0.7)
        
        # 6. FoV Distance
        axes[1, 2].plot(df['timestamp'], df['fov_distance'], label=label, color=c, alpha=0.7)
    
    # Formatting
    axes[0, 0].set_title('Trajectory'); axes[0, 0].set_xlabel('x (m)'); axes[0, 0].set_ylabel('y (m)')
    axes[0, 0].axhline(y=0, color='gray', linestyle='--', alpha=0.5)
    axes[0, 0].axvline(x=0, color='gray', linestyle='--', alpha=0.5)
    circle = plt.Circle((0, 0), 5.77, fill=False, color='red', linestyle='--', label='FoV')
    axes[0, 0].add_patch(circle)
    axes[0, 0].set_aspect('equal')
    axes[0, 0].legend()
    
    axes[0, 1].set_title('Tracking Error'); axes[0, 1].set_xlabel('Time (s)'); axes[0, 1].set_ylabel('Error (m)')
    axes[0, 1].legend()
    
    axes[0, 2].set_title('Covariance Trace'); axes[0, 2].set_xlabel('Time (s)'); axes[0, 2].set_ylabel('tr(Σ)')
    axes[0, 2].legend()
    
    axes[1, 0].set_title('Linear Velocity'); axes[1, 0].set_xlabel('Time (s)'); axes[1, 0].set_ylabel('v (m/s)')
    axes[1, 0].legend()
    
    axes[1, 1].set_title('Angular Velocity'); axes[1, 1].set_xlabel('Time (s)'); axes[1, 1].set_ylabel('ω (rad/s)')
    axes[1, 1].legend()
    
    axes[1, 2].set_title('Distance from FoV Center'); axes[1, 2].set_xlabel('Time (s)'); axes[1, 2].set_ylabel('d (m)')
    axes[1, 2].axhline(y=5.77, color='red', linestyle='--', label='FoV boundary')
    axes[1, 2].legend()
    
    plt.tight_layout()
    plt.savefig(os.path.expanduser('~/Desktop/Project_ws/experiment_logs/comparison.png'), dpi=150)
    plt.show()
def plot_bar_summary(summaries, labels):
    metrics = ['mean_tracking_error', 'control_smoothness', 'mean_cov_trace', 'fov_exit_rate']
    titles = ['Mean Tracking Error (m)', 'Control Smoothness', 'Mean Covariance', 'FoV Exit Rate (%)']
    
    fig, axes = plt.subplots(1, 4, figsize=(16, 4))
    
    for i, (metric, title) in enumerate(zip(metrics, titles)):
        values = [s[metric] for s in summaries]
        axes[i].bar(labels, values, color=['#1f77b4', '#ff7f0e', '#2ca02c', '#d62728'][:len(labels)])
        axes[i].set_title(title)
        axes[i].set_ylabel(metric)
    
    plt.tight_layout()
    plt.savefig(os.path.expanduser('~/experiment_logs/bar_comparison.png'), dpi=150)
    plt.show()
if __name__ == '__main__':
    if len(sys.argv) < 2:
        print('Usage: python plot_experiments.py file1.csv file2.csv ...')
        sys.exit(1)
    
    files = sys.argv[1:]
    labels = [os.path.basename(f).split('_')[0] for f in files]
    plot_comparison(files, labels)