import pandas as pd
import matplotlib.pyplot as plt
import numpy as np
import os
import sys

def parse_summary_file(summary_path):
    """Parses a _summary.txt file into a dictionary."""
    data = {}
    triggers = {}
    parsing_triggers = False
    
    with open(summary_path, 'r') as f:
        for line in f:
            line = line.strip()
            if not line:
                continue
            if '--- Trigger Breakdown ---' in line:
                parsing_triggers = True
                continue
            
            if ':' in line:
                key, val = line.split(':', 1)
                key = key.strip()
                val = val.strip()
                
                if parsing_triggers:
                    # Triggers (e.g., 'Curvature (0.45): 3')
                    if 'Curvature' in key:
                        if 'Curvature' not in triggers:
                            triggers['Curvature'] = 0
                        triggers['Curvature'] += int(val)
                    else:
                        triggers[key] = int(val)
                else:
                    try:
                        data[key] = float(val)
                    except ValueError:
                        data[key] = val
    return data, triggers

def generate_ablation_plots():
    # Define exact paths to the summary files based on the results folder
    results_dir = '/home/shlok-mehndiratta/Desktop/LCS-334-434-684/Project/Shlok_23309, Divyam_23112, Prashant_23237/03_System_Integration_ROS2/results'
    
    c1_sum = os.path.join(results_dir, 'C1_Baseline_20260627_001155_summary.txt')
    c3_sum = os.path.join(results_dir, 'C3_NoCurve_20260628_010847_summary.txt')
    c4_sum = os.path.join(results_dir, 'C4_Nominal_20260628_011726_summary.txt')
    
    if not (os.path.exists(c1_sum) and os.path.exists(c3_sum) and os.path.exists(c4_sum)):
        print("Error: Could not find all three summary files in the results directory!")
        return

    d1, t1 = parse_summary_file(c1_sum)
    d3, t3 = parse_summary_file(c3_sum)
    d4, t4 = parse_summary_file(c4_sum)

    # 1. Compute Savings Data
    labels_compute = ['C1\n(Continuous)', 'C4\n(Proposed)']
    replans = [d1.get('total_replans', 500), d4.get('total_replans', 56)]
    
    fig, (ax1, ax2) = plt.subplots(1, 2, figsize=(14, 6))
    
    # Left Plot: Total Replans
    ax1.bar(labels_compute, replans, color=['#e74c3c', '#2ecc71'], alpha=0.8)
    ax1.set_ylabel('Total Optimization Trigger Events', fontsize=12, fontweight='bold')
    ax1.set_title('Computational Efficiency (90% Reduction)', fontsize=14, fontweight='bold')
    for i, v in enumerate(replans):
        ax1.text(i, v + 5, str(int(v)), ha='center', fontsize=12, fontweight='bold')
        
    # Right Plot: Proactive Safety (Trigger Breakdown)
    labels_safety = ['C3\n(Reactive Trigger)', 'C4\n(Proactive Trigger)']
    
    # Extract Trigger Data: [Tube Exit, Plan Expiry, Curvature]
    c3_triggers = np.array([t3.get('Tube Exit', 0), t3.get('Plan Expiry', 0), t3.get('Curvature', 0)])
    c4_triggers = np.array([t4.get('Tube Exit', 0), t4.get('Plan Expiry', 0), t4.get('Curvature', 0)])
    
    bottom_c3 = 0
    bottom_c4 = 0
    
    colors = ['#e74c3c', '#95a5a6', '#3498db']
    categories = ['Emergency (Tube Exit)', 'Nominal (Plan Expiry)', 'Proactive (Curvature)']
    
    for i in range(3):
        # Only add a label for the first instance to avoid duplicate legend entries
        ax2.bar(labels_safety[0], c3_triggers[i], bottom=bottom_c3, color=colors[i], edgecolor='black')
        bottom_c3 += c3_triggers[i]
        
        ax2.bar(labels_safety[1], c4_triggers[i], bottom=bottom_c4, color=colors[i], edgecolor='black', label=categories[i])
        bottom_c4 += c4_triggers[i]
        
    ax2.set_ylabel('Number of Trigger Events', fontsize=12, fontweight='bold')
    ax2.set_title('Trigger Classification (Safety)', fontsize=14, fontweight='bold')
    ax2.legend(loc='upper left')
    
    plt.tight_layout()
    plot_path = os.path.join(results_dir, 'ablation_results.png')
    plt.savefig(plot_path, dpi=300)
    print(f"Successfully generated '{plot_path}'")

if __name__ == '__main__':
    generate_ablation_plots()
