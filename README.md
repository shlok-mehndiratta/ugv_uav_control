# UGV-UAV Multi-Agent Control & Perception

Project focused on coordinated path following and observability-aware aerial tracking.

## Overview
This project implements a ROS 2 based system where a UAV (Drone) autonomously tracks a UGV (Ground Robot) while the UGV follows a pre-defined path. The core innovation (v2.0) is a **Belief-Space MPC** that optimizes the UAV's position to maximize the observability of the UGV, minimizing estimation uncertainty.

---

## UAV Pipeline Extension (v2.0)

### 1. Key Changes from v1.1
- **Mobile Observation Pool**: UAV moved from fixed 10m altitude to dynamic 2D trailer at 5m.
- **Observability-Aware Planning**: UAV MPC now calculates the UGV's covariance trace over a horizon and plans moves that satisfy $q_k$ (view quality) targets.
- **Shared Dynamics Model**: `ugv_model.py` provides unified process/measurement jacobians for EKF and MPC nodes.
- **Topics**: Subscriptions to `/ugv_predicted_traj` (Path) and `/uav_gps_pose` (Pose) enable predictive tracking.

### 2. Key Parameters
- **UAV MPC Weights**: $\lambda=1.0$ (Uncertainty), $\gamma_q=1.5$ (View Quality), $\beta=0.1$ (Effort).
- **UAV Constraints**: $v_{max}=1.5 \text{m/s}$, $N=8$, $dt=0.3\text{s}$.
- **FOV Geometry**: $r_{fov} = 2.89\text{m}$ (calculated for downward camera at 5m altitude).

### 3. Final Performance
| Metric | Centroid Baseline | Full Belief-Space MPC |
| :--- | :--- | :--- |
| Mean UGV Tracking Err | 1.168m | **1.064m** |
| Mean Uncertainty $\text{tr}(\Sigma)$ | 0.0557 | **0.0491** |
| Steady-State Visibility | 98.1% | 93.5% |
| Mean View Quality $q_k$ | 0.801 | **0.849** |

*Note: The lower visibility in MPC is a geometric limitation of 5m altitude during high-speed cornering.*

### 4. Known Limitations
- **Visibility Paradox**: High-speed maneuvers at 5m can cause brief FOV exits even with optimal viewing angles.
- **Assumption of Perfect UAV Pose**: System currently assumes zero noise on the UAV's own GPS/IMU feedback.
- **CV Extrapolation**: Long-range planning reverts to constant velocity if the UGV MPC path is shorter than the UAV horizon.

---

## 🛠️ Usage

### Installation
Ensure you have ROS 2 Humble/Iron and Gazebo Sim installed.
```bash
colcon build --packages-select ugv_uav_control --symlink-install
source install/setup.bash
```

### Running the System
**Full Phase 2 Tube MPC (Recommended):**
```bash
ros2 launch ugv_uav_control middle_path_follower.launch.py controller:=belief_mpc use_timer:=false use_ancillary:=true
```

**Phase 1 Baseline Evaluation (Fixed 1Hz rate, no tube):**
```bash
ros2 launch ugv_uav_control middle_path_follower.launch.py controller:=belief_mpc use_timer:=true use_ancillary:=false
```
