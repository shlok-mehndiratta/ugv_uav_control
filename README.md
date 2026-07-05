# Robust Linear Control: A Belief-Coupled Tube-MPC Framework

### LCS 334 - Linear Control Systems
**Final Project Submission**

---

## Project Architecture and Roadmap
This repository presents a unified research framework for autonomous multi-agent tracking, bridging the gap between robust control theory, high-fidelity trajectory engineering, and mission-scale ROS 2 integration. The project is organized into three distinct phases, each building upon the scientific results of the previous.

### [01: Mathematical Foundations](./01_Mathematical_Foundations/)
*   **Scientific Goal**: Guaranteeing stability under bounded disturbances using Robust Positional Invariant (RPI) Sets.
*   **Key Deliverables**: Derivation of the tracking gain K and nominal constraint tightening.
*   *Implementation details and execution steps available in the sub-directory README.*

### [02: Trajectory Engineering](./02_Trajectory_Engineering/)
*   **Scientific Goal**: Validating high-dynamic flight trajectories for micro-UAV hardware.
*   **Key Deliverables**: Crazyflie SITL validation logs and optimized 3D path coordinates derived from stability constraints, docker file to ensure reproducibility.
*   *Implementation details and execution steps available in the sub-directory README.*

### [03: System Integration and Validation](./03_System_Integration_ROS2/)
*   **Scientific Goal**: Closing the loop between perception (OpenCV), state estimation (EKF), and belief-space control.
*   **Key Deliverables**: A high-fidelity ROS 2 and Gazebo mission demonstrating Adaptive Tube-MPC and Belief-Coupled tracking.
*   *ROS 2 workspace setup and execution steps available in the sub-directory README.*

---

## Result Verification Matrix
The following table summarizes the integrated performance comparison between the continuous baseline (C1), standard reactive event-triggered Tube-MPC (C2), and the proposed Belief-MPC (C3) under nominal conditions:

| Metric | C1 (Continuous MPC) | C2 (Reactive Tube-MPC) | C3 (Proposed Belief-MPC) |
| :--- | :--- | :--- | :--- |
| **Number of Replans** | 1,102 | 105 | **99 (-91.0%)** |
| **Mean Tracking Offset** | 0.6395 m | 0.4811 m | 0.6888 m |
| **Max Tracking Offset** | 1.0458 m | 0.5774 m | 1.1232 m |
| **Control Jerk** | 0.0195 | 0.0151 | 0.0286 |
| **Mean Covariance Trace $tr(\Sigma)$** | 0.0067 | 0.0065 | 0.0065 |
| **ArUco Visibility** | 100.0% | 100.0% | 100.0% |

---

## Scientific Inferences

### 1. The Observability-Control Link
We have demonstrated that by coupling the UGV's path-following with the UAV's ground projection (Belief-Space MPC), tracking error is significantly reduced. This proves that optimal control and state estimation are inseparable in robust multi-agent systems.

### 2. Computational Efficiency
The implementation of an Event-Triggered law, which only re-solves the MPC when the RPI boundary is approached, allowed the system to meet real-time constraints with a 16ms solve time.

### 3. Robustness via Adaptation
The UAV's ability to adapt its altitude (Adaptive Tube) based on UGV curvature is the primary factor in eliminating visual dropouts during high-dynamic maneuvers.

### 4. Theoretical Guarantees via Strict Robustness
By decoupling the true system state into a nominal trajectory and a bounded error state within a Tube MPC framework, we established absolute mathematical safety bounds. The computation of the minimal Robust Positively Invariant (mRPI) set utilizing support functions enabled rigorous constraint tightening via the Pontryagin difference. Coupled with an infinite-horizon LQR terminal cost to provide a formal Lyapunov stability certificate, the architecture mathematically guarantees recursive feasibility and system stability, explicitly absorbing worst-case disturbances even when raw measurement noise corrupts the direct state-feedback loop.

---
**Team Members:**
*   Shlok Mehndiratta (23309)
*   Divyam Sood (23112)
*   Prashant Gupta (23237)
