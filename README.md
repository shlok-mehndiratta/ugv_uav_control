# UAV-UGV Multi-Agent Tracking: A Belief-Coupled Tube-MPC Framework

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
*   **Key Deliverables**: Crazyflie SITL validation logs and optimized 3D path coordinates derived from stability constraints.

### [03: System Integration and Validation](./03_System_Integration_ROS2/)
*   **Scientific Goal**: Closing the loop between perception (OpenCV), state estimation (EKF), and belief-space control.
*   **Key Deliverables**: A high-fidelity ROS 2 and Gazebo mission demonstrating Adaptive Tube-MPC and Belief-Coupled tracking.
*   *ROS 2 workspace setup and execution steps available in the sub-directory README.*

---

## Result Verification Matrix
The following table summarizes the integrated performance of the full framework (Phase 3) against the Phase 1 baseline.

| Metric | Baseline (Phase 1) | Full System (Phase 3) | Improvement |
| :--- | :--- | :--- | :--- |
| Mean Tracking Error | 1.037 m | **0.463 m** | **-55.3%** |
| ArUco Visibility | 46.3% | **87.0%** | **+87.9%** |
| MPC Solve Time | ~50.0 ms | **~16.0 ms** | **3.1x Faster** |
| Tube Stability | Standard | **RPI-Bounded** | **Robust** |

---

## Submission Artifacts
*   **Research Report**: [PROJECT_REPORT.pdf](./PROJECT_REPORT.pdf)
*   **Phase 1 Code**: [Mathematical Foundations](./01_Mathematical_Foundations/)
*   **Phase 2 Code**: [Trajectory Engineering](./02_Trajectory_Engineering/)
*   **Phase 3 Code**: [System Integration (ROS 2)](./03_System_Integration_ROS2/)

## Scientific Inferences
1.  **Observability-Control Link**: We demonstrated that coupling the UGV's path-following with the UAV's visual belief state (Belief-Space MPC) significantly reduces tracking lag. Optimal control and state estimation are shown to be inseparable in robust multi-agent tracking.
2.  **Computational Efficiency**: The implementation of an Event-Triggered law, solving the MPC only when the RPI boundary is approached, enabled a 16ms solve time, ensuring feasibility for high-frequency flight hardware.
3.  **Robustness via Adaptation**: The UAV's adaptive altitude adjustment (Adaptive Tube) based on predicted UGV curvature is the primary factor in eliminating visual dropouts during non-linear maneuvers.

## Authors
*   **Shlok Mehndiratta** (23309)
*   **Divyam Sood** (23112)
*   **Prashant Gupta** (23237)

*Course: LCS 334 - Linear Control Systems*
