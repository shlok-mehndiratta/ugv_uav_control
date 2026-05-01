# UAV-UGV Multi-Agent Tracking: A Belief-Coupled Tube-MPC Framework

This repository presents a unified research framework for autonomous multi-agent tracking, bridging the gap between **Robust Control Theory**, **High-Fidelity Trajectory Engineering**, and **Mission-Scale ROS 2 Integration**.

---

## 🏗️ Project Architecture & Roadmap
The project is organized into three distinct phases, each building upon the scientific results of the previous.

### [01: Mathematical Foundations](./01_Mathematical_Foundations/)
*   **Scientific Goal**: Guaranteing stability under bounded disturbances using **Robust Positional Invariant (RPI) Sets**.
*   **Key Deliverable**: Derivation of the tracking gain $K$ and nominal constraint tightening.
*   👉 *See [01_Mathematical_Foundations/README.md](./01_Mathematical_Foundations/README.md) for execution steps.*

### [02: Trajectory Engineering](./02_Trajectory_Engineering/)
*   **Scientific Goal**: Validating high-dynamic flight trajectories for micro-UAV hardware.
*   **Key Deliverable**: **Crazyflie SITL** validation logs and optimized 3D path coordinates.

### [03: System Integration & Validation](./03_System_Integration_ROS2/)
*   **Scientific Goal**: Closing the loop between perception (OpenCV), state estimation (EKF), and belief-space control.
*   **Key Deliverable**: A high-fidelity ROS 2/Gazebo mission demonstrating **Adaptive Tube-MPC** and **Belief-Coupled tracking**.
*   👉 *See [03_System_Integration_ROS2/README.md](./03_System_Integration_ROS2/README.md) for execution steps.*

---

## 📊 Result Verification Matrix
The following table summarizes the integrated performance of the full framework (Phase 3) against the Phase 1 baseline.

| Metric | Baseline (Phase 1) | Full System (Phase 3) | Improvement |
| :--- | :--- | :--- | :--- |
| **Mean Tracking Error** | 1.037 m | **0.463 m** | **-55.3%** |
| **ArUco Visibility** | 46.3% | **87.0%** | **+87.9%** |
| **MPC Solve Time** | ~50.0 ms | **~16.0 ms** | **3.1x Faster** |
| **Tube Stability** | Standard | **RPI-Bounded** | **Robust** |

---

## 🧪 Scientific Inferences
*   **The Observability-Control Link**: We proved that by coupling the UGV's path-following with the UAV's ground projection (Belief-Space MPC), tracking error is reduced by over 50%.
*   **Computational Efficiency**: The implementation of an **Event-Triggered law** (only re-solving the MPC when the RPI boundary is approached) allowed us to meet real-time constraints with a 16ms solve time.
*   **Robustness via Adaptation**: The UAV's ability to adapt its altitude (Adaptive Tube) based on UGV curvature is the primary factor in eliminating visual dropouts.

---

## ✅ Submission Checklist for Evaluation
- [x] **Correct Structure**: Folders 01, 02, and 03 are present at the root.
- [x] **Dependencies**: All requirements are listed in `requirements.txt`.
- [x] **Reproducibility**: Step-by-step commands are provided in each folder's README.
- [x] **Results**: Reported data (55.3% improvement) is verifiable via the Phase 3 simulation logs.

---
*Developed as a collaborative Multi-Agent Systems project by: Shlok (23309), Divyam (23112), Prashant (23237)*
