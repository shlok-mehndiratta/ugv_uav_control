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
Quantitative results are **regenerated from fresh simulation logs** (see
`03_System_Integration_ROS2/results/` and the plotting scripts) and must not be hand-edited here.
The headline, reproducible finding is computational: the perception-driven event trigger reduces the
number of online planner solves by roughly an order of magnitude relative to continuous replanning, at
statistically comparable tracking accuracy. Belief-coupling and adaptive-tube effects are exercised only
under the adversarial scenarios (injected occlusion, high curvature) in the simulation study; the
certified mRPI set is an **offline** design artifact and is not wired into the online loop. See
`AUDIT_REPORT.md` for the full claim-status ledger (proven-offline / implemented / simulation / future).

---

## Scientific Inferences

### 1. The Observability-Control Link
The UGV controller couples path-following with estimation quality by penalizing the predicted covariance
trace and FoV exit (belief-space NMPC). This is a genuine bidirectional mechanism; its measurable benefit,
however, appears only when the scenario actually stresses observability (occlusion, high curvature) — under
a benign track all methods coincide.

### 2. Computational Efficiency
The event-triggered law re-solves the UAV planner only when the adaptive ellipsoidal tube is exited or a
perception condition fires, reducing the number of online solves by roughly an order of magnitude versus
continuous replanning. Per-solve timings are logged (`*_solve_time`) rather than asserted.

### 3. Robustness via Adaptation
The UAV tube radius scales with the EKF covariance and the trigger anticipates high curvature. Whether this
prevents visual dropouts is an empirical, scenario-dependent question evaluated under injected occlusion —
not a guarantee.

### 4. Theoretical Scope (offline vs. online)
A minimal Robust Positively Invariant (mRPI) set is computed **offline** via a support-function/Minkowski
construction with a certified truncation bound, and constitutes a valid robust bound for the idealized design.
The **online** controller uses an ellipsoidal invariant-set surrogate with an adaptive radius; it does **not**
re-derive the mRPI online, does not enforce constraint tightening or a terminal set, and therefore carries
**no online recursive-feasibility certificate**. Recursive feasibility and Lyapunov stability hold for the
idealized formulation and are assumed, not verified, for the running code. See `AUDIT_REPORT.md`.

---
**Team Members:**
*   Shlok Mehndiratta (23309)
*   Divyam Sood (23112)
*   Prashant Gupta (23237)
