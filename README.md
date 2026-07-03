# Perception-Constrained Altitude Control for Aerial–Ground Cooperative Tracking

A software-in-the-loop (SITL) study of UAV–UGV cooperative tracking in which a **single
downward UAV camera is the ground vehicle's only exteroceptive sensor** — serving *both* its
navigation reference (path segmentation) *and* its state estimate (ArUco/EKF). This dual role
induces a **coverage–resolution altitude tension**: low altitude sharpens estimation
(ground-sampling noise `R ∝ z²`) but starves navigation coverage (the path lookahead leaves the
camera footprint); high altitude does the reverse. We formalize this tension, derive the
altitude policy that resolves it (fly as low as segmentation allows, climb into turns), and
evaluate it in a genuinely-controlled, reproducible ROS 2 / Gazebo stack.

## Repository layout

```
ugv_uav_control/
├── paper/          Manuscript (self-contained; compiles standalone / on Overleaf)
│   ├── ICC12.tex / .pdf                 — the paper
│   ├── theory_active_altitude.tex/.pdf  — companion theory note (Props A/B/C)
│   ├── ieeeconf.cls                     — IEEE conference class
│   └── Fig_*.png                        — figures used by the manuscripts
└── ros2_ws/        ROS 2 workspace (colcon)
    ├── src/ugv_uav_control/   nodes, launch, models (UAV/UGV/ground plane), tracks
    ├── scripts/               analysis + figure generation (spawn finder, sweep runner, …)
    ├── results/               experiment data (per-run CSVs + summaries)
    ├── requirements.txt
    └── Makefile
```
`build/`, `install/`, `log/`, and raw console logs are generated artifacts and are git-ignored.

## Build & run

Requires ROS 2 Jazzy and Gazebo (Harmonic / `gz-sim` 8). From the workspace:

```bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash

# Run a mission (headless is required for reproducible closed-loop runs — see below):
GZ_HEADLESS=1 ros2 launch ugv_uav_control middle_path_follower.launch.py \
    experiment_name:=demo duration:=45
```

**Reproducibility (important).** Run **headless** (`GZ_HEADLESS=1`). With the Gazebo GUI, the
rendering thread contends with the ROS 2 executor and forks identical-configuration trajectories
by ≈2.7 m (and randomly starves the camera); headless runs agree to ≈0.03 m. All reported
results are headless.

## Key results (all regenerated from the committed run data in `ros2_ws/results/`)

- **Co-design.** A perception-driven event trigger invokes the expensive belief-space planner
  only when needed, cutting online planner solves ≈28× (1255→45 / run) and total planner CPU
  ≈14× at matched estimation quality (`tr Σ = 0.0063`, 100 % ArUco visibility), for a small loss
  of inter-trigger trajectory smoothness.
- **Coverage–resolution envelope.** Estimation cost falls with altitude (GSD, `R ∝ z²`) while
  the segmentation/navigation cost rises and worsens with path curvature — the two jaws of the
  altitude tension.

Per-track on-lane spawns are computed by `scripts/find_spawn.py`; the envelope sweep is driven
by `scripts/envelope_run_cell.sh`.

## Scope (stated honestly)

The demonstrated payoff is computational. The *isolated* observability benefit of
perception-awareness is **not** separable on this platform — the shared camera couples
navigation and estimation, so estimation covariance is dominated by altitude-dependent
measurement noise — and the low-altitude/high-speed corner is non-reproducibly unstable. The
offline mRPI/tube derivation is a design-time baseline that seeds the ellipsoidal shape matrix;
the online controller uses an adaptive ellipsoidal surrogate and carries no online
recursive-feasibility certificate. These are stated as findings in the paper, not hidden.

## Authors

Shlok Mehndiratta, Divyam Sood, Prashant Gupta, and Arijit Sen — Indian Institute of Science
Education and Research (IISER) Bhopal.
