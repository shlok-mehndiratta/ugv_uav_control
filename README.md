# Perception-Constrained Altitude Control for Aerial–Ground Cooperative Tracking

A software-in-the-loop (SITL) study of UAV–UGV cooperative tracking in which a **single
downward UAV camera is the ground vehicle's only exteroceptive sensor** — serving *both* its
navigation reference (path segmentation) *and* its state estimate (ArUco/EKF). This dual role
induces a **coverage–resolution altitude tension**: low altitude sharpens estimation
(ground-sampling noise `R ∝ z²`) but starves navigation coverage (the path lookahead leaves the
camera footprint); high altitude does the reverse. We formalize this tension, derive the altitude
policy that resolves it (fly as low as segmentation allows, climb into turns), and evaluate it in
a genuinely-controlled, reproducible ROS 2 / Gazebo stack.

This repository is the companion code and data for the paper (`paper/ICC12.pdf`). Everything
reported in the paper is regenerable from the committed run data and scripts below.

## Repository layout

```
ugv_uav_control/
├── paper/                     Manuscript (self-contained; compiles standalone / on Overleaf)
│   ├── ICC12.tex / .pdf       — the paper
│   ├── ieeeconf.cls           — IEEE conference class
│   └── media/Fig_*.png        — the three figures used by the paper
└── ros2_ws/                   ROS 2 workspace (colcon)
    ├── src/ugv_uav_control/   nodes, launch, models (UAV/UGV/ground plane), tracks
    ├── scripts/               run + analysis + figure generation (see "Reproducing" below)
    ├── paper_data/            the exact curated data behind each figure/table (see its README)
    ├── results/               scratch output dir for new runs (git-ignored contents)
    ├── requirements.txt
    └── Makefile
```
`build/`, `install/`, `log/`, and raw console logs are generated artifacts and are git-ignored.
The data actually used by the paper lives in `ros2_ws/paper_data/` (one subfolder per figure/table);
`ros2_ws/results/` is only scratch space for freshly generated runs.

## Build & run

Requires **ROS 2 Jazzy** and **Gazebo (Harmonic / `gz-sim` 8)**, plus the Python packages in
`ros2_ws/requirements.txt`. From the workspace:

```bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash

# One mission (headless is required for reproducible closed-loop runs — see note below):
GZ_HEADLESS=1 ros2 launch ugv_uav_control middle_path_follower.launch.py \
    experiment_name:=demo duration:=45
```

Each run writes a per-step CSV and a `_summary.txt` into `ros2_ws/results/`. The launch file
exposes every experimental knob as an argument (`track_name`, `active_altitude`, `z_opt`/`z_min`,
`use_gsd_noise`, the trigger thresholds, spawn pose, …); defaults reproduce the reported regime.

> **Reproducibility note.** Always run **headless** (`GZ_HEADLESS=1`). With the Gazebo GUI the
> rendering thread contends with the ROS 2 executor and forks identical-configuration
> trajectories by ≈2.7 m (and can starve the camera); headless runs agree to ≈0.03 m. **All
> reported results are headless.**

## Reproducing the paper's results

Every figure and the co-design table regenerate from the curated data in
`ros2_ws/paper_data/` (one subfolder per figure — see `ros2_ws/paper_data/README.md`). Run from
`ros2_ws/`:

| Paper element | Data (`paper_data/`) | Regenerate |
|---|---|---|
| Testbed — single camera, dual role | `fig1_testbed/testbed_frame.png` | `python3 scripts/make_testbed_figure.py` → `paper/media/Fig_Testbed.png` |
| Coverage–resolution envelope | `fig3_envelope/env_{track1,track4,track_slalom}_z*` | `python3 scripts/plot_envelope.py` → `paper/media/Fig_Envelope.png` |
| Adaptive vs. fixed altitude | `fig4_altitude/cga_{adaptive,fixedlow,fixedhi}_20260706*` | `python3 scripts/plot_altitude_codesign.py` → `paper/media/Fig_AltPolicy.png` |
| Event-triggered co-design table | `table2_codesign/gcod_{C1,C2}_*` | `python3 scripts/fill_paper_numbers.py` (prints C1 vs C2) |

The testbed and altitude figures regenerate **byte-identically**; the table reproduces exactly. The
envelope figure's canonical PNG is committed (`paper/media/Fig_Envelope.png`); `plot_envelope.py`
regenerates it from data (right panel exact; left panel matches up to the exact std window — see
`paper_data/README.md`).

To re-run the closed-loop experiments from scratch (all headless, GSD noise model, R=3) — output
lands in `results/`, then promote the validated runs into `paper_data/`:

```bash
# Co-design: continuous baseline (C1), error-only ablation (B), full event-trigger (C2)
scripts/run_compare.sh track2.png gcod 3 60

# Altitude policy on the hairpin — adaptive vs. two fixed baselines:
GZ_HEADLESS=1 ros2 launch ugv_uav_control middle_path_follower.launch.py \
    experiment_name:=cga_adaptive track_name:=track_hairpin.png duration:=60 \
    use_gsd_noise:=true active_altitude:=true                       # adaptive
#   ... active_altitude:=false z_opt:=2.5 z_min:=2.5   → cga_fixedlow
#   ... active_altitude:=false z_opt:=7.0 z_min:=7.0   → cga_fixedhi

# Coverage-resolution envelope: one (track, altitude, speed) cell at a time
scripts/envelope_run_cell.sh track4.png 4.0 0.22 env_track4_z4.0
```

Per-track on-lane spawn poses are computed by `scripts/find_spawn.py`; track textures are
generated by `scripts/make_tracks.py`.

## Key results

- **Adaptive altitude dominates fixed altitude** (hairpin, headless, GSD noise, R=3). The
  perception-constrained policy actuates altitude closed-loop over **[2.5, 5.1] m** at **100 %
  marker visibility**, beating both fixed baselines: a fixed-low policy (2.5 m) suffers
  coverage-loss transients (**≈9× larger peak `tr Σ`** as the target leaves the footprint in
  turns); a fixed-high policy pays a **≈61 % mean-estimation penalty** from degraded ground
  sampling. (`scripts/plot_altitude_codesign.py`)
- **Event-triggered co-design.** A perception-driven trigger invokes the expensive belief-space
  aerial planner only when needed, cutting online planner solves **≈25× (797→32 / run)** and total
  planner CPU **≈12×**, at **matched** estimation quality (`tr Σ = 0.0095`, 100 % ArUco
  visibility), for a small loss of inter-trigger trajectory smoothness.
- **Coverage–resolution envelope.** Estimation cost falls with altitude (GSD, `R ∝ z²`) while the
  segmentation/navigation cost rises and worsens with path curvature — the two jaws of the
  altitude tension that the policy trades off.

## Scope (stated honestly)

Results are in simulation with a velocity-controlled UAV abstraction (a full multicopter model is
future work). Because a *single* camera serves both navigation and estimation, the two are coupled
through altitude and cannot be varied independently on this platform — which is exactly why
altitude is the control handle; isolating an observability-only benefit would need a second sensor
to break the coupling. The altitude policy's response is reactive (its timing does not cleanly
*lead* curvature in SITL; the strict anticipation property is analytical). The offline mRPI/tube
derivation is a design-time synthesis that seeds the online adaptive ellipsoidal surrogate; the
online controller carries no online recursive-feasibility certificate. These are stated as scope
in the paper, not hidden.

## Authors

Shlok Mehndiratta, Divyam Sood, Prashant Gupta, and Arijit Sen — Indian Institute of Science
Education and Research (IISER) Bhopal.
