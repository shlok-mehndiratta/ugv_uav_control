# COAST: Altitude as a Coupled Perception Variable in Shared-Camera Aerial-Ground Tracking

Companion code and data for the paper. A software-in-the-loop (SITL) study of UAV–UGV cooperative
tracking in which a **single downward UAV camera is the ground vehicle's only exteroceptive sensor**,
serving *both* its navigation reference (lane segmentation) *and* its state estimate (ArUco + EKF). This
dual role couples navigation and estimation through **one scalar, the UAV altitude**: low altitude
sharpens estimation (ground-sampling noise `R ∝ z²`) but shrinks the footprint and starves navigation
coverage; high altitude does the reverse. The paper characterizes this coupling in closed form — the
estimation-optimal altitude is a curvature-scheduled coverage floor, and adapting altitude strictly
beats any fixed altitude exactly when that floor binds and the curvature varies — and verifies each
prediction in a reproducible ROS 2 / Gazebo stack.

Everything reported in the paper regenerates from the committed data and scripts below.

## Repository layout

```
.
├── paper/            The paper (compiles standalone / on Overleaf)
│   ├── ACODS.tex / .pdf     the manuscript (COAST)
│   ├── references.bib        bibliography
│   ├── ifacconf.cls / .bst   IFAC conference class + bib style
│   ├── build.sh              pdflatex + bibtex build script
│   ├── media/                the figures used by the paper
│   ├── scripts/              figure generators (read paper_data, write media)
│   └── track_set/            track-family assets
├── docs/
│   ├── PROJECT_HISTORY.md    research history: what was tried, what failed, why
│   ├── planning/             initial planning notes (reference only)
│   └── drafts/               superseded section drafts (reference only)
├── ros2_ws/          ROS 2 workspace (colcon)
│   ├── src/ugv_uav_control/  nodes, launch, Gazebo models, tracks
│   ├── scripts/              experiment runners + analysis
│   ├── paper_data/           the exact curated data behind each figure/table
│   ├── results/              scratch output for new runs (git-ignored)
│   ├── requirements.txt, Makefile
└── recordings/       curated SITL recordings (sitl_recordings.zip)
```

`build/`, `install/`, `log/`, raw console logs, and the loose recordings are generated or local and are
git-ignored. **The data used by the paper lives in `ros2_ws/paper_data/`** (one subfolder per
figure/table), kept separate from the scratch `results/` so the final data never mixes with old runs.

## Build the paper

```bash
cd paper
./build.sh          # pdflatex + bibtex + pdflatex ×2  (needs texlive-full; or use Overleaf)
```

## Build & run the simulation

Requires **ROS 2 Jazzy** and **Gazebo (Harmonic)**, plus the Python packages in
`ros2_ws/requirements.txt`. From the workspace:

```bash
cd ros2_ws
colcon build --symlink-install
source install/setup.bash

# One mission (headless is required for reproducible closed-loop runs):
GZ_HEADLESS=1 ros2 launch ugv_uav_control middle_path_follower.launch.py \
    experiment_name:=demo duration:=45
```

Each run writes a per-step CSV and a `_summary.txt` into `ros2_ws/results/`. The launch file exposes
every experimental knob as an argument (track, altitude policy, `z_min`/`z_max`, GSD noise, trigger
thresholds, spawn pose); the defaults reproduce the reported regime.

> **Reproducibility note.** Always run **headless** (`GZ_HEADLESS=1`). With the Gazebo GUI the render
> thread contends with the ROS 2 executor and forks identical-configuration trajectories by ≈2.7 m;
> headless runs agree to ≈0.03 m. **All reported results are headless.**

## Regenerate the paper figures

The figures regenerate from the curated data in `ros2_ws/paper_data/` (subfolders `fig3a_e0`,
`fig3_envelope`, `fig4_adaptive`, `table2_codesign`), using relative paths only:

```bash
python3 paper/scripts/make_paper_plots.py      # Fig 3 (E0), Fig 4 (envelope), Fig 5 (adaptive)
python3 paper/scripts/make_testbed_panels.py   # Fig 1 (testbed dual-role panels)
```

Both read `ros2_ws/paper_data/` and write into `paper/media/`. The co-design table numbers are computed
by `ros2_ws/scripts/fill_paper_numbers.py`.

## Key results (as stated in the paper)

- **Resolution follows `R ∝ z²`.** Measured ArUco error grows linearly with altitude (R² = 0.995), so
  the position variance scales as z² — the model's premise, measured directly.
- **Estimation cost is monotone and track-independent in altitude**, while the navigation cost is
  U-shaped — the two sides of the coverage–resolution coupling.
- **Altitude adaptation is regime-dependent.** Our testbed sits in the *non-binding* regime (the
  required lookahead keeps the coverage floor below `z_min`), where the theory predicts — and the data
  confirm — that adaptive altitude **ties** the best fixed altitude (Δ ≈ 0) while avoiding the large
  estimation penalty any higher fixed altitude pays. The strict-dominance (Δ > 0) regime is
  characterized analytically; demonstrating it needs a speed/curvature-scaled lookahead (future work).
- **Event-triggered co-design.** A perception-driven trigger invokes the expensive aerial planner only
  when needed, cutting planner solves ≈25× (797 → 32 / run) and planner CPU ≈12×, at matched estimation
  quality (`tr Σ ≈ 0.0095`, ≈99.9 % marker visibility).

## Scope (stated honestly)

Results are in simulation with a velocity-controlled UAV abstraction (a full multicopter model is
future work). Because a *single* camera serves both navigation and estimation, the two are coupled
through altitude and cannot be varied independently on this platform. The online controller monitors an
adaptive ellipsoidal surrogate seeded by a design-time tube synthesis; it carries no online
recursive-feasibility certificate. These are stated as scope in the paper, not hidden.

## Authors

Shlok Mehndiratta, Prashant Gupta, Divyam Sood, Agniva Banerjee, and Arijit Sen — Department of
Electrical Engineering and Computer Science, Indian Institute of Science Education and Research (IISER)
Bhopal.
