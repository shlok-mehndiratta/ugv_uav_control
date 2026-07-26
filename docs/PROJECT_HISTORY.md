# Project History — Shared-Camera Aerial-Ground Tracking (COAST)

**Purpose.** This note records the *research history* behind the paper that the code and the paper
alone do not show: what was tried, what failed, why the framing changed, and the reasoning that shaped
the final contribution. It is an internal working document (it deliberately records dead ends and
superseded claims) so that anyone continuing the work does not re-walk them.

Dates in the repository are simulated SITL timestamps, not wall-clock. Everything below is grounded in
the code, the run summaries, and the committed data; uncertain items are marked *(uncertain)*.

---

## 1. What the system is

A ROS 2 Jazzy / Gazebo software-in-the-loop (SITL) study of aerial-ground cooperative tracking. A
quadrotor (velocity-controlled abstraction, X500-class) carries the system's **only exteroceptive
sensor**, a single downward camera, above a TurtleBot3-class UGV. That one camera serves **two roles at
once**: (i) lane segmentation into the UGV's navigation waypoint, and (ii) ArUco marker detection into
a five-state EKF that gives the UGV's pose and covariance. A belief-space MPC drives the UGV
continuously; a UAV nominal planner plus an ancillary law drives the UAV. The central idea: because one
camera does both jobs, **UAV altitude couples navigation and estimation** — low altitude sharpens
estimation (ground-sampling noise R ∝ z²) but shrinks the footprint and starves navigation lookahead;
high altitude does the reverse.

---

## 2. The research arc — what was tried, in order

**Phase A — Original framing: event-triggered tube-MPC, UAV as passive relay.** The project began as a
robust-MPC paper: the UGV tracked under an event-triggered tube-MPC, the UAV a passive overhead
observer on a fixed trajectory. This did not yield a strong result — the compute-savings story was real
but thin, and the "robust tube" story did not hold up (see below).

**Phase B — Bugs that invalidated early results (found and fixed).**
- **UAV `/cmd_vel` topic collision:** the UAV's Gazebo velocity plugin shared the UGV's DiffDrive
  `/cmd_vel`, so the UAV mirrored the turtlebot and was never actually controlled by its planner. Fixed
  by giving the UAV plugin its own topic. This invalidated every prior UAV/altitude result.
- **Non-determinism (~2.7 m trajectory fork):** identical configs diverged with the Gazebo GUI open
  (the render thread contends with the executor). Headless (`GZ_HEADLESS=1`) → ~0.03 m agreement.
  Headless is mandatory; all reported numbers are headless.
- **The tube/mRPI was design-time scaffolding, not an online guarantee:** the offline mRPI set was
  never loaded online; a hand-picked `P₀` and gain `K` ran instead. Framing was corrected to
  "tube-inspired" with an adaptive ellipsoidal *trigger* surrogate, not a recursive-feasibility
  certificate.
- **Two inconsistent noise models unified** to the single GSD model R(z)=R₀(z/z₀)²(1+(d/r_fov)²).
- **Co-design attribution:** the belief-space UGV planner runs continuously and is not event-gated;
  only the aerial planner is gated by the trigger. The solve/CPU savings are attributed to the aerial
  planner.

**Phase C — Directions that did not pan out (do not retry blindly).**
- Occlusion injection to expose a belief-coupling robustness benefit: scenario-dependent and largely
  dormant under benign tracks.
- "Event-triggering filters chatter": false — the event-triggered scheme is jitterier, not smoother.
- Isolated observability benefit: not separable on this platform — one camera means covariance is
  altitude-dominated, so estimation cannot be varied independently of navigation. (This limitation
  later became the framing asset: altitude as the shared control handle.)
- Anticipatory climb (altitude *leads* curvature): not demonstrable in SITL (altitude lags κ). Kept
  only as an analytical property.

**Phase D — The pivot: perception-constrained altitude.** Introduced an altitude policy — fly as low as
segmentation allows, climb into turns (the coverage floor rises with curvature). Early runs on a sharp
track suggested adaptive altitude beat both fixed baselines, and this was initially written as a
"dominance" result.

**Phase E — Honest reframe → the coupling paper (final).** Two things reshaped the contribution:
1. On closer analysis the "dominance" is *regime-dependent*: adaptive altitude strictly beats every
   fixed altitude only when the coverage floor **binds** (rises above `z_min`) **and** the curvature
   varies. Our testbed sits on the *non-binding* side (required lookahead ≈ 0.5 m → floor ≈ 0.87 m <
   `z_min` = 2.5 m), so the honest result is **Δ ≈ 0** — adaptive ties the best fixed altitude here.
   This null is the theorem's prediction, not a failure, and became the paper's honest framing.
2. The genuine contribution is the **coupling and its regime map**, not the SITL demonstration. The
   final paper characterizes in closed form the estimation-optimal altitude (the curvature-scheduled
   coverage floor) and the exact regime where adaptation helps, then verifies each prediction in SITL.

The final paper is **`paper/ACODS.tex`** — "COAST: Altitude as a Coupled Perception Variable in
Shared-Camera Aerial-Ground Tracking" (IFAC `ifacconf`). Earlier ICC/SCAT drafts and their split-panel
figures were retired.

---

## 3. Key lessons

1. **The contribution is the coupling, not the SITL.** One camera → navigation *and* estimation →
   altitude is an active perception variable, not a tracking parameter.
2. **The compute co-design is the robust, reproducible headline:** event-trigger the aerial planner →
   ≈25× fewer solves, ≈12× less CPU at matched estimation quality.
3. **Altitude adaptation is regime-dependent.** Present the non-binding Δ ≈ 0 honestly; the binding
   Δ > 0 branch is characterized analytically but not demonstrated on this testbed — reaching it needs
   a speed/curvature-scaled lookahead (the `lookahead_la` recording shows that attempt going
   off-corridor).
4. **The tube-MPC machinery is design-time only;** the online object is an adaptive ellipsoidal
   trigger. Frame it as such.
5. **Reproducibility is fragile:** headless-only; a single shared camera cannot decouple the two roles
   without new sensing.
6. **Honesty repeatedly overruled convenient stories** — the occlusion benefit, chatter-filtering,
   anticipation, observability isolation, and the early dominance claim were all dropped or hedged.

---

## 4. Validated numbers (final paper)

- **Resolution (E0):** ArUco error ê(z) = 0.94 + 2.36 z mm, r = 0.997, R² = 0.995 over z ∈ [2, 7] m →
  metric variance ∝ z² (the R ∝ z² law). One altitude with an intermittent dropout is excluded.
- **Envelope:** steady-state trΣ rises monotonically 0.0059 (z = 2.5) → 0.0105 (z = 7), and is
  track-independent; the navigation (steering) cost is U-shaped in altitude.
- **Adaptive vs fixed (non-binding Var-κ track, 3 reps):** adaptive mean trΣ 0.00595, only 2.8% above
  the lowest fixed altitude (0.00579 at z = 2.5) → Δ ≈ 0; fixed cost rises 0.0058 → 0.0074 → 0.0104
  (+79% at z = 7); adaptive altitude spans [2.5, 3.9] m; ≥ 99% marker visibility.
- **Co-design (Table 2, z = 6 m, 3 reps):** aerial solves 797 ± 16 → 32 ± 1 (≈ 25×); aerial CPU
  3123 ± 56 → 253 ± 6 ms/run (≈ 12×); mean solve 3.9 → 7.9 ms; steady-state trΣ ≈ 0.0095 for both;
  ArUco ≈ 99.9%; UAV–UGV offset 0.128 → 0.060 m.
- **Key parameters:** R(z) = R₀(z/z₀)²(1 + (d/r_fov)²), R₀ = 2.5e-3, z₀ = 3.0; K = diag(0.6, 0.6, 0.8),
  P₀ = 0.186·I₃; FoV φ = 1.047 rad (60°); r_fov = z·tan(φ/2); altitude band [2.5, 7.0] m.
- **Off-limits (stability-critical):** the ancillary law u = ū − K·e and the gains K, P₀ were
  hand-tuned and validated; do not change them casually.

---

## 5. Repository structure

```
.
├── paper/                 final paper: ACODS.tex (COAST) + built PDF, references.bib,
│                          ifacconf.cls/.bst, build.sh, media/ (figures),
│                          scripts/ (figure generators), track_set/, README.md
├── docs/
│   ├── PROJECT_HISTORY.md this file
│   ├── planning/          initial planning notes, kept for reference only
│   └── drafts/            superseded section drafts, kept for reference only
├── ros2_ws/               ROS 2 workspace
│   ├── src/ugv_uav_control/   nodes + Gazebo models
│   ├── scripts/           analysis / experiment / figure scripts
│   ├── paper_data/        curated per-figure data (the reproducibility anchor)
│   ├── results/           scratch dir for new runs (git-ignored, local)
│   ├── Makefile, requirements.txt, README.md
└── recordings/            curated SITL recordings, shipped as sitl_recordings.zip
```

- **`ros2_ws/paper_data/`** holds the exact curated data behind each figure and table, one subfolder
  each (`fig3a_e0`, `fig3_envelope`, `fig4_adaptive`, `table2_codesign`), kept separate from the raw
  scratch `results/` so the final data is never mixed with old runs.
- **`paper/scripts/`** regenerate every figure from `paper_data/` using relative paths only.
- **`recordings/sitl_recordings.zip`** bundles the curated videos (including `lookahead_la`, the
  off-corridor binding attempt); raw runs and loose video files are kept local and git-ignored.

---

The constraint that has governed the project throughout: **honesty is the binding constraint — the
paper is changed to match the code, never the reverse.**
