# Simulation & plot plan + paper content flow

Companion to `PAPER_PLAN.md` (theory/claims). This is the *execution* plan: the paper's narrative
flow, the figure/table list, and the exact simulation matrix on the **standard 4-track set**.
Track names are **Low-κ / Mid-κ / High-κ / Var-κ** (decided; see §4).

Rule (per your instruction): **figures that don't need a 3-track comparison use the MIXED track**
(the varying-curvature one). The 3 graded tracks are used only where a curvature sweep is the point.

---

## 1. Paper content flow (broader narrative)

The paper is one argument in six moves: *idea → model → theory → controller → evidence → takeaway.*

1. **Introduction — the idea.** Perception-limited UGVs use a UAV as an aerial eye. Prior cooperative
   systems use *separate* sensors and treat altitude as a passive parameter. We observe that with a
   *single shared* downward camera, UAV altitude is the sole ground-sampling-distance and therefore
   sets **both** localization resolution **and** navigation coverage at once — so altitude is an
   *active-perception control variable*. Contributions: (C1) optimal-altitude theory for this coupling,
   (C2) a certifiable event-triggered realization, (C3) a reproducible SITL validation.

2. **Related work — the gap.** Three-way daylight: cooperative aerial–ground (separate sensors, passive
   altitude); perception-aware planning (FoV as a hard constraint, single robot); belief-space planning
   (family our UGV MPC belongs to). None reduces the coupling to one altitude scalar with a closed-form
   optimum.

3. **System & models — set up the coupling.** UGV unicycle; UAV single-integrator; the **GSD model
   `R(z) = (R0/z0²)(z² + const)`** (monotone in z) and the **coverage factor `η = z·tan(φ/2)/d_look(κ)`**.
   State the shared dual role of the one camera explicitly.

4. **Optimal altitude (C1, theory).** Constrained estimation problem → **Prop A** (optimal altitude =
   coverage floor `z* = d_look(κ)/tan(φ/2)`, via DARE monotonicity) → **Cor. 1** (climb into turns).
   The deployed altitude policy is the floor + an engineering margin (stated honestly).

5. **Event-triggered aerial planning (C2, realization + theory).** Belief-space UGV MPC (continuous);
   aerial planner + ancillary law; perception-driven trigger; **Prop E** (inter-trigger ISS /
   ultimate-boundedness + no-Zeno-by-construction); tube-*inspired* surrogate concession.

6. **Simulation study (C3, evidence).** Testbed; the measured `R∝z²` (E0); the reproducible two-jaw
   trade-off (envelope); adaptive-altitude dominance; event-triggered co-design; all with reps.

7. **Conclusion.** Altitude as an active-perception variable; honest limitations (velocity-controlled
   UAV, single-camera coupling, reactive timing, design-time tube); future = hardware/full multicopter.

Figure/section map: Fig.1→§Intro, Fig.2→§System, Fig.3(a,b)→§C1+§Sim, Fig.4→§Sim, Table II→§Sim,
(opt.) Fig.5→§C2.

---

## 2. Figures & tables (final list, with track usage)

| # | Item | Track(s) | Shows |
|---|---|---|---|
| Fig. 1 | Testbed: one camera, dual role | **Var-κ** (1 frame) | raw frame + lane-seg heatmap→waypoint + ArUco→EKF |
| Fig. 2 | System overview (TikZ) | — | signal flow; blue = coupling |
| **Fig. 3a** | **E0**: measured ArUco error vs altitude | **Var-κ** (static) | error ∝ z, r=0.996 → validates `R∝z²` |
| **Fig. 3b** | **Envelope**: two jaws vs altitude | **Low-κ/Mid-κ/High-κ** | estimation (meanCov)↓ + steering-effort↑ with altitude; steering rises with curvature |
| **Fig. 4** | **Adaptive vs fixed altitude** | **Var-κ** | altitude trace modulates; adaptive gets low-alt estimation + high-alt navigation |
| Fig. 5 (opt.) | Prop E illustration | **Var-κ** (reuse Table II run) | ‖e_k‖ vs ultimate bound; inter-event-time histogram |
| Table I | Parameters | — | from code |
| **Table II** | **Event-triggered co-design** | **Var-κ** | solves, CPU, matched estimation quality |
| Table III (opt.) | Perception-trigger ablation | **Var-κ** | perception trigger vs plain error-trigger (adds novelty to C2) |

---

## 3. Simulation matrix (what to run)

**Common:** headless (`GZ_HEADLESS=1`, `MPF_SHOW_WINDOW` unset), `use_gsd_noise:=true` for closed-loop runs
(E0 uses the raw sensor, no injection), common spawn per track (from `make_standard_tracks`/mapping),
3 reps unless noted, `/usr/bin/python3.12` for any standalone rclpy node, one sim at a time, cleanup
between (see the validated `scratchpad/*.sh` harnesses).

| Batch | Track(s) | Conditions | Reps | Runs | Metrics logged |
|---|---|---|---|---|---|
| **E0** (Fig 3a) | Var-κ (static hover, d≈0) | z ∈ {2,3,4,5,6,7} | 1 (deterministic) | 6 | raw ‖aruco−truth‖ vs z |
| **Envelope** (Fig 3b) | Low-κ, Mid-κ, High-κ | z ∈ {2.5,3,4,5,6,7}, `active_altitude:=false`, `z_opt=z` | 3 | 3×6×3 = **54** | meanCov, control_smoothness(steering), aruco vis, FoV-exit, dist |
| **Adaptive** (Fig 4) | Var-κ | {adaptive-reactive (`active_altitude:=true`, `altitude_kappa_gain:=0`), adaptive-anticipatory (`active_altitude:=true`, `altitude_kappa_gain:=K`), fixed-low z=2.5, fixed-high z=7} | 3 | 12 | altitude trace, meanCov, steering, vis, peak tracking offset |
| **Co-design** (Table II) | Var-κ | {C1 continuous (`use_timer:=true`), C2 event (`c1_baseline:=false`)} | 3 | 6 | aerial solves, CPU, meanCov, aruco vis, UAV–UGV offset, inter-event times (→ Fig 5) |
| **Ablation** (Table III, opt.) | Var-κ | {B = `error_only:=true`} (vs C2 reused) | 3 | 3 | solves, CPU, meanCov |
| **Fig 1 frame** | Var-κ | 1 short run, capture `/overhead_camera/image` + debug | 1 | 1 | one representative frame |
| **Total** | | | | **~79** (~76 core) | |

Notes:
- **Low-altitude cells:** with the 1.1 m lane, verify z=2.5 (and z=2 for E0) segments cleanly on the
  sharp track during execution; drop the lowest cell if it fails on straights (that would be a rig
  limit, not the coverage phenomenon — log it either way).
- **Envelope reps:** 3 is safe; if wall-clock is tight, 2 reps still gives error bars (note the choice).
- **Prop E (Fig 5):** derived from Table II C2 logs — **no extra runs**.
- **Altitude policy (DECIDED 2026-07-25, Option A):** report the reactive coverage-floor
  `z≥d/tan(φ/2)+margin` (`altitude_kappa_gain=0`) as the honest core AND an anticipatory
  κ-feedforward (`altitude_kappa_gain=K>0`) that literally realizes `z*(κ)` ("climb into turns").
  Both already implemented (`uav_mpc_node.py:162`, launch param `altitude_kappa_gain`) — **no
  refactor**. The reactive-vs-anticipatory ablation on Var-κ is a result (peak-offset reduction),
  not a caveat. Pick `K` so the sharpest Var-κ bend (κ≈1.4) gives ~0.5–1 m anticipatory climb;
  confirm κ_ahead magnitude on the first adaptive run before fixing K.
- Each run ≈ 45–60 s incl. startup + cleanup; batches run track-by-track via the file-based harnesses
  already validated (robust cleanup, DDS-shm purge, no inline `pkill` self-match).

Plot scripts to (re)write against `paper_data/` once runs land: `plot_envelope.py` (Fig 3b, add E0
panel 3a + error bars), `plot_altitude_codesign.py` (Fig 4 on reproducible metrics — drop the fragile
peak-covariance panel), `fill_paper_numbers.py` (Table II), `make_testbed_figure.py` (Fig 1).

---

## 4. Track naming (DECIDED): Low-κ / Mid-κ / High-κ / Var-κ

Names tie directly to the curvature grading (mean κ), introduced once in §System and used everywhere
(paper, plot legends, data folders, scripts). Filesystem-safe ids: `lowk / midk / highk / vark`
(texture files `track_lowk.png` … `track_vark.png`, already regenerated).

- **Low-κ**  (3 bends, R≈1.67 m, mean κ≈0.36)
- **Mid-κ**  (5 bends, R≈1.0 m, mean κ≈0.51)
- **High-κ** (7 bends, R≈0.71 m, mean κ≈0.65)
- **Var-κ**  (2 gentle + 3 sharp bends, mean κ≈0.50) — variable curvature; the default for any
  single-track figure.
