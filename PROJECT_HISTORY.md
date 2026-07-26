# Project history & handoff notes — UAV–UGV shared-camera tracking paper

**Purpose.** This document hands the project to a future agent/collaborator with the *research
history* that the code and paper alone don't show: what was tried, what failed, why we pivoted, and
the inferences that shaped the current paper. It exists because we are about to **re-scope the
paper** (strengthen the theory / sharpen the contribution; the current SITL-only offering is thin),
and the next session should not re-walk dead ends. It is an internal working doc (documents
failures) — it is **not** committed/pushed by default; keep it local or remove before any public push.

Written 2026-07 (dates in the repo are simulated/SITL timestamps, not wall-clock). Everything below
is grounded in the code, git history, run summaries, and prior findings — no speculation is stated
as fact; uncertain items are marked *(uncertain)*.

---

> # 🚨 OUTSTANDING CORRECTION — CO-DESIGN TRACK LABEL
> **The co-design experiment (Table II, C1 vs C2) was run on `track2` — NOT the "sharp track".**
> "sharp track" was a wrong label; **the correct name is `track2`.**
>
> - ✅ **Source tex fixed:** `paper/initial_vs1.tex` now says **track2** in the Table II caption
>   (line 571) and body text (line 546).
> - ❌ **STILL TO FIX — the compiled PDF:** `paper/AjS_ICC12-2.pdf` still prints **"sharp track"**
>   (≈3× on p.5). It was **not** recompilable locally (missing LaTeX packages + `\proof`/`\url`
>   version clashes that don't occur on Overleaf). **Recompile `initial_vs1.tex` on Overleaf** to
>   regenerate the PDF with **track2**.
> - ❗ **Carry this into the final version:** whatever paper version becomes canonical (SCAT /
>   ACODS / re-scoped), the co-design track MUST read **track2**.
> - ⚠️ Do NOT change the *altitude* / Fig. 4 references — those correctly say "sharp track"
>   (the hairpin altitude experiment, a different run).

---

## 1. What the system actually is (one paragraph)

A ROS 2 Jazzy / Gazebo (Harmonic) software-in-the-loop (SITL) study of aerial–ground cooperative
tracking. A quadrotor (velocity-controlled abstraction, X500-class) carries the **only exteroceptive
sensor** — a single downward camera — above a TurtleBot3-class UGV. That one camera serves **two
roles at once**: (i) lane/path segmentation → the UGV's navigation waypoint, and (ii) ArUco (ID 55,
`DICT_6X6_250`) detection → a 5-state EKF giving the UGV's pose/covariance. A belief-space NMPC drives
the UGV `/cmd_vel` (continuously), and a UAV nominal planner + ancillary law drives the UAV. The
intellectual hook: because one camera does both jobs, **UAV altitude couples navigation and
estimation** — low altitude sharpens estimation (GSD, R∝z²) but shrinks the footprint and starves
navigation lookahead; high altitude does the opposite.

---

## 2. The research arc — what we tried, in order (including dead ends)

**Phase A — Original framing: event-triggered tube-MPC (UGV), UAV as passive relay.**
The project started as a robust-MPC paper: UGV tracked under an event-triggered tube-MPC, UAV treated
as a passive overhead observer following a fixed/simple trajectory. Baselines/conditions compared were
continuous replanning vs event-triggered planning (and classical UGV controllers exist in the tree:
`pure_pursuit_node`, `stanley_node`, `mpc_node`, alongside `belief_mpc_node`). **This did not yield a
sound, publishable result** — the compute-savings story was real but thin, and the "robust tube"
story didn't hold up (see inferences). Several early result sets were later found invalid (below).

**Phase B — Bugs that invalidated early results (found & fixed).**
- **UAV `/cmd_vel` topic collision:** the UAV's Gazebo velocity plugin shared the UGV's DiffDrive
  `/cmd_vel`, so the UAV *mirrored the turtlebot and was never actually controlled by its planner*.
  Fixed by giving the UAV plugin its own topic (`/model/uav/cmd_vel`). **This invalidated every prior
  UAV/altitude result** and had to be re-run. (See memory `finding-uav-cmdvel-topic-collision`.)
- **"UAV can't actuate vertically" was false** — an artifact of the topic bug; after the fix the UAV
  altitude actuates over its full band.
- **Non-determinism (~2.7 m trajectory fork):** identical configs diverged with the Gazebo GUI open
  (render thread contends with the ROS 2 executor, camera occasionally starved). **Headless
  (`GZ_HEADLESS=1`) → ~0.03 m agreement.** Headless is mandatory for any reproducible run; all
  reported numbers are headless.
- **The tube/mRPI was scaffolding, not an online guarantee:** the offline mRPI set was never loaded
  online (dead code path); a hand-picked isotropic `P₀=0.186·I₃` and gain `K=diag(0.6,0.6,0.8)` ran
  instead, and the tube-exit trigger effectively never fired online. Framing was corrected to
  "tube-*inspired*" with an adaptive ellipsoidal *trigger* surrogate — **not** a recursive-feasibility
  certificate.
- **Two inconsistent noise models:** an early "view-quality bowl" `q_k` model coexisted with the GSD
  model `R(z)=R₀(z/z₀)²(1+(d/r_fov)²)`. Unified everything to the **single GSD model** (`q_k` demoted
  to a visibility gate); this made the co-design covariance (~0.0095) consistent with the envelope
  (~0.0094 at z=6).
- **Co-design mis-attribution:** the belief-space planner is the UGV's *continuous* `/cmd_vel`
  controller and is **not** event-gated; only the **UAV (aerial) nominal planner** is gated by the
  event trigger. Early text credited the belief planner; corrected so the co-design solve/CPU savings
  are attributed to the **aerial planner**.

**Phase C — Workarounds that did NOT pan out (important: don't retry blindly).**
- **Occlusion injection** (`block_aruco`) to expose a belief-coupling robustness benefit: the effect
  is **scenario-dependent and largely dormant** under benign tracks — not a robust headline.
- **"Event-triggering filters chatter":** FALSE. The event-triggered condition (C2) is *jitterier*
  (much more inter-trigger trajectory content), not smoother. We do **not** claim smoothness.
- **Isolated observability benefit:** NOT separable on this platform — because one camera serves both
  roles, covariance is altitude-dominated and you can't vary estimation independently of navigation
  without a second sensor. (This limitation later became a *framing asset*: altitude as the shared
  control handle.)
- **Anticipatory climb (altitude *leads* curvature):** NOT demonstrable in SITL — altitude *lags* κ
  (corr(z,κ)≈0.17). Kept only as an **analytical** property (a Proposition), never claimed empirically.
- **Supplied `tube_params.npz`** (K=2·I, P₀=6.86·I, isotropic): **rejected** as an AI-scratch
  heuristic inconsistent with the running controller; not wired in.

**Phase D — The pivot that worked: perception-constrained altitude.**
Introduced an **altitude policy**: fly as low as segmentation allows, climb into turns (coverage floor
rises with curvature). Demonstrated that **adaptive altitude dominates both fixed baselines** on a
sharp/hairpin track (headless, GSD, R=3): a fixed-low policy suffers coverage-loss transients
(~9× larger peak trΣ), a fixed-high policy pays a ~61% mean-estimation penalty (degraded GSD), and the
adaptive policy gets the best of both at ~100% marker visibility. This became the paper's core result.

**Phase E — Honest reframe → "shared-camera coverage–resolution" paper.**
Reframed from a tube-MPC paper to a perception-coupling paper: single GSD model; co-design presented
as **aerial-planner compute savings** (event-trigger the expensive planner); testbed figure from a
real captured frame; trimmed to 6 pages. Kept two headline results: (C1) altitude dominance, (C2)
event-triggered co-design.

**Phase F — Current revision. The canonical final source is `paper/initial_vs1.tex`** (user-confirmed).
- **`paper/initial_vs1.tex`** — title **"SCAT: Shared-Camera Adaptive Tracking for Perception-Coupled
  Aerial-Ground Systems"**, **5 authors** (Shlok, Divyam, Prashant, **Agniva Banerjee**, Arijit Sen),
  link **github.com/soododivyam/SCAT**, bib `ICC_kisan.bib`. It uses the **split/per-panel** figures:
  Fig. 1 = `Fig_Testbed1.png` + `Fig_Testbed2.png`; Fig. 2 = TikZ system overview (drawn, no image);
  Fig. 3 = `Fig_Envelope1.png` + `Fig_Envelope2.png`; Fig. 4 = `Fig_AltPolicy1/2/3.png`.
- `paper/AjS_ICC12-2.pdf` — the compiled PDF of this SCAT version.
- Parallel/older drafts also present: `paper/ICC12(1).tex` (title "Perception-Constrained Altitude
  Control…", 4 authors, uses the **combined** figures `Fig_Testbed/Envelope/AltPolicy.png`) and
  `New Paper/ACODS.tex` (ACODS-venue variant). Same underlying results/data; differ in
  title/authors/venue and combined-vs-split figure layout.

**⚠ Figure-file correction (supersedes an earlier wrong note):** the **numbered/split** figures
(`Fig_*1/2/3.png`) are the REAL figures of the final paper — NOT clutter. The **combined**
`Fig_Testbed.png`/`Fig_Envelope.png`/`Fig_AltPolicy.png` are used only by the older `ICC12(1).tex`.
All 7 split panels were verified to show the exact same validated content/data as the combined ones
(same `paper_data/`). **Reproducibility gap:** the committed scripts (`make_testbed_figure.py`,
`plot_envelope.py`, `plot_altitude_codesign.py`) emit the **combined** figures; the exact **split-panel**
generator is NOT in the repo (the split PNGs were produced separately). Data is correct and archived;
only the per-panel plotting recipe/styling is uncommitted.

---

## 3. Key inferences / lessons (the distilled takeaways)

1. **The genuine contribution is the coupling, not the SITL.** One camera → navigation *and*
   estimation → **altitude is an active perception variable**, not a tracking parameter. This is the
   idea worth strengthening theoretically in the next phase.
2. **The robust, reproducible headline is compute co-design** (event-trigger the aerial planner):
   ≈25× fewer solves, ≈12× less CPU at matched estimation quality. This survives scrutiny.
3. **The altitude-dominance result is real and demonstrated**, but its *anticipatory* character is
   analytical only (z lags κ in SITL). Don't over-claim timing.
4. **The tube-MPC machinery is design-time only.** Frame as tube-*inspired*; the online object is an
   adaptive ellipsoidal *trigger*. Any theory upgrade should either (a) make the online guarantee
   real, or (b) keep framing it honestly as a heuristic monitor.
5. **Reproducibility is fragile:** headless-only; single shared camera means you cannot decouple the
   two roles without new hardware/sensing.
6. **Honesty has repeatedly overruled convenient stories** (occlusion benefit, chatter-filtering,
   anticipation, observability isolation were all dropped or hedged). The next reframe must find
   *stronger, defensible* claims — likely on the theory side — rather than resurrect these.
7. **User's own current assessment:** the SITL-only framing is not strong enough for the venue; the
   plan is to keep the (mostly unchanged) experimental work but **re-scope around a more
   theoretically robust contribution.**

---

## 4. Current validated numbers (so the next agent has the ground truth)

- **Co-design (Table II), gcod C1 continuous vs C2 event, R=3, headless, GSD:**
  aerial solves **797±16 → 32±1** (≈25×); aerial CPU **3123±56 → 253±6 ms/run** (≈12×);
  mean solve **3.9±0.1 → 7.9±0.4 ms**; steady-state trΣ **≈0.0095** both; ArUco **≈99.8 / 99.9 %**;
  UAV–UGV offset **≈0.128 → 0.056 m** (matches the paper's 0.128 → 0.060; the offset uses the
  **EKF-estimated** UGV position `(x,y)`, not ground truth — confirmed and wired into `fill_paper_numbers.py`).
- **Altitude (Fig. 4), hairpin, R=3:** adaptive mean/max trΣ **0.0065 / 0.016**; fixed-low
  **0.0062 / 0.14** (≈9× peak); fixed-high **0.0105 / 0.030** (≈61% mean penalty); adaptive z spans
  **[2.5, 5.1] m**; ~100% ArUco.
- **Envelope (Fig. 3):** mean trΣ **0.0053 (z=2) → 0.0095 (z=6)**; steering effort std(ω) declines
  with η; η = z·tan(φ/2)/d_look with **d_look≈0.70 m**, φ=1.047.
- **Key params:** GSD R(z)=R₀(z/z₀)²(1+(d/r_fov)²), R₀=2.5e-3, z₀=3.0; K=diag(0.6,0.6,0.8),
  P₀=0.186·I₃, A_K=I−Δt·K=diag(0.97,0.97,0.96) (Δt=0.05); FoV φ=1.047 (60°); r_fov=z·tan(φ/2).
- **⚠ OFF-LIMITS (stability-critical):** the ancillary law `u=ū−K·e` and the gains **K, P₀** must not
  be changed casually — they were hand-tuned and validated.

---

## 5. Current repo state (after the 2026-07 cleanup/reorg)

- **`paper/`** — **`initial_vs1.tex` is the canonical final source** (SCAT, 5 authors) + `ICC_kisan.bib`,
  `AjS_ICC12-2.pdf` (its compiled PDF), the parallel draft `ICC12(1).tex` (Perception-Constrained,
  4 authors), `ieeeconf.cls`, and `media/`. The old `ICC12.tex`/`ICC12.pdf` were deleted. `media/`
  holds: the **split/per-panel figures used by the final paper** — `Fig_Testbed1/2.png`,
  `Fig_Envelope1/2.png`, `Fig_AltPolicy1/2/3.png` — and the **combined** figures
  `Fig_Testbed/Envelope/AltPolicy.png` used only by the older `ICC12(1).tex`. All verified to carry
  the same validated content/data. Fig. 2 (system overview) is TikZ, no image asset.
- **`ros2_ws/paper_data/`** — NEW: the exact curated data behind each figure/table, one subfolder
  each (`fig1_testbed`, `fig3_envelope`, `fig4_altitude`, `table2_codesign`, `reproducibility`), with
  its own `README.md`. This is the reproducibility anchor.
- **`ros2_ws/results/`** — now just a **scratch** dir for new runs (git-ignored contents).
- **`ros2_ws/scripts/`** — figure/table scripts point at `paper_data/`:
  `make_testbed_figure.py` (Fig. 1), `plot_envelope.py` (Fig. 3, rewritten — see caveat),
  `plot_altitude_codesign.py` (Fig. 4), `fill_paper_numbers.py` (Table II, rewritten).
  Experiment/utility scripts: `run_compare.sh` (co-design), `envelope_run_cell.sh` (envelope sweep),
  `find_spawn.py`, `make_tracks.py`, `capture_camera_frame.py`. `numsim_tradeoff.py` is a **theory
  validator** (Props A/B/C) that produces figures **not used** in the paper — candidate for removal.
- **Reproducibility status:** Fig. 1 and Fig. 4 regenerate **byte-identically** from `paper_data/`;
  Table II reproduces **exactly**. **Fig. 3 caveat:** the canonical PNG is the committed one; its
  original one-off plotting script was lost, so `plot_envelope.py` is a *faithful reconstruction* —
  right panel exact, left panel (std(ω)) matches the same curves/trend but the exact per-run std
  window (`--window`, default full-run) is not certain; sharp-track z=2 differs slightly (0.82 vs the
  figure's ~0.73). Treat the committed PNG as authoritative.
- **State is uncommitted** (data moved via `git mv`; scripts/README modified). Nothing pushed.
  Governing constraints that persist: **honesty is the constraint; change the paper to match the code,
  not vice-versa; no "claude" in git commit messages; ask before committing/pushing.**
- **Frozen checkpoints** exist as sibling folders under `/home/omen/CRAZYFLY_SITL/`
  (`ugv_uav_control__CHECKPOINT_2026-07-15`, `...__CHECKPOINT2_honest-reframe_2026-07-15`,
  `...__honest_reframe`) — do NOT push from them.

---

## 6. What the next phase is meant to do

Keep the experimental work essentially as-is, but **re-scope the paper toward a stronger, more
theoretically robust contribution** (the SITL-only framing is judged too weak for the venue). Likely
directions to explore (to be decided with the user): formalize the coverage–resolution altitude
coupling as an optimization/observability result; give the online altitude policy a real guarantee or
a cleaner optimality argument; or reframe the event-trigger co-design with tighter theory. Minor
experimental tweaks are acceptable; wholesale re-runs should be avoided unless a claim requires it
(and must be headless).

**First things a new agent should read:** this file, then `ros2_ws/paper_data/README.md`, the current
`AjS_ICC12-2.pdf`, and `README.md`. The auto-memory under
`~/.claude/projects/.../memory/` (`icc12-claude`, `project-tempc-iccc-paper`,
`audit-tempc-code-vs-paper-gaps`, `finding-uav-cmdvel-topic-collision`) has finer-grained history.
