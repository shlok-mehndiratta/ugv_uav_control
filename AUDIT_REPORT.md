# TEMPC (UAV+UGV) — Fidelity, Defensibility & Validation Audit
**Target venue:** ICC-12 (2026). **Scope:** does the code implement the paper's 3 claims; are the current results defensible; how to design the validation study. Phases 1–3 only (stop for approval before building).

Paper read: `ICC12-1.pdf` (an **unfinished draft** — empty Related Work/Problem Formulation, template residue, `[TODO]`s, **no results/figures**). Prior concept review: `TEMPC_review_ICC2026.pdf` (Reject/major-revision, theory-level). This audit is the missing **code↔paper** layer.

---

## Phase 1 — Fidelity Audit (code vs. paper)

Status legend: **✅ implemented & faithful** · **⚠️ implemented but diverges from paper / not exercised** · **❌ claimed but not implemented online**.

| # | Paper claim / Eq | Code location | Status | Note |
|---|---|---|---|---|
| Eq 1 | UGV unicycle + actuator lag | `ugv_model.py:9` (`process_model`), `:38` (`compute_jacobian_F`) | ✅ | Trapezoidal integration, first-order lag τ_v,τ_w. Faithful. |
| Eq 2 | View-quality `q_k` | `view_quality.py:18-76` | ⚠️→✅ | Code is **more correct than the paper**: adds altitude Gaussian `s_z=exp(-(z-z_opt)²/2σ²)` (`:74`), so `q` does **not** →1 as z→∞. Directly fixes reviewer concern #6. Paper Eq 2 should be updated to match code. |
| Eq 3 | `R_k(q_k)=R0+Rmax(1-q)` | `ugv_model.py:71-90` (`get_dynamic_R`) | ⚠️ | Implemented as `R_base/(q+1e-3)`, **not** `R0+Rmax(1-q)`. Same qualitative behavior (R↑ as q↓) but a different functional form than the equation. Reconcile paper↔code. |
| Eq 4 | Belief-space NMPC `J_g` with **tr(Σ) propagated** + FoV penalty | `belief_mpc_node.py:89-170` | ✅ (mechanism) / ⚠️ (dormant) | **Σ IS propagated through the horizon**: `Σ_pred=FΣFᵀ+Q` (`:107`), FoV-blended EKF update (`:120-137`), `trace(Σ)` added each step (`:153,165`), asymptotic-ish FoV penalty (`:156-166`). This is a genuine belief-space NMPC (SLSQP, N=5). **Caveat:** it uses a **static** UAV pose over the whole horizon (`self.uav_pos`, `:113-117`) — no prediction of UAV future motion, so the "bidirectional" coupling is one-step in practice. Never stressed in the runs (see Phase 2). |
| Eq 5 | Aerial nominal planner `J_a`, "Q_a scales with predicted UGV maneuvers" | `uav_mpc_node.py:92-116` | ⚠️ | Planner exists, but **Q_a is a fixed weight** (`w_alt=50`, `Q_vec` fixed) — it does **not** scale with predicted maneuvers or covariance. Uses a hand-rolled `lat_dist²/z²` visibility heuristic (`:110`), and **imports but never calls** `view_quality`. The Eq-5 claim ("naturally trades off altitude") is not what the code does. |
| Eq 6 | `α_k = max(1, β·tr(Σ_xy))` re-scaling the tube | `uav_ancillary_node.py:195-196` | ⚠️ | Different formula: linear interp `α∈[α_min,α_max]=[0.5,2.0]` over `tr∈[tr_min,tr_max]`. So **α can be <1** (0.5), contradicting `max(1,·)`. More importantly, α only resizes the **trigger boundary** (`:199`); see Eqs 7–13. |
| Eqs 7–10 | Offline mRPI via truncated Minkowski series + `1/(1-ρ^{N+1})` scaling | `tube_calculation.py:149-234` | ✅ (offline) / ❌ (online) | The **offline script is correct**: support-function mRPI, N=100 truncation, `1/(1-ρ^{N+1})` compensation (`:227`), ρ computed (`:199`). **BUT the online controller never uses it.** |
| — | `w_eff = w_a − BKv_k` used; Gaussian bounded before mRPI | `tube_calculation.py:108-114` | ⚠️ | Offline, `w_eff` is built correctly and `v_k` is **truncated to a box** `[±0.05,±0.05,±0.1]` (`:105`). But the box is arbitrary — **no probability level stated, no link to the live EKF Σ**. Online, the unbounded Gaussian Σ never enters any bounded set. Confirms reviewer concern #2. |
| Eqs 11–13 | Polyhedral Ω, `X̄=X⊖Ω`, `Ū=U⊖KΩ` | `tube_calculation.py:236-272` | ✅ (offline) / ❌ (online) | Pontryagin tightening implemented offline and saved to `tube_data.npz`. **Never loaded by any online node.** |
| — | **Offline tube actually loaded online?** | `uav_ancillary_node.py:58-69` | ❌ **BUG** | Node tries to load `K,P_0` from a **hardcoded absolute path on another machine** (`/home/shlok-mehndiratta/.gemini/...`), then lines `68-69` (at method indentation) **unconditionally overwrite** them with `K=diag(.6,.6,.8)`, `P_0=0.186·I`. The rigorous mRPI is decorative; the online "tube" is a **hand-picked ellipsoid**. |
| Eqs 14–15 | `x=x̄+e`, `u=ū−Ke` | `uav_ancillary_node.py:192,202` | ✅ | `e_k=uav_pos−x̄`, `u_anc=u_bar−K e_k` (clipped). Faithful in structure. |
| Eq 16/17 | **σ-contracted trigger** `max(HΩe−σhΩ)>0`, set `Ω_σ` | `uav_ancillary_node.py:199,215` | ❌ | **Not implemented.** Trigger is ellipsoidal tube-exit `eᵀP_0e ≤ α_k²` — **no σ, no HΩ polyhedron, no Ω_σ**. σ is not a parameter anywhere in the online stack, so the paper's headline tuning knob (and its Pareto curve) does not exist in code. |
| Eqs 18–19 | QP with tightened `X̄,Ū`, terminal set `X_f`, DARE terminal cost `P` | `uav_mpc_node.py:92-155` | ❌ | Online planner is an SLSQP with **only a z-box constraint** (`:143-145`). **No tightened constraints, no terminal set X_f, no DARE terminal cost** used online (P is computed offline, unused). The "recursive feasibility / stability" apparatus is **absent from the running controller**. |
| Eq 16 | Trigger gates QP; solves counted | `uav_mpc_node.py:88-90,159`; `experiment_logger.py:301` | ✅ | Trigger→`solve_mpc()`; `total_solves`↑ per solve. Gating works; counting is correct (logger counts trigger publishes at 10 Hz vs ancillary 20 Hz — minor rate mismatch, not fatal). **This is the one contribution the code genuinely delivers.** |

### Answers to the five required verifications
1. **Is tr(Σ) propagated (true belief NMPC) or frozen?** → **Propagated** (`belief_mpc_node.py:105-137`). Real belief-space NMPC. But driven by a **static** UAV pose over the horizon, and never stressed by the scenario.
2. **Is Ω computed offline via truncated Minkowski + `1/(1-ρ^{N+1})`?** → **Yes, offline, correctly** (`tube_calculation.py`). **But it is never used online** (overwritten by a hardcoded ellipsoid).
3. **Does `α_k` re-scale the tube, and is `X⊖α_kΩ` / invariance re-checked?** → α rescales **only the trigger ellipsoid**, not any nominal constraint. **No re-tightening, no invariance re-check.** Exactly reviewer concern #1 — confirmed in code. (And the paper's "α expands → prevents infeasibility" is backwards: a larger tube ⇒ larger `X⊖αΩ` ⇒ *smaller* feasible set.)
4. **Is `w_eff=w_a−BKv_k` used; is the Gaussian bounded before the mRPI?** → Offline yes, with an **arbitrary box truncation** of `v_k` (no probability level, no tie to live Σ). Online: **not used at all**; unbounded Gaussian Σ never bounded.
5. **Does Eq-16 (σ-set) gate the QP; solves counted correctly?** → The **Eq-16 σ-mechanism is not implemented**; a **different** ellipsoidal tube-exit + heuristics gates the QP. Solve counting itself is correct.

**Phase-1 bottom line:** C1 (belief NMPC) is genuinely implemented but dormant. C3 (event-trigger reduces solves) is real. **C2 (adaptive robust tube) is not the paper's method**: the online tube is a hand-tuned ellipsoid, the offline mRPI is dead code, α doesn't touch constraint tightening, and Eq-16/σ, terminal set, and DARE terminal cost are absent online.

---

## Phase 2 — Defensibility Audit (why the plots look the same)

**What's plotted** (`plot_final_figures.py`): Fig 1 spatial XY overlay (**one** run, C3, with a `scipy.optimize` **rotation fit of the UAV track onto the UGV track** — cosmetic alignment, `:95-105`); Fig 2 time-series of `|target_y|`, `tr(Σ)`, trigger raster; Fig 3 violins of error & jerk; Table 1 summary. **`BASE_DIR` is hardcoded to `/home/shlok-mehndiratta/...`** — the plotting/`reproduce` path does not run on this machine.

**Seeds / scenarios:** **N = 1 run per condition** (`get_latest_csv`), single benign track (`track2.png`), ~110 s each. No Monte Carlo.

**Are the compared runs actually different?** Barely, and inconsistently wired:

| Metric (from summaries) | C1 Continuous | C2 Reactive | C3 Proposed |
|---|---|---|---|
| Mean tracking error (m) | 0.0568 | 0.0595 | 0.0591 |
| Max tracking error (m) | 0.600 | 0.599 | 0.598 |
| Mean tr(Σ) | 0.0070 | 0.0101 | 0.0069 |
| **FoV exit %** | **0.0** | **0.0** | **0.0** |
| ArUco visible % | 99.9 | 99.2 | 99.9 |
| **UAV z (min/mean/max)** | **6.0 / 6.0 / 6.0** | **6.0 / 6.0 / 6.0** | **6.0 / 6.0 / 6.0** |
| Replans | 1116 | 107 | 102 |
| Mean solve (UGV/UAV ms) | 41 / 12 | **0 / 0** | 37 / 15 |

- **The scenario is benign:** `fov_exit_pct=0` and ArUco≈99.9% for all three → no FoV risk, no occlusion (`block_aruco` was off). Every mechanism that's supposed to *react to lost observability* is never provoked.
- **Adaptive altitude never moves** (z≡6.0 in all three; and in C1 it's frozen by construction — `uav_mpc_node.py:130` sets `vz=0` when `use_timer`). C2's entire selling point produces **zero** observable effect.
- **Tracking error, max error, tr(Σ) are statistically indistinguishable** across methods → Figs 1 & 3 *must* overlap. That's the user's complaint, and it's structural, not a plotting bug.
- **The runs aren't a clean controlled A/B:** C2 logged `0/0` solve times and an empty trigger-reason breakdown, i.e. it was wired differently (non-belief UGV / different topics) from C1 and C3. Not a paired comparison.
- **The only real signal:** event-triggering cuts replans ~10× (1116→~105) at equal tracking error. Legitimate and worth reporting — but it's C3 only.

**README vs. reality:** the README "Result Verification Matrix" (Baseline 1.037→**0.463 m**, ArUco 46.3→**87.0 %**, solve 50→**16 ms**, "-55.3%") **matches none of the logged CSVs** (which show ~0.06 m error and ~99.9 % ArUco for *all* conditions). Those headline numbers are unsupported by any data in the repo and read as fabricated/aspirational. This must not go near a submission.

### Are the current results defensible? **No.**
Precisely why: (1) **n=1**, no Monte Carlo, no statistics; (2) a **benign scenario** that never triggers FoV loss, occlusion, or altitude adaptation, so C1 and C2 are provably inert (z≡6, FoV-exit≡0); (3) **C1/C2/C3 are not a paired, identically-wired comparison**; (4) the reproduction path is **hardcoded to another machine**; (5) the headline metrics in the README are **not backed by the logged data**; (6) the plotted "improvement" is confined to replan count, while error/tr(Σ)/visibility are identical — so the figures actively hide, rather than expose, the contributions. Plus the Phase-1 gaps mean C2 as run is not the paper's method.

---

## Phase 3 — Validation Design (proposal; nothing built yet)

Goal: force each contribution to show (or fail to show) its effect, with isolation + paired statistics. Reuse existing knobs where possible; new knobs flagged **[NEW]**.

### (a) Baseline + Ablation matrix
UGV axis (perception): `passive` = `mpc_node`/`stanley`; `active` = `belief_mpc`. UAV axis (tracking) and Trigger axis as below.

| ID | UGV (perception) | UAV tube | Trigger | How to run | Isolates |
|----|----|----|----|----|----|
| **B0** | passive (`mpc`) | — (fixed hover / preplanned orbit **[NEW passive-UAV mode]**) | n/a | new `uav_passive` node | decoupled strawman (intro target) |
| **B1** | passive (`mpc`) | adaptive | perception-driven | `controller:=mpc` | one-directional (smart UAV, dumb UGV) |
| **B2** | active (`belief_mpc`) | adaptive | perception-driven | `controller:=belief_mpc` | full bidirectional (proposed) |
| T-fix-wc | active | **fixed worst-case** (α≡α_max) **[NEW freeze-α]** | perception-driven | param | conservatism cost of static tube |
| T-fix-nom | active | **fixed nominal** (α≡1) **[NEW]** | perception-driven | param | fragility of un-adapted tube (violations) |
| T-adapt | active | adaptive | perception-driven | = B2 | value of adaptation |
| G-cont | active | adaptive | **continuous** | `use_timer:=true` | compute ceiling |
| G-periodic | active | adaptive | **periodic (fixed rate)** **[NEW]** | param | naive baseline |
| G-error | active | adaptive | **error-only** (Level-1a only; disable 1b/2/3) **[NEW flag]** | param | value of perception terms |
| G-percep | active | adaptive | **perception-driven** (all levels) | = B2 | proposed trigger |

**Prereqs to make the matrix meaningful:** (i) load the **offline mRPI** online and delete the hardcode (`uav_ancillary_node.py:68-69`); (ii) implement the **Eq-16 σ-trigger** + expose σ; (iii) either re-derive the adaptive tube as **homothetic/parameterized tube MPC** with online `X⊖α_kΩ` re-tightening + a feasibility check, **or** explicitly downscope C2's claim to "trigger-boundary scaling" and report feasibility empirically. Without (iii), C2 remains a claim the code cannot support.

### (b) Adversarial scenarios (each stresses a specific mechanism)
| ID | Scenario | Mechanism it exists to break | Hook |
|----|----|----|----|
| S0 | benign lane (current) | sanity / equivalence check | `track2.png` |
| S1 | **FoV-tempting**: lateral excursions approaching `z·tan(θ/2)`; or lower `z_opt` to shrink FoV | C1 FoV penalty, %time-in-FoV | new track + `z_opt` |
| S2 | **Injected occlusion**: `block_aruco` windows (25–30 s, + repeats) | C1 tr(Σ) penalty, C2 α-expansion, C3 covariance-spike trigger | `block_aruco:=true` (**mechanism already exists**, unused) |
| S3 | **Hairpin / high-κ** track (κ>κ_thresh) | C1 observability maneuver, C2 tube under aggressive motion, C3 curvature trigger | new hairpin track image |
| S4 | **Intermittent sensor dropout** (periodic ArUco loss) | recovery time, feasibility retention | extend `block_aruco` to periodic **[NEW]** |

### (c) Claim → metric → plot (signature metrics)
| Claim | Metrics | Headline plot | Scenario × configs |
|----|----|----|----|
| **C1** info-aware UGV | %time-in-FoV, peak/mean tr(Σ), target-loss events | **Catastrophic decoupled failure vs bounded coupled estimate**: tr(Σ) blow-up + target loss for B0/B1 vs bounded for B2 | S2, S3 × {B0,B1,B2} |
| **C2** adaptive tube | α_k(t), feasibility-retention %, constraint-violation rate, post-occlusion recovery time | violations(fixed-nominal) vs conservatism/error(fixed-worst) vs adaptive; α_k(t) overlaid on occlusion windows | S2, S3 × {T-fix-nom, T-fix-wc, T-adapt} |
| **C3** perception trigger | #QP solves, solve-time p50/p95/p99, tracking RMSE | **σ Pareto: replan frequency vs RMSE**; solve-count bars; trigger raster | S3 × {G-cont, G-periodic, G-error, G-percep}, σ∈{0.3…0.95} |

### (d) Statistical protocol
- **≥30 Monte-Carlo seeds**, **paired**: same seed ⇒ identical process/measurement-noise + occlusion-timing + spawn realization across all configs. Seed the RNG explicitly in EKF/sim (currently unseeded).
- Report **mean±std + box/violin** per metric; **paired significance test** (Wilcoxon signed-rank, or paired-t if normal) on the headline claims (tr(Σ), %FoV, #solves, RMSE); report effect sizes, not just p.
- **Fixed budgets documented per config:** horizon (UGV N=5, UAV N=10), solver (SLSQP maxiter/ftol), dt, all weights, α/σ/τ thresholds — one table, equal tuning budget, no per-method tuning.
- **Compute realism:** 30 seeds × ~10 configs × 5 scenarios in Gazebo real-time is ~heavy. Recommend a **headless numeric sim** (extend `01/.../sim_*.py`) for the MC statistics, and keep Gazebo for 2–3 showcase runs. Report both.

### (e) Priorities
**Must-have (defensibility floor):**
1. Fix README fabricated table + hardcoded plotting/`reproduce` paths.
2. Use the offline mRPI online (kill the `uav_ancillary_node.py:68-69` override); implement Eq-16 σ-trigger; expose σ.
3. Resolve C2: homothetic tube with online re-tightening + feasibility check, **or** honestly rescope the claim.
4. Run S2 (occlusion) + S3 (hairpin) — these are what make C1/C2 visible; both largely reuse existing hooks.
5. ≥30 paired seeds with signature metrics; the **catastrophic-decoupled vs bounded** headline figure; the **σ Pareto**.
6. Identically-wired B0/B1/B2 (fix the C2-run wiring inconsistency).

**Nice-to-have (best-paper polish):**
- Hardware/Crazyflie demo (ICC prizes this).
- Coupled-loop non-chattering / fixed-point argument (reviewer #3); homothetic-tube recursive-feasibility proof + Lemma-1 proof (reviewer #5).
- Real ArUco GSD/resolution model at altitude (partly already in `view_quality`); document R_k form vs Eq 3.
- S1/S4 scenarios; finer σ sweep; cross-track RMSE vs arc-length.

---
**Recommendation:** the event-trigger compute saving (C3) is real and defensible on its own; C1 is implemented but needs adversarial scenarios to demonstrate; **C2 as coded is not the paper's method and is the critical-path item.** Do not submit current results. I've stopped here per instructions — approve/adjust the Phase-3 plan and I'll build the harness (Phase 4).
