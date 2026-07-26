# ACODS paper — detailed presentation plan (evidence-locked)

**Purpose.** How we present this paper so it is theoretically robust, rigorously proved, and
simulation-verified — with every claim backed by measured data or a citable theorem, nothing vague,
nothing naive, and nothing weakened in the name of honesty. Supersedes the strategic sketch in
`REFRAME_PLAN.md`. Written 2026-07-25 after the E0 + Pillar-2 measurement campaign.

**Governing principle (unchanged):** honesty is a constraint, but honesty means *scoping claims to what
we can prove/measure*, not hedging strong results into mush. We make the strongest claims the evidence
supports, and we retire the one claim the evidence does not (the fragile peak-covariance spike) so a
reviewer cannot break the paper.

---

## 1. Thesis and contributions

**Thesis.** In a shared-single-camera aerial–ground system, the UAV's altitude is the sole
ground-sampling-distance of the only exteroceptive sensor, and that one scalar simultaneously sets
(i) the *resolution* of marker-based localization and (ii) the *footprint* available for lane
navigation. Altitude is therefore an **active-perception control variable** that trades estimation
resolution against navigation coverage. The trade-off admits a **closed-form optimum — the coverage
floor** — and an altitude policy tracking it dominates any fixed altitude. Both sides of the coupling
are now *measured*, not assumed.

**Contributions (ordered; the idea leads, theory + measured data back it):**

- **C1 — Optimal-altitude theory for shared-camera tracking (primary).** We formalize the
  coverage–resolution coupling as a constrained steady-state estimation problem and prove the optimal
  altitude is the coverage floor `z* = d_look(κ)/tan(φ/2)` (Prop A), rising with curvature (Cor. 1).
  The measurement model underlying it, `R(z) ∝ z²`, is **validated by direct measurement** from the
  rendered camera (E0: measured localization error ∝ z, r = 0.996).

- **C2 — Certifiable event-triggered realization (secondary).** A perception-driven event trigger
  makes the expensive aerial planner affordable, with a proved bound on the inter-trigger tracking
  error and no-Zeno-by-construction (Prop E), and a measured ≈25× fewer solves / ≈12× less CPU at
  matched estimation quality. (Event-triggered MPC is prior art — we claim the *perception-driven
  coupling and its certified realization*, not the technique.)

- **C3 — Reproducible SITL validation (support).** A ROS 2 / Gazebo testbed with a shared vision/ArUco
  front end, view-quality EKF, belief-space UGV MPC, and UAV planner/ancillary, under a common logging
  protocol; all headline numbers reproduced across repetitions with tight error bars.

---

## 2. Theoretical spine (this is the "robust theory" — full statements + proofs + citations)

Two propositions + one corollary. Appropriately scoped for a 6-page IFAC paper; each is provable, none
overreaches. **The honesty pivot must be stated correctly** (see 2.0) — it is what makes Prop A rigorous
rather than hand-wavy.

### 2.0 The pivot: the objective is monotone, so the optimum is a constraint boundary
Substituting `r_fov = z·tan(φ/2)` into the GSD model collapses it to a manifestly monotone form:
```
R(z) = R0 (z/z0)² (1 + (d/r_fov)²) = (R0/z0²)·( z² + d²/tan²(φ/2) )   →   dR/dz = 2R0 z/z0² > 0.
```
`R(z)` is **strictly increasing in z**, with no interior minimum. Hence `z*` is **not** a stationary
point (`∂trΣ/∂z ≠ 0`); it is the **coverage-floor boundary optimum**. State Prop A as a *constrained
monotone optimization*, never as first-order stationarity. (Neatly, the FoV-edge term becomes a
z-independent constant at fixed d, so it does not disturb monotonicity.) **This is corroborated by E0**:
measured error grows ∝ z with r = 0.996, i.e. the estimator strictly prefers lower altitude — it is the
navigation constraint that stops the descent.

### 2.1 Proposition A — optimal altitude = coverage floor  (backs C1)
*Assumptions:* frozen-parameter linear-Gaussian target filter (constant z, offset d, process noise Q);
`(F, Q^{1/2})` stabilizable, `(F, H)` detectable (⇒ unique stabilizing PSD DARE solution `Σ_ss(R)`);
isotropic per-axis position measurement with variance `R(z)` as in 2.0; coverage feasibility
`η(z) = z·tan(φ/2)/d_look(κ) ≥ 1`.
*Statement:* (i) `z ↦ tr Σ_ss(R(z))` is nondecreasing on the feasible set; (ii) the constrained problem
`min_z tr Σ_ss(R(z)) s.t. η(z)≥1, z∈[z_min,z_max]` attains its minimum at `z* = d_look(κ)/tan(φ/2)`,
the smallest feasible altitude; (iii) `z*` is nondecreasing in curvature κ.
*Proof (self-contained, ≈½ column):* scalar per-axis DARE `σ_ss(r)` is an explicit strictly-increasing
function of the measurement variance `r` (one-line calculus); isotropy ⇒ `tr Σ_ss = Σ_axes σ_ss`;
composition with increasing `R(z)` gives (i). `η(z)≥1 ⟺ z ≥ z_floor := d_look(κ)/tan(φ/2)`; minimizing a
nondecreasing function on `[z_floor, z_max]` gives the minimizer `z_floor` (monotone KKT: multiplier
`λ = f'(z*)/tan(φ/2) ≥ 0`, complementary slackness active). Needs monotonicity only — **no convexity**.
`d_look` increasing in κ ⇒ (iii).
*Citations:* DARE/Riccati monotonicity in R — Wimmer (Syst. Control Lett., 1985); Anderson & Moore,
*Optimal Filtering* (1979); Lancaster & Rodman (1995). Active-perception framing (Fisher-info gloss,
one sentence): per-axis info `I(z)=1/R(z)` decreasing in z — Bishop et al. (Automatica, 2010).
*Empirical anchor:* E0 (Fig. 3a) — measured `R(z) ∝ z²` confirmed (r = 0.996).

### 2.2 Corollary 1 — "climb into turns" is optimal, not heuristic
Since `z* = d_look(κ)/tan(φ/2)` and `d_look` increases with κ, the optimal altitude *rises with path
curvature*. The deployed policy `z_k = d̂/tan(φ/2) + β_κ κ̂ + z_m` is the coverage-floor optimum plus an
explicit robustness/preview margin `(β_κ κ̂ + z_m)`. **We claim optimality of the floor term only**; the
margin is an engineering add-on (state this — do not claim the margin inherits optimality).

### 2.3 Proposition E — inter-trigger boundedness + no Zeno  (backs C2)
*Setup:* ancillary error map `e_{k+1} = A_K e_k + w_k`, `A_K = diag(0.97,0.97,0.96)`, `ρ := ‖A_K‖₂ =
0.97 < 1`, `‖w_k‖ ≤ w̄`.
*Statement:* (i) `‖e_k‖ ≤ ρ^k‖e_0‖ + w̄/(1−ρ)` (ISS w.r.t. w); (ii) ultimate bound `limsup‖e_k‖ ≤
w̄/(1−ρ)`, so the error enters and remains in the monitor ellipsoid `E(α)` for any `α ≥
(w̄/(1−ρ))√λ_max(P0)` — certifying ellipsoid membership as a *sound* trigger; (iii) inter-event time ≥
the 20 Hz sampling period (and ≥ 1 s for lockout-guarded conditions), so **Zeno is excluded by
construction**.
*Proof:* `‖e_{k+1}‖ ≤ ρ‖e_k‖ + w̄`; iterate geometric series; (iii) by periodic + time-lockout design.
*Scope (state explicitly):* covers the **linear ancillary error subsystem only**, not the full coupled
loop.
*Citations:* discrete-time ISS — Jiang & Wang (Automatica, 2001); periodic event-triggered / MIET ≥
sampling — Heemels, Donkers & Teel (IEEE TAC, 2013); event-triggering — Tabuada (2007),
Heemels–Johansson–Tabuada (2012); ultimate-bound ⊆ mRPI tie to the tube baseline — Mayne et al. (2005),
Raković et al. (2005).

---

## 3. Measured evidence (every claim → data → number)

All headline numbers reproduced with reps (headless, `use_gsd_noise` on for closed-loop runs; E0 uses
raw sensor, no injection). Track/label consistency to be fixed (see §8).

| Claim (paper) | Evidence | Measured result |
|---|---|---|
| `R(z) ∝ z²` (model premise of Prop A) | **E0** raw ArUco error vs z, static hover, d≈0, 6 altitudes, ~150 samples each | error ∝ z, `err_mm ≈ 2.20·z − 0.85`, **r = 0.996**; median 3.7 mm (2 m) → 15.0 mm (7 m). Fig. 3a. |
| Estimation improves at low altitude (Jaw 1) | closed-loop hairpin, 3 reps, z=2.5 vs 7 | meanCov **0.0064 ± 0.0004** (low) vs **0.0108 ± 0.0001** (high); ~1.7× |
| Navigation cost rises at low altitude (Jaw 2) | same runs, steering effort (control_smoothness) | **0.285 ± 0.007** (low) vs **0.044 ± 0.001** (high); **~6.5×**, reproducible |
| Event-trigger co-design | Table II runs, 3 reps (C1 vs C2) | solves **370→32** (all reps), CPU ≈12×, matched steady-state trΣ ≈0.0095, ArUco ≈99.8% |
| Inter-trigger error bounded, no Zeno | Prop E + logged e_k / inter-event times | ρ=0.97 contraction; ultimate bound; MIET ≥ 0.05 s (opt. Fig. panel) |

**DROPPED (fragile, non-reproducible):** the "peak-covariance spike / target leaves footprint / 9×"
headline (old Fig. 4). Reps: maxCov at z=2.5 was 0.042, 0.042, **0.397** — 1 of 3; non-monotone in speed
(appeared at V=1.0, absent at 0.8 and 1.2); visibility stayed ≥ 99.2%. A reviewer re-running would not
see it. We retire it and rest dominance on the reproducible jaws.

---

## 4. Claim discipline (print this list; check the manuscript against it)

**We DO claim (rigorous + backed):**
- Coverage-floor optimality of altitude (Prop A) with measured `R∝z²` (E0).
- Optimal altitude rises with curvature (Cor. 1).
- A reproducible coverage–resolution trade-off: low altitude → ~1.7× better estimation, ~6.5× more
  steering effort (with error bars).
- Adaptive altitude attains near low-altitude estimation quality *and* near high-altitude navigation
  smoothness (best-of-both, shown on the reproducible metrics).
- Event-trigger: bounded inter-trigger error + no Zeno (Prop E); measured ≈25×/≈12× compute at matched
  estimation quality.

**We DO NOT claim (overclaim guard):**
1. Interior/stationary altitude optimum (it is a boundary optimum).
2. Prop B (active beats *any* fixed on integrated cost) as a theorem — empirical, or a corollary under
   an explicit quasi-static assumption only.
3. Anticipatory > reactive climb (Prop C) as a guarantee — z lags κ in SITL; remark only.
4. Separation principle / certainty-equivalence / LQG optimality of the joint scheme (R depends on the
   control z ⇒ dual/active sensing).
5. CRLB/Fisher bound *achieved* (EKF not efficient); EKF minimum-variance optimality.
6. Continuous-time Zeno-freeness / hard positive MIET beyond "≥ sampling period."
7. ISS/boundedness of the *full coupled* loop (Prop E is the linear ancillary subsystem only).
8. Recursive feasibility / robust invariance of the online controller (keep the tube-*inspired*
   surrogate concession).
9. Convexity of `tr Σ_ss(z)` (not needed — monotonicity suffices).
10. Optimality of the anticipatory margin `β_κ κ̂ + z_m`.

**We DROP:** peak-covariance-spike / coverage-loss headline (see §3).

---

## 5. Related work & positioning (three-way daylight; not a list)

Must-cite-and-distinguish (one sentence of daylight each):
- **ColAG** (Li et al., ICRA 2024) — nearest *system* (perceptive UAV for blind UGVs); daylight:
  *separate* sensors, altitude *passive*, no coupling.
- **Stache et al.** (ECMR 2021 / RAS 2023) — nearest on *altitude as a resolution knob under coverage*;
  daylight: mapping/segmentation, no closed-loop estimator, no navigation-lookahead constraint, no
  shared dual-role sensor, no closed-form optimum.
- **Zhang & Scaramuzza (ICRA 2019) + Falanga PAMPC (IROS 2018)** — the active-perception "theory
  currency"; daylight: single-robot self-localization, FoV as *hard constraint* not a continuous
  GSD–coverage trade-off, no reduction to one altitude scalar, no cooperative navigation role.
Belief-space cluster (Platt; van den Berg; Prentice) — the family our UGV MPC belongs to (no guarantee
claimed). Theory cites: Wimmer 1985 / Anderson–Moore 1979 (Riccati monotonicity), Bishop 2010 (FIM),
Jiang–Wang 2001 (discrete ISS), Heemels–Donkers–Teel 2013 (periodic ETC), Tabuada 2007, Mayne 2005 /
Raković 2005 (tube/mRPI). **To do:** a proper 1–2 h literature pass to (a) confirm every citation's exact
venue/year, (b) surface any 2024–2025 shared-sensor / altitude-active-perception work we must address.

---

## 6. Paper structure (6-page `ifacconf`, section by section)

1. **Introduction** — perception-limited UGV → aerial eye → *shared single camera* → the coupling →
   altitude as one active-perception scalar → coverage-floor optimum → contributions (C1 leads).
2. **Related work** — the three-way positioning of §5.
3. **System & models** — UGV unicycle; UAV single-integrator; **GSD model with the collapse to
   `R=(R0/z0²)(z²+const)` made explicit** (sets up Prop A); coverage factor η; the shared dual role.
4. **Optimal altitude for shared-camera tracking (C1)** — constrained problem; **Prop A + proof**;
   **Cor. 1** (climb into turns); the altitude policy as the floor + margin (honest split); Fisher-info
   framing sentence.
5. **Event-triggered aerial planning (C2)** — belief-space UGV MPC (continuous); aerial planner +
   ancillary; the perception-driven trigger; **Prop E + proof**; the tube-*inspired* surrogate
   concession.
6. **Simulation study (C3)** — testbed + parameter table; **Fig. 3** (measured two jaws); **Fig. 4**
   (adaptive vs fixed on reproducible metrics); **Table II** (co-design). All with reps/error bars.
7. **Conclusion + limitations** — velocity-controlled abstraction, single-camera coupling, reactive
   timing, design-time tube; future = hardware/full multicopter, perception-specific-trigger ablation.

---

## 7. Figures & tables (rebuilt around measured data)

- **Fig. 1** Testbed — single camera, dual role (existing `Fig_Testbed1/2`).
- **Fig. 2** System overview — TikZ (existing).
- **Fig. 3 (NEW, the theory's evidence).** (a) **E0**: measured ArUco error vs altitude, linear fit,
  r = 0.996 → "R ∝ z²". (b) the two jaws:
  meanCov ↓ and steering-effort ↑ vs altitude, **with error bars from reps**. This is the paper's
  empirical backbone for Prop A + the trade-off.
- **Fig. 4 (REVISED).** Adaptive vs fixed altitude, dominance shown on **reproducible** meanCov +
  steering effort (bar plots with error bars). **Remove** the log-scale peak-covariance panel.
- **Fig. 5 (optional).** Prop E: `‖e_k‖` with ultimate-bound envelope + inter-event-time histogram.
- **Table I** Parameters (existing, verify against code).
- **Table II** Event-trigger co-design (existing, reproducible).

---

## 8. Remaining work (prioritized)

**A. Data (headless, reps, error bars):**
1. **Fig. 3b two-jaw sweep with reps** — altitude z ∈ {2,3,4,5,6,7} at a fixed elevated speed on one
   representative track (hairpin or slalom), N=3 reps, log meanCov + steering effort → error-barred
   curves.
2. **Fig. 4 revision** — re-plot adaptive vs fixed from existing `fig4_altitude` data using
   meanCov/steering, add reps if needed; drop peak panel.
3. **(Optional, strengthens C2) perception-trigger ablation** — `error_only:=true` "B" condition vs full
   C2, to show the perception-specific triggers add value beyond a plain error trigger.

**B. Theory (write + verify):**
4. Write Prop A proof (scalar DARE monotonicity) + Cor. 1; Prop E proof (geometric series). Verify the
   scalar-DARE monotonicity statement and cite precisely.
5. Estimate `w̄` from logs for the Prop E ultimate bound (optional Fig. 5).

**C. Literature:**
6. 1–2 h citation-verification + 2024–25 gap check (§5).

**D. Manuscript hygiene:**
7. **Fix track-label inconsistency** — "sharp" is overloaded (envelope=track4, Fig.4=track_hairpin,
   co-design=track2). Map the Gentle/Slalom/Sharp names to the actual data folders and make Table II /
   figure captions consistent.
8. Port everything into `New Paper/ACODS.tex` (ifacconf); keep the honesty concessions already in the
   draft (tube-inspired surrogate).

**E. Keep the reproducibility fix:** the headless `MPF_SHOW_WINDOW` guard in `middle_path_follower.py`
(committed to src, rebuilt `--symlink-install`) — needed for clean batch runs.

---

## 9. Venue specifics (ACODS)
`ifacconf` document class; IFAC-PapersOnLine; ~6 pages. Applications-oriented but theory-respecting —
the 2-proposition + reproducible-simulation shape is exactly the expected profile. Keywords:
perception-aware control, active perception, event-triggered MPC, cooperative aerial–ground robotics,
state estimation, optimal altitude. (Confirm current deadline/page limit with organizers — user says
time remains.)

---

## 10. Open decisions for the user
1. **Which track for the Fig. 3b two-jaw sweep** — hairpin (sharpest, biggest jaw contrast) vs slalom
   (moderate, cleaner). Recommend hairpin for contrast + one gentle track to show track-independence of
   Jaw 1.
2. **Include the perception-trigger ablation (item A3)?** — strengthens C2 to a real contribution vs a
   comparison; costs a few more runs. Recommend yes if time permits.
3. **Title.** Recommend foregrounding the idea + theory, e.g. *"Altitude as an Active-Perception
   Variable: Optimal Coverage–Resolution Trade-off in Shared-Camera Aerial–Ground Tracking."*
4. **Fig. 5 (Prop E illustration)** — include, or keep the 6-page budget lean? Recommend include only if
   space allows.
