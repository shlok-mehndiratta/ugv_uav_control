# ACODS re-framing plan — from "SITL testbed" to "shared-sensor perception-coupled control"

**Status:** planning only. No code/paper edits yet. Written 2026-07-24. Grounded in three literature/venue
studies (see §7 for sources). Governing constraint unchanged: **honesty — change the paper to match the
code, never overclaim.** `K`, `P_0`, ancillary law are OFF-LIMITS (hand-tuned, stability-critical).

---

## 0. The one-line diagnosis

The current paper presents *the SITL implementation* as the contribution ("we built a reproducible ROS2/Gazebo
testbed and an altitude policy"). Reviewers at a control venue read that as engineering, not a result. The
fix is **not** new experiments — it is to promote the idea that our system already embodies into the
foreground, give it a clean formal spine, and demote the SITL to *validation*.

The work already supports a real, defensible, and (pleasantly) **novel** control contribution. We just have
to say it correctly.

---

## 1. The re-framed idea (the "offering")

**Core claim (novel — a problem formulation + closed-form result, not a new tool):**
Because a *single* downward camera is shared between the two jobs, **one scalar — UAV altitude z — simultaneously
sets the estimator's measurement covariance and the navigation footprint.** This couples localization resolution
and navigation coverage in a way that (a) no cooperative aerial–ground system does (they use separate sensors),
and (b) no perception-aware planner does (they treat field-of-view as a hard *visibility* constraint, not a
continuous *resolution* knob traded against coverage). The coupling yields a **closed-form optimal altitude — the
"coverage floor"** — and a policy that provably (in the frozen-parameter sense) minimizes estimation uncertainty
subject to keeping the path navigable.

**Defensible one-sentence positioning (use ~verbatim in the intro):**
> In contrast to cooperative aerial–ground systems that use separate sensors for mapping and localization (e.g.
> ColAG) and to perception-aware planners that treat the camera field-of-view as a hard visibility constraint
> (PAMPC, Fisher-information-field planning), we study a *shared-sensor* regime in which one downward camera
> simultaneously supplies the UGV's navigation waypoint (via its FoV footprint) and its EKF pose measurement (via
> GSD-limited covariance R(z) ∝ z²), so that UAV altitude becomes a single scalar trading localization resolution
> against navigational coverage — admitting a closed-form optimal "coverage-floor" altitude z\* = d_look/tan(φ/2).

**What we must NOT claim as novel** (all prior art — say so plainly): the altitude→GSD relation (photogrammetry),
altitude→footprint (surveillance), coverage-vs-resolution via altitude (Stache et al.), active-perception via
Fisher information/covariance (Zhang–Scaramuzza, Falanga), the "sole aerial eye for a blind UGV" system (ColAG),
and event-triggered replanning (event-triggered MPC; RAPTOR). **The event trigger is an efficiency component, not
a core theoretical claim.**

**Three contributions, re-ordered so the idea leads:**
- **C1 (primary — the idea + the theorem).** Formalize the shared-sensor coverage–resolution coupling as a
  constrained estimation problem and prove the optimal altitude is the coverage floor (Prop A, §2). "Fly as low
  as segmentation allows, climb into turns" stops being a heuristic and becomes the *argmin*.
- **C2 (secondary — efficient, certifiable realization).** A perception-driven event-triggered scheme that makes
  the aerial planner affordable, with an honest closed-loop guarantee on the ancillary loop (Prop E, §2:
  inter-trigger ISS / ultimate boundedness + no-Zeno-by-construction) — plus the measured ≈25×/≈12× compute win.
- **C3 (validation — not the headline).** A reproducible ROS2/Gazebo SITL study that confirms C1 and C2:
  adaptive-altitude dominance over fixed baselines, and matched estimation quality at a fraction of the compute.

---

## 2. The theory spine (exactly what we prove — and the honesty pivot)

Two formal statements. Both are conference-grade, both are honestly provable, both map onto machinery already in
the code. This is the canonical IFAC-symposium shape (1 optimality prop + 1 closed-loop property + sim study).

### ⭐ HONESTY PIVOT (must get this right — the current framing is subtly wrong)
Substitute r_fov = z·tan(φ/2) into the deployed noise model and it collapses:
```
R(z) = R0 (z/z0)² (1 + (d / r_fov)²)  =  (R0/z0²) · ( z² + d²/tan²(φ/2) )
```
This is **strictly increasing in z** (dR/dz = 2R0 z/z0² > 0) — there is **no interior minimum**. (Neat side
effect: the FoV-edge term becomes a z-independent, d-dependent constant, so it doesn't disturb the argument.)
Therefore **z\* = d_look/tan(φ/2) is NOT a stationary point of tr(Σ); it is a constraint-boundary (coverage-floor)
optimum.** The estimator alone wants to fly as low as physically possible; it is *navigation coverage* that sets
the floor. State Prop A as a **constrained monotone optimization**, never as ∂tr(Σ)/∂z = 0. (This is also
consistent with `numsim_tradeoff.py`, which enforces a hard visibility gate: below the floor there is no
measurement and tr(Σ) blows up; above it, R grows.)

### Proposition A — optimal altitude = coverage floor  (backs C1)
*Setup:* frozen-parameter (constant z, offset d, process noise Q) linear-Gaussian target filter, per-axis position
measurement with variance R(z) as above; assume (F, Q^½) stabilizable and (F, H) detectable so the DARE has a
unique stabilizing PSD solution Σ_ss(R). Coverage constraint η(z) = z·tan(φ/2)/d_look(κ) ≥ 1.
*Claim:* (i) z ↦ tr Σ_ss(R(z)) is monotone nondecreasing; (ii) min_z tr Σ_ss s.t. η(z)≥1, z∈[z_min,z_max] is
attained at **z\* = d_look(κ)/tan(φ/2)**, the smallest feasible z; and z\* rises with curvature κ.
*Proof route (self-contained, ~half column):* do the **scalar/decoupled per-axis** case — the scalar DARE σ_ss(r)
is an explicit increasing function of r (one line of calculus), which suffices since the model is isotropic per
axis. Then η≥1 ⟺ z ≥ z_floor; minimizing a nondecreasing objective over [z_floor, z_max] ⇒ minimizer = z_floor
(monotone KKT: multiplier λ = f'(z\*)/tan(φ/2) ≥ 0, complementary slackness active). **Needs monotonicity only —
no convexity.**
*Cite:* Riccati/DARE monotonicity in R — Wimmer (Syst. Control Lett., 1985); Lancaster & Rodman, *Algebraic
Riccati Equations* (1995); filtering form Anderson & Moore, *Optimal Filtering* (1979). Fisher-information gloss
(one sentence, for the "active perception" framing): per-axis info I(z)=1/R(z) is decreasing in z — Bishop et al.,
*Automatica* (2010).

### Proposition E — inter-trigger boundedness + no Zeno  (backs C2)
*Setup:* ancillary error map (code eq. `error_dynamics`) e_{k+1} = A_K e_k + w_k, A_K = diag(0.97,0.97,0.96),
ρ := ‖A_K‖₂ = 0.97 < 1, disturbance ‖w_k‖ ≤ w̄.
*Claim:* (i) ‖e_k‖ ≤ ρ^k‖e_0‖ + w̄/(1−ρ) (ISS w.r.t. w); (ii) ultimate bound limsup‖e_k‖ ≤ w̄/(1−ρ), so the error
enters and stays in the monitor ellipsoid E(α) for any α ≥ (w̄/(1−ρ))·√λ_max(P_0) — certifying ellipsoid
membership as a *sound* trigger metric; (iii) inter-event time ≥ the 20 Hz sampling period (and ≥ 1 s for
lockout-guarded conditions), so **Zeno is excluded by construction.**
*Proof:* ‖e_{k+1}‖ ≤ ρ‖e_k‖ + w̄; iterate geometric series; (iii) by periodic + time-lockout design.
*Cite:* discrete-time ISS — Jiang & Wang (*Automatica*, 2001); periodic event-triggered / MIET≥sampling — Heemels,
Donkers & Teel (*IEEE TAC*, 2013); event-trigger framing — Tabuada (*IEEE TAC*, 2007), Heemels–Johansson–Tabuada
(CDC, 2012); ultimate-bound⊆mRPI tie to the tube baseline — Mayne–Seron–Raković (*Automatica*, 2005), Raković et
al. (*IEEE TAC*, 2005, the existing `mRPI` cite).

### DO NOT CLAIM (the overclaim list — keep us honest)
1. z\* is an *interior/stationary* optimum (it's a boundary optimum — see pivot).
2. Prop B (tracking z\*(d_k) beats *any* fixed altitude) as a **theorem** — keep it **empirical**, or at most a
   corollary under an explicit *quasi-static / slow-variation* assumption. Steady-state monotonicity ≠ integrated
   trajectory dominance.
3. Prop C (anticipatory > reactive) as a guarantee — it *lags* in SITL; analytic-only under perfect preview.
   Remark, not property.
4. Separation principle / certainty-equivalence / LQG optimality of the joint scheme — R depends on z (a control),
   so this is dual/active-sensing control, generally intractable.
5. CRLB/Fisher bound *achieved* (EKF isn't efficient) or EKF minimum-variance optimality (nonlinear).
6. Continuous-time Zeno-freeness / hard positive MIET beyond "≥ sampling period" — our trigger is periodic+locked.
7. ISS/boundedness of the *full coupled* closed loop — Prop E covers only the **linear ancillary error subsystem**;
   say so.
8. Recursive feasibility / robust invariance for the *online* controller — keep the existing honest concession
   that the adaptive ellipsoidal surrogate relaxes it.
9. Convexity of tr Σ_ss(z) — not needed, likely false; rely on monotonicity.
10. Optimality of the anticipatory margin β_κκ̂ + z_m — the *optimum* is the bare coverage floor; the margin is a
    robustness/preview add-on.

---

## 3. Related-work restructure (position, don't list)

Three papers we **must cite and explicitly distinguish** — one sentence of daylight each:
- **ColAG** (Li et al., ICRA 2024) — nearest *system* (one perceptive UAV, blind UGVs). Daylight: *separate*
  sensors (LiDAR map + tag pose), altitude *passive*, no coverage–resolution coupling.
- **Stache et al.** (ECMR 2021 / RAS 2023, multi-resolution semantic segmentation) — nearest on *altitude as a
  resolution knob under a coverage tradeoff*. Daylight: offline-ish mapping/segmentation, no closed-loop
  estimator, no navigation-lookahead constraint, no shared dual-role sensor, no closed-form optimum.
- **Zhang & Scaramuzza (ICRA 2019) + Falanga et al. (PAMPC, IROS 2018)** — define the "theory currency" (Fisher
  info / covariance / observability). Daylight: single-robot self-localization; FoV a *hard constraint* not a
  continuous GSD–coverage tradeoff; no reduction to one altitude scalar; no cooperative nav-reference role.
Keep the belief-space cluster (Platt, van den Berg, Prentice) as the family our UGV MPC belongs to — no guarantee
claimed. Add Bishop et al. (2010) for the FIM framing, and the discrete-ISS / periodic-event-triggered cites.

---

## 4. Narrative arc / section skeleton (6-page ifacconf)

1. **Intro** — perception-limited UGV → aerial eye → *shared single camera* → the coupling → altitude as one
   control scalar → coverage-floor optimum. End with the 3 contributions (C1 lead).
2. **Related work** — the three-way positioning of §3.
3. **System & models** — UGV unicycle; UAV single-integrator; **GSD model with the collapse to R=(R0/z0²)(z²+
   const)** made explicit (sets up Prop A); coverage factor η; the shared-sensor dual role.
4. **Coverage–resolution coupling & optimal altitude (C1)** — the constrained problem, **Prop A + proof**, then
   the altitude policy as its realization (bare floor = optimum; margin/anticipation = engineering add-on, stated
   honestly). Fisher-info sentence for framing.
5. **Event-triggered aerial planning (C2)** — belief-space UGV MPC (continuous); aerial planner + ancillary law;
   the trigger; **Prop E + proof**; the honest surrogate concession.
6. **Simulation study (C3)** — setup/params table; **Fig: envelope (two jaws)**; **Fig: Prop A validation**
   (promote `Fig_PropA` — steady-state tr(Σ) vs z with argmin at floor; this *is* the theorem's evidence);
   **Fig: adaptive vs fixed altitude**; **Table: event-trigger co-design (25×/12×)**. Testbed frame figure.
7. **Conclusion + limitations** — velocity-controlled abstraction, single-camera coupling, reactive timing,
   design-time tube; future = hardware/full multicopter.

---

## 5. Work plan — what each claim needs (code / sim / analysis)

**Already supported by committed data — NO re-run:**
- Envelope (two jaws), adaptive-vs-fixed altitude (Fig. 4), event-trigger table (25×/12×) — all in `paper_data/`,
  regenerate from existing scripts.

**New analysis/figures needed (no new SITL runs — analytic/existing data):**
- **Prop A validation figure.** `numsim_tradeoff.py` already produces `Fig_PropA` (tr(Σ_ss) vs z, argmin at
  d/tanφ) and `Fig_PropBC`. These are currently *unused*. Under the re-frame they become the theorem's empirical
  backbone. Action: adopt `Fig_PropA` into the paper; restyle to ifacconf; caption ties argmin→coverage floor.
- **Prop E illustration (optional, cheap).** A small plot of ‖e_k‖ with the ultimate-bound envelope w̄/(1−ρ), and
  inter-event-time histogram from an existing co-design run — evidence for boundedness + no-Zeno. Derivable from
  logged data; likely no re-run.
- **Verify the R(z) collapse numerically** (one-liner) so the "monotone, no interior min" claim is airtight.

**Code changes (minimal, all default-off / non-stability-critical):**
- None required for the theory. Possible tidy: expose w̄ (disturbance bound) estimate for the Prop E plot.
- Do **not** touch K, P_0, ancillary law, GSD constants.

**Re-simulation — only if we decide to, and only headless:**
- Not needed for C1/C2/C3 as scoped. Candidate *only* if we want a cleaner Prop A sweep in the real SITL (vs the
  analytic validator) — a nice-to-have, not required.

**Housekeeping bug to fix during the rewrite (carried from PROJECT_HISTORY):**
- **Track-label inconsistency.** "sharp track" is overloaded: envelope=track4, altitude Fig.4=track_hairpin,
  co-design Table II=track2. ACODS.tex Table II still says "sharp track" but the co-design ran on `track2`.
  Reconcile labels (the new Gentle/Slalom/Sharp naming must map to the actual data folders) before submission.

---

## 6. Open decisions for the user (need your call)
1. **Deadline/venue reality check** (blocking) — is ACODS 2026 still open? If not: ACODS 2028, or a different
   IFAC venue (NecSys/ADHS/IFAC WC), or back to an IEEE conf with the *same* re-frame? The re-frame is
   venue-portable.
2. **Which contribution leads?** Recommendation: **C1 (altitude coverage-floor optimality) leads**, event-trigger
   secondary. Alternative: keep event-trigger co-design co-equal. This changes title + abstract + section order.
3. **Theory ambition.** Recommendation: the **two-proposition spine (Prop A + Prop E)** — it's the ideal ACODS fit
   and fully honest. Heavier theory (e.g. a real online feasibility certificate) would need new work and risks
   overclaiming; lighter (Prop A only) is thinner. Confirm the two-prop spine.
4. **Title.** Current "SCAT: Shared-Camera Adaptive Tracking…" is fine but implementation-flavored. Consider one
   that foregrounds the idea, e.g. *"Altitude as an Active-Perception Variable: Optimal Coverage–Resolution
   Trade-off in Shared-Sensor Aerial–Ground Tracking."* (≤10 words for IFAC — needs trimming.)

---

## 7. Sources (from the three grounding studies)
- Novelty/positioning: ColAG (arXiv:2310.13324, ICRA 2024); Stache et al. (arXiv:2203.01642; RAS 2023); Fisher
  Information Field (Zhang–Scaramuzza, ICRA 2019 + T-RO ext.); Spasojevic et al. (arXiv:2005.13986, ICRA 2020);
  Falanga PAMPC (IROS 2018); crowd-surveillance coverage/accuracy (Drones 2025).
- Theory: DARE monotonicity (Wimmer 1985; Lancaster–Rodman 1995; Anderson–Moore 1979; Wonham 1968); FIM geometry
  (Bishop et al., Automatica 2010); discrete ISS (Jiang–Wang 2001); periodic event-triggered/MIET (Heemels–
  Donkers–Teel 2013); event-trigger (Tabuada 2007; Heemels–Johansson–Tabuada 2012); tube/mRPI (Mayne et al. 2005;
  Raković et al. 2005).
- Venue: ACODS 2026 official (acods-conference.org), IFAC listing, NIT Warangal CFP; IFAC-PapersOnLine ACODS
  2020/2024 proceedings for structure. ⚠️ deadline 22 Jul 2026 per live site — verify.
