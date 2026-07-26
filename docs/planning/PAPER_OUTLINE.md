# ACODS paper — section outline v2 (reviewed)

**Format:** IFAC `ifacconf`, ~6 pp, natbib author–year. **Theory-first**, standard structure.
**Working title (T1, may revise):** *Altitude as a Coupled Perception Variable in Shared-Camera
Aerial–Ground Tracking.*

**Thesis:** In a shared-camera aerial–ground system the UAV's altitude is one control variable that
antagonistically sets localization resolution and navigation coverage; we characterize in closed
form the optimal altitude and *when* adapting it helps, realize it with an event-triggered co-design
carrying stability guarantees, and verify the model and the theory's predictions in SITL.

**Novelty (verified, present as such):** first to treat one shared camera's altitude as a single
scalar coupling resolution and coverage in opposite directions, with a closed-form curvature-scheduled
optimum + perception-triggered co-design (ISS, no-Zeno). No precedent across the four neighbor fields.

---

## Global writing rules (enforce everywhere — from your guidance)
1. **Theory uses symbols, not sim values.** Every gain/weight is a named symbol
   ($K,\ \alpha\!\in\![\alpha_{\min},\alpha_{\max}],\ \beta_\kappa,\ z_{\mathrm m},\ \lambda,\ w_{\mathrm{fov}},
   w_z,\ \varepsilon,\ Q,R,\ldots$). **Numeric values appear only in §4 (Table I).**
2. **Singularity guards → $\varepsilon$.** The aerial visibility cost denominator $z^2+0.1$ becomes
   $z^2+\varepsilon$ with $\varepsilon>0$ (regularizer avoiding the $z\!\to\!0$ singularity); §4 states $\varepsilon=0.1$.
3. **Every term of every equation is explained** in words on first use.
4. **One notation table**, symbols fixed once; consistent thereafter.
5. **Flow:** each subsection ends by motivating the next; no orphan results.
6. **Claim discipline:** always "curvature-scheduled coverage floor" / "when adaptation helps," never
   an unconditional "adaptive beats fixed." Every §4 result cites the proposition it verifies.
7. Tracks always **Low-κ/Mid-κ/High-κ/Var-κ**; numbers only from `paper_data/`; drop `maxCov`.

---

## §1 Introduction (Introduction + Related Work, merged)  *(to write — strengthen motivation)*
**Purpose:** motivate, position, and stake the claim in one flowing narrative (prof's suggestion).
**Motivation strategy — learn from the base papers' intros** (mirror, then escalate):
- *Perception-aware control* (PAMPC, Falanga'18): the move is "perception and control are traditionally
  decoupled → suboptimal → co-optimize." We adopt this rhetorical arc, then escalate: not just coupling
  perception to motion, but **one shared parameter setting two perception products in opposite directions.**
- *Cooperative aerial–ground* (Grocholsky'06): "aerial coverage vs. ground accuracy — complementary." We
  note that trade is classically split across *robots/sensors*; we collapse it to **one altitude scalar.**
- *Belief-space* (Prentice-Roy, van den Berg): "act to shape your own future observations." We externalize
  the information decision to a **teammate's viewpoint.**
**Content order:** (i) perception-limited UGVs + the aerial eye; (ii) the observation that a *single shared*
downward camera makes altitude set both GSD-resolution and footprint-coverage — oppositely; (iii) a tight
related-work sweep woven in as the gap (the four fields above, each closed with one gap sentence, citing the
verified refs); (iv) the idea; (v) contributions C1 (coupling theory), C2 (event-triggered co-design),
C3 (SITL verification). End on Fig 1 (testbed, dual role).
**Figs:** Fig 1.  **Length:** ~1.25 pp.

## §2 System Model and the Altitude Coupling  *(drafted — SYSTEM_MODELS_DRAFT.tex; refine)*
**Purpose:** formalize the coupling; fix notation.
**Content (term-by-term):** shared-camera dual role (ArUco→EKF belief $(\mu,\Sigma)$; lane-seg→waypoint);
UGV unicycle w/ first-order actuator lag (state, inputs, noise defined); UAV velocity-controlled single
integrator $\mathbf p_{k+1}=\mathbf p_k+\Delta t\,\mathbf u_k+\mathbf w_k$, $A_d=I,B_d=\Delta t I$;
**resolution model** $R(z)=R_0(z/z_0)^2(1+(d/r_{\mathrm{fov}})^2)$ — each factor explained, monotone in $z$;
**coverage model** $\eta=r_{\mathrm{fov}}(z)/d_{\mathrm{look}}(\kappa)$, floor $z_{\mathrm{fl}}(\kappa)=d_{\mathrm{look}}(\kappa)/\tan(\phi/2)$.
**Figs/Tables:** Fig 2 (system TikZ); notation table.  **Length:** ~1.0 pp.

## §3 Coupled Altitude Control: Analysis and Realization  *(MERGED Theory+Controller — the contribution)*
**Purpose:** the core — prove the optimal-altitude theory, then present the controller that deploys it.
Theory first (symbols only), realization second.
- **3.1 Constrained estimation problem.** Minimize time-averaged $J[z(\cdot)]=\frac1T\int \operatorname{tr}P_\infty(R(z))\,dt$
  s.t. coverage $z\ge z_{\mathrm{fl}}(\kappa)$, box $[z_{\min},z_{\max}]$.
- **3.2 Optimal altitude.** Lemma (DARE monotonicity in $R$) → **Prop 1** (optimum = coverage floor $z^\star(\kappa)$)
  → **Cor 1** (climb into turns) → **Prop 2** (adaptive strictly dominates any fixed altitude *iff* the floor binds
  and varies; gain ∝ curvature spread) → **Cor 2** (boundary $d_{\mathrm{look}}(\kappa_{\max})=z_{\min}\tan(\phi/2)$).
  Remark: this *predicts* the SITL outcome.
- **3.3 Belief-space UGV controller.** Covariance rollout (Joseph update + visibility gate $\gamma$), cost
  $J_g=\sum(\lVert e\rVert_Q^2+\lVert u\rVert_R^2+\lambda\operatorname{tr}\Sigma+w_{\mathrm{fov}}c_{\mathrm{fov}})$
  — every weight symbolic and explained.
- **3.4 Aerial planner + altitude policy.** Nominal planner cost (goal + altitude regularizer $w_z(z-z^\star)^2$
  + visibility term $w_{\mathrm{alt}}\,\lVert\cdot\rVert^2/(z^2+\varepsilon)$ — note $\varepsilon$); the policy
  $z_k=\hat d_k/\tan(\phi/2)+\beta_\kappa\hat\kappa_k+z_{\mathrm m}$ realizing Prop 1/Cor 1.
- **3.5 Event-triggered co-design.** Ancillary law $u=\bar u-Ke$ ⇒ $A_K=A_d-B_dK=I-\Delta t\,K$ (Schur, symbolic $K$);
  adaptive ellipsoidal surrogate $\mathcal E(\alpha_k)$; perception-driven trigger (tube-exit + covariance +
  curvature + drift); **Prop 3** (inter-trigger ISS/ultimate-bound + no-Zeno; unsaturated-tube scope).
**Figs/Tables:** none (equations).  **Length:** ~2.5 pp (the paper's center of gravity).

## §4 Simulation Study  *(to write — data ready)*
**Purpose:** verify the model + the theorem's predictions; assign the numbers here.
- Setup: ROS2/Gazebo, standard 4-track set, common logger; **Table I** (all numeric parameter values, incl.
  $K=\mathrm{diag}(\cdot)$, $\varepsilon=0.1$, weights, $\alpha$ range).
- **Fig 3a — E0:** ArUco error ∝ z ($R^2{=}0.994$) ⇒ verifies A1 ($R\propto z^2$).
- **Fig 3b — Envelope:** meanCov monotone ↑ z, track-independent ⇒ verifies Lemma/Prop 1 premise.
- **Fig 4 — Prop 2 non-binding:** adaptive $=$ fixed-low, $\Delta\approx0$, 0% off-corridor ⇒ confirms Prop 2's
  non-binding branch (our config has $z_{\mathrm{fl}}<z_{\min}$); altitude trace shows Cor 1 climbs. State the null *as* confirmation.
- **Table II + Fig 5 — Prop 3:** C1 vs C2 (≈25× solves, ≈12× CPU, matched cov); Fig 5 (sparsity + no-Zeno MIET).
  Footnote: co-design on the earlier track; continuous baseline is footprint-sensitive.
**Length:** ~1.1 pp.

## §5 Conclusion  *(to write)*
Takeaway (altitude as coupled active-perception variable; the *characterization* is the contribution; the
co-design makes it deployable with guarantees). **Limitations:** velocity-controlled UAV abstraction; tube-
*inspired* online surrogate; the binding-regime benefit ($\Delta>0$) is characterized, not SITL-shown.
**Future:** demonstrate the binding branch (speed-scaled lookahead), full multicopter, hardware.  **Length:** ~0.3 pp.

---

## Figures & tables
| Item | Section | Verifies/shows |
|---|---|---|
| Fig 1 testbed | §1 | shared-camera dual role |
| Fig 2 system TikZ | §2 | signal flow; the coupling |
| Fig 3a E0 | §4 | A1: `R∝z²` |
| Fig 3b envelope | §4 | monotone estimation cost |
| Fig 4 adaptive vs fixed | §4 | Prop 2 non-binding + Cor 1 |
| Fig 5 (keep?) co-design | §4 | Prop 3 sparsity + no-Zeno |
| Table I parameters | §4 | all numeric values |
| Table II co-design | §4 | Prop 3 compute saving |

## Section order & budget (~6 pp)
§1 Intro+Related (1.25) · §2 Model (1.0) · **§3 Analysis+Realization (2.5)** · §4 Simulation (1.1) · §5 Conclusion (0.3).
Merged §3 is the center of gravity, as it should be for a theory-first paper.
