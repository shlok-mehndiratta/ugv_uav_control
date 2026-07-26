# Literature survey & positioning (ACODS / IFAC)

Companion to `PAPER_PLAN.md` (theory/claims) and `SIM_AND_PLOTS.md` (evidence). This is the
*positioning* layer: the verified citation base, the gap our coupling occupies across five adjacent
literatures, and an **evidence-based framing recommendation**. Built from five parallel, web-verified
surveys (2026-07-25). Every citation below was checked against a primary source; soft fields are
flagged in §6 and must be reconfirmed against the DOI record before submission.

---

## 1. Positioning thesis (one paragraph)

Prior cooperative aerial–ground systems resolve the **coverage-vs-accuracy** tension by *splitting it
across separate robots or separate sensors*; perception-aware control makes a *single* robot move so its
*own* estimate improves *monotonically*; belief-space planners let the ego-robot's *own motion* gather
information; event-triggered control fires on a *state-error* condition over an *exogenous* disturbance;
and the GSD/altitude literature optimizes altitude for coverage *or* resolution, empirically. **None of
them has a single shared downward camera whose one scalar — altitude — simultaneously and
*antagonistically* sets localization resolution (`R(z)∝z²`, measured `r=0.996`) and navigation coverage,
resolved in closed form by a curvature-scheduled optimum.** That coupling is the unoccupied territory,
and it is the paper's identity.

---

## 2. The five neighbor literatures

### 2.1 Cooperative aerial–ground ("aerial eye")
**Canonical:** Grocholsky, Keller, Kumar, Pappas, *Cooperative Air and Ground Surveillance*, IEEE R&A
Magazine 13(3):16–25, 2006 (the canonical "coverage-vs-accuracy synergy" statement); Chaimowicz et al.,
*Deploying Air-Ground Multi-Robot Teams in Urban Environments*, Springer MRS-III, 2005; Hsieh et al.,
*Adaptive Teams of Autonomous Aerial and Ground Robots…*, JFR 24(11–12), 2007; Michael et al.,
*Collaborative Mapping of an Earthquake-Damaged Building…*, JFR/FSR 2012.
**Recent:** Delmerico, Mueggler, Nitsch, Scaramuzza, *Active Autonomous Aerial Exploration for Ground
Robot Path Planning*, IEEE RA-L 2017 (UAV down-camera → ground path, navigation only); Sivarathri &
Shukla, *Autonomous Non-Communicative Navigation Assistance…*, Machines 13(2):152, 2025 (**closest
match** — UAV down-camera ArUco on AGV, but a *second* sky-facing camera on the AGV; altitude a fixed
10 m ceiling); Fisker et al., *UAV See, UGV Do*, arXiv:2505.16912, 2025 (separate sensors, two phases).
**Motivation move:** "give the low-horizon ground robot a bird's-eye teammate" / "complementary
strengths — UAV coverage vs. UGV accuracy/payload."
**Gap:** the two functions we fuse (localize the UGV + generate its waypoints) are *always* split across
separate sensors/robots; altitude is fixed, hovered, or bounded by an empirical ceiling — never solved for.

### 2.2 Perception-aware / active-perception control
**Canonical:** Falanga, Foehn, Lu, Scaramuzza, *PAMPC: Perception-Aware MPC for Quadrotors*, IROS 2018,
pp. 5200–5207 (perception + action coupled through the dynamics as a soft cost — *"a framework able to
consider both perception and action objectives and couple them through the system dynamics is
necessary"*); Bajcsy, Aloimonos, Tsotsos, *Revisiting Active Perception*, Autonomous Robots 42:177–196,
2018 (field's intellectual root). **Active localization:** Costante et al., *Perception-aware Path
Planning*, arXiv:1605.04151, 2016 (preprint — cite as such); Zhang & Scaramuzza, *Perception-aware
Receding Horizon Navigation for MAVs*, ICRA 2018; Zhang & Scaramuzza, *Beyond Point Clouds: Fisher
Information Field…*, ICRA 2019. **Control-theoretic:** Murali, Spasojevic, Guerra, Karaman,
*Perception-aware Trajectory Generation… via Differential Flatness*, ACC 2019 (closest to ACODS taste).
Also Zhou et al., *RAPTOR*, IEEE T-RO 37(6):1992–2009, 2021; Placed et al., *A Survey on Active SLAM*,
IEEE T-RO 39(3):1686–1705, 2023 (positioning anchor).
**Motivation move:** "perception and control are traditionally decoupled → suboptimal → we co-optimize"
(PAMPC). **This is the template to mirror, then differentiate from.**
**Gap:** perceiving agent = acting agent (single-robot); perception is *monotone-good* (better viewpoint
⇒ lower covariance, full stop) and independently weightable; the optimized variable is trajectory/yaw,
never an altitude scalar that sets *two* perception products in *opposite* directions.

### 2.3 Belief-space planning (our UGV MPC's family)
**Canonical lineage:** Prentice & Roy, *The Belief Roadmap…*, IJRR 2009; van den Berg, Abbeel, Goldberg,
*LQG-MP*, IJRR 2011; van den Berg, Patil, Alterovitz, *Motion Planning under Uncertainty using Iterative
Local Optimization in Belief Space*, IJRR 31(11), 2012; Platt, Tedrake, Kaelbling, Lozano-Pérez, *Belief
Space Planning Assuming Maximum Likelihood Observations*, RSS 2010; Indelman, Carlone, Dellaert,
*Planning in the Continuous Domain: A Generalized Belief Space Approach…*, IJRR 34(7):849–882, 2015.
**Recent uncertainty-aware MPC:** Kayalıbay et al., *Filter-Aware Model-Predictive Control*, L4DC 2023
(PMLR v211:1441–1454); Hu & Fisac, *Active Uncertainty Reduction for HRI: An Implicit Dual Control
Approach*, WAFR 2022 / IJRR 2024.
**Motivation move:** "plan in belief space so the robot actively shapes its *own* future observations"
(the light–dark domain: move into the light to localize).
**Gap — our honest delta:** in all of these the single lever over measurement quality is the *ego-robot's
own motion*. In ours the observation covariance is set by an **external teammate's altitude**, exogenous
to the UGV — the UGV cannot "move into the light"; the light level is dialed overhead — and that same
altitude also sets the navigation reference the planner must track. We inherit the belief-space MPC
machinery but **relocate the information-design decision from the ego-robot's trajectory to a teammate's
viewpoint**, and expose the coverage/covariance coupling the ego-only formulations never face.

### 2.4 Event-triggered control + ISS/no-Zeno (Prop E's base)
**Verified citation base (all fields confirmed except flagged issue numbers):**
Tabuada 2007, *Event-Triggered Real-Time Scheduling of Stabilizing Control Tasks*, IEEE TAC 52:1680–1685
(paradigm + positive inter-event time); Heemels, Johansson, Tabuada 2012, *An Introduction to
Event-Triggered and Self-Triggered Control*, CDC pp. 3270–3285 (definitions/motivation); Girard 2015,
*Dynamic Triggering Mechanisms…*, IEEE TAC 60(7):1992–1997; **Jiang & Wang 2001, *Input-to-state
stability for discrete-time nonlinear systems*, Automatica 37(6):857–869 (PRIMARY backbone for our
discrete-time ISS/ultimate-bound claim)**; Heemels, Donkers, Teel 2013, *Periodic Event-Triggered
Control for Linear Systems*, IEEE TAC 58(4):847–861 (**best "no-Zeno-by-construction in discrete time"
cite** — MIET ≥ one sampling period); Sontag 1989, *Smooth Stabilization Implies Coprime Factorization*,
IEEE TAC 34(4):435–443 (ISS origin); Johansson et al. 1999, *On the regularization of Zeno hybrid
automata*, Systems & Control Letters 38(3):141–150 (Zeno definition).
**Nearest precedent — MUST cite & position against:** Trimpe & D'Andrea 2014, *Event-Based State
Estimation With Variance-Based Triggering*, IEEE TAC 59(12):3266–3281 — triggers on predicted error
*variance*. Also Anta & Tabuada 2010, *To Sample or Not to Sample*, IEEE TAC 55(9):2030–2042 (positive
MIET, nonlinear).
**Motivation move:** "periodic execution wastes computation/communication; act only when a
state-dependent condition demands it, with guarantees." (Map to: re-plan only when *perception* demands.)
**Gap (honesty-narrowed):** covariance-*driven* triggers already exist (Trimpe–D'Andrea), so our novelty
is **not** "trigger on covariance." It is: a trigger that is a *geometric function of altitude* (FoV/GSD),
firing a control *re-plan*, over a disturbance whose covariance `R∝z²` is itself *set by the chosen
altitude* — i.e. **co-design of gain `K` and altitude `z`**, with a discrete-time ISS/UB bound on the
ancillary error. Trimpe–D'Andrea triggers an estimator *transmit* (not a re-plan), is not tied to camera
geometry, and does no gain+altitude co-design.

### 2.5 GSD / altitude vs perception quality + Riccati monotonicity (Prop A's base)
**GSD∝altitude (citable):** Di Franco & Buttazzo, *Coverage Path Planning for UAVs Photogrammetry with
Energy and Resolution Constraints*, J. Intelligent & Robotic Systems 83:445–462, 2016 (writes GSD =
pixel·H/(f·width), uses GSD as a hard *resolution constraint*). Supporting empirical altitude-vs-detection:
*Drones* 9(9):595, 2025 and a *Sensors* 2025 zoom-CPP study (author lists unconfirmed — supporting only);
ArUco-error-vs-range preprint Hinderer et al., arXiv:2509.17345, 2025 (supporting, not archival).
**DARE monotonicity in R (Prop A's analytical lever):**
- **PRIMARY: Ran & Vreugdenhil 1988, *Existence and comparison theorems for algebraic Riccati equations
  for continuous- and discrete-time systems*, Linear Algebra and its Applications 99:63–83** — explicit
  **comparison theorem for the DISCRETE ARE** (ordering the data orders the maximal solutions). This is
  the correct backing for "steady-state covariance `P(R)` monotone nondecreasing in `R`."
- **Support:** Anderson & Moore, *Optimal Filtering*, Prentice-Hall 1979 (canonical Kalman
  covariance-monotonicity statement); Lancaster & Rodman, *Algebraic Riccati Equations*, Oxford 1995
  (dedicated comparison-theorem chapter).
- ⚠️ **CORRECTION carried into memory:** Wimmer 1985 (*Monotonicity of maximal solutions of ARE*, Syst.
  Control Lett. 5:317–319) is **continuous-time and monotone in the constant term Q — NOT the discrete
  filter's R.** Use it only as the "classic monotonicity template," *not* as the exact result. Earlier
  plan notes citing Wimmer for Prop A are superseded by **Ran–Vreugdenhil**.
**Motivation move (partial, exists):** "lower ⇒ finer GSD, better detection, less coverage; higher ⇒
more coverage, worse per-pixel accuracy." **Our sharper move:** treat altitude as a *first-class control
input* setting `R(z)∝z²` (measured) and coverage, and resolve the trade *analytically*.
**Gap:** altitude is optimized for coverage OR resolution; where both appear, resolution is a *constraint*
or the trade is *empirical/numerical*. No prior work does (1) measured `R(z)∝z²` ⇒ (2) coverage
constraint ⇒ (3) closed-form boundary optimum via DARE monotonicity. The two literatures (perception
GSD, Riccati comparison theory) are **currently disjoint**; connecting them is the contribution.

---

## 3. Consolidated gap statement (drop-in for Related Work)

> Cooperative aerial–ground systems assign coverage and accuracy to *different* robots or sensors
> [Grocholsky'06, Chaimowicz'05, Michael'12]; the recent single-camera exception still adds a second
> sensor and fixes altitude [Sivarathri'25]. Perception-aware control couples perception to action for a
> *single* robot, where a better viewpoint monotonically improves the robot's *own* estimate
> [PAMPC'18, Zhang'19]. Belief-space planners let the ego-robot's *own motion* gather information
> [Prentice-Roy'09, van den Berg'12, Platt'10]. Event-triggered control fires on a *state-error*
> condition over an *exogenous* disturbance [Tabuada'07, Girard'15]; even variance-based triggering
> [Trimpe-D'Andrea'14] drives an estimator transmit, not a geometry-coupled re-plan. And UAV
> altitude is optimized for coverage *or* resolution, empirically [DiFranco'16]. **We are the first to
> treat a single shared camera's altitude as one control variable that antagonistically sets both
> localization resolution (`R(z)∝z²`, measured) and navigation coverage, with a closed-form,
> curvature-scheduled optimum (Prop A) realized by a perception-triggered altitude/gain co-design with
> discrete-time ISS and no-Zeno guarantees (Prop E).**

---

## 4. What the survey changes in our plan

1. **Prop A citation fix:** Ran–Vreugdenhil 1988 (discrete ARE comparison) is PRIMARY; Anderson–Moore /
   Lancaster–Rodman support; Wimmer demoted to "template." (Update `PAPER_PLAN.md` + memory.)
2. **Prop E novelty narrowed & strengthened:** cite Trimpe–D'Andrea 2014 as nearest precedent; our claim
   is geometry-coupled trigger + gain/altitude co-design + endogenous `R∝z²`, not "covariance triggering."
3. **Motivation template chosen:** mirror PAMPC's "decoupled → co-optimize," then escalate — *one shared
   parameter, two products, opposite directions.*
4. **Prop A is a *connection/modeling* result, not a deep theorem** — frame it that way; the weight is
   the measured `R(z)` + the closed-form curvature schedule + the pairing with Prop E, never the
   monotonicity lemma alone.

---

## 5. Framing recommendation (evidence-based; final call deferred to post-plots)

**Where the evidence points:** every one of the five literatures confirms daylight around the *coupling*,
and none around either theorem taken alone. Prop A is, honestly, a modeling/connection result (monotone
⇒ boundary optimum); Prop E sits next to an existing covariance-trigger precedent. So **leading with
either theorem exposes its individual modesty, while leading with the coupling plays to the one asset the
whole field concedes is unoccupied.** Recommendation: **lead with the shared-sensor coupling as the
identity**, present Prop A ("where to fly") and Prop E ("when to re-plan") as a *matched pair* that
together answer the two questions a shared-sensor co-design must answer, and anchor credibility on the
*measured* `R(z)∝z²`.

**What the plots must confirm to lock this** (the second input you asked to wait for):
- The **two-jaw trade-off** must be visible and reproducible on the envelope (estimation improves as
  altitude drops; steering effort/coverage-loss rises) — this is the empirical face of the coupling.
- Adaptive-altitude must **dominate** both fixed-low and fixed-high on Var-κ (validates "climb into turns").
- If the two jaws are clean → coupling-led framing is fully earned. If the estimation jaw is weak but the
  co-design/event-trigger result is strong → we reconsider a Prop-E-forward emphasis. **Decide after the
  matrix runs.**

---

## 6. Pre-submission citation verification checklist (soft fields)
- Tabuada'07 & Johansson'99 issue numbers — drop `number=` for zero risk (vol/pages/year solid).
- Costante et al. — arXiv:1605.04151 only; cite as preprint (no confirmed peer-reviewed venue).
- Ran–Vreugdenhil pages (99:63–83), Wimmer pages (5:317–319) — from DB listing; reconfirm on DOI.
- *Drones* 9(9):595 2025, *Sensors* 2025 zoom-CPP, swamp-deer *Ecol. Informatics* 2024 — author lists
  unconfirmed; supporting evidence only, reconfirm before archival citation.
- Bircher NBV — two forms (ICRA'16 vs Autonomous Robots'18); pick per style.
- Belief Roadmap — two title variants; cite the IJRR 2009 version.
- Excluded as unverifiable: a 2026 aerial-ground preprint and two incomplete author lists surfaced by the
  aerial-ground search — do not cite.
