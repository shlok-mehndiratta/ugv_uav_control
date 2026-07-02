# Task

Generate a **rigorous standalone mathematical derivation document** for our UAV-UGV cooperative tracking system. This is **not** a code report — that already exists in `report.html`. This is the theoretical companion: a graduate-thesis-style write-up that a reviewer or examiner could read on its own and understand exactly what we are doing mathematically, why, and under what assumptions.

Output: a single **PDF file** saved to the project root as `theory_report.pdf`. Keep the source (`.tex` or `.md`) alongside it so I can edit later.

## Context

You have already inspected this codebase (`ugv_uav_control`) and produced `report.html`. Do **not** repeat the code-summary content here. This document is purely theoretical.

The work extends Ferreira Santos, Castillo & Corrêa Victorino — *"Aerial Vision Based Guidance and Control for Perception-Less Ground Vehicle,"* ECC 2024. Two extensions matter for this document:

1. **Tube MPC for the UAV.** ECC used fixed pre-defined waypoints with a tracking controller; we plan the UAV trajectory online with event-triggered Tube MPC.
2. **Observability coupling.** The EKF covariance is fed into both controllers' cost functions, creating a bidirectional perception–control loop that ECC does not have.

Read the ECC paper for the UGV side. Read the codebase to confirm what is actually implemented vs. idealized. Then derive the math properly from first principles.

## Required structure

The document must contain the following sections, in this order. Each derivation starts from textbook basics — do not assume the reader already knows Tube MPC or belief-space planning.

### 1. Notation and symbol table
Every variable used anywhere in the document, defined once upfront. Include dimensions (scalar / vector / matrix), units where applicable, and the section where the symbol is first introduced. Distinguish state-space symbols, control inputs, disturbances, error variables, and tuning parameters.

### 2. System representations
- **UGV model.** Start from the full bicycle model (ECC eq. 6). Apply the same simplifications (no-slip, small lateral forces) to reach ECC eq. 7. Then state the discrete-time form used in our code (extract from `ugv_model.py` — note the unicycle simplification we actually use).
- **UAV model.** State the linear double-integrator form used for Tube MPC (extract from the code; this is the discrete-time `x_{k+1} = A_d x_k + B_d u_k + w_k` formulation). Define the state $[p_x, p_y, p_z, v_x, v_y, v_z]^T$ and the gravity-compensated acceleration command $[a_x, a_y, a_z]^T$.

### 3. Reference and disturbance models
Define the reference trajectory $\bar{x}_k$, the desired waypoints, and the disturbance set $\mathbb{W}$ as a bounded compact set. State assumptions on $\mathbb{W}$ explicitly (typically: convex, compact, contains the origin).

### 4. Constraints
- State constraints $\mathbb{X}$ (workspace bounds, altitude limits)
- Input constraints $\mathbb{U}$ (acceleration limits, steering limits)
- Field-of-view constraint (formal definition — what does "UGV in FoV" mean as a geometric condition involving UAV altitude, camera intrinsics, and UGV position?)
- Any other implicit constraints found in the code

### 5. Standard MPC formulation
Textbook nominal MPC for context. State the finite-horizon optimal control problem, the cost function structure, and the receding-horizon implementation. One page; cite a standard reference (Rawlings, Mayne & Diehl). The point is to show what we are *building on*.

### 6. Tube MPC — full derivation from basics
This is one of the two central sections. Derive everything:
- The nominal system $\bar{x}_{k+1} = A_d \bar{x}_k + B_d \bar{u}_k$
- The tracking error $e_k = x_k - \bar{x}_k$
- The feedback control law $u_k = \bar{u}_k - K e_k$
- The closed-loop error dynamics $e_{k+1} = (A_d - B_d K) e_k + w_k$, and the requirement that $(A_d - B_d K)$ be Schur stable
- How $K$ is chosen (discrete-time LQR — extract weights from code)
- The minimal Robust Positive Invariant set $\Omega$: definition, properties, why it exists
- Constraint tightening via Pontryagin difference: $\bar{\mathbb{X}} = \mathbb{X} \ominus \Omega$, $\bar{\mathbb{U}} = \mathbb{U} \ominus K\Omega$
- The Tube MPC optimization problem — full statement of the QP / NLP solved at each trigger

For each step, state the assumption being used and cite the seminal source (Mayne, Rakovic, Seron, Kerrigan for tube MPC; Rakovic for the mRPI computation specifically).

### 7. Event-triggered Tube MPC
Derive the event-triggering condition used in our code (extract from `uav_ancillary_node.py`):
- The set-membership trigger $\max(\Omega_H e_k - \Omega_h) > 0$
- The chattering-avoidance threshold ($n_{\text{thresh}}$ consecutive violations)
- The safety time-trigger ensuring recursive feasibility
- The four-level trigger system the code actually implements — derive or state each condition formally

State and briefly argue why event-triggering preserves the robust invariance guarantees of standard Tube MPC.

### 8. Extended Kalman Filter with view-quality coupling
- State the UGV motion model used in the EKF (unicycle, $[x, y, \theta, v, \omega]^T$)
- Derive the Jacobians $F_k = \partial f/\partial x$ and $H_k = \partial h/\partial x$ (extract from `ugv_model.py` and `ekf_node.py`)
- State the standard EKF predict and update equations
- Derive the view-quality-coupled measurement noise: $R_k = R_{\text{base}} \cdot (1 + \alpha \|d_{\text{pixel}}\|^2)$, where $d_{\text{pixel}}$ is the marker's pixel distance from frame center. Explain *why* this scaling is sensible (a single paragraph of intuition before the formula).

### 9. Belief-space MPC for the UGV
- Define the augmented state including covariance: $\xi_k = (x_k, \Sigma_k)$
- State the full UGV MPC cost function:
$$J = \sum_{k=0}^{N-1} \left( w_p \|x_k - x_k^{\text{ref}}\|^2 + w_s \|u_k - u_{k-1}\|^2 + w_\sigma\, \text{tr}(\Sigma_k) + w_{\text{fov}}\, c_{\text{fov}}(x_k) \right)$$
with each weight $w_\bullet$ extracted from the code. Define $c_{\text{fov}}(\cdot)$ explicitly — what penalty function is used? Soft barrier, quadratic, log-barrier?
- Discuss why each term exists (one paragraph per term).
- State the resulting NLP and the solver used (SLSQP).

### 10. View-quality metric and altitude adaptation
Derive the differentiable view-quality score:
$$q(d) = \left( \frac{1}{1 + \exp(\kappa(d - r_{\text{fov}}))} \right) \cdot \exp\!\left(-\frac{d^2}{2\sigma^2}\right)$$
- Explain each factor (sigmoid cutoff + Gaussian centering preference)
- Derive the relationship between altitude $z$ and effective FoV radius $r_{\text{fov}}$ from the pinhole camera model: $r_{\text{fov}} = z \tan(\eta / 2)$ where $\eta$ is the camera's vertical FoV
- State the UAV altitude-adaptation logic: how does the high-level UAV MPC use $q$ to decide when to climb?

### 11. Validation: stability, recursive feasibility, and ISS
This is the section where we have to be honest. Cover:
- **Recursive feasibility of Tube MPC.** State the standard theorem (if the initial state is feasible and $\Omega$ is correctly computed, the problem remains feasible at every step). Cite Mayne 2005 or Rakovic 2005.
- **Input-to-state stability (ISS).** State the standard Tube MPC ISS result for the closed-loop true state under bounded $w_k$.
- **Belief MPC stability.** Acknowledge that adding the covariance term to the cost does not automatically preserve standard MPC stability guarantees. Cite the relevant belief-space planning references (Platt 2010, van den Berg 2012). Be honest: we rely on the cost being a valid Lyapunov candidate under mild assumptions; we have not proven this formally.
- **Numerical validation.** Reference the experimental results from `report.html` (55% tracking error reduction, 87% visibility) as empirical evidence, not as formal proof.

### 12. Implementation notes
A short closing section noting deviations between idealized theory and actual code:
- Discretization step $\Delta t$
- MPC horizon $N$ (extract from code)
- Solver tolerances
- Linearization points for the EKF
- Any place the code approximates something the theory states exactly

## Format requirements

- **LaTeX preferred.** Use `\documentclass[11pt]{article}`, `amsmath`, `amssymb`, `mathtools`, `geometry` (1-inch margins), `hyperref`, `cleveref`, `natbib` for citations. Single column.
- **Fallback if LaTeX is not installed**: write a Markdown source with LaTeX math, then convert with `pandoc theory_report.md -o theory_report.pdf --pdf-engine=xelatex` or `weasyprint` if a PDF engine is available. If no PDF engine is available, save as `theory_report.html` with KaTeX and tell me.
- **Bibliography.** Include real citations for: ECC 2024 (Ferreira Santos et al.), Mayne et al. on Tube MPC, Rakovic on mRPI sets, Platt 2010 and van den Berg 2012 on belief-space planning, Rawlings/Mayne/Diehl textbook for standard MPC. Use BibTeX entries you can verify; do not fabricate DOIs.
- **Diagrams.** Use TikZ for any block diagrams (controller architecture, tube concept). Keep them minimal — function over decoration.
- **Length.** Estimate 20–35 pages. Do not pad. If a section is naturally short (e.g. notation table), keep it short.

## Rules

- **Derive from basics.** If you compress a derivation, state explicitly: *"For brevity, the standard derivation of [X] is omitted; see Mayne et al. (2005)."* Do not silently skip steps.
- **Match math to code.** Where the implementation deviates from the idealized derivation, document the deviation in §12 (Implementation notes). Do not hide approximations.
- **No fabricated validation.** If we have not proven a property, do not write "we have proven." Use phrases like *"follows from standard arguments under assumptions A1–A3"* or *"empirically validated but not formally proven."*
- **No marketing language.** This is a technical document. No "novel," no "powerful," no "innovative." State what we do and what guarantees we have.
- **Variables defined before use.** Every symbol introduced in an equation must be defined in §1 (notation table) and on first use in prose.
- **Cite ECC where you use their setup.** The UGV dynamics and the perception-less paradigm are theirs.

When done, print the path to `theory_report.pdf`, the .tex/.md source path, the total page count, and a one-line summary of which sections are complete vs. which sections had gaps (e.g. "§11 recursive feasibility relies on cited theorem only; we have not numerically verified mRPI computation").

## Priority

Sections in priority order if time or context is short: §6 (Tube MPC derivation), §7 (event-triggered), §8 (EKF), §9 (belief MPC cost), §1 (notation), §11 (validation). The remaining sections fill out the document but are less critical.
