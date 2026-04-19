# Time-integration papers

Two clusters of prior art. The first is the alternative time-integration schemes [Part 5 Ch 01](../../50-time-integration/01-projective.md) works through and rejects — Projective Dynamics, Position-Based Dynamics, and their extensions. PD and PBD are parallel lineages with overlapping intuitions but architecturally distinct solvers; the 2017 extensions (Liu quasi-Newton, Overby ADMM) close PD's expressiveness gap for hyperelastic at an architectural cost. The second is the adaptive-stepping control-theory literature [Part 5 Ch 02](../../50-time-integration/02-adaptive-dt.md) draws on for the halve-on-failure asymmetry discussion. Each anchor below is traced to the specific claim the cluster uses it for.

## PD, PBD, and extensions

## Bouaziz et al. 2014 {#bouaziz-2014}

*Projective Dynamics: Fusing Constraint Projections for Fast Simulation.* ACM Transactions on Graphics 33(4), Article 154 (SIGGRAPH 2014). Authors: Sofien Bouaziz (EPFL), Sebastian Martin (U Penn), Tiantian Liu (U Penn / Utah), Ladislav Kavan (U Penn / Utah), Mark Pauly (EPFL). DOI [10.1145/2601097.2601116](https://doi.org/10.1145/2601097.2601116).

The original Projective Dynamics paper. Introduces the local-global iteration: each constraint projects its sub-state onto its feasible manifold (local step, embarrassingly parallel), then one linear solve fuses per-constraint targets into a consistent global update (global step). The key architectural property is that the global-solve matrix $M + \sum_i A_i^T A_i$ is **constant across iterations and across timesteps** — sparse-Cholesky-factorized once at setup and re-used forever. Scope in the 2014 paper: cloth (edge-length, bending), mass-spring, As-Rigid-As-Possible (ARAP) FEM with SVD-based closest-rotation projection. Neo-Hookean hyperelasticity is **not** handled in the 2014 paper; that extension arrived in [Liu et al. 2017](#liu-2017). Cited inline from [Part 5 Ch 01 §00](../../50-time-integration/01-projective/00-projections.md).

## Müller et al. 2007 {#muller-2007}

*Position Based Dynamics.* Journal of Visual Communication and Image Representation 18(2), 2007. Authors: Matthias Müller, Bruno Heidelberger, Marcus Hennix, John Ratcliff.

The original Position-Based Dynamics paper. Architectural ancestor of XPBD, and a **separate framework from Projective Dynamics** — PBD comes from the Müller constraint-projection line (each step guesses positions via inertial prediction, then sweeps Gauss-Seidel constraint projections until positions satisfy them), while PD comes from the Bouaziz variational-optimization line. PBD's stiffness was iteration-count- and timestep-dependent, a coupling the user had to manage empirically. Cited inline from [Part 5 Ch 01 §01 xpbd](../../50-time-integration/01-projective/01-xpbd.md) for the historical "what XPBD extended" context; the study's rejection argument operates on XPBD, not PBD directly.

## Macklin et al. 2016 {#macklin-2016}

*XPBD: Position-Based Simulation of Compliant Constrained Dynamics.* Motion in Games (MIG 2016), pp. 49–54. Authors: Miles Macklin, Matthias Müller, Nuttapong Chentanez (all NVIDIA). DOI [10.1145/2994258.2994272](https://doi.org/10.1145/2994258.2994272).

The XPBD paper. Extends Position-Based Dynamics (PBD) with compliance regularization: a per-constraint inverse-stiffness $\alpha_j = 1/k_j$ enters the Lagrange-multiplier update as a $\Delta t^2$-scaled term $\tilde\alpha_j = \alpha_j/\Delta t^2$. The central architectural contribution is **removing PBD's iteration-count dependence** — at fixed $\Delta t$ and sufficient Gauss-Seidel sweeps, the converged state is a well-defined function of $\alpha_j$ alone. The Lagrange-multiplier update is $\Delta\lambda_j = -(C_j(x) + \tilde\alpha_j \lambda_j) / (\nabla C_j^T M^{-1} \nabla C_j + \tilde\alpha_j)$. $\Delta t$ dependence is structural and *not* removed by XPBD. Rigid-body extension came later (Müller et al. 2020, "Detailed Rigid Body Simulation using Extended Position Based Dynamics"), not in the 2016 paper. Cited inline from [Part 5 Ch 01 §01 xpbd](../../50-time-integration/01-projective/01-xpbd.md).

## Liu et al. 2017 {#liu-2017}

*Quasi-Newton Methods for Real-Time Simulation of Hyperelastic Materials.* ACM Transactions on Graphics 36(3), Article 23 (2017). Authors: Tiantian Liu (U Utah), Sofien Bouaziz (EPFL), Ladislav Kavan (U Utah). DOI [10.1145/3072959.2990496](https://doi.org/10.1145/3072959.2990496). arXiv preprint [1604.07378](https://arxiv.org/abs/1604.07378) (2016).

Reinterprets Projective Dynamics as a **quasi-Newton method** on the true hyperelastic potential and accelerates it with L-BFGS. The quasi-Newton Hessian approximation is the constant PD matrix $M + \sum_i A_i^T A_i$ — exactly the matrix PD prefactors at setup — so each L-BFGS step reuses the cached factor. **Extends PD to handle neo-Hookean and spline-based materials** that vanilla Bouaziz 2014 could not express. Reported speedups are typically several times faster per iterate than full Newton on the same materials. The extension closes PD's expressiveness gap for hyperelastic but at the architectural cost that the approximation is no longer exact and multiple L-BFGS iterates are needed per step for stiff non-separable materials. Cited inline from [Part 5 Ch 01 §02 convergence](../../50-time-integration/01-projective/02-convergence.md) as one of the two parallel 2017 reframings.

## Overby et al. 2017 {#overby-2017}

*ADMM ⊇ Projective Dynamics: Fast Simulation of Hyperelastic Models with Dynamic Constraints.* IEEE Transactions on Visualization and Computer Graphics 23(10), pp. 2222–2234, 2017. Authors: Matthew Overby, George E. Brown, Jie Li, Rahul Narain (U Minnesota). DOI [10.1109/TVCG.2017.2730875](https://doi.org/10.1109/TVCG.2017.2730875). Extended journal version of the 2016 SCA paper (*ADMM ⊇ Projective Dynamics: Fast Simulation of General Constitutive Models*).

Shows that Projective Dynamics is a **special case of ADMM** (Alternating Direction Method of Multipliers), and that the generalization to full ADMM recovers general hyperelastic energies including neo-Hookean. ADMM's standard convergence theory (Boyd et al. 2011) then applies: convergence to the true energy's minimizer under mild conditions, including for non-separable hyperelastic. The architectural cost is the ADMM outer loop, which for general constitutive models adds per-step work beyond vanilla PD's "one prefactored matrix, back-sub forever" accounting. Complementary to [Liu 2017](#liu-2017) rather than competing — both papers independently extend PD to hyperelastic, via different optimization-theoretic reframings. Cited inline from [Part 5 Ch 01 §02 convergence](../../50-time-integration/01-projective/02-convergence.md) as the ADMM reframing.

## Adaptive stepping

## Gustafsson, Lundh, Söderlind 1988 {#gustafsson-soderlind-1988}

*A PI Stepsize Control for the Numerical Solution of Ordinary Differential Equations.* BIT Numerical Mathematics 28(2), pp. 270–287, June 1988. Authors: Kjell Gustafsson, Michael Lundh, Gustaf Söderlind (Lund University). DOI [10.1007/BF01934091](https://doi.org/10.1007/BF01934091).

The origin of the **PI (proportional-integral) step-size controller** for ODE solvers. Treats adaptive step-size selection as a feedback-control problem: the predicted step size for iterate $n+1$ combines the current local-error ratio with the previous iterate's ratio, smoothing the noise that the simpler single-ratio step-size schemes exhibit on stiff and near-stiff problems. The control-theoretic framing is the contribution; the specific tuning of the PI gains is not load-bearing. Cited inline from [Part 5 Ch 02 §01 ccd-shrink](../../50-time-integration/02-adaptive-dt/01-ccd-shrink.md) in the alternatives-rejected discussion of smooth step controllers — `sim-soft` rejects PI/PID in favor of binary halve-on-failure because the contact-dominated failure mode is categorical (CCD-clipping) rather than a smoothly-varying error norm.

## Söderlind 2002 {#soderlind-2002}

*Automatic Control and Adaptive Time-Stepping.* Numerical Algorithms 31(1–4), pp. 281–310, 2002. Author: Gustaf Söderlind (Lund University). DOI [10.1023/A:1021160023092](https://doi.org/10.1023/A:1021160023092).

Extended control-theoretic treatment of adaptive stepping for ODE and DAE solvers. Builds on [Gustafsson-Söderlind 1988](#gustafsson-soderlind-1988) with stability analysis of the coupled discretization-controller system, applicability to stiff and A-stable schemes, and a design-of-controllers framework. Söderlind's 2003 follow-up in ACM TOMS extends this to PID controllers and digital-filter-based step-size selection (not separately anchored here; referenced inline if needed). The 2002 paper is the most comprehensive single-source reference for the control-theoretic perspective on adaptive stepping; cited inline from [Part 5 Ch 02 §01 ccd-shrink](../../50-time-integration/02-adaptive-dt/01-ccd-shrink.md) as the secondary framework reference.

## Hairer, Nørsett, Wanner (Vol I) and Hairer, Wanner (Vol II) {#hairer-wanner}

*Solving Ordinary Differential Equations I: Nonstiff Problems.* Springer Series in Computational Mathematics 8, 2nd revised ed. 2009. Authors: Ernst Hairer, Syvert P. Nørsett, Gerhard Wanner. ISBN 978-3-642-05163-0.

*Solving Ordinary Differential Equations II: Stiff and Differential-Algebraic Problems.* Springer Series in Computational Mathematics 14, 2nd revised ed. 1996. Authors: Ernst Hairer, Gerhard Wanner. ISBN 978-3-540-60452-5. (Nørsett is not a Vol-II author.)

The canonical textbook treatment of Runge-Kutta methods, embedded pairs, automatic step-size control, and — in Vol II — stiff-solver adaptive stepping via stage-based error estimators (Radau IIA and related implicit RK families). Referenced as the canonical background for adaptive step-size control broadly; the specific "safety factor" terminology (typically 0.8–0.9 in implementations) is widespread across the adaptive-stepping literature without a single coined-term origin, and this anchor stands in for the textbook family in which that heuristic is at home. Cited inline from [Part 5 Ch 02 §01 ccd-shrink](../../50-time-integration/02-adaptive-dt/01-ccd-shrink.md) as the textbook background, distinct from the specific `sim-soft` binary-halving scheme.

## Fehlberg 1969 {#fehlberg-1969}

*Low-Order Classical Runge-Kutta Formulas with Stepsize Control and Their Application to Some Heat Transfer Problems.* NASA Technical Report R-315, July 1969. Author: Erwin Fehlberg (NASA Marshall Space Flight Center). [NTRS 19690021375](https://ntrs.nasa.gov/citations/19690021375).

The canonical RKF4(5) paper. Introduces the embedded-RK idea: two Runge-Kutta solutions of different orders (4 and 5) share the same stage evaluations, so their difference gives a cheap local-truncation-error estimate that drives adaptive step selection. Explicit method, applicable only to non-stiff IVPs with smooth right-hand sides. A precursor 1968 NASA TR R-287 covers higher-order Fehlberg variants (5-8); the 1969 R-315 is the RKF4(5) reference proper. Cited inline from [Part 5 Ch 02 §01 ccd-shrink](../../50-time-integration/02-adaptive-dt/01-ccd-shrink.md) as a rejected-alternative example — the embedded-error-estimation framework does not extend to the contact-barrier-dominated regime where accuracy is limited by contact geometry and CCD topology, not by elasticity-side truncation error.

## Pass 3 anchors (not yet inline-cited)

Reserved slots for works a Pass 3 bibliography expansion may populate: Müller et al. 2020 (*Detailed Rigid Body Simulation using Extended Position Based Dynamics*) if a future chapter references the XPBD rigid-body extension directly; and any more recent 2024–2025 XPBD follow-ups the study references by anchor.
