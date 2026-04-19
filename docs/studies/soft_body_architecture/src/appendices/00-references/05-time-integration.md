# Time-integration papers

Projective Dynamics, Position-Based Dynamics, and their extensions — the alternative time-integration schemes [Part 5 Ch 01](../../50-time-integration/01-projective.md) works through and rejects. PD and PBD are parallel lineages with overlapping intuitions but architecturally distinct solvers; the 2017 extensions (Liu quasi-Newton, Overby ADMM) close PD's expressiveness gap for hyperelastic at an architectural cost. Each anchor below is traced to the specific claim [Part 5 Ch 01](../../50-time-integration/01-projective.md) uses it for.

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

## Pass 3 anchors (not yet inline-cited)

Reserved slots for works a Pass 3 bibliography expansion may populate: Müller et al. 2020 (*Detailed Rigid Body Simulation using Extended Position Based Dynamics*) if a future chapter references the XPBD rigid-body extension directly; and any more recent 2024–2025 XPBD follow-ups the study references by anchor.
