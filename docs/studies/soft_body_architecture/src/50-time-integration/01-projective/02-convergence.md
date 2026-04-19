# Convergence characteristics

Vanilla Projective Dynamics' quadratic-constraint-energy framework is [structurally restricted to energies separable and quadratic in state](00-projections.md#scope) — exactly right for cloth and ARAP-FEM, exactly wrong for neo-Hookean. Two parallel 2017 works reformulated PD to close that gap, each at a different architectural cost. This leaf summarizes both extensions, names the cost, then argues that even with the extensions applied PD does not meet `sim-soft`'s [Ch 03 thesis](../../10-physical/03-thesis.md) — on gradient quality at the differentiability boundary, and on integration with IPC contact.

## Two 2017 reframings of PD

**Liu, Bouaziz, Kavan 2017 — quasi-Newton.** [*Quasi-Newton Methods for Real-Time Simulation of Hyperelastic Materials*](../../appendices/00-references/05-time-integration.md#liu-2017) (ACM TOG 36(3), 2017). Reinterprets the PD local-global iteration as a **quasi-Newton method** on the true hyperelastic potential, and accelerates it with L-BFGS. The quasi-Newton Hessian approximation is the constant PD matrix $M + \sum_i A_i^T A_i$ — exactly the matrix PD prefactors at setup — so each L-BFGS step reuses the cached factor. **Extends PD to handle neo-Hookean, spline-based, and polynomial materials** that vanilla [Bouaziz 2014](../../appendices/00-references/05-time-integration.md#bouaziz-2014) cannot express. Reported to be several times faster per iterate than full Newton on the same materials.

**Overby, Brown, Li, Narain 2017 — ADMM.** [*ADMM ⊇ Projective Dynamics: Fast Simulation of Hyperelastic Models with Dynamic Constraints*](../../appendices/00-references/05-time-integration.md#overby-2017) (IEEE TVCG 23(10), 2017; extended journal version of a 2016 SCA paper). Shows that Projective Dynamics is a **special case of ADMM** (Alternating Direction Method of Multipliers), and that the generalization to full ADMM recovers general hyperelastic energies including neo-Hookean. ADMM's standard convergence theory (Boyd et al. 2011) then applies: convergence to the true energy's minimizer under mild conditions, including for non-separable materials.

**Both extensions close the expressiveness gap.** Neither restores the clean "one prefactored matrix, no re-assembly, no iteration-count budget" architecture PD had for the separable case:

- Liu's quasi-Newton reuses the PD matrix as an approximate Hessian. Cheaper than full Newton per iterate, but the quasi-Newton approximation is no longer exact for non-separable materials, and more L-BFGS iterates are needed per step on stiff problems. The architectural payoff per iterate remains — cached factor, no re-assembly — but total per-step cost rises with material stiffness.
- Overby's ADMM introduces a proximal outer loop. The inner ADMM primal update retains PD's cached-factor advantage, but the outer loop adds per-step work that vanilla PD did not have — eroding the constant-time-per-step architectural advantage. The exact cost ratio relative to Newton-on-Hessian depends on constitutive model and implementation; the structural point is that ADMM-on-PD is not the same fast-and-prefactored operation as vanilla PD.

Neither 2017 paper reports pathological divergence from the true hyperelastic equilibrium — both close the gap in the limit of sufficient iterates. The cost is in the iterate budget, not in the converged solution's fidelity.

## The differentiability problem extensions don't fix

PD's local step, in all variants (vanilla, quasi-Newton-augmented, ADMM-augmented), projects onto per-constraint feasible manifolds via closed-form or SVD-based operations. The projection is **non-differentiable** at the manifold boundary:

- Edge-length projections sign-flip when the edge passes through zero length.
- SVD-based tet projections have gradients defined in the interior of the positive-stretch cone but diverge as any singular value approaches zero.
- Collision-pair projections are discontinuous at first contact and at separation.

For a downstream optimizer doing Bayesian or gradient-based design over material fields ([Part 10](../../100-optimization/00-forward.md)), gradients computed by autograd through an unrolled local-global iteration are (a) zero through most of the iteration once projections have converged, (b) singular at any active manifold boundary, and (c) biased by the quasi-Newton or ADMM approximation on non-converged iterates. This limitation is independent of which 2017 extension is used; the local-step non-differentiability is the common surface.

Energy-based Newton on the total potential, by contrast, produces gradients at equilibrium via the [implicit function theorem](../../60-differentiability/02-implicit-function.md). The IFT's preconditions are (i) the residual is a smooth function of state, and (ii) the residual's Jacobian is nonsingular at the solution — both of which a scalar potential with SPD Hessian delivers, and both of which a projection-based iteration destroys at the manifold boundaries.

"Getting the physics right" and "getting the gradients" are the same prerequisite in this framing, not by coincidence; PD is where that prerequisite is most concretely tested, and it is the test PD fails.

## What "XPBD is differentiable" actually means

Two named differentiable-XPBD implementations exist:

- [**DiffXPBD**](../../appendices/00-references/03-diff-sim.md#stuyck-2023) (Stuyck & Chen 2023, ACM TOG). Derives analytical gradients through the XPBD Lagrange-multiplier updates. The gradients are well-defined for optimization over XPBD's own compliance parameters ($\alpha_j$, damping $\beta_j$) — useful for inverse-simulation tasks framed in XPBD's own parameterization. They are **not** gradients of the true hyperelastic potential; they are gradients of the XPBD-objective surface, which approximates the potential up to the compliance-regularization error.
- [**NVIDIA Warp's XPBD integrator**](../../appendices/00-references/03-diff-sim.md#warp) (`warp.sim.IntegratorXPBD`). Ships gradients via Warp's autograd machinery. The framework's own issue tracker flags practical gradient limitations (joint-target gradients not yet differentiable, zero-gradient issues in cloth examples), and the gradients that do flow are, by the same architectural logic as DiffXPBD, gradients of the XPBD-objective surface rather than of the true potential.

Both are legitimate tools for what they are. For `sim-soft`'s [Part 10 optimization loop](../../100-optimization/00-forward.md), gradients of an approximate surface descend to a local minimum of the approximation, not of the physics — a failure mode [Part 6 Ch 00](../../60-differentiability/00-what-autograd-needs.md) names explicitly.

## Relationship to the Ch 03 thesis

[Part 1 Ch 03's thesis](../../10-physical/03-thesis.md) argues that the games/science split has dissolved and that no soft-body stack built today should pick a side. This chapter is where the thesis is most honestly tested: **there is a real soft-body time integrator — PD, XPBD, and their 2017 extensions — that is faster, better tooled, and chosen by almost every production game pipeline, and this book rejects it.** The rejection is not a rejection of the games tradition; it is a claim that the specific *convergence-and-differentiability structure* of PD/XPBD is a 2010s artifact of real-time cost constraints that the thesis argues have expired. In 2026, on a consumer GPU, [Newton-on-neo-Hookean-plus-IPC](../00-backward-euler.md) + [Part 4 Ch 05 real-time IPC](../../40-contact/05-real-time.md) + [Part 8's GPU layout](../../80-gpu/00-wgpu-layout.md) runs at interactive rates. The compromises PD was designed to make are no longer compromises `sim-soft` has to make, and so we do not make them.
