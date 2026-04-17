# Projective Dynamics

Projective Dynamics (Bouaziz et al. 2014) and its position-based-dynamics descendant XPBD (Macklin et al. 2016) are the dominant time-integration schemes in the real-time soft-body literature. Both are backed by production implementations (NVIDIA Flex, PhysX, most game-engine cloth modules), both are honest research contributions, and both are considerably faster per-step than the [backward-Euler Newton scheme of Ch 00](00-backward-euler.md). This chapter explains why `sim-soft` rejects them anyway — not because they are slower (they are not) or less polished (they are more) or less documented (they are better), but because their *convergence theory* is incompatible with the rest of the stack the [Ch 03 thesis](../10-physical/03-thesis.md) commits to. Cheap and polished is not the same as correct for this problem.

This is the honest-alternative-rejected chapter, parallel to [Part 2 Ch 02 (linear)](../20-materials/02-linear.md) and [Part 2 Ch 03 (corotational)](../20-materials/03-corotational.md). No `sim-soft` PD or XPBD implementation is planned; the chapter exists so a reader can see that the rejection is structural, not a hand-wave.

| Section | What it covers |
|---|---|
| [Constraint projections](01-projective/00-projections.md) | Local step: each constraint projects its sub-state onto its feasible manifold, independently and in parallel; global step: one fixed linear solve against a constant matrix |
| [Relation to XPBD](01-projective/01-xpbd.md) | Macklin et al.'s eXtended PBD as the Gauss-Seidel variant; compliance-based constraint stiffness; how XPBD generalizes PD to rigid bodies and cloth |
| [Convergence characteristics](01-projective/02-convergence.md) | The Liu et al. 2017 result: PD converges to a biased equilibrium for non-separable hyperelastic energies; iteration count, stiffness schedule, and variational gap |

## How PD/XPBD are supposed to work

The core trick is disarmingly simple. Every constraint — each tet's elastic energy, each collision pair's non-penetration — is written as a function that *projects* a local sub-state onto its feasible manifold. A neo-Hookean tet projects its current deformation gradient onto the manifold where the element is at rest. A collision constraint projects the two touching vertices onto the manifold where the gap is zero. Each projection is local, small, and embarrassingly parallel. After all projections run, a *global* step fuses the per-constraint targets into a consistent position update by minimizing

$$ \|x - x_\text{predictor}\|_M^2 + \sum_i \|A_i\, x - p_i\|^2 $$

where $p_i$ is constraint $i$'s projection target, $A_i$ is a selector, and $M$ is the lumped mass. The minimizer solves $(M + \sum_i A_i^T A_i)\, x = M\, x_\text{predictor} + \sum_i A_i^T\, p_i$. The matrix on the left is **constant across iterations and across timesteps** — material stiffness does not appear in it — so it can be factorized once at simulation startup and re-used forever. XPBD reformulates the same idea as Gauss-Seidel sweeps with per-constraint compliance, trading global coherence for even more parallelism. Both approaches yield per-step costs dominated by $O(\#\text{constraints})$ parallel kernel launches plus one cached matrix back-substitution.

The speed advantage is real and structural. On a 30k-tet scene, PD's per-step cost is 5–20× lower than Newton-on-Hessian at equal timestep size — not because Newton is badly implemented, but because PD *never forms the elastic tangent* and *never factorizes anything material-dependent*. It is the right answer to a narrower question than the one `sim-soft` is asking.

## Why sim-soft rejects it anyway

Four structural incompatibilities. The first is the decisive one; the other three compound.

### 1. The convergence target is biased for non-separable energies

Liu, Bouaziz, and Kavan 2017 formalized Projective Dynamics as a proximal method: the local-global iteration minimizes an **ADMM-style augmented objective** that agrees with the true potential energy *only for separable, quadratic-in-each-constraint energies.* Linear elasticity in the small-strain limit is separable in this sense. Corotational elasticity is approximately separable (separable after polar decomposition). Neo-Hookean hyperelasticity is **not** separable — its energy density $\Psi(F) = \tfrac{\mu}{2}(I_1 - 3) - \mu\, \log J + \tfrac{\lambda}{2}(\log J)^2$ couples the invariants of $F$ in a way that no finite set of per-element quadratic constraints can reproduce exactly.

The practical consequence is that PD converges not to the true neo-Hookean equilibrium but to a **biased equilibrium whose bias scales with the mismatch between the quadratic constraint energy and the true $\Psi(F)$.** The bias is zero for linear, small for corotational at moderate stretch, *structural* for neo-Hookean and growing super-linearly in stretch. Published PD benchmarks on silicone-stiffness cubes report converged-position errors in the 5–20% range at moderate-to-large stretch versus Newton-on-neo-Hookean oracles. The gap does not close with more iterations — it closes only by changing the constraint energy, i.e., by abandoning the quadratic local-step projection, which is exactly the property that made PD fast in the first place.

For an engineering-grade differentiable simulator whose [reward function](../10-physical/01-reward.md) reads per-element stress, this is disqualifying. The [Ch 03 thesis](../10-physical/03-thesis.md) ties visual-quality-plus-physical-correctness to solving the *true* energy; a solver that converges to a biased version of it has both the visual failure ("the silicone under-softens at finite strain") and the physical failure ("the stress field is systematically wrong by $O(\text{bias})$") that [Part 1 Ch 02](../10-physical/02-what-goes-wrong.md) indicts linear elasticity for. PD is linear-elasticity-plus-fancier-projections; it inherits linear elasticity's large-stretch errors structurally.

### 2. The projection step is not differentiable in any usable sense

PD's local step projects a per-constraint sub-state onto a feasible manifold via a closed-form projection (for cloth-like edge-length constraints) or a tiny inner solve (for neo-Hookean tets, Bouaziz et al.'s "constraint projection via SVD"). The closed-form projections have discontinuities at the manifold boundary — e.g., an edge-length constraint's projection sign-flips when the edge passes through zero length. The SVD-based tet projections have gradients defined everywhere in the interior of the positive-stretch cone but blow up as any singular value approaches zero.

The practical consequence is that PD gradients computed by autograd through the unrolled local-global iteration are (a) zero through most of the iteration when the projections have converged (saturation), (b) singular at any active constraint boundary, and (c) biased by the same Liu-2017 variational gap as the primal solve. The research literature that claims "XPBD is differentiable" (DiffTaichi XPBD, Warp-PBD) is reporting gradients that are empirically non-zero and empirically useful on their benchmarks; what it is *not* reporting is that those gradients are **not the gradients of the true potential energy** that the solver is nominally targeting. For a downstream optimizer doing Bayesian or gradient-based design, that distinction matters: the [optimization loop](../100-optimization/00-forward.md) descends a surface that is not the reward surface, and convergence to a local optimum of the wrong surface is the failure mode.

Energy-based Newton, by contrast, produces gradients at equilibrium via [the implicit function theorem](../60-differentiability/02-implicit-function.md). The IFT's precondition is that the *residual* is a smooth function of state — which is exactly what a scalar energy gives you and exactly what a projection-based iteration destroys. "Getting the physics right" and "getting the gradients" are the same prerequisite, not by coincidence; the [Ch 03 thesis](../10-physical/03-thesis.md) names the prerequisite, and PD is where the thesis is most concretely tested.

### 3. XPBD's compliance schedule is timestep- and iteration-count-coupled

Macklin et al.'s XPBD resolves PD's "what does stiffness mean at finite iteration count" problem by introducing a per-constraint *compliance* $\alpha = 1/k$ that is scaled by $\Delta t^2$ and by the iteration count to give a well-defined constraint-satisfaction limit. The formulation is mathematically clean, and for fixed $\Delta t$ and fixed iteration count it is self-consistent.

The consequence is that the effective material stiffness a user observes in an XPBD simulation is a function of $(\Delta t, n_\text{iter}, \alpha)$ jointly, not of material parameters alone. Changing $\Delta t$ for adaptive-timestep reasons (Ch 02) changes the effective stiffness; changing $n_\text{iter}$ for performance reasons changes the effective stiffness; the three knobs must be re-tuned together any time one changes. For a design tool where the user expects "silicone Shore 30A" to behave the same way across timestep ranges and across CPU/GPU backends, this coupling is operationally untenable. Ch 00's backward-Euler-on-total-potential-energy does not have this coupling: material stiffness is the material stiffness, $\Delta t$ affects only accuracy and stability, the knobs are independent.

### 4. The IPC integration story is incompatible

IPC contact ([Part 4](../40-contact/00-why-ipc.md)) encodes non-penetration as a $C^2$-smooth barrier term *in the potential energy*, with [adaptive stiffness](../40-contact/01-ipc-internals/01-adaptive-kappa.md) and [CCD-limited step lengths](../40-contact/01-ipc-internals/02-ccd.md). The barrier is not a constraint that projects to a manifold; it is a continuous energy contribution that diverges at zero gap and vanishes outside tolerance. It has no meaningful "projection" onto a feasible manifold — what would you project onto? — and no clean translation into XPBD's compliance formalism because the barrier's stiffness is not constant across the active set (it adapts per contact pair).

Research attempts to combine IPC-style barriers with XPBD exist but pay for it by either (a) reverting contact to penalty form while keeping elasticity as XPBD, which re-introduces the [popping failure from Part 1 Ch 02](../10-physical/02-what-goes-wrong/02-popping.md), or (b) wrapping the barrier in an ADMM-style outer loop that replicates Newton's per-step cost, losing the speed advantage that motivated XPBD in the first place. Neither is a satisfying architecture.

## What PD/XPBD are good for

The rejection above is scoped. PD and XPBD are genuinely the right answer for a class of problems `sim-soft` does not target:

- **Clothing and hair simulation in games.** Long-skinny constraint graphs, no requirement for finite-strain accuracy, no differentiability requirement, 60 FPS on mobile hardware — XPBD is the production answer.
- **Secondary motion on top of skeletal animation.** The character rig is correct; the softbody is decoration. A converged-stress bias in the 5–20% range is invisible when the rendered artifact is an authored animation curve with physics perturbations added.
- **VR demos and interactive pre-viz.** User experience threshold is "feels responsive"; numerical accuracy is not graded.
- **Large-scale cloth scenes where per-particle cost must stay in single-digit microseconds.** PD's per-step scaling dominates.

The common thread is that the final product's quality metric is not "does this match the measured neo-Hookean response of the material" but "does this look plausible at speed." For a simulator whose [reward](../10-physical/01-reward.md) directly reads per-element stress and feeds into an optimizer, the quality metric is different, and PD/XPBD's speed advantage does not compensate for the convergence bias.

## What this does NOT reject

Two ideas from PD/XPBD that `sim-soft` *does* keep, in non-load-bearing roles.

**Warm-starting.** XPBD's observation that constraint Lagrangians persist across timesteps is correct and applies equally to Newton-on-Hessian: `sim-soft`'s Newton initial guess is $x_n^{(0)} = \widehat x_n$ (inertial predictor), and the IPC active set from step $n-1$ is the initial active-set guess for step $n$. Cheap, uncontroversial, keeps iteration counts low.

**Parallel local evaluations.** Per-element elastic energy and per-contact barrier energy evaluation is embarrassingly parallel; [Part 8's GPU layout](../80-gpu/00-wgpu-layout.md) launches per-element kernels that look very much like PD's local step. The parallelism pattern is good; the fixed-matrix-global-solve that PD pairs it with is what this chapter rejects, not the per-element kernel.

## Relationship to the Ch 03 thesis

[Part 1 Ch 03's thesis](../10-physical/03-thesis.md) argues that the games/science split has dissolved and no soft-body stack built today should pick a side. This chapter is where the thesis is most honestly tested: **there is a real soft-body time integrator that is faster, better tooled, better documented, and chosen by almost every production game pipeline — and this book rejects it.** The rejection is not a rejection of the games tradition; it is a claim that PD/XPBD's specific *convergence structure* is a 2010s artifact of the cost constraint the thesis argues has expired. In 2026, on a consumer GPU, Newton-on-neo-Hookean-plus-IPC runs at interactive rates (Ch 00 + [Part 4 Ch 05](../40-contact/05-real-time.md) + [Part 8](../80-gpu/00-wgpu-layout.md)). The compromises PD was designed to make are no longer compromises `sim-soft` has to make, and so we do not make them.
