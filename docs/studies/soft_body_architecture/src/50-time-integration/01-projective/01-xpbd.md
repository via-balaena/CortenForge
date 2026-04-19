# Relation to XPBD

XPBD — eXtended Position-Based Dynamics — is [Macklin, Müller, and Chentanez's 2016 paper](../../appendices/00-references/05-time-integration.md#macklin-2016) extending Position-Based Dynamics (PBD, [Müller et al. 2007](../../appendices/00-references/05-time-integration.md#muller-2007)). **PBD is not Projective Dynamics.** They are architecturally distinct frameworks with overlapping intuitions — both prize parallelism and per-constraint local updates — but they evolved independently. XPBD sits in PBD's lineage, not in PD's. This leaf walks through the XPBD reformulation and names the structural reason `sim-soft` declines XPBD alongside PD.

## PBD's history in one paragraph

Müller, Heidelberger, Hennix, and Ratcliff's 2007 Position-Based Dynamics was the original position-space constraint solver: each integration step guesses the new position via an inertial predictor, then iteratively projects all constraints in a Gauss-Seidel sweep until positions satisfy them (or until the iteration budget runs out). The convergence was empirical; the observed stiffness was whatever the combination of iteration count and $\Delta t$ happened to produce — a coupling that PBD's paper openly acknowledged and that ties the user's stiffness specification to solver budget rather than to material parameters. It was the right answer for real-time game physics in 2007 (cloth in PhysX, Havok) and the wrong answer for anything whose stiffness the user wanted to specify in material units.

## What XPBD adds

[Macklin et al. 2016](../../appendices/00-references/05-time-integration.md#macklin-2016)'s central contribution is to reformulate the constraint solve as a Lagrange-multiplier update with **compliance regularization**. For a scalar constraint $C_j(x) = 0$ with target stiffness $k_j$, XPBD defines the compliance $\alpha_j = 1/k_j$ (inverse stiffness) and the $\Delta t^2$-scaled compliance $\tilde\alpha_j = \alpha_j / \Delta t^2$. The per-constraint Lagrange-multiplier update is

$$\Delta \lambda_j \;=\; \frac{-C_j(x) - \tilde\alpha_j\, \lambda_j}{\nabla C_j^T\, M^{-1}\, \nabla C_j + \tilde\alpha_j},$$

and the position correction follows: $\Delta x = M^{-1}\,\nabla C_j^T\, \Delta\lambda_j$. Sweeping through all constraints once per outer iteration forms the Gauss-Seidel loop. The key property is that the converged state, at any iteration count past the Gauss-Seidel convergence plateau, is a well-defined function of $\alpha_j$ and the current $\Delta t$ — **iteration-count independence is XPBD's headline result relative to PBD's ambiguous stiffness**. The paper reports cloth error dropping from 6% to 2% to 0.5% at 50, 100, and 1000 iterations respectively, converging in iteration count rather than drifting with it as PBD did.

XPBD is structurally different from PD: PD's global step is a single linear solve against a prefactored matrix, while XPBD runs Gauss-Seidel sweeps with no global factorization. The compliance $\tilde\alpha_j$ exists to decouple stiffness from iteration count; it plays no role in PD, which had no such coupling to remove (PD converges to the minimum of its own augmented objective regardless of iteration count).

A secondary compliance-like parameter $\beta_j$ enters XPBD's velocity-level update for Rayleigh-style damping — a block-diagonal coefficient rather than an inverse stiffness, distinct from $\alpha_j$ in its algebraic role but introduced for the same design motive (make the damping specification iteration-count independent).

## Two things XPBD does NOT claim

Two narrower scope claims worth flagging because the wider versions occasionally appear in secondary sources:

- **XPBD is not rigid-body-ready in 2016.** The original paper targets deformable bodies — cloth, soft bodies, mass-spring networks. A rigid-body extension arrived later (Müller et al. 2020, *Detailed Rigid Body Simulation using Extended Position Based Dynamics*) and is a follow-on contribution built on top of the 2016 framework rather than part of it.
- **XPBD extends PBD, not PD.** The looser phrasing "XPBD generalizes PD to rigid bodies and cloth" conflates PBD and PD. The two frameworks are parallel lineages — PD comes from the [Bouaziz 2014](../../appendices/00-references/05-time-integration.md#bouaziz-2014) variational-optimization line, PBD from the [Müller 2007](../../appendices/00-references/05-time-integration.md#muller-2007) constraint-projection line — and XPBD is a PBD extension.

## Why sim-soft rejects XPBD

The rejection is *related to* the PD rejection but distinct in its mechanics.

### Same expressiveness limit as vanilla PD

XPBD's constraint $C_j(x)$ is a scalar function of state, and the effective constraint "energy" is an augmented-Lagrangian form $\tfrac{\tilde\alpha_j}{2}\lambda_j^2 + \lambda_j\, C_j(x)$ — compliance-weighted regularization. For cloth and mass-spring, $C_j$ is a natural match (edge length, bending angle). For neo-Hookean or Ogden, XPBD extensions define per-tet constraints on deformation-gradient singular values or invariants, but the underlying solver is still regularized-quadratic in $C_j$ and the same architectural restriction applies: the material is handled iteratively through constraint regularization rather than by minimizing the true hyperelastic potential. `sim-soft`'s [Part 1 Ch 03 thesis](../../10-physical/03-thesis.md) commits to the true potential, not a compliance-regularized proxy.

### Effective stiffness is Δt-coupled

XPBD removed PBD's iteration-count dependence — but it did not remove timestep dependence. $\tilde\alpha_j = \alpha_j / \Delta t^2$ means the effective stiffness a user observes scales with the current $\Delta t$: doubling $\Delta t$ quarters $\tilde\alpha_j$ and multiplies the effective stiffness by four. Changing $\Delta t$ for adaptive-timestep reasons ([Ch 02](../02-adaptive-dt.md)) therefore changes the effective stiffness in lock-step, and the user must retune $\alpha_j$ whenever $\Delta t$ changes. For a design tool where the user expects "silicone Shore 30A" to behave the same across timestep ranges and across CPU/GPU backends, this coupling is operationally untenable. Ch 00's backward-Euler-on-total-potential does not have this coupling: material stiffness is material stiffness, $\Delta t$ affects only accuracy and stability, the two knobs are independent.

### No IPC integration story

IPC's barrier ([Part 4 Ch 01 §00](../../40-contact/01-ipc-internals/00-barrier.md)) is a continuous energy contribution that diverges at zero gap and vanishes outside tolerance $\hat d$. It has no well-defined projection onto a feasible manifold — what would you project onto? — and no clean fit with XPBD's compliance formalism, because the barrier's effective stiffness $\kappa$ varies per-pair through the [adaptive-κ machinery of Part 4 Ch 01 §01](../../40-contact/01-ipc-internals/01-adaptive-kappa.md), while XPBD's compliance is constant per constraint. **No published XPBD+IPC integration exists**: [Lan 2022](../../appendices/00-references/00-ipc.md#lan-2022) combines *Projective Dynamics* with IPC (not XPBD), and [GIPC 2024](../../appendices/00-references/00-ipc.md#gipc-2024) and [StiffGIPC 2025](../../appendices/00-references/00-ipc.md#stiffgipc-2025) are Newton-on-Hessian GPU IPC implementations. The architectural incompatibility is the reason: IPC lives in the potential energy; XPBD lives in the constraint Lagrangian.

## Where XPBD IS the right answer

The rejection is scoped. XPBD remains the production answer for:

- **Real-time cloth in games** (Macklin et al.'s original target — shipping in PhysX, Unity's XPBD cloth solver, most modern game-engine cloth modules).
- **Hair and rope simulation** where the constraint graph is long and skinny, differentiability is not required, and frame budgets are sub-millisecond.
- **VR demos and interactive pre-viz** where "feels responsive" is the quality metric rather than "matches measured material response."

The common thread is the [same as for PD](00-projections.md#scope): real-time simulation where the final product's quality metric is visual plausibility at speed, not convergence to a measured material law. For a simulator whose [reward](../../10-physical/01-reward.md) directly reads per-element stress and feeds into a downstream optimizer, the quality metric is different — and XPBD's speed advantage does not compensate for the expressiveness limit.

Next: [02-convergence](02-convergence.md) — what happens when PD (or XPBD) is pushed to hyperelastic via the 2017 extensions, why those extensions do not close the rejection gap, and what "XPBD is differentiable" claims actually deliver.
