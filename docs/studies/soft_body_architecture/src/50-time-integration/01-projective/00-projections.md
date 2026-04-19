# Constraint projections

Projective Dynamics' architectural trick is to split each integration step into two alternating phases. The **local step** projects every constraint's sub-state onto its feasible manifold *independently and in parallel* — edge-length constraints snap to their rest length, FEM-tet ARAP constraints snap the deformation gradient $F$ to its closest rotation, positional constraints snap to their pinned target. The **global step** fuses the per-constraint projection targets into a consistent global position update by solving one linear system whose left-hand-side matrix is **constant across iterations and across timesteps**. This leaf walks through both steps and names the architectural restriction the framework pays for the speed: the constraint energy must be quadratic in state, which is fine for cloth and ARAP-FEM and not fine for neo-Hookean hyperelasticity.

## Local step

[Bouaziz et al. 2014](../../appendices/00-references/05-time-integration.md#bouaziz-2014) formulates each constraint as a projection function. For a constraint $c_i$ acting on a selector-image $A_i\,x$ of the global state, the projection $p_i = \Pi_{c_i}(A_i\,x)$ finds the point on $c_i$'s feasible manifold closest to $A_i\,x$ in some constraint-local metric. The projections are per-constraint and embarrassingly parallel — the 2014 paper's implementation is CPU-multithreaded, and follow-on GPU work (notably [Lan 2022](../../appendices/00-references/00-ipc.md#lan-2022)) dispatches one kernel per constraint type to handle all instances of that type in parallel.

The cases the 2014 paper ships are:

- **Edge-length constraints** (cloth, mass-spring): $p_i$ is the endpoint pair rescaled to rest length $L_0$. Closed-form projection.
- **As-Rigid-As-Possible (ARAP) FEM-tet constraints**: $p_i$ is the closest rotation $R$ of the deformation gradient $F$, found via a single 3×3 SVD per tet ($F = U\Sigma V^T \Rightarrow R = UV^T$). Closed-form projection.
- **Positional / pin constraints**: $p_i$ is the pinned target.
- **Bending constraints** (cloth, thin shells): $p_i$ minimizes a quadratic bending energy against the rest dihedral.

Each projection is **small** (3×3 SVD for tets; scalar rescale for edges) and **local** (reads only the constraint's own degrees of freedom). The 2014 paper's scope does **not** include neo-Hookean hyperelasticity — that extension arrived in the 2017 follow-ons covered in [02-convergence](02-convergence.md).

## Global step

After all projections run, the global step solves one linear system for the consistent global position update. The backward-Euler variational principle is

$$x^{(k+1)} \;=\; \arg\min_x\; \bigl\|x - \widehat x\bigr\|_M^2 \;+\; \sum_i \bigl\|A_i\,x - p_i^{(k)}\bigr\|^2,$$

where $\widehat x$ is the inertial predictor from [Ch 00 §01 newton](../00-backward-euler/01-newton.md), $p_i^{(k)}$ is constraint $i$'s current local-step target, and the first term is the inertial residual weighted by the lumped mass $M$. The minimizer satisfies the linear system

$$\Bigl(M + \sum_i A_i^T A_i\Bigr)\, x^{(k+1)} \;=\; M\,\widehat x \;+\; \sum_i A_i^T\, p_i^{(k)}.$$

The **left-hand matrix is constant across iterations and across timesteps**. $M$ is the lumped mass — a diagonal matrix that doesn't change unless the mesh is remeshed. The $\sum_i A_i^T A_i$ term is a selector-product sum that encodes the *topology* of the constraint graph but not the *material stiffness*: a tet's $A_i^T A_i$ contribution is determined by which four vertices the tet is incident to, not by the Lamé parameters. The global matrix can therefore be sparse-Cholesky-factorized *once at simulation setup* and the factor re-used forever.

This is the architectural claim the PD speed advantage rests on: **per-step cost is dominated by parallel local-step kernels plus one sparse back-substitution against a cached factor**. No Jacobian assembly, no re-factorization, no line search.

## Scope: what kind of energies this expresses

The global variational principle's per-constraint term $\|A_i\,x - p_i\|^2$ says the constraint energy is **quadratic in the state** and **separable** across constraints. This is exactly right for:

- Linear / small-strain elasticity ($\varepsilon$ is linear in $x$, and the energy is quadratic).
- ARAP-FEM ($F$ is linear in $x$, and $\|F - R\|_F^2$ is quadratic at fixed $R$; the polar decomposition is the local-step update).
- Mass-spring and cloth (edge length is quadratic in endpoints at a fixed projection target).

It is **structurally insufficient** for neo-Hookean, Ogden, or Mooney-Rivlin hyperelasticity — their energy densities couple invariants of $F$ nonlinearly (neo-Hookean's $\tfrac{\lambda}{2}(\ln J)^2$ term, for instance, is nonlinear in $x$ even at a fixed local-step target). Two parallel 2017 works — [Liu, Bouaziz, Kavan's quasi-Newton reframing](../../appendices/00-references/05-time-integration.md#liu-2017) and [Overby, Brown, Li, Narain's ADMM reframing](../../appendices/00-references/05-time-integration.md#overby-2017) — independently extended the PD architecture to handle non-separable hyperelastic, each at an architectural cost. [02-convergence](02-convergence.md) walks through both and argues neither closes the rejection gap for `sim-soft`.

## What this commits

- The local step sets `sim-soft`'s [Part 8 GPU layout](../../80-gpu/00-wgpu-layout.md) precedent: per-element kernels are the right parallelism pattern — PD got that part right, and Ch 05's [real-time IPC pipeline](../../40-contact/05-real-time.md) reuses the idea in Newton form.
- The global step's constant-matrix property is the specific architectural property `sim-soft` is *not* adopting. Our Newton iteration re-assembles $H$ on every iterate because $H$ is material-dependent and IPC-adaptive — that re-assembly is what [Part 4 Ch 01's barrier-stiffness adaptation](../../40-contact/01-ipc-internals/01-adaptive-kappa.md) requires, and it is the structural reason PD's prefactored-matrix architecture does not fit.

Next: [01-xpbd](01-xpbd.md) walks through XPBD — a separate framework from PD that shares the parallel-local intuition but rebuilds the solver on compliance-regularized Lagrange multiplier updates rather than on variational global solves.
