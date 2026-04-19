# Energy-based formulation

The [barrier function](00-barrier.md), the [adaptive stiffness schedule](01-adaptive-kappa.md), and the [CCD-filtered line search](02-ccd.md) are three components. This sub-leaf is the fourth and closes the composition: how the contact term enters the total potential energy the time integrator already minimizes, what primitive pairs contribute, and how the gradient and Hessian assemble from per-pair contributions. Once this integration is stated, every piece downstream — the Newton loop, sparse factorization, autograd VJP — inherits contact correctness automatically, because the contact term has become just another energy in the same minimization.

## The total potential with contact

Li et al. 2020's formulation writes the backward-Euler step as a minimization of:

$$ U_\text{total}(x) = \tfrac{1}{2}(x - \hat x)^T M (x - \hat x) + h^2 \Psi_\text{elastic}(x) + h^2 \kappa B(x) + h^2 D(v(x)) $$

where $M$ is the mass matrix, $\hat x = x_{t-1} + h\, v_{t-1} + h^2 M^{-1} f_\text{ext}$ is the inertial predictor for step $t$, $h = \Delta t$ is the timestep, $\Psi_\text{elastic}$ is the hyperelastic energy density integrated over the mesh, $B(x) = \sum_i b(d_i(x), \hat d)$ is the barrier sum over active pairs, and $D(v)$ is the [friction dissipation](../02-friction/01-smoothed.md) (deferred to Ch 02). The minimizer $x^\ast$ is the next timestep's state $x_t$.

This is the same backward-Euler structure as [Part 5 Ch 00](../../50-time-integration/00-backward-euler.md) with two new energy terms: the barrier $\kappa B(x)$ and the friction $D(v(x))$. Neither changes the shape of the Newton loop — the loop still solves $\nabla U_\text{total}(x^\ast) = 0$; both new terms contribute gradient and Hessian blocks that add to the existing elastic + inertial blocks.

## Primitive pairs

The barrier sum $B(x) = \sum_i b(d_i(x), \hat d)$ is over surface-mesh primitive pairs within gap distance $\hat d$. For 3D triangle meshes, the only two pair types are:

- **Point-triangle pairs.** A surface vertex $v$ and a non-incident triangle $t$; gap $d$ is the shortest distance from $v$ to the closest point in $t$ (interior or boundary). Same-mesh self-pairs are allowed; pairs where $v$ is a vertex of $t$ are excluded (those would give $d = 0$ spuriously).
- **Edge-edge pairs.** Two non-adjacent edges $e_1, e_2$; gap $d$ is the shortest segment-to-segment distance. Adjacency-based exclusions apply: edges sharing a vertex are excluded.

The [Part 4 Ch 03 proximity sub-chapter](../03-self-contact/01-proximity.md) formalizes the exclusion rules. Cross-mesh pairs (e.g., sleeve-vertex to probe-triangle) and self-pairs (same-mesh primitives on different elements) go through the same filtering; the barrier has no special case for self vs. cross — same algorithm, same energy shape, same gradient path.

Pairs with $d \ge \hat d$ contribute zero and are culled at the BVH broadphase (the [BVH sub-chapter](../03-self-contact/00-bvh.md)). Only pairs reported by the broadphase as within-$\hat d$ are evaluated in the sum above.

## Gradient and Hessian of the barrier term

The gradient $\nabla_x [\kappa B(x)]$ is a sum over active pairs of the chain-ruled derivative:

$$ \nabla_x \big[\kappa B(x)\big] = \kappa \sum_i b'(d_i) \, \nabla_x d_i(x) $$

where $b'(d)$ is the barrier derivative from [the barrier sub-chapter](00-barrier.md) and $\nabla_x d_i(x)$ is the distance-function gradient for pair $i$, a sparse vector with non-zero entries only at the nodes of the primitives involved in pair $i$. For a point-triangle pair, $\nabla_x d_i$ has four non-zero $\mathbb{R}^3$ blocks (one per triangle vertex plus the point); for an edge-edge pair, it has four (two per edge).

The Hessian is:

$$ \nabla^2_x \big[\kappa B(x)\big] = \kappa \sum_i \big[ b''(d_i) \, \nabla_x d_i \otimes \nabla_x d_i + b'(d_i) \, \nabla^2_x d_i \big] $$

Two terms per pair: a rank-1 outer product ($b'' \cdot \nabla d \otimes \nabla d$, which is always rank-1 regardless of primitive type) and a second-order distance-function Hessian ($b' \cdot \nabla^2 d$). Each pair contributes a sparse $12 \times 12$ block to the global Hessian — 4 vertex blocks of 3 DOFs each, either (triangle vertices plus the point) for a point-triangle pair or (both edges' endpoints) for an edge-edge pair. The [Part 8 Ch 01 BSR-3 layout](../../80-gpu/01-sparse-matrix/01-bsr.md) is sized for exactly this block structure — the $3 \times 3$ BSR block size fits both the element-level tangent contributions from hyperelasticity and the pair-level contact contributions from the barrier, with no separate data layout required for contact.

## Why the energy-based composition is the decisive property

A force- or impulse-based contact formulation supplies forces at contact pairs (penalty) or velocity corrections at contact events (impulse) and relies on the time integrator to enforce whatever continuity property it needs. The composition fails at the integrator level: the time integrator sees a non-smooth force or velocity history and has no way to reconcile it with an energy-minimizing objective. The energy-based formulation works in the opposite direction: contact supplies an energy, the time integrator composes it with the other energies, and every integrator-level property (Newton's convergence, autograd's chain rule, sparse-factorization reuse) comes along automatically.

This is the composition the [Part 1 Ch 03 thesis](../../10-physical/03-thesis.md) argued makes the integrated stack simpler than either of its parents — not more complicated. Contact is not a special case in the solver; it is one more term in the sum. The solver doesn't know whether a particular energy contribution came from the neo-Hookean bulk, the barrier contact, or the smoothed Coulomb friction — it just adds them up and minimizes.

## What this sub-leaf commits the book to

- **Contact is an energy, not a force.** `sim-soft`'s `ContactBody` trait surface (conceptually, a contribution to $U_\text{total}$) returns an energy density, a gradient, and a Hessian — not a force or an impulse. The choice is consequential: the [Part 11 Ch 00 contact module](../../110-crate/00-module-layout/03-contact.md) API is shaped around this trait surface; a force-based variant would be a different crate.
- **The four IPC components are composed, not stacked.** The barrier function (per-pair energy shape), the adaptive $\kappa$ schedule (Newton-loop stiffness control), the CCD filter (line-search step bound), and the energy-based composition (this sub-leaf) are four independent algorithmic choices. Each is replaceable in isolation — a different smooth barrier, a different stiffness schedule, a different CCD, a different integrator-level composition — without breaking the others. The specific Li 2020 choices are what `sim-soft` ships; the abstraction boundaries are the `ContactBody` trait plus the Newton loop, and each choice is an implementation detail behind those boundaries.
- **The gradient and Hessian structure is sparse in a pattern compatible with BSR-3.** [Part 8 Ch 01 sparse assembly](../../80-gpu/01-sparse-matrix/01-bsr.md) does not need a separate contact-pair data layout on top of the element-level sparse layout; both write into the same BSR-3 matrix with the same $3 \times 3$ block size.
