# Tangent stiffness

The [energy-and-stress sibling](00-energy.md) gave first Piola $P = \mu\,(F - F^{-T}) + \lambda\,(\ln J)\, F^{-T}$. This leaf takes the next derivative to get the fourth-order tangent $\partial P / \partial F$ in closed form and states the per-element block sizes the [`element/`](../../../110-crate/00-module-layout/01-element.md) layer assembles from it.

## Closed-form fourth-order tangent

Differentiating $P(F)$ term by term using the identities $\partial F_{ij} / \partial F_{kl} = \delta_{ik} \delta_{jl}$, $\partial F^{-T}_{ij} / \partial F_{kl} = -F^{-T}_{kj}\, F^{-T}_{il}$, and $\partial \ln J / \partial F_{kl} = F^{-T}_{kl}$:

$$ \mathbb{C}_{ijkl} \;=\; \frac{\partial P_{ij}}{\partial F_{kl}} \;=\; \mu\,\delta_{ik}\,\delta_{jl} \;+\; (\mu - \lambda \ln J)\, F^{-T}_{il}\, F^{-T}_{kj} \;+\; \lambda\, F^{-T}_{ij}\, F^{-T}_{kl} $$

Three tensor products: the identity on $F$, the "swapped" product of $F^{-T}$ with itself indexed as $il, kj$, and the direct tensor product of $F^{-T}$ with itself indexed as $ij, kl$. The first carries the shear-plus-$\mu$ part, the second carries the combined derivative of the $-\mu F^{-T}$ and $\lambda (\ln J) F^{-T}$ pieces via the $F^{-T}$-derivative identity, and the third carries the $\ln J$-derivative piece.

At $F = I$ (reference configuration) the tangent reduces to the isotropic elasticity tensor of [Ch 02 linear](../../02-linear.md):

$$ \mathbb{C}_{ijkl}\big|_{F = I} \;=\; \mu\,(\delta_{ik}\,\delta_{jl} + \delta_{il}\,\delta_{jk}) \;+\; \lambda\,\delta_{ij}\,\delta_{kl} $$

This is the structural reason the linear impl and the neo-Hookean impl agree to six digits at 0.1% strain in the [gradcheck suite](../../../110-crate/04-testing/03-gradcheck.md) — they share the same tangent at zero strain.

## Symmetry and positive-definiteness

**Major symmetry: $\mathbb{C}_{ijkl} = \mathbb{C}_{klij}$.** Because neo-Hookean is hyperelastic — $P$ derives from a scalar energy $\psi$ — the tangent is the Hessian of $\psi$ with respect to $F$, and Hessians are symmetric by Clairaut's theorem. Inspecting the closed form confirms it: the first term is already symmetric under $(ij) \leftrightarrow (kl)$, and the second and third terms are as well after relabeling the dummy indices. This means the flattened $9\times 9$ tangent is a symmetric matrix, which is what lets the Newton system's per-element stiffness block be symmetric and the sparse factorization Cholesky rather than LU ([Part 5 Ch 00 Claim 3](../../../50-time-integration/00-backward-euler.md)).

**Positive-definiteness inside the validity domain.** At small-to-moderate stretch and non-near-inverted configurations, $\mathbb{C}$ is positive-definite on the space of symmetric $F$-perturbations. Outside the domain — at large stretch or as $J \to 0^+$ — the $-\lambda \ln J$ coefficient in the second term can make the tangent lose positive-definiteness in specific modes, which shows up as indefinite diagonal blocks in the Newton system. [The line-search of Part 5 Ch 00](../../../50-time-integration/00-backward-euler.md) is what handles these steps robustly rather than relying on the tangent alone to stay SPD everywhere.

## Per-element block sizes

The shape-function-gradient pull-back ([Part 6 Ch 01 FEM assembly VJP](../../../60-differentiability/01-custom-vjps/01-fem-assembly.md)) converts $\mathbb{C}$ at each Gauss point to the per-element stiffness block via $K^{\text{el}} = \int_V B^T\, \mathbb{C}\, B\, \mathrm{d}V$, where $B$ is the strain-displacement matrix for the element. Per [Part 3 Ch 00 element choice](../../../30-discretization/00-element-choice.md):

- **Tet4** — 4 nodes × 3 DOFs = 12 DOFs per element → $12 \times 12$ symmetric stiffness block, evaluated at 1 Gauss point per tet (single-point centroid rule, exact for linear strain).
- **Tet10** — 10 nodes × 3 DOFs = 30 DOFs per element → $30 \times 30$ symmetric stiffness block, evaluated at 4 Gauss points per tet (the Phase H default per [Part 3 Ch 00 cost/accuracy tradeoff](../../../30-discretization/00-element-choice/04-tradeoff.md)).

The blocks fan out into the global Newton tangent via per-element-to-global vertex indexing, which is `solver/`-layer work, not `material/`. A multi-material mesh with Tet4 and Tet10 regions in different parts fans out mixed block sizes; the [spatial-material-fields machinery](../../09-spatial-fields.md) routes `Material` impls to elements but does not touch the block structure.

## Cached intermediates

$F^{-T}$ appears in $P$ and in every term of $\mathbb{C}$. The `first_piola` and `tangent` calls at the same Gauss point share an $F^{-T}$ cache — the [implementation sub-leaf](03-impl.md) stores it on an `EvaluationScratch` passed by reference to avoid recomputing the $3\times 3$ inverse. $\ln J$ is likewise shared between the stress and tangent evaluations.

## What this sub-leaf commits the book to

- **Neo-Hookean's tangent is closed-form.** No numerical differentiation, no linearization around a previous step. The only inputs are $F$ and the Lamé pair $(\mu, \lambda)$; every Gauss-point tangent evaluation is a fixed number of floating-point operations.
- **The tangent is symmetric by hyperelastic construction.** Downstream code ([Part 5 Ch 00 Cholesky factorization](../../../50-time-integration/00-backward-euler.md), [Part 8 Ch 02 sparse solvers](../../../80-gpu/02-sparse-solvers.md)) relies on this symmetry; a bug producing an asymmetric tangent would break the Cholesky path and fall back to LU with a 2–3× cost penalty.
- **Per-element block sizes are fixed per element type.** $12 \times 12$ for Tet4, $30 \times 30$ for Tet10. Global Newton tangent sparsity is therefore deterministic given the mesh topology; the [sparse factorization on tape](../../../60-differentiability/02-implicit-function.md) can pre-allocate the CSR pattern once per mesh.
- **$F^{-T}$ and $\ln J$ are Gauss-point-local caches shared between stress and tangent.** The [implementation sub-leaf](03-impl.md)'s `EvaluationScratch` is the concrete form.
