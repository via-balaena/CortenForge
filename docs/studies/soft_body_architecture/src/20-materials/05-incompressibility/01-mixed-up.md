# Mixed u-p formulation

The [locking sibling](00-locking.md) named the pathology: Tet4 at $\nu \to 0.5$ over-constrains the per-element system, the solver cannot simultaneously satisfy volumetric and deviatoric requirements, and the result is rigid-looking bending with simultaneous volume loss. The mixed u-p formulation cures this by promoting pressure from a derived quantity to an independent field — the displacement is no longer responsible for enforcing $J = 1$ alone, because a Lagrange-multiplier pressure field shares the work. This is `sim-soft`'s primary near-incompressibility cure, per the [Ch 05 parent's Claim 2](../05-incompressibility.md).

## The two-field variational principle

Write the [modified hyperelastic energy from Ch 04](../04-hyperelastic/00-neo-hookean/02-incompressible.md) in its deviatoric-plus-volumetric split $\psi(F) = \psi_\text{dev}(\bar F) + U(J)$, and introduce pressure $p$ as an independent scalar field. Replace $U(J)$ with a Lagrangian whose stationary points enforce the near-incompressibility constraint through $p$:

$$ \Pi(u, p) \;=\; \int_\Omega \psi_\text{dev}(\bar F(u))\, \mathrm{d}V \;+\; \int_\Omega \left[ p\,(J(u) - 1) \;-\; \tfrac{1}{2\kappa}\, p^2 \right] \mathrm{d}V \;+\; \text{boundary work} $$

with $\kappa$ the bulk modulus (for near-incompressibility, $\kappa \gg \mu$; for strict incompressibility, $1 / \kappa \to 0$). Stationarity in $u$ and in $p$ gives two coupled equations:

- **Displacement equation.** $\partial \Pi / \partial u = 0$ yields the deviatoric residual plus $p\,\partial J / \partial u$ — the pressure enters the force balance directly, carrying the role the volumetric term $\partial U / \partial J$ carried in the single-field formulation.
- **Pressure equation.** $\partial \Pi / \partial p = 0$ yields $J - 1 = p / \kappa$, i.e., $p = \kappa\,(J - 1)$ in the stationary limit. In the strict-incompressibility limit $1 / \kappa \to 0$, this becomes $J = 1$ with $p$ free (a Lagrange multiplier).

The key change is that $p$ is no longer derived from $u$ pointwise — it is its own field with its own degrees of freedom. The over-constraint of the single-field Tet4 system ("all twelve DOFs plus per-tet $J = 1$ cannot satisfy the deviatoric boundary conditions") becomes a compatible two-field system ("twelve displacement DOFs plus one pressure DOF per tet, with the pressure carrying the volumetric constraint"). The number of independent displacement modes available for isochoric bending is preserved.

## Discretization — Tet4 + constant pressure

`sim-soft`'s mixed-u-p uses Tet4 with a per-element constant pressure: linear displacement ($P_1$) plus $P_0$ (piecewise-constant) pressure, on the [Tet4 mesh](../../30-discretization/00-element-choice/00-tet4.md) that the [SDF pipeline](../../70-sdf-pipeline/01-tet-strategies.md) produces.

One pressure DOF per tet plus the existing twelve displacement DOFs per tet. The global unknown vector grows by $n_\text{tets}$ scalars; the sparse Newton tangent grows correspondingly with a block structure that mixes the displacement-displacement block $K_{uu}$ (as before), a displacement-pressure block $K_{up}$, a pressure-displacement block $K_{pu} = K_{up}^T$, and a pressure-pressure block $K_{pp}$.

Stability of the $P_1$–$P_0$ pair on tetrahedral meshes is a well-known technical concern. The pair does not generically satisfy the Babuška–Brezzi inf-sup condition in the strict-incompressibility ($\kappa \to \infty$) limit, where certain mesh topologies admit spurious pressure modes that pollute the solution. At finite $\kappa$ — which is always the case for `sim-soft`'s near-incompressibility regime at $\nu < 0.499$ — the $1/\kappa$ term in the Lagrangian provides pressure stabilization (a Tikhonov-style regularization on $p$) that damps the spurious modes to a level consistent with the physical compressibility the material actually has. Strict incompressibility at $\nu = 0.5$ is outside `sim-soft`'s Phase A–I scope and would require either a stabilized $P_1$–$P_1$ pair (with an added pressure-penalty term) or a Taylor–Hood $P_2$–$P_1$ pair on Tet10. The equal-order $P_1$–$P_1$ pair without stabilization is rejected: it produces checkerboard pressure modes distinct from locking, with a different cure.

## Saddle-point Newton system

The $(K_{uu}, K_{up}, K_{pu}, K_{pp})$ block structure is indefinite — mixed u-p is a *saddle-point* system, not a purely positive-definite one. Newton's direction solve is:

$$ \begin{pmatrix} K_{uu} & K_{up} \\ K_{up}^T & K_{pp} \end{pmatrix} \begin{pmatrix} \Delta u \\ \Delta p \end{pmatrix} \;=\; -\begin{pmatrix} r_u \\ r_p \end{pmatrix} $$

$K_{pp}$ is $-(1/\kappa)$ times the per-tet mass matrix for pressure (a diagonal block on $P_0$-pressure), which is negative-definite for positive $\kappa$ and zero in the strict-incompressibility limit. The system is not SPD, so the [factor-on-tape Cholesky path](../../50-time-integration/00-backward-euler.md) from Part 5 Ch 00 does not apply directly to the combined system.

`sim-soft`'s approach is *static condensation* of the pressure degrees of freedom. $K_{pp}$ is block-diagonal (per-tet), so it can be inverted tet-by-tet and substituted into the displacement block:

$$ \left[K_{uu} - K_{up}\, K_{pp}^{-1}\, K_{up}^T\right] \Delta u \;=\; -r_u + K_{up}\, K_{pp}^{-1}\, r_p $$

The reduced system in $\Delta u$ alone is SPD inside the IPC-guarded basin (same argument as the single-field Newton system in [Part 5 Ch 00](../../50-time-integration/00-backward-euler.md)), so the reduced Cholesky path applies. $K_{pp}^{-1}$ is a per-tet $1 \times 1$ scalar inversion, trivially storable. Once $\Delta u$ solves, $\Delta p = K_{pp}^{-1}(-r_p - K_{up}^T\,\Delta u)$ recovers the pressure update.

Static condensation preserves [factor-on-tape](../../60-differentiability/02-implicit-function.md)'s cost-amortization argument: the Cholesky factor of the reduced $K_{uu}$-minus-Schur-complement is stored on the autograd tape; the IFT adjoint re-uses it. One setup per Newton step, one factor per converged step, many backward-solves per step against the stored factor.

The displacement-pressure cross-term $K_{up}\, K_{pp}^{-1}\, K_{up}^T$ changes the sparse-assembly shape — it adds a per-tet rank-one update to $K_{uu}$ that is a pressure-stabilized perturbation. The [CSR pattern](../../80-gpu/01-sparse-matrix/00-csr.md) is pre-allocatable once per mesh because the rank-one pattern is determined by the Tet4 topology; at runtime the coefficient shifts per step but the nonzero pattern does not.

## Validity-domain widening

The standalone hyperelastic [`ValidityDomain`s in Ch 04](../04-hyperelastic/00-neo-hookean.md) cap at $\nu < 0.45$ because above that ratio the standalone compressible form locks. Wrapping a base `M: Material` in the mixed-u-p decorator widens the Poisson upper bound to $\nu < 0.499$:

```rust
pub struct MixedUP<M: Material> {
    base: M,
    // No new parameters — the Lagrangian structure is universal across hyperelastic laws
}

impl<M: Material> Material for MixedUP<M> { /* ... */ }
```

The widening holds because the saddle-point system's compatibility no longer depends on the $\lambda / \mu$ ratio inside the displacement block — the pressure field absorbs the volumetric penalty. The composed validity descriptor reports $\nu < 0.499$ on the stretch axis and inherits the base law's stretch-range from its own declaration (per the [validity composition-widening pattern](../00-trait-hierarchy/02-validity.md)).

At strict incompressibility ($\nu = 0.5$, $\kappa = \infty$), the $1 / \kappa$ regularization vanishes and the system becomes a pure saddle-point. `sim-soft`'s mixed-u-p is implemented for near-incompressibility with $\kappa$ large but finite; the strict limit is reached by $\kappa$ asymptotics rather than by special-casing, which keeps the code paths uniform across $\nu \in [0.45, 0.5)$.

## What this sub-leaf commits the book to

- **Mixed u-p is `sim-soft`'s primary near-incompressibility cure.** Claim 2 of the [parent chapter](../05-incompressibility.md); named here with its structural form (Lagrangian + Tet4-plus-$P_0$-pressure element pair + saddle-point Newton).
- **The element pair is $P_1$–$P_0$ (Tet4 displacement, piecewise-constant pressure).** Stable on well-shaped Tet4 meshes, generates one pressure DOF per tet, avoids both locking and checkerboard-pressure instability. Equal-order $P_1$–$P_1$ is rejected.
- **Static condensation recovers an SPD reduced system.** The per-tet $K_{pp}^{-1}$ is a scalar; the Schur complement adds a pre-allocatable rank-one structure to $K_{uu}$; the [factor-on-tape Cholesky path](../../50-time-integration/00-backward-euler.md) carries over without modification to the reduced system.
- **Poisson upper bound widens to $\nu < 0.499$ when wrapped.** The composed `ValidityDomain` reports the widened bound per the [Ch 00 validity composition pattern](../00-trait-hierarchy/02-validity.md); standalone base laws remain capped at $\nu < 0.45$.
- **The formulation is constitutive-law-agnostic.** `MixedUP<M: Material>` wraps any base hyperelastic law; neo-Hookean, Mooney-Rivlin, and Ogden all compose with it via the modified-form evaluation hooks each exposed in [Ch 04](../04-hyperelastic/00-neo-hookean/02-incompressible.md).
