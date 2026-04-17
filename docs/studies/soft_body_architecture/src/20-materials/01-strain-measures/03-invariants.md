# Invariants I₁, I₂, I₃

The isotropic-hyperelastic family in [Ch 04](../04-hyperelastic.md) writes every energy as a function of three scalar invariants of the [right Cauchy-Green tensor $C$](01-C.md). This leaf writes the invariants down, gives their principal-stretch forms, names the anisotropic extension $I_4$, states the isochoric split used by [Ch 05's incompressibility treatments](../05-incompressibility.md), and flags the near-identity numerical-stability concern every invariant-based impl has to handle.

## Definitions

For a symmetric positive-definite $C = F^T F$:

$$ I_1 = \mathrm{tr}\,C $$

$$ I_2 = \tfrac{1}{2}\left((\mathrm{tr}\,C)^2 - \mathrm{tr}\,(C^2)\right) $$

$$ I_3 = \det C $$

All three are rotation-invariant: under $F \mapsto R F$ for $R \in SO(3)$, $C$ is invariant (see [the $C$ sibling](01-C.md)), and therefore so are $I_1, I_2, I_3$. This is what makes them the natural arguments for an isotropic-hyperelastic energy: any frame-indifferent isotropic scalar function of $C$ can be expressed as $\psi(I_1, I_2, I_3)$ by the Rivlin-Ericksen representation theorem.

## Principal-stretch representation

Writing $C$'s eigenvalues as the squared principal stretches $\lambda_1^2, \lambda_2^2, \lambda_3^2$ (the squares of the stretches from the polar decomposition of [$F$](00-F.md)):

$$ I_1 = \lambda_1^2 + \lambda_2^2 + \lambda_3^2 $$

$$ I_2 = \lambda_1^2 \lambda_2^2 + \lambda_2^2 \lambda_3^2 + \lambda_3^2 \lambda_1^2 $$

$$ I_3 = \lambda_1^2 \lambda_2^2 \lambda_3^2 = J^2 $$

The last equation gives the connection $J = \det F = \sqrt{I_3} = \lambda_1 \lambda_2 \lambda_3$: the volume ratio is the product of principal stretches, and $I_3$ is its square. Every `sim-soft` evaluation of an invariant-based energy uses this relation — when an impl writes $\ln J$ ([Ch 04 neo-Hookean](../04-hyperelastic/00-neo-hookean.md)), $J$ is computed as $\det F$ directly rather than as $\sqrt{I_3}$, to avoid the square-root derivative near $J = 1$.

## Anisotropic $I_4$

Anisotropic laws ([Ch 06](../06-anisotropic.md), [HGO](../06-anisotropic/01-hgo.md)) add a fourth invariant that depends on a fiber direction $a$:

$$ I_4 = a \cdot C\, a $$

$I_4$ is the squared stretch of a unit vector aligned with the fiber direction in the reference configuration. At the undeformed state $F = I$, $I_4 = 1$; at a stretch of $\lambda_f$ along the fiber, $I_4 = \lambda_f^2$. Two-fiber-family models (HGO for arteries) add a second $I_4^{(2)} = a^{(2)} \cdot C\, a^{(2)}$ with a second fiber direction; the two invariants are independent.

$I_4$ is rotation-invariant only if $a$ is a reference-frame vector — the "fiber direction is fixed in the reference configuration" convention locked in [Ch 06 Claim 2](../06-anisotropic.md). Under that convention, $a$ is reference-frame, $C$ is reference-frame, and $a \cdot C\, a$ is a rotation-invariant scalar.

## Isochoric split

[Ch 05's mixed u-p formulation](../05-incompressibility/01-mixed-up.md) and [F-bar method](../05-incompressibility/02-f-bar.md) work on a volume-preserving version of $C$:

$$ \bar C = J^{-2/3}\, C $$

with $\det \bar C = 1$ by construction. The isochoric invariants are then:

$$ \bar I_1 = J^{-2/3}\, I_1 $$

$$ \bar I_2 = J^{-4/3}\, I_2 $$

$$ \bar I_3 = 1 $$

Invariant-based hyperelastic energies for near-incompressible materials split into a deviatoric term in $\bar I_1, \bar I_2$ and a volumetric term in $J$:

$$ \psi(C) = \psi_\text{dev}(\bar I_1, \bar I_2) + \psi_\text{vol}(J) $$

This split is what lets the volumetric penalty (enforcing $J \to 1$) be handled separately from the deviatoric response — the core technical move [Ch 05](../05-incompressibility.md) builds on.

## Numerical stability near the reference configuration

At $F = I$, the invariants take fixed values: $I_1 = 3$, $I_2 = 3$, $I_3 = 1$. Energy functions are typically written with offsets — $I_1 - 3$, $\ln J$, $(\bar I_1 - 3)^2$ — so the reference configuration has zero energy and zero stress. These offsets are catastrophic-cancellation-prone when computed in `f64` at small strains: $I_1 = \lambda_1^2 + \lambda_2^2 + \lambda_3^2$ with stretches near 1.0 can lose many digits before the "$-3$" offset is applied.

`sim-soft` computes the offset measures directly from the displacement gradient $\nabla u = F - I$ where it matters:

$$ I_1 - 3 = 2\,\mathrm{tr}(\varepsilon) + \|\nabla u\|_F^2 $$

where $\varepsilon = \tfrac{1}{2}(\nabla u + (\nabla u)^T)$ is the small-strain tensor and $\|\cdot\|_F$ denotes the Frobenius norm. The analogous move for $\ln J$ at small $J - 1$ goes through `log1p`, avoiding the direct $\ln(1 + \epsilon)$ cancellation. These rearrangements keep the relative error below machine $\varepsilon$ at small strain — the regime where the [Phase B gradcheck](../../110-crate/03-build-order.md) asserts six-digit agreement with linear elasticity.

## What this sub-leaf commits the book to

- **`sim-soft`'s isotropic-hyperelastic energies are written as $\psi(I_1, I_2, I_3)$.** The three invariants are pre-computed once per Gauss-point evaluation and consumed by whichever energy the impl declares; an Ogden impl additionally computes principal stretches by eigen-decomposition. The `Material` trait signs in $F$, but the invariant evaluation is the hot path for every invariant-based impl.
- **Anisotropic laws add $I_4 = a \cdot C\, a$; $I_4$ is reference-frame.** The fiber direction $a$ is a reference-frame unit vector assigned per element by [Ch 09's spatial material fields](../09-spatial-fields.md); it does not rotate with deformation. This is what [Ch 06 Claim 2](../06-anisotropic.md) calls "fiber direction is a field, not a parameter".
- **The isochoric split $(\bar I_1, \bar I_2, J)$ is the canonical form for near-incompressible hyperelasticity.** [Ch 05](../05-incompressibility.md)'s locking treatments operate on this split; every hyperelastic impl that claims validity above $\nu = 0.45$ is the composition of its base with a mixed-u-p or F-bar locking-fix decorator that uses the split.
- **Reference-configuration numerical stability is per-impl.** Each invariant-based impl computes its offset measures ($I_1 - 3$, $\ln J$) via cancellation-safe expressions over $\nabla u$ rather than over $F$ directly. Failure to do so shows up as a [Phase B gradcheck](../../110-crate/03-build-order.md) miss at small strain, not as an error growing with stretch.
