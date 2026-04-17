# Corotational formulation

Corotational elasticity is the middle step between [linear elasticity](02-linear.md) and full [hyperelasticity](04-hyperelastic.md) on the complexity axis. Per-element, it extracts the rotation $R$ from the deformation gradient $F$ via polar decomposition, applies linear elasticity in the un-rotated configuration, and rotates the resulting stress back into the deformed frame. The fictitious-rotation-stress failure from [Ch 02](02-linear.md#whats-wrong-about-it-in-the-soft-body-regime) goes away. The geometric-over-softening and the incompressibility-locking failures do not.

## The formulation

**Polar decomposition.** For every element at every timestep, decompose the deformation gradient:

$$ F = R\, U $$

where $R \in SO(3)$ is the rotation and $U$ is the symmetric positive-definite stretch tensor. $U$ can be computed as $U = \sqrt{F^T F}$ directly or by one of several iterative methods — Higham's Newton iteration on $F\, F^{-T}$ is the standard cheap choice, converging to 5 digits in ≈3 iterations from an identity start.

**Apply linear elasticity in the un-rotated frame.** Define the corotational strain as the linear strain measured against the un-rotated configuration:

$$ \varepsilon_R = U - I $$

This is exactly the linear strain measure that would be computed if the element were in configuration $U X$ instead of $F X$. Apply Hooke's law:

$$ \sigma_R = \lambda\, (\mathrm{tr}\, \varepsilon_R)\, I + 2\mu\, \varepsilon_R $$

**Rotate the stress back.** The first Piola stress in the deformed configuration is then:

$$ P = R\, \sigma_R $$

and the stiffness tangent $\partial P / \partial F$ combines the tangent of the linear Hooke map with the derivative of the polar decomposition. The tangent derivation is in [Part 2 Ch 00 sub-chapter 00](00-trait-hierarchy/00-trait-surface.md)'s `Material` trait's default `tangent()` method for corotational implementations; the math is ≈20 lines and verifiable against finite differences.

## What this fixes

**Rotation invariance is exact.** A pure rotation $F = R$ has $U = I$, so $\varepsilon_R = 0$ and $\sigma_R = 0$, and the predicted first Piola stress is zero regardless of the rotation angle. The fictitious-rotation-stress failure from linear elasticity is structurally gone.

**Moderate-stretch accuracy improves.** Because the stretch $U$ rather than the displacement gradient appears in the strain measure, corotational is correct for moderate stretches where linear is not — a rotated-and-stretched rod of 10% stretch at 30° rotation has corotational stress within 1% of hyperelastic, where linear has a ≈15% rotation-dominated error. For the rigid-body-motion-plus-small-deformation regime of mechanical parts at large overall displacement, corotational is a real upgrade over linear.

## What this does not fix

**Geometric over-softening under large stretch persists.** The strain measure $\varepsilon_R = U - I$ is still linear in stretch; at 50% stretch ($\lambda_{\text{stretch}} = 1.5$), $\varepsilon_R = 0.5$, whereas the neo-Hookean energy's effective strain term scales logarithmically in $\lambda$. At 50% stretch, corotational over-softens by ≈10% relative to neo-Hookean; at 100% stretch, ≈25%. This is the same class of error linear elasticity makes at large stretch, just with the rotation contribution subtracted out.

**No volumetric incompressibility at finite strain.** Corotational inherits linear elasticity's volumetric-locking failure ([Part 2 Ch 05](05-incompressibility.md)). Silicone's near-incompressibility is not representable in a Hooke-law-in-un-rotated-frame formulation without a separate volumetric treatment (mixed u-p, F-bar, higher-order elements); corotational does not supply one.

**Not derived from an energy.** Hyperelastic laws are defined by an energy density $\psi(F)$ from which stress and tangent follow by differentiation, guaranteeing thermodynamic consistency and a symmetric tangent by construction. Corotational defines stress directly, and the tangent must be hand-derived. Symmetry of the tangent is provable but not structural; a bug in the polar-decomposition derivative would produce an asymmetric tangent that breaks conjugate-gradient convergence in the solver. The [gradcheck suite](../110-crate/04-testing/03-gradcheck.md) catches this, but the corotational implementation carries more risk than an energy-based implementation for a comparable amount of code.

## Why corotational exists in the literature

Corotational is what [XPBD](../00-context/02-sota/05-nvidia-flex.md) and most position-based-dynamics implementations reach for when they want "better than linear" without committing to a full nonlinear energy. The computational profile is attractive: one polar decomposition per element per timestep (≈30 floating-point ops via Higham iteration, negligible), plus a linear Hooke evaluation, plus a rotate-back — maybe 2× the per-element cost of linear, still 3–5× cheaper than neo-Hookean with its closed-form energy evaluation. For games that need rigid-body-plus-deformation behavior at 60 FPS on tight hardware budgets, corotational has been the going rate for a decade.

The trade-off is explicit: corotational is an incomplete fix. It patches the most visually obvious failure (fictitious rotation stress) while leaving the physically larger errors (over-softening, lack of incompressibility) in place. Games-industry literature often treats this as acceptable because games rarely push materials to 50% stretch; engineering-grade simulation does push that hard, and corotational's large-stretch error becomes the dominant source of disagreement with measured data.

## Why sim-soft ships it

Two reasons, both narrow.

**Benchmark baseline.** [Part 11 Ch 04's regression tests against MuJoCo flex](../110-crate/04-testing/01-regression.md) need a matched-assumption comparison — flex uses a corotational-flavored PBD solver, and comparing `sim-soft`'s neo-Hookean directly to flex's output mixes constitutive-law differences with solver-family differences. Having a `Corotational` `Material` impl in `sim-soft` lets us compare flex against `sim-soft` with matched constitutive assumptions, isolating the solver-family delta from the constitutive-law delta.

**Baseline for the thesis.** [Part 1 Ch 03's thesis](../10-physical/03-thesis.md) commits to hyperelastic as the default. The corotational `Material` impl is how the book makes "corotational is not enough" reproducible in `sim-soft` itself, the same way linear makes its own failures reproducible. An honest baseline is worth its small code cost.

## Validity domain declaration

The `Corotational` `Material` declares validity as: stretch magnitude below 25% ($\lambda_{\text{stretch}} \in [0.8, 1.25]$), rotation unrestricted, Poisson ratio strictly below 0.45. Outside this domain the gradcheck-verified error bound versus neo-Hookean is quadratic in excess stretch; runtime warnings fire when an element exceeds its validity domain.
