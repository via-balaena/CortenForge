# Concentration → electrical conductivity

The electrical conductivity of carbon-black-loaded silicone is the property that makes this family a sensor, and it is the property whose filler-fraction dependence is sharpest — a near-discontinuous transition across the [percolation threshold](03-percolation.md) that the [modulus](00-modulus.md) and [thermal-conductivity](02-thermal.md) axes do not show. This leaf is the first place in the book that carries the electrical-conductivity symbol $\sigma_e$ — subscripted per the convention locked in [`appendices/03-notation.md`](../../../appendices/03-notation.md) to avoid collision with Cauchy stress $\sigma$ everywhere in Parts 2–6.

## The data table

$\sigma_e$ is reported as the bulk electrical conductivity of the composite, in S/m. The tabulated values are geometric-mean anchors in a broad lot-and-dispersion-dependent band; per-print calibration via the [Part 10 Ch 05 sim-to-real loop](../../../100-optimization/05-sim-to-real.md) narrows the band for each fabricated part.

| Filler fraction $\phi$ | Electrical conductivity $\sigma_e$ (S/m) | Regime |
|---|---|---|
| 5 wt% | $< 10^{-8}$ | Insulator (below percolation) |
| 10 wt% | ≈$10^{-3}$ | Transitioning through percolation |
| 15 wt% | ≈$10^{0}$ | Resistive conductor (above percolation) |
| 20 wt% | ≈$10^{1}$ | Well above percolation |

The span across the 5–20 wt% range is roughly nine orders of magnitude, and the bulk of that span is concentrated in a narrow few-percent filler-fraction window near the [percolation threshold](03-percolation.md) at ≈8–10 wt%. Above threshold the conductivity rises steeply but smoothly with further loading; well above threshold (15 wt% and above) the rise flattens toward the carbon-black particles' intrinsic bulk conductivity scaled by the composite's effective connectivity.

## The mechanism, in one sentence

Electrical current flows through the composite when carbon-black particles form a connected path spanning the body; below the filler fraction required for such a path, the silicone matrix dominates and the composite is an insulator, and above it a resistive-network regime sets in. The [percolation-threshold sub-chapter](03-percolation.md) walks the statistical-physics argument for why the threshold is sharp and how the conductivity scales with $\phi$ above it. This leaf focuses on the calibration data and its `MaterialField` implications.

## The near-discontinuity at threshold

Unlike the modulus and thermal-conductivity axes, which are smooth monotonic functions of $\phi$ across the filler-fraction range, $\sigma_e(\phi)$ has a structural near-discontinuity. A few-percent shift in filler fraction — within the measurement precision the material-database table is quoted at — can push the composite across the insulator/conductor boundary, changing $\sigma_e$ by several orders of magnitude.

Two consequences for the solver and the optimizer.

- **The conductivity is not a smooth function of $\phi$ through the percolation band.** The [implicit-function-theorem gradient machinery of Part 6](../../../60-differentiability/02-implicit-function.md) assumes a smooth $\sigma_e(\phi)$ in its per-element sampling; through the percolation band it is not smooth, and the gradient-through-$\sigma_e$ flowing through the [`MaterialField`](../../../70-sdf-pipeline/00-sdf-primitive.md) into an optimizer's electrical-sensing reward term fails the standard smoothness assumption.
- **The optimizer must either stay on one side of the threshold or handle the discontinuity explicitly.** The default strategy in the book is "stay on one side" — the designer specifies a filler-fraction range that sits clearly above threshold (15 wt% and above) or clearly below (5 wt% and below), and the optimizer's search stays within that side. Crossing the threshold during optimization is an explicit design choice that engages the [FD-wrapper pattern](../../../60-differentiability/05-diff-meshing/03-fd-wrappers.md) — the same pattern [Part 6 Ch 05](../../../60-differentiability/05-diff-meshing.md) commits to for topology-changing re-mesh events, with the full stochastic-adjoint upgrade scheduled alongside Part 6's [other stochastic-adjoint deferrals](../../../60-differentiability/03-time-adjoint/02-stochastic.md) in Phase H.

## What the sim-soft `Material` stores

A carbon-black-loaded `Material` carries the electrical axis as an `ElectricalProperties` sub-field alongside the mechanical and thermal properties:

- The filler fraction $\phi$, shared with the [modulus](00-modulus.md) and [thermal-conductivity](02-thermal.md) property shifts.
- The conductivity function $\sigma_e(\phi)$ as a closed-form expression — a power-law $\sigma_e \propto (\phi - \phi_c)^t$ above threshold (where $\phi_c$ is the percolation threshold and $t$ is the universal percolation exponent), a flat insulator value below threshold, and a cross-over that the [percolation-threshold leaf](03-percolation.md) parameterizes.
- A validity-metadata flag indicating whether the calibrated $\phi$ range sits above, below, or straddles the percolation threshold. Straddling the threshold engages the FD-wrapper on the gradient path through $\sigma_e$.

The Pass-1 `ElectricalProperties` ships with a `t` exponent and $\phi_c$ value calibrated to the material-database nominal values; Pass-3 refinement swaps these for per-print-calibrated numbers from the sim-to-real loop.

## The sensing loop

The electrical-sensing loop in [Part 10 Ch 06 — full design-print-rate loop](../../../100-optimization/06-full-loop.md) uses $\sigma_e(\phi)$ in one of two modes.

- **Capacitive sensing below threshold.** The composite's dielectric constant shifts with filler fraction and with strain; a capacitive readout between two carbon-black-loaded regions returns a signal that tracks contact pressure and deformation. This mode keeps $\phi$ below percolation so that the composite remains an insulator and the capacitive measurement is not shorted out.
- **Resistive sensing above threshold.** The bulk resistance of a carbon-black path changes with strain (the conducting network is strain-sensitive), and a two-point or four-point resistance measurement returns a strain-gauge-like signal. This mode requires $\phi$ above threshold for the conducting network to exist at all; the measurement precision scales with the composite's baseline conductivity, which is why 15 wt% and above is the typical design target.

Joule heating uses the same above-threshold regime: current through a resistive carbon-black region dissipates as heat at rate $\sigma_e |E|^2$ per unit volume (with $E$ the electric field), which couples into [`sim-thermostat`](../../../20-materials/08-thermal-coupling.md) as a heat source. The Joule-heating path is what closes the full electrical-to-thermal coupling the [carbon-black branch](../02-carbon-black.md) Claim 3 names.

## Alternatives considered

**Smooth $\sigma_e(\phi)$ through the percolation band as a fitting-convenience approximation.** Rejected because the near-discontinuity is physical, not a fitting artifact — it comes from the connectivity transition in the particle network, not from limited sampling of a smooth function. Approximating it as smooth would let the optimizer's gradient flow through a region where the actual conductivity jumps by orders of magnitude, which produces physically misleading search directions.

**Ship only resistive-regime data and exclude sub-threshold rows.** Rejected because the capacitive-sensing application at sub-threshold filler fractions is a real canonical-problem path, and omitting the sub-threshold rows would close that path off at the material-data level. The four tabulated $\phi$ values span both regimes deliberately.

**Use a second filler chemistry (conductive silver flakes) for the sensing role and reserve carbon-black for mechanical stiffening only.** Rejected because the cross-chemistry design introduces two percolation thresholds, two thermal profiles, two modulus shifts, and a bonded-interface story that the [single-chemistry `MaterialField` framing](../../../20-materials/09-spatial-fields.md) is built to avoid. Carbon-black handles all three axes with one $\phi$ field and one chemistry; the operational simplicity is the reason.
