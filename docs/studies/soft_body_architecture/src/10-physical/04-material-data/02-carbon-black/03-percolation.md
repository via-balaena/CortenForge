# Percolation threshold

Percolation is the structural reason the [electrical conductivity](01-conductivity.md) of a carbon-black-loaded silicone is not a smooth function of filler fraction. Below a critical loading $\phi_c$ the carbon-black particles are dispersed in the silicone matrix without forming a connected path spanning the body, and the composite is an insulator. Above $\phi_c$ a percolating cluster exists and the composite conducts. The transition is sharp — most of the conductivity range sits in a few-percent filler-fraction window around $\phi_c$ — and it shapes how the carbon-black family enters the optimizer, the solver, and the design flow.

## The threshold

For the carbon-black-loaded Ecoflex and Dragon Skin composites the book ships, the percolation threshold sits in the ≈8–10 wt% filler fraction window. The specific threshold depends on:

- **Carbon-black particle type.** Different carbon-black grades (conductive carbon black like Vulcan XC-72 versus furnace blacks versus acetylene black) have different particle sizes, aggregate structures, and surface chemistries. The threshold shifts by a few percent filler fraction across grades.
- **Dispersion quality.** A well-dispersed composite (high-shear mixing, no clumping) has a higher threshold than a poorly-dispersed composite — agglomerated particles form connected paths at lower bulk filler fractions. This is why the [branch Claim 4](../02-carbon-black.md) names dispersion quality as a primary driver of lot-to-lot variation.
- **Cure profile.** Thermal cure can mobilize the particles, allowing them to cluster into connected paths after dispersion, which lowers the effective threshold. Room-temperature cure preserves the as-dispersed distribution.
- **Aging.** Over timescales of months, the particles can migrate under gravitational settling and interfacial forces, shifting the effective threshold. The [sim-to-real calibration loop](../../../100-optimization/05-sim-to-real.md) handles aging via periodic re-measurement.

The Pass-1 material database's $\phi_c \approx 0.08$–$0.10$ range reflects the nominal geometric mean across these factors. Per-print calibration narrows the range for a specific fabricated part.

## The scaling law above threshold

Statistical-physics percolation theory predicts that the electrical conductivity scales with filler fraction above threshold as a power law:

$$ \sigma_e(\phi) \;\propto\; (\phi - \phi_c)^t \qquad \text{for } \phi > \phi_c $$

with $t$ the universal percolation exponent. For three-dimensional continuum percolation the universal exponent is near $t \approx 2$, though measured values in filled elastomers can drift higher due to tunnelling contributions across narrow gaps between nearly-touching particles.

Below threshold, the conductivity is dominated by tunnelling between isolated or weakly-clustered particles, giving a stretched-exponential rise with filler fraction that is many orders of magnitude below the above-threshold regime.

The cross-over between these two regimes is what the `ElectricalProperties` field's closed-form $\sigma_e(\phi)$ expression captures, typically as:

- A low-$\phi$ flat insulator value (dominated by the pure matrix's leakage).
- A sigmoidal cross-over through the percolation band.
- A power-law above-threshold rise with the exponent $t$ and threshold $\phi_c$ calibrated against the material-database nominal rows.

## Why the threshold matters for the optimizer

The [Part 10 Ch 06 full design-print-rate loop](../../../100-optimization/06-full-loop.md) and its [sim-to-real sub-loop](../../../100-optimization/05-sim-to-real.md) search over filler-fraction fields $\phi(x)$ when the electrical axis is part of the reward. The percolation threshold enters in one of three ways, which the designer chooses at problem setup time.

- **Stay below threshold.** The design uses the capacitive-sensing mode the [conductivity leaf](01-conductivity.md) names, and the optimizer's search is bounded $\phi \in [0, \phi_c - \text{margin}]$. The gradient through $\sigma_e$ is smooth in this regime; no special handling.
- **Stay above threshold.** The design uses resistive sensing or Joule heating, and the optimizer's search is bounded $\phi \in [\phi_c + \text{margin}, 0.20]$. The gradient through $\sigma_e$ is smooth in this regime too; the power-law rise is monotonic and differentiable.
- **Straddle the threshold.** The design wants to explore both regimes — a capacitive-to-resistive transition under active control, or a filler field that crosses threshold within a single part. The gradient through $\sigma_e$ is not smooth in the crossing region; the `ElectricalProperties` field flags this as an FD-wrapper-engaged path, same pattern as [Part 6 Ch 05's diff-meshing discussion](../../../60-differentiability/05-diff-meshing.md) uses for non-smooth events, scheduled alongside the other stochastic-adjoint upgrades in Phase H.

The first two modes are the default Pass-1 operating modes; the third is an explicit opt-in.

## Why the threshold is not a smooth curve even with fine sampling

A key reader question is whether the sharp threshold reflects a real discontinuity or just a steep smooth slope that fine sampling would resolve. It reflects a real structural discontinuity.

Percolation is a topological-connectivity transition. Below $\phi_c$ no carbon-black path connects opposite faces of the body; above $\phi_c$ at least one does. The existence of a connecting path is a binary property of the particle network's geometry, not a continuous function of filler fraction. What smooths the bulk $\sigma_e(\phi)$ curve near threshold in measured data is (i) the finite body size (a large enough body with filler fraction slightly below the bulk threshold can still have a locally-percolating cluster spanning it), (ii) tunnelling between nearly-touching particles (which contributes conductivity before physical contact), and (iii) measurement-averaging across lot-to-lot dispersion variation. None of these is a smooth physical mechanism at the underlying particle level; they are finite-size-effect and statistical smoothings of a structurally binary transition.

For `sim-soft`'s per-element sampling at canonical-problem resolution, each element's volume is large enough that the finite-size smoothing dominates, so the $\sigma_e(\phi)$ that the element-level `ElectricalProperties` field reads is a smoothed approximation to the underlying binary transition. The smoothed curve still has a steep region around $\phi_c$ that the FD-wrapper treatment handles; it is not as discontinuous as the idealized percolation limit, but it is sharp enough that gradient-based optimization needs care.

## What the sim-soft `Material` stores

The `ElectricalProperties` field carries the percolation parameters directly:

- The threshold $\phi_c$, calibrated per base-silicone from the material-database rows.
- The universal exponent $t$, defaulted to the 3D continuum-percolation value and refined per-print.
- A cross-over width $\Delta\phi$ that parameterizes the sigmoidal smoothing.
- A matrix-leakage floor for the below-threshold regime.

The [`MaterialField`](../../../70-sdf-pipeline/00-sdf-primitive.md) samples $\phi(x)$ per element and evaluates the closed-form $\sigma_e(\phi)$ with these parameters. The gradient-path-through-$\sigma_e$ flows smoothly when $\phi(x)$ stays on one side of $\phi_c$ across the whole body; crossing engages the FD-wrapper flag per [Part 6 Ch 05](../../../60-differentiability/05-diff-meshing.md).

## Alternatives considered

**Use a smooth-enough $\sigma_e(\phi)$ that no FD-wrapper is needed.** The smoothed percolation cross-over in finite-element samples is already smoother than the idealized theory, so the FD-wrapper could in principle be avoided by choosing a sufficiently gradual cross-over width. Rejected because the actual measured cross-over width is physically bounded by the composite's microstructure, not by the curve-fitting convenience — using a wider cross-over than the measured physics would misrepresent $\sigma_e(\phi)$ in the regime the capacitive-sensing mode actually operates in.

**Separate the percolation parameters into their own per-material calibration table outside the `ElectricalProperties` field.** Rejected because the percolation parameters are intrinsically tied to the composite (base + filler + cure profile), not to the filler alone — a carbon-black loading in Ecoflex 00-30 has a different threshold than the same loading in Dragon Skin 10A. Keeping $\phi_c$ and $t$ inside the composite's `ElectricalProperties` respects the composite-level identity and composes correctly with the [spatial-fields framing](../../../20-materials/09-spatial-fields.md).

**Treat the threshold as a fixed constant across all carbon-black-loaded rows.** Rejected at Pass 1 because the material-database's Pass-1 row set explicitly flags lot and dispersion variance, and the Pass-3 refinement via per-print calibration is expected to land per-print $\phi_c$ values. A fixed constant would short-circuit the calibration pathway the [branch Claim 4](../02-carbon-black.md) is built on.
