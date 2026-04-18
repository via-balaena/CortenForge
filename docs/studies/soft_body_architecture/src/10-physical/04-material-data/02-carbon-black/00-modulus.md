# Concentration → modulus

Loading carbon-black into a platinum-cure silicone stiffens it. The relationship between filler fraction and small-strain modulus is monotonic, smooth, and — unlike the [electrical conductivity response](01-conductivity.md) — does not go through a discontinuous transition. This is the mechanical axis of the [carbon-black branch's](../02-carbon-black.md) "one filler parameter moves three property axes" claim: a single weight-fraction $\phi$ shifts modulus by a factor that the `Material<NeoHookean>` or `Material<MooneyRivlin>` of the base silicone consumes as a scalar multiplier on its Lamé parameters.

## The data table

Modulus data is reported as a multiplicative factor $M(\phi)$ relative to the base silicone's small-strain $E$. The same fractions the [material-database carbon-black table](../../../appendices/02-material-db.md) ships for conductivity and thermal conductivity also drive the modulus shift.

| Filler fraction $\phi$ | Relative modulus factor $M(\phi)$ | Regime |
|---|---|---|
| 5 wt% | ≈1.2 | Well below percolation |
| 10 wt% | ≈1.6 | Near percolation |
| 15 wt% | ≈2.1 | Above percolation |
| 20 wt% | ≈2.8 | Well above percolation |

The factor multiplies the base silicone's small-strain $E$ — an Ecoflex 00-30 base with 15 wt% carbon-black loading gives an effective $E \approx 125 \times 2.1 = 263$ kPa (using the [Ecoflex mechanical leaf](../00-ecoflex/00-mechanical.md)'s nominal value). The Lamé pair $(\mu, \lambda)$ that the solver consumes scales with $E$ under the near-incompressible $\nu \approx 0.499$ assumption the Ecoflex and Dragon Skin mechanical leaves establish — Poisson ratio does not change meaningfully with carbon-black loading at these fractions, which is why a single scalar factor captures the shift.

The factor is smooth through the percolation threshold. Mechanical stiffening does not depend on whether the particles form a conducting network; it depends on the particles' volume fraction and interfacial interaction with the silicone matrix, both of which evolve continuously with loading.

## The mechanism

Carbon-black particles are rigid relative to the silicone matrix — the particle modulus is several orders of magnitude above silicone's, so the composite's effective modulus is an average weighted toward the matrix at low filler fraction and dragged upward as the filler volume grows. Three mechanisms contribute, in order of increasing importance as $\phi$ rises.

- **Volume-fraction averaging.** The bare Einstein-Smallwood rule predicts $E_\text{composite} / E_\text{matrix} \approx 1 + 2.5\,\phi_\text{vol}$ for dilute rigid spheres in a soft matrix. Weight-to-volume conversion at the silicone-versus-carbon-black density ratio gives a volume fraction somewhat below the weight fraction, and the occluded-rubber correction (rubber trapped inside carbon-black aggregates that moves with the filler rather than the matrix) amplifies the effective filler volume further. At 5 wt% loading the combined effect is consistent with the tabulated 1.2× figure at the Einstein-Smallwood-plus-aggregate dilute limit.
- **Filler–matrix interaction at the interface.** Carbon-black surfaces bind to silicone chains via physisorption, restricting chain mobility near the particle. This adds a stiffening contribution beyond pure volume-fraction averaging, which is why the measured factor at 10 wt% (1.6×) sits above the Einstein-Smallwood prediction. The interfacial effect is what makes the composite's stiffening closer to the Guth-Gold rule $E_\text{composite} / E_\text{matrix} \approx 1 + 2.5\,\phi_\text{vol} + 14.1\,\phi_\text{vol}^2$ than to the purely-dilute form.
- **Particle-network formation near and above percolation.** Above the percolation threshold, carbon-black particles form mechanical contact chains that carry stress directly through rigid-particle-to-rigid-particle paths. This contribution is what drives the factor above 2× at 15 wt% and further at 20 wt%. The mechanical percolation threshold sits close to but not identical to the electrical one, and the stiffening contribution grows continuously rather than jumping at the threshold.

The three mechanisms compound, so the measured factor is an increasing nonlinear function of $\phi$ that a single Guth-Gold-style quadratic captures well enough for the book's default `MaterialField` framing.

## What the sim-soft `Material` stores

A carbon-black-loaded `Material<MooneyRivlin>` on an Ecoflex 00-30 base carries:

- The base silicone's Mooney-Rivlin coefficients ($C_{10}, C_{01}$) from the [Ecoflex hyperelastic-fits leaf](../00-ecoflex/01-hyperelastic-fits.md).
- The filler fraction $\phi$ as a scalar field over the body, sampled per element by the [`MaterialField` machinery of Part 7 Ch 00](../../../70-sdf-pipeline/00-sdf-primitive.md).
- The modulus factor $M(\phi)$ applied multiplicatively to $(C_{10}, C_{01})$ at each sample, matching the Lamé-scaling convention.
- A validity-metadata block declaring the $\phi$ range the factor is calibrated over ($\phi \in [0.05, 0.20]$ at Pass 1, with per-print-calibration refinement expected via the [sim-to-real loop](../../../100-optimization/05-sim-to-real.md)).

The factor lives as a closed-form function of $\phi$ rather than a lookup table. `MaterialField`'s continuous sampling through the SDF pipeline needs the factor to be differentiable in $\phi$ for the [Part 6 implicit-function-theorem gradients](../../../60-differentiability/02-implicit-function.md) to flow through it; a Guth-Gold quadratic with a small strain-amplification correction is differentiable everywhere on the calibrated $\phi$ range.

## The stiff-skin-on-soft-core design pattern

The canonical application of the carbon-black modulus shift is not uniform loading — it is spatially varying loading that creates a stiffer skin on a softer core. The [Part 2 Ch 09 spatial-fields chapter](../../../20-materials/09-spatial-fields.md) is the machinery that supports it: an Ecoflex 00-30 core with a thin carbon-black-loaded Ecoflex 00-30 skin is a single-chemistry, single-cure part with a continuous $\phi(x)$ field that drops from 15 wt% at the surface to 0 wt% at a small depth.

Two things this design pattern buys for the canonical problem.

- **Contact-stiffness tunability without a separate material.** The skin's effective $E \approx 260$ kPa sits between Ecoflex 00-50 and Dragon Skin 10A on the stiffness ladder, but without the cross-family chemistry transition the Dragon Skin-Ecoflex interface would carry. The [compliant-cavity probe-conformity objective](../../../10-physical/01-reward.md) consumes the contact-stiffness profile directly, so spatial carbon-black loading is a way to tune conformity versus abrasion-resistance with one parameter field.
- **Electrical sensing coupled to the stiff region by construction.** The same $\phi$ field that raises the modulus at the skin also raises electrical conductivity (past percolation — see [the conductivity sibling leaf](01-conductivity.md)), putting the sensing layer exactly where the contact pressure concentrates. [Part 10 Ch 06's full design-print-rate loop](../../../100-optimization/06-full-loop.md) uses this pattern to close the sensing loop physically.

## Alternatives considered

**Ship $M(\phi)$ as a lookup table at measured filler fractions.** Rejected because interior points on the $\phi$ range are consumed by the `MaterialField`'s continuous SDF sampling, and the solver's implicit-function-theorem gradient requires differentiable scaling. A closed-form Guth-Gold-style fit is smooth; table lookup with interpolation introduces either piecewise non-smoothness or higher interpolation machinery for no fidelity gain at the canonical-problem scale.

**Use a separate non-carbon-black stiff material (e.g., silica-loaded silicone) for the stiff-skin use case.** Rejected because the electrical-sensing role the carbon-black family plays ([Claim 3](../02-carbon-black.md)) would require a second material with its own percolation threshold and thermal characterization. One filler chemistry handles all three axes (modulus, conductivity, thermal conductivity) with a single $\phi$ field — this is structurally simpler, and the canonical problem's sensing-at-the-stiff-skin pattern is exactly what the single-filler story is built for.

**Treat modulus shift as a constant multiplier across the filler range.** Rejected because the Pass-1 tabulated factor ranges from 1.2 to 2.8, a factor of 2.3 across the 5–20 wt% window; approximating this as a constant would introduce modulus error of up to a factor of 2 at the endpoints, which exceeds the tolerance the [canonical-problem reward terms](../../../10-physical/01-reward.md) resolve.
