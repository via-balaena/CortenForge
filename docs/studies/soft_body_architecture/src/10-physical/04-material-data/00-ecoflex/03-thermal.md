# Thermal properties

The Ecoflex branch's [thermal envelope](../00-ecoflex.md) is narrow and predictable — small expansion coefficient, low thermal conductivity, modest specific heat — which is why the silicone family works as a substrate for [`sim-thermostat`'s](../../../20-materials/08-thermal-coupling.md) bidirectional coupling at all. This leaf names the specific numbers the thermal decorator reads, names their provenance, and declares the regime in which they are valid.

## The data table

| Property | 00-10 | 00-20 | 00-30 | 00-50 |
|---|---|---|---|---|
| Thermal expansion $\alpha$ (1/K) | ≈3 × 10⁻⁴ | ≈3 × 10⁻⁴ | ≈3 × 10⁻⁴ | ≈3 × 10⁻⁴ |
| Thermal conductivity $k$ (W/m·K) | ≈0.17 | ≈0.17 | ≈0.18 | ≈0.18 |
| Specific heat $c_p$ (J/kg·K) | ≈1400 | ≈1400 | ≈1400 | ≈1400 |
| Volumetric heat capacity $\rho c_p$ (MJ/m³·K) | ≈1.44 | ≈1.46 | ≈1.50 | ≈1.50 |
| Source — $\alpha$, $k$, $c_p$ | Smooth-On TDS + PDMS literature | same | same | same |
| Glass-transition temperature $T_g$ | ≈ −120 °C | same | same | same |

The $(\alpha, k, c_p)$ row triple is the minimum `sim-thermostat` needs to compute the heat-diffusion step the [Part 2 Ch 08 thermal-expansion leaf](../../../20-materials/08-thermal-coupling/01-expansion.md) and [dissipation leaf](../../../20-materials/08-thermal-coupling/02-dissipation.md) consume. All three are flat-across-the-ladder to within the precision the data-sheet numbers are quoted at — the grade ladder is a cross-link-density ladder, not a thermal-conductivity ladder, so the thermal properties shift little across it.

## Thermal expansion coefficient

$\alpha \approx 3 \times 10^{-4}\,\mathrm{K}^{-1}$ is the single number the [Part 2 Ch 08 expansion leaf](../../../20-materials/08-thermal-coupling/01-expansion.md) reads when it constructs the multiplicative thermal-deformation split

$$ F = F_\text{mech}\,F_\text{therm}, \qquad F_\text{therm} = (1 + \alpha (T - T_0))\,I $$

with $T_0$ the reference temperature at which $F_\text{therm} = I$. For $\Delta T = 20\,\mathrm{K}$ this gives a linear-expansion factor of $1.006$ — a 0.6% length-scale change, which is measurable but small relative to the stretches the canonical-problem cavity-conformity sees. Where thermal expansion matters is the no-load-but-warm regime: an Ecoflex seal pre-warmed by 20 K before a probe is inserted starts from a geometry 0.6% larger than the room-temperature design, and the reward terms in [Part 1 Ch 01](../../../10-physical/01-reward.md) score the conforming fit off the expanded geometry, not the reference.

Two caveats on $\alpha$ for Ecoflex.

- The value is a manufacturer-data-sheet number cross-checked against the PDMS literature's characterization of platinum-cure silicones; it is within the ≈$2.8$–$3.2 \times 10^{-4}\,\mathrm{K}^{-1}$ band the literature reports for this chemistry class. The book uses the round value; a per-batch measurement is Pass-3 refinement.
- The coefficient is isotropic at the grade-ladder level. Carbon-black-loaded composites reduce $\alpha$ anisotropically; [the carbon-black family sibling leaf](../02-carbon-black/02-thermal.md) handles that.

## Thermal conductivity

$k \approx 0.17$–$0.18\,\mathrm{W/m \cdot K}$ across the ladder. This is in the low end of the polymer range — silicone is a thermal insulator — and it is what makes dissipative heating matter for extended-load canonical-problem runs. A viscoelastic Ecoflex cavity pulsed at human-hand-cyclic frequencies for minutes accumulates heat in the cavity wall faster than the wall can conduct it to the probe and the ambient, which is the coupling loop [dissipative heating](../../../20-materials/08-thermal-coupling/02-dissipation.md) closes.

The canonical-problem short-transient regime the book tests primarily in (Phase-B-through-D fidelity validation) leaves the heat-diffusion step a small correction. The extended-load regime for [Part 10 Ch 05 sim-to-real](../../../100-optimization/05-sim-to-real.md) calibration is where the low conductivity dominates the thermal steady state.

## Specific heat and volumetric heat capacity

$c_p \approx 1400\,\mathrm{J/kg\cdot K}$ is consistent across the Ecoflex ladder. The volumetric heat capacity $\rho c_p$ is what `sim-thermostat` actually consumes in its diffusion step — conservation of energy on an element of mass $\rho V$ is $\rho V c_p \dot T = k \nabla^2 T \cdot V + \dot q$. Because $\rho$ is not flat across the ladder (the [mechanical-data leaf](00-mechanical.md) carries 1030–1070 kg/m³), the volumetric heat capacity shifts by ≈4% across the ladder, which is at the edge of what matters and is tracked grade-by-grade.

The [dissipative heating leaf](../../../20-materials/08-thermal-coupling/02-dissipation.md) writes the heat source term $\dot q = \sigma : \dot\varepsilon_\text{visc}$, which comes from the [viscoelastic sibling leaf](02-viscoelastic.md)'s Prony-branch internal forces. The loop closes: elastic work → viscoelastic hysteresis → dissipative heat rate → `sim-thermostat` diffusion → temperature field → modulus modulation via the [temperature-dependent modulus leaf](../../../20-materials/08-thermal-coupling/00-modulus-T.md) → back into the elastic response.

## Glass-transition temperature

$T_g \approx -120\,\mathrm{°C}$ for platinum-cure PDMS. It is well below any regime the canonical problem operates in, and the book's thermal-coupling validity domain is bounded well above it — the material behaves as a rubbery elastomer throughout its working range. The value is published here as the lower thermal-validity limit: below $T_g$ the elastic response is a different material entirely, a glassy PDMS with a modulus three orders of magnitude larger than the rubbery-state modulus the [mechanical-data leaf](00-mechanical.md) tabulates. The `Material`'s validity metadata declares $T \in [-20\,\mathrm{°C}, +150\,\mathrm{°C}]$ as the declared thermal range; $[-120, -20]$ is out-of-domain and not shipped.

## Per-grade validity

The thermal-property row set is the least grade-sensitive of the four Ecoflex data leaves. Every value in the table transfers across the ladder within ≈5% — smaller than the data-sheet precision the values are quoted at. This is the single-set-transfers-across-ladder claim that lets the book use a single set of thermal parameters in `sim-thermostat` for the whole Ecoflex family, with per-grade refinement deferred to Pass 3 for specific applications where thermal behaviour is the critical degree of freedom (e.g., resistively-heated soft actuators built on the carbon-black composites rather than pure Ecoflex).

## What the sim-soft `Material` stores

The `Material<Thermal<Ogden<3>>>` decorator ([Part 2 Ch 08 Claim 1](../../../20-materials/08-thermal-coupling.md)) holds the $(\alpha, k, c_p, T_0)$ quadruple alongside the elastic Ogden coefficients and the viscoelastic Prony parameters. The reference temperature $T_0$ is the design-time ambient temperature the `MaterialField` was calibrated at, and defaults to 20 °C for the canonical problem.

The temperature-dependent modulus term (softening at ≈0.5–2% per °C above $T_0$, per the [Part 2 Ch 08 modulus-T leaf](../../../20-materials/08-thermal-coupling/00-modulus-T.md)) is *not* a direct Ecoflex data-sheet number; it is a regime-level correction the book handles in the constitutive-form decorator, with coefficients calibrated at Pass 3 from the DMA data's frequency–temperature shift factors. The Ecoflex family data on this page is the static-temperature slice at the reference value; the temperature-dependence is the decorator's domain.

## Alternatives considered

**Treat thermal properties as non-material and read them from an ambient scalar.** Every `Material` reads the same $(\alpha, k, c_p)$. Rejected because the carbon-black family's thermal properties diverge sharply from the pure silicone family's — a designer mixing Ecoflex-only and carbon-black-loaded regions in the same mesh (which is exactly the compliant-cavity-with-a-sensor-lining problem) needs per-material thermal parameters, not a global ambient.

**Ship lot-specific thermal numbers out of the gate.** Per-batch measurement of $(\alpha, k, c_p)$ for every Ecoflex lot. Rejected as a Pass-1 deliverable because the lot-to-lot variance on thermal properties is below what the coupling loop resolves at the canonical-problem's transient timescale; deferring to [sim-to-real calibration](../../../100-optimization/05-sim-to-real.md) for extended-load cases is honest.

**Decouple mechanical and thermal timesteps (multi-rate solver).** `sim-thermostat` runs at a slower step than `sim-soft` because the thermal diffusion timescale is longer than the mechanical oscillation timescale. Rejected at Pass 1 for the default configuration — the [Part 5 Ch 03 coupling fixed-point-iteration leaf](../../../50-time-integration/03-coupling/01-fixed-point.md) exchanges temperature and displacement across sub-steps, and a unified timestep is simpler to validate. Multi-rate is reachable as a Pass-3 upgrade if the coupled run time becomes the bottleneck in [Phase E](../../../110-crate/03-build-order.md).
