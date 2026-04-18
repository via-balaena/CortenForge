# Thermal properties

Dragon Skin's thermal response is close to [Ecoflex's](../00-ecoflex/03-thermal.md) — same cure chemistry family, nearly identical expansion coefficient, nearly identical thermal conductivity — with small drifts the data sheets quote at the per-family level. This leaf names the numbers the [`sim-thermostat` coupling](../../../20-materials/08-thermal-coupling.md) reads when the thermal decorator wraps a Dragon Skin `Material`, and makes explicit the co-cure story that the [Dragon Skin branch's](../01-dragonskin.md) chemistry-continuity claim rests on.

## The data table

| Property | 10A | 20A | 30A |
|---|---|---|---|
| Thermal expansion $\alpha$ (1/K) | ≈3 × 10⁻⁴ | ≈3 × 10⁻⁴ | ≈3 × 10⁻⁴ |
| Thermal conductivity $k$ (W/m·K) | ≈0.18 | ≈0.19 | ≈0.19 |
| Specific heat $c_p$ (J/kg·K) | ≈1400 | ≈1400 | ≈1400 |
| Volumetric heat capacity $\rho c_p$ (MJ/m³·K) | ≈1.50 | ≈1.51 | ≈1.51 |
| Source — $\alpha$, $k$, $c_p$ | Smooth-On TDS + PDMS literature | same | same |
| Glass-transition temperature $T_g$ | ≈ −120 °C | same | same |

The thermal values cluster tightly across the Dragon Skin grade ladder — within a couple of percent on each property, smaller than the data-sheet precision — and sit very close to the [Ecoflex thermal values](../00-ecoflex/03-thermal.md) across the family boundary. The small shifts that do exist run in the direction a denser-cross-linking mechanism predicts: slightly higher thermal conductivity (the denser network conducts phonons marginally better), slightly higher volumetric heat capacity (driven by the slightly higher density).

## Thermal expansion coefficient

$\alpha \approx 3 \times 10^{-4}\,\mathrm{K}^{-1}$, effectively the same as Ecoflex to within the precision the data sheets quote. The [multiplicative thermal-deformation split](../../../20-materials/08-thermal-coupling/01-expansion.md) reads this value the same way for Dragon Skin as for Ecoflex — an Ecoflex-Dragon-Skin multi-material part pre-warmed by 20 K expands uniformly at $1 + \alpha \Delta T = 1.006$ across both material regions, with no differential expansion stress at the co-cure interface. This is part of what makes the chemistry-continuity story operationally usable: thermal loading does not preferentially stress the cross-family interface.

Per-batch $\alpha$ refinement is a Pass-3 deliverable for applications where the thermal gradient across a multi-material part matters at a precision tighter than the data-sheet precision supports.

## Thermal conductivity

$k \approx 0.18$–$0.19\,\mathrm{W/m \cdot K}$, modestly higher than Ecoflex's 0.17–0.18 band. The difference is at the edge of what matters for the canonical-problem transient timescale, but it is systematic — Dragon Skin conducts heat marginally faster than Ecoflex does at the same thickness, consistent with the denser cross-link network. For a multi-material part with a Dragon Skin core and an Ecoflex skin, the steady-state temperature distribution under a distributed heat source is slightly more uniform than a pure-Ecoflex part would be, and [dissipative heating](../../../20-materials/08-thermal-coupling/02-dissipation.md) accumulates slightly more slowly in the Dragon Skin regions.

## Specific heat and volumetric heat capacity

$c_p \approx 1400\,\mathrm{J/kg\cdot K}$ across the ladder, same as the Ecoflex family. $\rho c_p$ shifts upward by a few percent across the Dragon Skin ladder relative to the softer Ecoflex grades (because Dragon Skin's ≈1070–1080 kg/m³ density sits a touch above Ecoflex's 1030–1070 band), with the shift varying by which grade-pair is compared. `sim-thermostat`'s diffusion step consumes $\rho c_p$ directly per-element, so the volumetric-heat-capacity shift lands in the diffusion solver without any special handling at the material-family boundary.

## Glass-transition temperature and validity

$T_g \approx -120\,\mathrm{°C}$, the same as Ecoflex and as platinum-cure PDMS generally. The `Material<Thermal<MooneyRivlin>>` decorator ([Part 2 Ch 08 Claim 1](../../../20-materials/08-thermal-coupling.md)) for Dragon Skin declares the same $T \in [-20\,\mathrm{°C}, +150\,\mathrm{°C}]$ working-range validity as the Ecoflex decorators — the platinum-cure chemistry's working range is a family-level property, not a grade-level one, so Dragon Skin inherits the range unchanged.

## Co-cure and multi-material thermal behaviour

The [Dragon Skin branch's Claim 1](../01-dragonskin.md) rests on the chemistry-continuity with Ecoflex: same platinum-cure chemistry, compatible cure profiles, robust co-cure bond. The thermal-property near-match reinforces the claim operationally. At the thermal-decorator level, a multi-material part mixing Ecoflex and Dragon Skin regions has:

- **No thermal-expansion mismatch stress.** $\alpha$ is identical at the Ecoflex-Dragon Skin interface, so a temperature change produces uniform expansion across the part.
- **A smooth thermal-conductivity gradient.** $k$ shifts by a few percent across the family boundary, which produces a mild discontinuity in the steady-state heat-flux profile but no thermal-stress concentration at the interface.
- **A continuous volumetric heat capacity.** $\rho c_p$ varies by a few percent across the family boundary, which means the transient thermal diffusion step runs without any interface-specific treatment.

This is a different thermal-interface story from the one the [carbon-black family sibling leaf](../02-carbon-black/02-thermal.md) tells: carbon-black composites change thermal conductivity by a multiplicative factor relative to the base silicone, which does introduce thermal-stress concentrations at the composite-pure-silicone boundary, and the [Part 2 Ch 08 thermal-coupling chapter](../../../20-materials/08-thermal-coupling.md) handles the composite case with extra care. The Ecoflex-Dragon Skin case is clean.

## Alternatives considered

**Ship separate $(k, c_p)$ values per grade.** The per-grade drift within Dragon Skin is below the data-sheet precision the values are quoted at; shipping per-grade thermal rows adds noise without adding information. The single-family set is honest.

**Defer thermal properties to Pass 3 per-batch calibration.** Rejected because the canonical-problem transient regime does not require precision tighter than the data-sheet values support, and the [sim-thermostat](../../../20-materials/08-thermal-coupling.md) coupling needs numbers at Phase B for the validation runs that verify the coupling is bidirectional in the first place. The Pass-1 numbers are the starting anchor; Pass-3 per-batch measurement refines for specific applications.

**Model the Dragon Skin thermal response as a shift from the Ecoflex thermal baseline (parameterized by the chemistry difference).** Intellectually tempting, but shipped parameter tables tend to duplicate the baseline-plus-shift decomposition anyway at the `MaterialField` consumption point, and naming the Dragon Skin values directly is simpler for downstream consumers. The book ships direct values and notes the family-level near-match in prose.
