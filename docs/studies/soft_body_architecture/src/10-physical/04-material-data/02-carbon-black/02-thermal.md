# Concentration → thermal conductivity

Thermal conductivity is the third axis the carbon-black filler fraction moves. Like [modulus](00-modulus.md) and unlike [electrical conductivity](01-conductivity.md), the thermal-conductivity shift is smooth, monotonic, and structurally continuous across the [percolation threshold](03-percolation.md) — the carbon-black network does not need to be fully connected for thermal transport, because phonons and diffusive heat flow through the silicone matrix as well as through the carbon-black chains.

## The data table

Thermal conductivity is reported as a multiplicative factor on the base silicone's $k$, following the same convention the [modulus leaf](00-modulus.md) uses.

| Filler fraction $\phi$ | Thermal conductivity factor $K(\phi)$ | Approximate composite $k$ (W/m·K, Ecoflex 00-30 base) |
|---|---|---|
| 5 wt% | ≈1.1 | ≈0.20 |
| 10 wt% | ≈1.3 | ≈0.23 |
| 15 wt% | ≈1.6 | ≈0.29 |
| 20 wt% | ≈2.0 | ≈0.36 |

The factor $K(\phi)$ multiplies the base silicone's thermal conductivity ($k \approx 0.18\,\mathrm{W/m \cdot K}$ for Ecoflex 00-30, per the [Ecoflex thermal leaf](../00-ecoflex/03-thermal.md)) to give the composite's effective $k$. The 2× factor at 20 wt% moves the composite from silicone-insulator territory toward the low end of the filled-elastomer range but does not approach the carbon-black particles' intrinsic bulk thermal conductivity — the silicone matrix still dominates thermal transport at these fractions.

Specific heat $c_p$ shifts modestly with filler fraction (carbon-black's $c_p$ is about half of silicone's, so $c_p$ of the composite drops weakly with loading). Density rises with loading (carbon-black is denser than silicone). The volumetric heat capacity $\rho c_p$ that `sim-thermostat`'s diffusion step actually consumes shifts by a few percent across the 5–20 wt% range — small enough that the composite's $c_p$ and $\rho$ can be treated per-grade at the Ecoflex/Dragon Skin level and adjusted by $K(\phi)$ on $k$ alone at the Pass-1 precision the material-database carries.

## The mechanism

Thermal transport in a carbon-black-loaded silicone is a mixed mechanism dominated by the matrix at low filler fraction and shifting toward mixed-path transport at higher fractions. Three contributions.

- **Matrix-dominated diffusion.** At low $\phi$ the silicone matrix carries most of the heat, and the filler particles act as high-conductivity inclusions that modestly raise the effective conductivity by the Maxwell-Eucken mean-field formula. For 5 wt% loading the predicted factor is near 1.1, matching the tabulated value.
- **Particle-chain phonon transport.** As $\phi$ rises and particles cluster into chains and networks (below, at, and above the electrical percolation threshold), heat can flow through carbon-particle-to-carbon-particle contacts via phonon transport — a mechanism that does not require the network to be fully connected to contribute, unlike electrical conduction which requires a spanning path. This is the reason thermal conductivity rises smoothly through the percolation band while electrical conductivity jumps.
- **Interfacial thermal resistance.** The silicone-carbon-black interface has a nonzero thermal boundary resistance — heat flowing from particle to matrix loses efficiency at the interface. At high filler fractions where interface density is high, this resistance keeps the composite's thermal conductivity well below the carbon-black bulk value. It is the reason the factor at 20 wt% is only 2× rather than the much larger value a simple volume-fraction average would predict.

A two-parameter semi-empirical fit (Maxwell-Eucken at low fraction, Nielsen-style correction at high fraction) captures the factor across the 5–20 wt% range to within the precision the material-database values are quoted at.

## What the sim-soft `Material` stores

A carbon-black-loaded `Material` carries thermal transport in its `ThermalProperties` sub-field alongside the mechanical and electrical properties:

- The shared filler-fraction field $\phi$ from [the branch `MaterialField` framing](../../../20-materials/09-spatial-fields.md).
- The thermal-conductivity factor $K(\phi)$ as a closed-form expression — smooth, differentiable, and valid across the whole 5–20 wt% range (no threshold behaviour to work around).
- The base silicone's $c_p$ and $\rho$ carried per-grade with a small multiplicative correction for filler loading.
- A validity-metadata block declaring the calibrated $\phi$ range, same pattern as the [modulus leaf](00-modulus.md).

Unlike the conductivity leaf's `ElectricalProperties` field, the `ThermalProperties` field does not carry a threshold flag or an FD-wrapper engagement — thermal conductivity is smooth across the percolation band, so the [implicit-function-theorem gradient](../../../60-differentiability/02-implicit-function.md) flows through $K(\phi)$ without special handling.

## Joule heating closes the electrical-to-thermal loop

The thermal-conductivity leaf is the consumer side of the [Joule-heating loop](01-conductivity.md) the conductivity sibling names. When current flows through a resistive carbon-black region, it dissipates as heat at a rate $\sigma_e |E|^2$ per unit volume, which enters [`sim-thermostat`](../../../20-materials/08-thermal-coupling.md) as a heat source. The dissipated heat then diffuses according to the local $k(x)$ — which is exactly $K(\phi(x)) \times k_\text{base}$ at each point — and the temperature distribution that results feeds back into the [temperature-dependent modulus](../../../20-materials/08-thermal-coupling/00-modulus-T.md) and the [thermal expansion](../../../20-materials/08-thermal-coupling/01-expansion.md) of the silicone matrix.

Two consequences for the canonical problem's design flow.

- **A high-$\phi$ region is both an electrical-current path and a thermal hot spot.** The same carbon-black skin the [modulus leaf](00-modulus.md) names as a stiffening layer, when run above the percolation threshold and driven electrically, also heats. The local temperature rise softens the silicone matrix, which softens the composite's mechanical response, which changes the contact-conformity the cavity is designed for. This is the multi-physics coupling loop Joule heating is supposed to exercise.
- **The thermal-diffusion gradient sharpens at the composite-base interface.** A carbon-black skin sitting on a pure-Ecoflex core has $k$ dropping by factor of 1.6–2.0 across the interface. This produces a thermal-conductivity-discontinuity that [`sim-thermostat`](../../../20-materials/08-thermal-coupling.md)'s per-element diffusion solver resolves through the [spatial-fields machinery](../../../20-materials/09-spatial-fields.md) as a normal per-element transport-coefficient shift, not as a special-cased interface. The resulting steady-state temperature profile has a kink at the interface that is physically correct and numerically well-behaved.

## Alternatives considered

**Use a lookup table for $K(\phi)$.** Same rejection rationale as the [modulus leaf](00-modulus.md): the [`MaterialField`](../../../70-sdf-pipeline/00-sdf-primitive.md)'s continuous SDF sampling consumes $K(\phi)$ at interpolated $\phi$ values, and the implicit-function-theorem gradient needs smooth differentiability. A Maxwell-Eucken-plus-Nielsen closed-form fit gives both without interpolation machinery.

**Ship only the composite's effective $k$ per measured filler fraction without the factor framing.** Rejected because the factor framing is what keeps the carbon-black family composable with both Ecoflex and Dragon Skin base elastomers — the factor is the composite-level shift that the base-material's $k$ gets multiplied by. Direct-$k$-per-fraction tables would need a separate table for each base, which duplicates the data pointlessly.

**Omit the thermal axis entirely and rely on the base silicone's $k$.** Rejected because the 2× factor at 20 wt% is not negligible for the Joule-heating regime's thermal transient timescale — the heat that electrical dissipation deposits diffuses at a rate proportional to $k$, and a 2× change in $k$ is a factor-of-two change in the thermal-relaxation timescale. This is material for the multi-physics coupling the [branch Claim 3](../02-carbon-black.md) is built on.
