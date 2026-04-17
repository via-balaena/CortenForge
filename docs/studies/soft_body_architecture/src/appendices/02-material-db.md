# Material property database

Reference tables for the three material families [Part 1 Ch 04](../10-physical/04-material-data.md) commits to: the Ecoflex platinum-cure silicone family, the Dragon Skin platinum-cure silicone family, and carbon-black-loaded conductive composites. Part 1 Ch 04 is the narrative treatment — *why* these families, *how* the numbers were measured, *what* the limits of each family are. This appendix is the reference-table form — a dense lookup of nominal values each `MaterialField` calibration starts from.

## Scope of this table

Pass 1 ships the nominal-value rows the [Part 9 Ch 00 shipping commitment](../90-visual/00-sss.md) binds the appendix to: Ecoflex 00-10, 00-20, 00-30, 00-50; Dragon Skin 10A, 20A, 30A; carbon-black composites. Pass 3 expands the table with measurement sources (per-value citation), full Ogden and Prony coefficient sets, temperature-dependent modulus coefficients, and the full measured diffusion profiles.

Nominal values below are drawn from manufacturer technical data sheets at standard conditions (23 °C, ~50% relative humidity, quasi-static) where available, and from the published characterization studies [Part 1 Ch 04](../10-physical/04-material-data.md) names in prose where not. Entries marked *Pass 3* are slots reserved for the full-depth bibliography pass.

## Ecoflex family (Smooth-On, platinum-cure)

| Property | 00-10 | 00-20 | 00-30 | 00-50 |
|---|---|---|---|---|
| Shore hardness | 00-10 | 00-20 | 00-30 | 00-50 |
| Young's modulus $E$ (small-strain) | ≈55 kPa | ≈85 kPa | ≈125 kPa | ≈220 kPa |
| Density $\rho$ (kg/m³) | 1030 | 1040 | 1070 | 1070 |
| Poisson ratio $\nu$ | ≈0.499 | ≈0.499 | ≈0.499 | ≈0.499 |
| Tensile strength | ≈800 kPa | ≈1100 kPa | ≈1380 kPa | ≈2170 kPa |
| Elongation at break | ≈800% | ≈845% | ≈900% | ≈980% |
| Neo-Hookean $\mu$ | ≈18 kPa | ≈28 kPa | ≈42 kPa | ≈73 kPa |
| Neo-Hookean $\lambda$ (near-incompressible) | large | large | large | large |
| Ogden $N=3$ coefficients | Pass 3 | Pass 3 | Pass 3 | Pass 3 |
| Prony series $(\tau_i, g_i)$ | Pass 3 | Pass 3 | Pass 3 | Pass 3 |
| Thermal expansion $\alpha$ (1/°C) | ≈3 × 10⁻⁴ | ≈3 × 10⁻⁴ | ≈3 × 10⁻⁴ | ≈3 × 10⁻⁴ |
| Thermal conductivity (W/m·K) | ≈0.17 | ≈0.17 | ≈0.18 | ≈0.18 |
| Diffusion profile reference | Pass 3 measured | Pass 3 measured | Pass 3 measured | Pass 3 measured |

Source convention: small-strain $E$ is computed from Shore 00 hardness via the standard correlation and is consistent with manufacturer data-sheet moduli. Neo-Hookean $\mu$ is $E / (2 (1 + \nu))$ at the near-incompressible $\nu$. Ogden and Prony entries are deliberately deferred to Pass 3 — the specific published fits (Marechal et al. for 00-30; Liao et al. for 00-50) require a source-check the Pass 1 process has not run. See [Part 1 Ch 04 — Ecoflex](../10-physical/04-material-data/00-ecoflex.md) for the narrative treatment.

## Dragon Skin family (Smooth-On, platinum-cure)

| Property | 10A | 20A | 30A |
|---|---|---|---|
| Shore hardness | 10A | 20A | 30A |
| Young's modulus $E$ (small-strain) | ≈300 kPa | ≈700 kPa | ≈1.5 MPa |
| Density $\rho$ (kg/m³) | 1070 | 1080 | 1080 |
| Poisson ratio $\nu$ | ≈0.499 | ≈0.499 | ≈0.499 |
| Tensile strength | ≈3.3 MPa | ≈3.8 MPa | ≈3.6 MPa |
| Elongation at break | ≈1000% | ≈620% | ≈364% |
| Neo-Hookean $\mu$ | ≈100 kPa | ≈230 kPa | ≈500 kPa |
| Neo-Hookean $\lambda$ (near-incompressible) | large | large | large |
| Ogden $N=3$ coefficients | Pass 3 | Pass 3 | Pass 3 |
| Prony series $(\tau_i, g_i)$ | Pass 3 | Pass 3 | Pass 3 |
| Thermal expansion $\alpha$ (1/°C) | ≈3 × 10⁻⁴ | ≈3 × 10⁻⁴ | ≈3 × 10⁻⁴ |
| Thermal conductivity (W/m·K) | ≈0.18 | ≈0.19 | ≈0.19 |
| Diffusion profile reference | Pass 3 measured | Pass 3 measured | Pass 3 measured |

Source convention: same as Ecoflex. Shore-A-to-modulus correlation differs from Shore-00 and is applied at the higher durometer range. See [Part 1 Ch 04 — Dragon Skin](../10-physical/04-material-data/01-dragonskin.md) for the narrative treatment and the applicable modulus-correlation formula.

## Carbon-black-loaded composites

Base elastomer is either Ecoflex 00-30 or Dragon Skin 10A; the carbon-black loading modifies the mechanical, electrical, and thermal properties. Percolation threshold is at ≈8–10 wt% carbon-black for these base silicones; below threshold the composite is insulating, above threshold it is a resistive conductor whose conductivity rises steeply with loading.

| Property | 5 wt% | 10 wt% | 15 wt% | 20 wt% |
|---|---|---|---|---|
| Regime | Below percolation | Near percolation | Above percolation | Well above percolation |
| Relative modulus factor vs. base | ≈1.2 | ≈1.6 | ≈2.1 | ≈2.8 |
| Electrical conductivity (S/m) | < 10⁻⁸ | ≈10⁻³ | ≈10⁰ | ≈10¹ |
| Thermal conductivity factor vs. base | ≈1.1 | ≈1.3 | ≈1.6 | ≈2.0 |
| Density factor vs. base | ≈1.04 | ≈1.08 | ≈1.12 | ≈1.16 |
| Shore hardness shift | +2 | +5 | +8 | +12 |
| Ogden coefficients | Pass 3 | Pass 3 | Pass 3 | Pass 3 |
| Prony series | Pass 3 | Pass 3 | Pass 3 | Pass 3 |
| Diffusion profile reference | Pass 3 measured | Pass 3 measured | Pass 3 measured | Pass 3 measured |

The "relative modulus factor" and "thermal conductivity factor" columns multiply the base silicone's entry in the corresponding family table above. Values are nominal — carbon-black composite behaviour depends sensitively on dispersion quality, cure profile, and aging, and per-print calibration via the [Part 10 Ch 05 sim-to-real loop](../100-optimization/05-sim-to-real.md) is expected.

See [Part 1 Ch 04 — Carbon-black](../10-physical/04-material-data/02-carbon-black.md) for the narrative on the percolation transition, sensing and Joule-heating applications, and the thermal-coupling bridge to [`sim-thermostat`](../20-materials/08-thermal-coupling.md).

## Calibration note

Every value in this appendix is a *starting point*, not a calibrated per-print value. The [Part 10 Ch 05 sim-to-real machinery](../100-optimization/05-sim-to-real.md) expects per-batch refinement: the designer prints a reference coupon, runs the measurement protocol, and the residual GPs adjust the simulation's effective parameters toward the specific batch's realized behaviour. The table here is the anchor the calibration loop starts from; it is not the final number the optimizer consumes.

## Source-fetch status

Pass 1 entries are drawn from manufacturer data sheets and summarized correlations. Pass 3 will replace "Pass 3" cells with specifically cited fits (anchor-linked to entries in [the hyperelasticity reference leaf](00-references/01-hyperelastic.md)), replace the qualitative "large $\lambda$" with concrete bulk-modulus numbers, and add the full per-batch diffusion profiles the [Part 9 Ch 00 SSS shader](../90-visual/00-sss.md) consumes. The row set itself (locked by Part 9 Ch 00) will not change.
