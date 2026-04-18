# Mechanical data — 00-10, 00-20, 00-30, 00-50

The [Ecoflex branch](../00-ecoflex.md) picks the family because its four grades span roughly a 4× small-strain modulus range in a single chemistry. This leaf names the specific numbers — modulus, density, Poisson ratio, tensile strength, elongation at break — the [`sim-soft` material database](../../../appendices/02-material-db.md) cites as the nominal values each `MaterialField` calibration starts from, and declares the validity domain per grade. The per-grade hyperelastic curve fits, viscoelastic spectra, and thermal properties are siblings — this leaf is about small-strain elasticity in the quasi-static limit only.

## The data table

The following numbers are the nominal values the book's canonical problem and the [Part 10 Ch 05 sim-to-real loop](../../../100-optimization/05-sim-to-real.md) start from. Every row is traceable to a source or explicitly flagged as a Pass-3 measurement deliverable.

| Property | 00-10 | 00-20 | 00-30 | 00-50 |
|---|---|---|---|---|
| Shore 00 hardness | 10 | 20 | 30 | 50 |
| Small-strain Young's modulus $E$ | ≈55 kPa | ≈85 kPa | ≈125 kPa | ≈220 kPa |
| Density $\rho$ (kg/m³) | ≈1030 | ≈1040 | ≈1070 | ≈1070 |
| Poisson ratio $\nu$ | ≈0.499 | ≈0.499 | ≈0.499 | ≈0.499 |
| Tensile strength at break | ≈800 kPa | ≈1100 kPa | ≈1380 kPa | ≈2170 kPa |
| Elongation at break | ≈800% | ≈845% | ≈900% | ≈980% |
| Neo-Hookean $\mu$ | ≈18 kPa | ≈28 kPa | ≈42 kPa | ≈73 kPa |
| Source — modulus | Shore-correlation | Shore-correlation | Measured (Pass-3 anchor) | Measured (Pass-3 anchor) |
| Source — tensile, elongation | Smooth-On TDS | Smooth-On TDS | Smooth-On TDS | Smooth-On TDS |

The same numbers appear in [`appendices/02-material-db.md`](../../../appendices/02-material-db.md). The appendix row set is authoritative; this table is the provenance-annotated narrative view the downstream sub-chapters cite.

## Shore-to-modulus correlation

Shore 00 hardness is a penetration-resistance measurement, not a modulus measurement. The conversion to $E$ runs through a published Shore-00-to-$E$ correlation of the form used throughout the soft-robotics characterization literature — an empirical monotonic map from the Shore 00 reading to the small-strain Young's modulus, calibrated against a set of published Shore-and-modulus pairs on platinum-cure silicones. The specific functional form and its source are a Pass-3 anchor in [the hyperelasticity reference leaf](../../../appendices/00-references/01-hyperelastic.md); applied to $S_{00} = 10, 20, 30, 50$ it reproduces the modulus row to the precision the data sheets themselves quote.

Two things to know about the correlation at the leaf-level. First, it is a correlation, not a derivation — a best-fit curve through published Shore-and-modulus pairs, not a mechanistic conversion. Lot-to-lot variation on a single grade is within roughly $\pm 15\%$ of the correlation's output, which is larger than the digit precision the table shows. Second, it is less accurate at the very low end of the Shore 00 scale where the durometer's indenter response becomes nonlinear; Ecoflex 00-10 at the edge of the usable durometer range inherits the widest correlation uncertainty, which the [hyperelastic curve fits sub-chapter](01-hyperelastic-fits.md) closes with a measurement deliverable and the [sim-to-real loop](../../../100-optimization/05-sim-to-real.md) corrects per-print.

## Poisson ratio — why ≈0.499 and not 0.5

Ecoflex is incompressible to within the precision any continuum measurement can distinguish, and the book's default near-incompressible constitutive treatment ([Part 2 Ch 05](../../../20-materials/05-incompressibility.md)) is calibrated for $\nu$ in the 0.495–0.499 window. Strict incompressibility $\nu = 0.5$ is rejected for two reasons, both of which [Part 2 Ch 05](../../../20-materials/05-incompressibility.md) argues in full:

- The Lamé parameter $\lambda = E\nu / ((1+\nu)(1-2\nu))$ diverges at $\nu = 0.5$, and a displacement-only linear-elastic formulation locks the mesh well before that. Setting $\nu = 0.499$ keeps $\lambda$ finite and at a $\lambda / \mu$ ratio of $2\nu / (1 - 2\nu) \approx 500$, which is enough to reproduce the near-incompressible response and leaves the mixed u-p formulation headroom.
- Measured data for platinum-cure silicones under hydrostatic compression shows a bulk modulus consistent with $\nu \approx 0.4995$, not exact. The 0.001 gap is below what matters for the canonical problem, but preserving it keeps the model well-posed and matches the data.

The $\nu = 0.499$ value is used across all four grades. The grade ladder varies $\mu$ through the modulus column; the volumetric response does not change meaningfully across the ladder.

## Density — why it shifts across the ladder

The ≈1030 → ≈1070 kg/m³ spread across 00-10 to 00-50 reflects the different silane–siloxane ratios the grades use to reach different cross-link densities. It is small ($\approx 4\%$) but it matters for two reasons the canonical problem exercises. First, inertial terms in the dynamic solver ([Part 5 Ch 00 backward-Euler](../../../50-time-integration/00-backward-euler.md)) scale with $\rho$, and a 4% density error produces a 2% error in first-mode natural frequency — enough to fail the [wobble-failure](../../02-what-goes-wrong/01-wobble.md) regression case. Second, the [thermal properties sibling leaf](03-thermal.md) reads $\rho$ when computing volumetric heat capacity $\rho c_p$, and the $c_p$ values in the manufacturer data sheets assume the grade-specific density.

## Tensile strength and elongation at break

These values come from uniaxial tensile tests in the Smooth-On technical data sheets. They are published at a fixed strain rate (typically 500 mm/min) and a fixed specimen geometry (ASTM D412 die C), and are rate-dependent through the viscoelastic response the [sibling leaf](02-viscoelastic.md) quantifies — quasi-static tensile strength is lower than the data-sheet number, by a factor on the order of the storage-modulus loss at 1 Hz.

For `sim-soft` the two values serve different roles.

- **Elongation at break** is the upper bound of the strain regime the constitutive fits remain valid in. The [hyperelastic curve fits leaf](01-hyperelastic-fits.md) publishes Ogden fits for 00-30 and 00-50 up to stretches of $\lambda \approx 4$, well short of the 9× break elongation, which means the constitutive-law validity domain is data-limited below the geometric break. The book does not use the elongation-at-break number as a validity-domain boundary itself.
- **Tensile strength at break** is the ultimate failure criterion a designer uses to reject a candidate conformity solution. Rejection runs as a smooth barrier, not a hard indicator, following [Part 1 Ch 01 reward composition](../../../10-physical/01-reward/04-composition.md) — the reward term for peak-stress bounds uses the tensile strength as the soft ceiling. `sim-soft` stores the value in the `Material`'s validity metadata; the reward composition reads it at evaluation time.

## Per-grade validity declaration

Each grade carries its own validity statement, consistent with the [Part 2 Ch 00 validity regime requirement](../../../20-materials/00-trait-hierarchy/02-validity.md).

- **00-10** — modulus from Shore correlation, bounded $\pm 20\%$ below $\lambda = 2$; no published uniaxial curve fit the book relies on; Pass-3 measurement deliverable. Viscoelastic and thermal properties assumed to transfer from 00-30 with $E$-scaling.
- **00-20** — same status as 00-10. Neighbouring grade to the two characterized grades; the Shore-correlation error is bracketed rather than extrapolated.
- **00-30** — published hyperelastic fit to $\lambda \approx 4$ ([hyperelastic fits leaf](01-hyperelastic-fits.md)); modulus measurable to $\pm 10\%$ of the correlation value; DMA data available for the viscoelastic baseline.
- **00-50** — published hyperelastic fit to $\lambda \approx 4$; modulus measurement has the lowest correlation uncertainty of the four grades because the durometer response is linear across this region. Canonical-problem default when a single Ecoflex grade is chosen.

The sim-soft `Material` returns these metadata at the `validity()` trait method, and the gradcheck suite ([Part 11 Ch 04 gradcheck sub-chapter](../../../110-crate/04-testing/03-gradcheck.md)) skips grade-specific verification when a call-site exceeds the declared domain.

## Alternatives considered

Two alternative data strategies were considered and rejected at Pass 1:

**Measure every grade, tag every cell.** Rejected because the deliverable slope of the book does not justify measuring Ecoflex 00-10 and 00-20 uniaxially ahead of Phase B. The canonical problem at the default scale lives in the 00-30-to-00-50 band; 00-10/00-20 are walkthroughs on the grade ladder, not primary working materials. Shore-correlation with $\pm 15\%$ uncertainty is honest and leaves the 00-10/00-20 measurement deliverable on the Pass-3 / sim-to-real side of the book where it belongs.

**Publish only characterized grades.** Rejected because the Ecoflex ladder's value is the continuity it provides. Dropping 00-10 and 00-20 from the material database turns the ladder into a two-grade step with a 1.75× jump between 00-30 and 00-50, which defeats the [Part 10 Ch 02 BayesOpt](../../../100-optimization/02-bayesopt.md) "single continuous stiffness parameter" story the branch ([Ecoflex parent](../00-ecoflex.md)) rests on.

The chosen strategy — publish all four, mark provenance per cell, schedule the measurement — is the one the [Part 1 Ch 04 material-data parent](../../04-material-data.md)'s "real measured data, not textbook placeholders" claim is built on.
