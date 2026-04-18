# Mechanical data — 10A, 20A, 30A

The [Dragon Skin branch](../01-dragonskin.md) picks the family for its tougher-and-tear-resistant position above Ecoflex on the durometer scale, sharing the same platinum-cure chemistry. This leaf names the per-grade mechanical numbers — modulus, density, Poisson ratio, tensile strength, elongation at break — the [`sim-soft` material database](../../../appendices/02-material-db.md) cites, and declares the stretch-versus-toughness trade that makes the three-grade ladder narrower than Ecoflex's four.

## The data table

| Property | 10A | 20A | 30A |
|---|---|---|---|
| Shore A hardness | 10 | 20 | 30 |
| Small-strain Young's modulus $E$ | ≈300 kPa | ≈700 kPa | ≈1.5 MPa |
| Density $\rho$ (kg/m³) | ≈1070 | ≈1080 | ≈1080 |
| Poisson ratio $\nu$ | ≈0.499 | ≈0.499 | ≈0.499 |
| Tensile strength at break | ≈3.3 MPa | ≈3.8 MPa | ≈3.6 MPa |
| Elongation at break | ≈1000% | ≈620% | ≈364% |
| Neo-Hookean $\mu$ | ≈100 kPa | ≈230 kPa | ≈500 kPa |
| Source — modulus | Shore-A correlation | Shore-A correlation | Shore-A correlation (Pass-3 anchor) |
| Source — tensile, elongation | Smooth-On TDS | Smooth-On TDS | Smooth-On TDS |

The same numbers live in [`appendices/02-material-db.md`](../../../appendices/02-material-db.md). The appendix row set is authoritative; this table is the provenance-annotated narrative view the downstream sub-chapters cite.

## Shore-A-to-modulus correlation

Shore A is a different durometer scale from Shore 00, targeting the firmer end of the elastomer range. The Shore-A-to-$E$ correlation the soft-robotics and industrial-elastomer literature converges on is a published empirical form distinct from the Shore-00 correlation in the [Ecoflex mechanical-data sibling leaf](../00-ecoflex/00-mechanical.md) — the functional shape is related but the constants differ, and the two correlations should not be interchanged. The specific form and source anchor are a Pass-3 slot in [the hyperelasticity reference leaf](../../../appendices/00-references/01-hyperelastic.md); applied to $S_A = 10, 20, 30$ the correlation reproduces the modulus row to the precision the data sheets themselves quote.

Two properties of the Shore-A correlation worth naming explicitly.

First, it is more accurate than the Shore-00 correlation across its working range. Shore A durometer readings above ~10 correspond to indenter regimes where the response is more nearly linear in modulus than the Shore 00 scale's low-end readings — the durometer's design target — so the correlation's output is closer to lot-measured modulus than the Shore 00 equivalent. Lot-to-lot variation on a Dragon Skin grade is typically within $\pm 10\%$ of the correlation's output, tighter than the Ecoflex band.

Second, it saturates near Shore A 100 and is not valid beyond. Dragon Skin stops at 30A because above that the material enters a harder-rubber regime where the constitutive story shifts (strain-hardening shapes change, near-incompressibility is less precise, and the [hyperelastic-fits leaf](01-hyperelastic-fits.md)'s published characterization coverage drops). The `Material` validity metadata bounds Dragon Skin usage at Shore A 30A by declaration, not by hard runtime block.

## Poisson ratio and near-incompressibility

$\nu \approx 0.499$ across all three grades, the same value the [Ecoflex sibling](../00-ecoflex/00-mechanical.md) uses, rejected from strict $\nu = 0.5$ for the Lamé-divergence and locking reasons that [Part 2 Ch 05](../../../20-materials/05-incompressibility.md) walks in full. Dragon Skin's higher modulus does not change the volumetric response meaningfully — the silane–siloxane cross-link density shifts $\mu$ substantially but leaves the bulk-to-shear ratio in the near-incompressible regime. The $\lambda / \mu \approx 500$ ratio of the [Ecoflex mechanical leaf](../00-ecoflex/00-mechanical.md) carries across, with both numerator and denominator scaled by the grade's $E$.

## Density

The 1070 → 1080 kg/m³ drift across the ladder reflects the same cross-link-density mechanism as Ecoflex's 1030 → 1070 drift, shifted up by the harder grades' different formulation. The inertial-term implications are the same as in the [Ecoflex mechanical leaf](../00-ecoflex/00-mechanical.md) — first-mode frequency scales as $\omega \propto 1/\sqrt{\rho}$ — but the Dragon Skin range is tighter (≈1%), which matters less for natural-frequency calibration and the [wobble-failure regression test](../../02-what-goes-wrong/01-wobble.md) reproduces cleanly on a single-density Dragon Skin model.

## Tensile strength and elongation at break — the toughness trade

This is where the Dragon Skin ladder shows its distinguishing character. Tensile strength at break sits in a narrow ~3.3–3.8 MPa band across all three grades with no monotonic trend — harder grades do not break at meaningfully different stress than the softer grades, a departure from what a single strength-versus-stiffness correlation would predict. What varies instead is the elongation at which break occurs: 10A reaches ≈1000% elongation before tearing, 20A drops to ≈620%, 30A drops further to ≈364%.

The mechanism is that the harder grades' denser cross-linking reduces chain extensibility — each polymer chain between adjacent cross-links is shorter, so the chain network reaches its limiting stretch at a smaller overall material stretch. At the chain-limit, stress rises steeply until the intrinsic tensile strength is reached; the intrinsic strength itself is a chemistry-level property that does not shift much across the grade ladder, which is why the break-stress band is narrow while the break-stretch band is wide. For the canonical-problem cycling-under-squeeze regime this matters in two directions at once.

- **Stiffer is not always tougher.** A designer optimizing for a small-displacement cavity that reaches only 50% stretch finds 30A a perfectly adequate material at its smaller stretch-at-break, because the operating regime does not approach that limit. The harder grade's ≈5× modulus over 10A is a direct stiffness win.
- **Repeated-cycle fatigue cuts back the usable stretch window further.** The break values above are single-cycle-to-failure numbers; under repeated cyclic loading the effective break stretch drops further, and the drop is more severe for the harder grades because the stress amplitude per cycle is higher. The [sim-to-real loop](../../../100-optimization/05-sim-to-real.md) tracks cycle-count-to-failure for per-batch drift; the Pass-3 measurement deliverable for Dragon Skin adds a cycle-count column that the single-cycle elongation table does not carry.

## Per-grade validity declaration

Each grade carries its own validity metadata, consistent with the [Part 2 Ch 00 validity regime requirement](../../../20-materials/00-trait-hierarchy/02-validity.md).

- **10A** — modulus from Shore-A correlation with $\pm 10\%$ lot variance; moderate literature coverage for hyperelastic fits; DMA data derived from per-family transfer with Pass-3 refinement scheduled. Canonical-problem default when the stretch window regularly exceeds 100%.
- **20A** — same Shore-A correlation status. Canonical-problem default when the regime is moderate-stretch (below 100%) and stiffness is the dominant concern.
- **30A** — Shore-A correlation at the firm end of the scale; weakest published characterization of the three grades, so the [hyperelastic-fits leaf](01-hyperelastic-fits.md) ships both Mooney-Rivlin and Ogden fits to bracket the uncertainty; per-print calibration expected to close the fit-quality gap.

The gradcheck suite ([Part 11 Ch 04 gradcheck sub-chapter](../../../110-crate/04-testing/03-gradcheck.md)) honours the per-grade validity declaration and emits warnings when a Dragon Skin 30A `Material` exceeds its declared domain under a canonical-problem solve.

## Alternatives considered

**Four-grade ladder (add Dragon Skin 40A or 50A).** Smooth-On does not ship platinum-cure Dragon Skin above 30A in the same chemistry continuity as 10A–30A; the firmer grades are different formulations. Extending the ladder to 40A would break the chemistry-continuity claim the [Dragon Skin branch](../01-dragonskin.md) Claim 1 rests on, which is why the book stops at 30A.

**Merge with Ecoflex into a single "Smooth-On platinum-cure" family.** Ecoflex and Dragon Skin share chemistry and manufacturer but cover different durometer ranges and different characterization-literature coverage. Treating them as one family obscures the Shore-00-vs-Shore-A correlation distinction and merges the material-data provenance story; keeping them as adjacent families with an explicit cross-family chemistry-continuity claim preserves both granularities.

**Use Dragon Skin as the default for all canonical-problem runs.** Dragon Skin's stiffness trade makes it a poor match for the cavity-conformity-dominated regime where Ecoflex's softness is the load-bearing property. The book's canonical default is Ecoflex 00-30 or 00-50; Dragon Skin is the opt-in for cycling and tear-resistance regimes.
