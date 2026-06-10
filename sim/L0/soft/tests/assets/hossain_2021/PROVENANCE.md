# Vendored asset — Hossain group measured Ecoflex 00-30 multi-mode data

Vendored 2026-06-10 for the soft-FEM fidelity arc (Mission Layer-1), M3
deformation-modes phase. Consumed by `tests/multimode_measured_accuracy.rs`.

## What this is

`ecoflex_00_30_multimode.json` — curated, **figure-digitized** first-cycle loading
curves for **Ecoflex 00-30** in three deformation modes (uniaxial, planar/pure-shear,
equibiaxial), digitized with WebPlotDigitizer from the published figures. The raw
digitized `(nominal_strain, nominal_stress_MPa)` points are alongside as
`ecoflex_00_30_{uniaxial,planar,equibiaxial}_digitized.csv`.

## Source

Z. Liao, J. Yang, M. Hossain, *On the stress recovery behaviour of Ecoflex silicone
rubbers*, International Journal of Mechanical Sciences **206**:106624 (2021), DOI
`10.1016/j.ijmecsci.2021.106624`. Open access: Swansea Cronfa `cronfa57203`.
Figures: 4(a) uniaxial, 8(a) planar, 12(a) equibiaxial (Shore 00-30, first cycle).

## Provenance tier & the material-variability finding

This is **figure-digitized** data (~1–3 % reading error) — one tier below M1/M2's
machine-readable Marechal raw data. **It measures ~1.75× stiffer than the Marechal
2021 dataset M1/M2 calibrate to** (steady ratio across the window). Silicone modulus
genuinely varies ~1.5–2× with batch / mix ratio / cure / lab — so *absolute* modulus
carries a ~1.75× inter-source uncertainty that dwarfs the model effects (ν, I₂).
Accordingly this asset is used to validate the **cross-mode relationship** (calibrate
to one mode, predict the others — material-batch-invariant), NOT absolute modulus.
See `docs/soft_fidelity/03_phases/m3_deformation_modes/recon.md` (§REFRAME).
