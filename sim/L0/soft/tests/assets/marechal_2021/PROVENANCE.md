# Vendored asset — Marechal et al. 2021 measured silicone tensile data

Vendored 2026-06-09 for the **soft-FEM fidelity arc** (Mission Layer-1, substrate fidelity).
Consumed by the M1 uniaxial measured-accuracy gate — see
`docs/soft_fidelity/03_phases/m1_silicone_uniaxial/recon.md`.

## What this is

`ecoflex_00_30_uniaxial.json` and `ecoflex_00_10_uniaxial.json` — **curated subsamples**
(stretch ≤ 3.3, 100 points, monotonic **in stretch**; the first sample's stress is a near-zero
toe value, slightly negative from measurement noise) of the measured **uniaxial tensile**
true-stress–stretch curves for **Ecoflex 00-30** (M1) and **Ecoflex 00-10** (M2-S3 — the
softest Ecoflex grade; the only other anchor with real data in the snapshot), used to validate
that `sim-soft`'s hyperelastic material response reproduces a *measured* curve rather than a
single datasheet point. The full published curves run to stretch ≈ 17 (to failure); we vendor
only the device-relevant low-stretch portion needed for the gates. Each JSON also records the
current datasheet-anchored `silicone_table` params for that grade as the baseline this arc
replaces.

**Convention** (as published, and as encoded in the JSON): the source reports **true
(Cauchy) stress** vs **true strain**; we store **stretch = exp(true_strain)** and
**cauchy_stress_pa = true_stress_MPa × 1e6**. Test: ASTM D412, laser extensometer,
450 mm/min.

## Source & license

- **Paper:** Luc Marechal, Pascale Balland, Lukas Lindenroth, Fotis Petrou, Christos
  Kontovounisios, Fernando Bello, *"Toward a Common Framework and Database of Materials for
  Soft Robotics"*, **Soft Robotics 8(3):284–297, 2021** (DOI `10.1089/soro.2019.0115`).
- **Data:** Luc Marechal, *"Soft Robotics Materials"*, **Zenodo, 2020**, DOI
  `10.5281/zenodo.3611329` (record `LucMarechal/Soft-Robotics-Materials v0.0`).
- **License:** Zenodo access **open**; license tag **`other-open`** (no SPDX-standard file
  shipped with the dataset). The data was published by the authors expressly to enable
  finite-element simulation reuse by the soft-robotics community (the paper's stated purpose).
  We vendor a small **curated subsample as a derived validation reference**, with full
  attribution + DOI, consistent with academic-citation norms. If exact redistribution terms
  ever need tightening, the asset can be regenerated on-demand from the public Zenodo record
  instead of vendored.

## How to regenerate

```sh
curl -sL "https://zenodo.org/api/records/3611329/files/LucMarechal/Soft-Robotics-Materials-v0.0.zip/content" -o srm.zip
unzip srm.zip
# parse "Tensile Tests Data/Ecoflex 00-30.csv": skip 9 header rows, ';'-delimited,
# cols = Time; True Strain; True Stress (MPa); Engineering Strain; Engineering Stress (MPa)
# stretch = exp(True Strain); cauchy_pa = True Stress (MPa) * 1e6; keep stretch <= 3.3, monotonic.
```

(Only Ecoflex 00-10 and Ecoflex 00-30 carry real data in the v0.0 snapshot; the Dragon Skin
files are unresolved 1-byte Git-LFS placeholders.)
