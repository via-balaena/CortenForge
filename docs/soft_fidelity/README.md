# Soft-FEM fidelity — `sim-soft` to *measured* accuracy

*Established 2026-06-09.*

> **Take `sim-soft` from a datasheet-seeded *relative-comparison tool* to a model
> *validated against measured stress–strain behavior* within a stated regime — using
> published experimental material data as the differential oracle, exactly as the MSK
> builder used OpenSim 4.6.**

This is **Mission Layer 1 — substrate fidelity** (`MISSION.md`: *"bring the soft-tissue FEM
to measured accuracy. The twin's fidelity caps everything downstream — every gradient and
every device inherits its error."*). It is the soft-tissue counterpart to the rigid-body
substrate that the MSK builder validated against OpenSim. We do it **before** the Layer-2
keystone (differentiable soft↔rigid coupling) so the coupling does not inherit unvalidated
soft-tissue error.

## The gap, precisely

`sim-soft`'s **solver math** is validated against *analytical* benchmarks — Hertz
sphere–plane (`tests/hertz_sphere_plane.rs`, rel-err < 20 %, monotonic convergence),
concentric Lamé shells, the Yeoh constitutive contract. Its **material accuracy** is not
validated against anything measured:

- Every silicone's Yeoh `(C₁, C₂, λ)` is derived from a **single data-sheet point** — the
  100 % modulus `σ₁₀₀` — via small-strain identities (`material/silicone_table.rs:45–95`),
  with an acknowledged **~1.7× catalog-value uncertainty** (`1.75 μ` vs `3 μ`) and a
  deliberate **ν = 0.40** volumetric-locking compromise (real silicones ν ≈ 0.4999). The
  module states plainly: *"sim-soft is a relative-comparison tool, not an absolute predictor."*
- The entire "row" validation corpus (rows 6–24) validates **composition/machinery**
  (multi-material blending, scan-fit sleeves, axial zoning) — never whether the simulated
  stress–strain response matches a **measured** curve.

## The oracle (no hardware)

**Marechal et al. 2021**, *"Toward a Common Framework and Database of Materials for Soft
Robotics"* (Soft Robotics 8(3):284–297). ASTM D412 uniaxial tensile tests, laser
extensometer, **true (Cauchy) stress vs stretch to failure**, on exactly our anchor
materials (Dragon Skin 10/20/30, Ecoflex 00-10/00-30/00-50, …). Provides:

- **Raw measured stress–stretch data**, publicly downloadable — Zenodo
  [`10.5281/zenodo.3611329`](https://doi.org/10.5281/zenodo.3611329).
- **Independently-fit Yeoh (3-term), Ogden, Mooney-Rivlin, Neo-Hookean** parameters per
  material, with residual + AIC (paper Table III).
- **Closed-form incompressible uniaxial Cauchy-stress** expressions per model (paper Table II).

A published, peer-reviewed measurement standing in for our own instrument — the same
discipline as the OpenSim oracle. (The eventual gate of testing *our own cast* silicone on a
real tensile rig remains a later, hardware step; this arc closes the software half against the
best available measurement, honestly scoped.)

## Folder map (mirrors `docs/msk_builder/`)

| Path | Contents |
|---|---|
| `03_phases/m1_silicone_uniaxial/recon.md` | The active RECON: empirical baseline → thesis → end-state → gap table → decisions → sub-leaf ladder (S0→Sn) → risks → validation gate |

## Phase map

| Phase | Gate | Definition of done |
|---|---|---|
| **M1 — Silicone uniaxial** *(✅ done — PR #295)* | software, published-data oracle | Ecoflex 00-30 under uniaxial tension in `sim-soft` reproduces Marechal's measured true-stress–stretch curve within tolerance over the device window; the one-point-datasheet gap quantified (~85 % → ~6 % RMS) |
| **M2 — Roller BCs + multi-material + mode recon** *(✅ done — PRs #296/#297)* | software, published-data oracle | Per-axis (roller) Dirichlet BCs (unblock the Layer-2 keystone + a genuine free-lateral coupon that re-discovers λ_t — the solve M1's kept patch test stood in for); Ecoflex 00-10 validated through the same machinery (195 % → ~6 % RMS); a mode-oracle recon (S4) confirming credible multi-mode silicone data exists (Hossain Ecoflex, digitization-tier) and promoting deformation **modes** to its own phase. Recon: `03_phases/m2_roller_bc_and_modes/{recon.md, s4_mode_oracle_recon.md}` |
| **M3 — Deformation modes** *(next candidate)* | software, published-data oracle | Validate equibiaxial + planar (pure-shear) accuracy against published multi-mode silicone data (Hossain Ecoflex, digitized) using the M2 roller-BC coupons — the canonical test a uniaxial-only fit fails; expected to pull in near-incompressibility (Tet10 + F-bar / mixed-u-p) and possibly a richer model. **Competing direction:** the Layer-2 keystone (soft↔rigid coupling), now unblocked by the roller BCs — a mission-direction fork (see `s4_mode_oracle_recon.md`) |
| **M4 — Human soft tissue** *(future)* | software → physical | Published human soft-tissue data (e.g. residual-limb indentation, relevant to the socket waypoint); eventually a real instrumented measurement |

## Sequencing within the loop

This validates the **body half** of the eventual soft↔rigid coupling. We bring the material
response to measured fidelity first so the keystone coupling — and every co-design gradient
downstream — does not inherit an unvalidated constitutive error. Quality and ceiling, not speed.
