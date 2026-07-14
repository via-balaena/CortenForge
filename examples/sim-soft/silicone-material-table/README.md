# silicone-material-table

**PR3 row 19 — engineering-grade lookup of Smooth-On platinum-cure silicone Lamé pairs + density, with each entry's compressible Neo-Hookean stress + energy at the σ_100 anchor validated against closed-form.** Direct-eval consumer of [`sim_soft::material::silicone_table`](../../../sim/L0/soft/src/material/silicone_table.rs) (PR3 F4): iterates the seven `pub const SiliconeMaterial` entries (`{ECOFLEX_00_10, _20, _30, _50, DRAGON_SKIN_10A, _20A, _30A}`), dispatches each via `SiliconeMaterial::to_neo_hookean()` (`const` bridge into the [`Material`](../../../sim/L0/soft/src/material/mod.rs) trait surface), and probes the resulting `NeoHookean` at `F = diag(2.0, 1, 1)` (simple uniaxial stretch at `λ = 2.0`, the data-sheet `σ_100 = 100 % engineering strain` anchor). Per inventory Q4 row 19 visualization, JSON-only (no `cf-view`, the table IS the artifact); museum-plaque-tour shape per [`feedback_museum_plaque_readmes`][m] + [`feedback_visual_pass_collapses_for_json_rows`][v].

Companion to row 5 (the `neo_hookean` module of [`stretch/stress-test`](../stretch/stress-test)): row 5 sweeps a single material across `λ ∈ [0.15, 1.95]` under traction-free uniaxial (transcendental `λ_t`); row 19 sweeps seven materials at one fixed `F` under simple stretch (closed-form scalar — no inner Newton). The two rows together close the constitutive-coverage story: row 5 validates `Material::first_piola` + `Material::energy` across the in-domain bracket on one material; row 19 validates the production material library's `to_neo_hookean()` const bridge by checking each entry's `Material`-trait dispatch against closed-form NH at the data-sheet σ_100 anchor.

Every claim sits behind an `assert!` / `assert_eq!` / `assert_relative_eq!` in `src/main.rs::verify_*`; per [`feedback_math_pass_first_handauthored`][m2], a clean `cargo run --release` exit-0 IS the correctness signal. Output is `out/silicone_materials.json` (programmatic-consumption lookup; schema documented inline at `src/main.rs::save_json`).

[m]:  ../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_museum_plaque_readmes.md
[v]:  ../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_visual_pass_collapses_for_json_rows.md
[m2]: ../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_math_pass_first_handauthored.md

## What this example demonstrates

Engineering-grade material lookup, not "guess your Lamé params from a search bar":

```rust
use sim_soft::Material;
use sim_soft::material::silicone_table::ECOFLEX_00_30;
use nalgebra::{Matrix3, Vector3};

let nh = ECOFLEX_00_30.to_neo_hookean();                            // const bridge into Material trait
let f  = Matrix3::from_diagonal(&Vector3::new(2.0, 1.0, 1.0));     // F = diag(2, 1, 1) — this row's σ_100 probe
let p  = nh.first_piola(&f);                                        // first Piola stress (Pa)
let psi = nh.energy(&f);                                            // strain-energy density (J/m³)
```

The seven `pub const SiliconeMaterial` entries cover the platinum-cure silicones the [layered silicone device arc](../../../sim/L0/soft/EXAMPLE_INVENTORY.md) currently considers as body-layer or rigid-mold materials. The Lamé pairs are sourced from Smooth-On's published technical data sheets ([www.smooth-on.com](https://www.smooth-on.com)) under the `ν = 0.40` compressible-NH framing per Fork B (sim-soft = relative-comparison tool, not absolute predictor; ν = 0.40 locking is absorbed by post-cast modulus fit).

## Probe — simple uniaxial stretch at λ = 2.0

`F = diag(λ, 1, 1)` with `λ = 2.0` mirrors the Smooth-On `σ_100 = 100 % engineering strain` data-sheet anchor (`ε = 1.0` ⇒ `λ = 1 + ε = 2`). Simple stretch (NOT traction-free) — transverse stretches pinned at 1, so the configuration carries volumetric strain (`J = λ = 2`); not what a tensile-test apparatus measures, but the closed form is scalar arithmetic with no inner Newton, which is what makes it a clean `Material::first_piola` + `Material::energy` direct-eval gate across the seven materials.

### Closed forms (compressible NH, μ shear, Λ first-Lamé)

At `F = diag(λ, 1, 1)` with `J = λ`, `F⁻ᵀ = diag(1/λ, 1, 1)`, `I₁ = λ² + 2`:

```text
ψ    = (μ/2)(I₁ − 3) − μ ln J + (Λ/2)(ln J)²
     = (μ/2)(λ² − 1) − μ ln(λ) + (Λ/2)(ln λ)²
P_11 = μ(λ − 1/λ) + Λ ln(λ) / λ
P_22 = P_33 = Λ ln(λ)
```

At `λ = 2.0`, `Λ = 4μ` (`ν = 0.40`):

```text
P_11 = μ(2 − 1/2) + 4μ · ln(2) / 2 = 1.5 μ + 2 μ ln 2 ≈ 2.886 μ
P_22 = 4μ · ln(2)                                     ≈ 2.773 μ
ψ    = (μ/2)(3) − μ · ln(2) + (4μ/2)(ln 2)²           ≈ 1.768 μ
```

`P_11 ≈ 2.886 μ` sits below the `3μ` small-strain identity (incompressible NH at `ε = 1.0` — the linearization Smooth-On's `σ_100` is conventionally interpreted under, per [`silicone_table.rs`](../../../sim/L0/soft/src/material/silicone_table.rs) §Conversion). The ~3.8 % shortfall is the finite-strain compressible-NH simple-stretch correction at `ν = 0.40`; `Λ ln(λ) / λ < Λ (λ − 1) / λ` makes the volumetric contribution sub-linear in `λ − 1`. The constitutive gap PLUS the data-sheet's intrinsic catalog-value uncertainty is exactly what Fork B's post-cast calibration loop absorbs into the effective `μ`; the demo's engineering-grade-lookup contract is "the table reproduces the data sheet up to a known finite-strain correction," not "the table is the data sheet bit-exact."

## Numerical anchors

Each anchor is encoded as an `assert!` / `assert_eq!` / `assert_relative_eq!` in `src/main.rs` under `verify_*`. All 7 groups are called from `main()` in dependency order; `cargo run --release` exit-0 means every assert passed.

### 1. `nu_invariant` — per material, λ_pa.to_bits() == (4·μ_pa).to_bits()

| Anchor | Bound |
|---|---|
| `lambda_pa.to_bits() == (4 * mu_pa).to_bits()` per material | bit-equal |

Mirrors F4's [`lambda_is_four_times_mu_at_nu_0_40`](../../../sim/L0/soft/src/material/silicone_table.rs) unit test verbatim — lifted here as a cross-crate regression net so any future drift in the F4 conversion (e.g. `ν` changed, `λ` formula edited) fires loudly at row 19's anchor pass.

### 2. `rest_config_zero` — at F = I, every output is 0u64 bit-exact

| Anchor | Bound |
|---|---|
| `nh.first_piola(&I)[(0, 0)].to_bits()` per material | `0u64` |
| `nh.first_piola(&I)[(1, 1)].to_bits()` per material | `0u64` |
| `nh.energy(&I).to_bits()` per material              | `0u64` |

`try_inverse(I) = I`, `F − F⁻ᵀ = 0`, `ln(det I) = 0`; every NH constitutive output is `0u64` bit-exact at `F = I`. Pin directly across all seven materials. Same invariant row 5 pins for its `λ = 1.0` rest point.

### 3. `closed_form_p11` — observed vs μ(λ − 1/λ) + Λ ln(λ)/λ

| Anchor | Bound |
|---|---|
| `assert_relative_eq!(p_11_observed, μ(λ−1/λ) + Λ ln(λ)/λ, max_relative = 1e-12)` per material | rel ≤ `1e-12` |

Same gate row 5 uses for its closed-form `P_11` anchor. The `Material::first_piola` impl evaluates `self.mu * (f - f_inv_t) + self.lambda * ln_j * f_inv_t` (general matrix arithmetic); `1e-12` admits any cross-platform FMA-fusion drift in nalgebra's matrix path while catching real regressions.

### 4. `closed_form_p22` — observed vs Λ ln(λ); P_33 == P_22

| Anchor | Bound |
|---|---|
| `assert_relative_eq!(p_22_observed, Λ ln(λ), max_relative = 1e-12)` per material | rel ≤ `1e-12` |
| `(p_33_observed - p_22_observed).abs() <= 1e-8 Pa` per material | transverse symmetry |

`P_22 = P_33 = Λ ln(λ)` is the axial-stretch identity at `F = diag(λ, 1, 1)`. Transverse symmetry pinned within `EPS_ABS_PA` rather than `to_bits` strict — nalgebra's matrix arithmetic visits `(1, 1)` and `(2, 2)` on different code paths that may FMA-fuse differently across platforms.

### 5. `closed_form_psi` — observed vs (μ/2)(I₁−3) − μ lnJ + (Λ/2)(lnJ)²

| Anchor | Bound |
|---|---|
| `assert_relative_eq!(psi_observed, ψ_analytic, max_relative = 1e-12)` per material | rel ≤ `1e-12` |

The analytic helper's FMA chain (`half_lambda.mul_add(ln_j*ln_j, half_mu.mul_add(i_1 - 3.0, -mu * ln_j))`) matches `NeoHookean::energy`'s internal FMA exactly, so the `1e-12` rel-tol is for cross-platform headroom, not for round-off slack at the local FP path.

### 6. `hardness_ordering` — P_11 non-decreasing along source-PSI order

| Anchor | Bound |
|---|---|
| `records[i+1].p_11 >= records[i].p_11` for `i ∈ [0, 6)` | non-decreasing (Ecoflex 00-10 / 00-20 tie permitted at 8 PSI) |

F4's [`mu_is_non_decreasing_along_hardness_order`](../../../sim/L0/soft/src/material/silicone_table.rs) invariant lifted into stress space — `P_11` non-decreasing along the same source-PSI order as the F4 `ALL` array.

### 7. `captured_bits` — per-material (P_11, P_22, ψ) to_bits self-pin

| Anchor | Bound |
|---|---|
| `p_11_observed.to_bits() == CAPTURED_BITS[i].0` per material | bit-equal |
| `p_22_observed.to_bits() == CAPTURED_BITS[i].1` per material | bit-equal |
| `psi_observed.to_bits() == CAPTURED_BITS[i].2` per material  | bit-equal |

Pure-analytic probe with no sparse-solver path → IV-1 dense bit-equal tier applies (NOT the IV-1 sparse-tier rel-tol that admits faer's per-column FMA-fusion drift). **Failure-mode protocol per IV-1**: if a captured-bits drift fires, do NOT re-bake. Rule out toolchain drift first; if same toolchain, real regression in `Material::first_piola` / `Material::energy` OR in F4's `to_neo_hookean` const bridge. Re-bake only on intentional code edit (e.g. F4 const value updated to a new data-sheet revision).

Re-bake helper: `CF_CAPTURE_BITS=1 cargo run -p example-sim-soft-silicone-material-table --release` skips the self-pin and prints the `CAPTURED_BITS` block ready to paste into source.

## Visuals

None — JSON-only per [`feedback_visual_pass_collapses_for_json_rows`][v]. The table IS the artifact; cf-view is irrelevant for material-reference data.

`out/silicone_materials.json` — programmatic-consumption lookup. Object-key order follows `serde_json::json!` (alphabetical):

```json
{
  "materials": [
    {
      "density_kg_per_m3": 1040.0,
      "lambda_pa":         72000.0,
      "mu_pa":             18000.0,
      "name":              "Ecoflex 00-10",
      "p_11_pa":           51953.29850015803,
      "p_22_pa":           49906.59700031606,
      "psi_j_per_m3":      31819.659250976234,
      "sigma_100_pa":      55158.058345346886,
      "sigma_100_psi":     8.0
    },
    ...
  ],
  "probe": { "form": "F = diag(lambda, 1, 1) (simple uniaxial stretch)", "lambda": 2.0 },
  "psi_to_pa": 6894.757293168361
}
```

## Run

```sh
cargo run -p example-sim-soft-silicone-material-table --release
```

Per [`feedback_release_mode_heavy_tests`][rel] — release mode is conventional across the sim-soft examples arc. This row is purely analytic (seven `Material::first_piola` + `Material::energy` evaluations on a `Matrix3<f64>`); debug runs in milliseconds, but the workspace convention is `--release` for the constitutive / FEM examples uniformly.

[rel]: ../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_release_mode_heavy_tests.md
