//! silicone-material-table — engineering-grade lookup of Smooth-On
//! platinum-cure silicone Lamé pairs + density, with each entry's
//! compressible-Neo-Hookean stress + energy at the σ_100 anchor
//! validated against closed-form.
//!
//! Direct-eval consumer of [`sim_soft::material::silicone_table`] (PR3
//! F4): iterates the seven `pub const SiliconeMaterial` entries
//! (`{ECOFLEX_00_10, _20, _30, _50, DRAGON_SKIN_10A, _20A, _30A}`),
//! dispatches each via `SiliconeMaterial::to_neo_hookean()` (`const`
//! bridge into `Material`-trait surface), and probes the resulting
//! `NeoHookean` at `F = diag(2.0, 1, 1)` (simple uniaxial stretch at
//! `λ = 2.0`, the data-sheet `σ_100 = 100 % engineering strain`
//! anchor). Per inventory Q4 row 19 visualization, JSON-only (no
//! `cf-view`, the table IS the artifact); museum-plaque-tour shape
//! per [`feedback_museum_plaque_readmes`][m] + [`feedback_visual_pass_collapses_for_json_rows`][v].
//!
//! Companion to row 5 [`neo-hookean-uniaxial`][r5]: row 5 sweeps a
//! single material across `λ ∈ [0.15, 1.95]` under traction-free
//! uniaxial (transcendental `λ_t`); row 19 sweeps seven materials at
//! one fixed `F` under simple stretch (closed-form scalar — no inner
//! Newton). The two rows together give the full constitutive-coverage
//! story: row 5 validates the surface across the in-domain bracket;
//! row 19 validates the table against itself across the production
//! material library.
//!
//! [m]: ../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_museum_plaque_readmes.md
//! [v]: ../../../.claude/projects/-Users-jonhillesheim-forge-cortenforge/memory/feedback_visual_pass_collapses_for_json_rows.md
//! [r5]: ../neo-hookean-uniaxial
//!
//! # Probe — simple uniaxial stretch at `λ = 2.0`
//!
//! `F = diag(λ, 1, 1)` with `λ = 2.0`. Simple stretch (NOT
//! traction-free) — transverse stretches pinned at 1, so the
//! configuration carries volumetric strain (`J = λ = 2`); not what a
//! tensile-test apparatus measures, but the closed form is scalar
//! arithmetic with no inner Newton, which is what makes it a clean
//! `Material::first_piola` + `Material::energy` direct-eval gate
//! across the seven materials.
//!
//! # Closed forms (compressible NH, `μ` shear, `Λ` first-Lamé)
//!
//! At `F = diag(λ, 1, 1)` with `J = λ`, `F⁻ᵀ = diag(1/λ, 1, 1)`,
//! `I₁ = λ² + 2`:
//!
//! ```text
//!     ψ    = (μ/2)(I₁ − 3) − μ ln J + (Λ/2)(ln J)²
//!          = (μ/2)(λ² − 1) − μ ln(λ) + (Λ/2)(ln λ)²
//!     P_11 = μ(λ − 1/λ) + Λ ln(λ) / λ
//!     P_22 = P_33 = Λ ln(λ)
//! ```
//!
//! At `λ = 2.0`, `Λ = 4μ` (`ν = 0.40`):
//!
//! ```text
//!     P_11 = μ(2 − 1/2) + 4μ · ln(2) / 2 = 1.5 μ + 2 μ ln 2 ≈ 2.886 μ
//!     P_22 = 4μ · ln(2) ≈ 2.773 μ
//!     ψ    = (μ/2)(3) − μ · ln(2) + (4μ/2)(ln 2)² ≈ 1.5 μ − 0.693 μ + 0.961 μ ≈ 1.768 μ
//! ```
//!
//! `P_11 ≈ 2.886 μ` sits below the `3μ` small-strain identity
//! (incompressible NH at `ε = 1.0` — the linearization Smooth-On's
//! `σ_100` is conventionally interpreted under, per F4 module
//! `silicone_table.rs` §Conversion). The ~3.8 % shortfall is the
//! finite-strain compressible-NH simple-stretch correction at
//! `ν = 0.40`; `Λ ln(λ) / λ < Λ (λ − 1) / λ` makes the volumetric
//! contribution sub-linear in `λ − 1`.
//!
//! # σ_100 data-sheet anchor (engineering-grade lookup)
//!
//! Each material's `σ_100` (Smooth-On 100 % modulus, in PSI) is
//! tabulated in [`silicone_table.rs`][st]'s `# Table` section; row 19
//! converts to pascals via `1 PSI = 6894.757 Pa` and prints alongside
//! the compressible-NH simple-stretch `P_11` for visual comparison.
//! The Lamé pair derivation in [`silicone_table.rs`][st] §Conversion
//! uses `σ_100 ≈ 3μ` (small-strain incompressible-NH identity). At
//! `ν = 0.40` the standalone compressible NH at simple stretch
//! produces `P_11 ≈ 2.886 μ` (this row's actual probe), so the row's
//! `P_11` lands ~3.8 % below the small-strain `3μ` reference. The
//! `~3.8 %` constitutive gap PLUS the data-sheet's intrinsic
//! catalog-value uncertainty is exactly what Fork B's post-cast
//! calibration loop absorbs into the effective `μ`; the demo's
//! engineering-grade-lookup contract is "the table reproduces the
//! data sheet up to a known finite-strain correction," not "the
//! table is the data sheet bit-exact."
//!
//! [st]: ../../../sim/L0/soft/src/material/silicone_table.rs
//!
//! # Anchor groups (all assertions exit-0 on success)
//!
//! - **`nu_invariant`** — per material, `λ_pa.to_bits() ==
//!   (4 · μ_pa).to_bits()`. Mirrors F4's `lambda_is_four_times_mu_at_nu_0_40`
//!   unit test verbatim, lifted into the row's anchor set as a
//!   cross-crate regression net (any drift in F4 fires here).
//! - **`rest_config_zero`** — per material, at `F = I` every output
//!   is `0u64` bit-exact (`F − F⁻ᵀ = 0`, `ln(det I) = 0`). Same
//!   invariant row 5 pins for its `λ = 1.0` rest point.
//! - **`closed_form_p11`** — per material, observed `P_11` at
//!   `F = diag(2, 1, 1)` matches closed-form `μ(λ − 1/λ) + Λ ln(λ)/λ`
//!   at relative `1e-12`. Same gate row 5 uses for its closed-form
//!   `P_11` anchor.
//! - **`closed_form_p22`** — per material, observed `P_22` matches
//!   `Λ ln(λ)`; `P_33 == P_22` by transverse symmetry within
//!   `EPS_ABS_PA`.
//! - **`closed_form_psi`** — per material, observed ψ matches
//!   closed-form ψ at relative `1e-12`.
//! - **`hardness_ordering`** — `P_11` non-decreasing along source-PSI
//!   order with the Ecoflex 00-10 / 00-20 tie permitted (both at 8
//!   PSI; F4's `mu_is_non_decreasing_along_hardness_order` invariant
//!   lifted into stress space).
//! - **`captured_bits`** — per-material `(P_11, P_22, ψ)` exact
//!   `to_bits()` self-pin (21 pins). Pure-analytic probe with no
//!   sparse-solver path → IV-1 dense bit-equal tier applies. Failure-
//!   mode protocol verbatim per IV-1: rule out toolchain drift first;
//!   if same toolchain, real regression in `Material::first_piola` /
//!   `Material::energy` OR in F4's `to_neo_hookean` const bridge.

// `doc_markdown` flags Unicode math notation (`λ`, `μ`, `Λ`, `σ_100`,
// `ν`) as if they were unbacktrick-quoted code identifiers. Same
// allowance as row 5; backticking every Greek symbol clutters the
// prose without adding signal. Code identifiers (`mat.first_piola`,
// `ECOFLEX_00_10`, etc.) ARE backticked.
#![allow(clippy::doc_markdown)]
// `print_summary` is a single museum-plaque stdout writer; splitting
// fragments the visual format without information gain. Same allowance
// as rows 4+5+6+9+10+11+15+16.
#![allow(clippy::too_many_lines)]

use std::path::Path;

use anyhow::{Context, Result};
use approx::assert_relative_eq;
use nalgebra::{Matrix3, Vector3};
use serde_json::json;
use sim_soft::{
    Material, SiliconeMaterial,
    material::silicone_table::{
        DRAGON_SKIN_10A, DRAGON_SKIN_20A, DRAGON_SKIN_30A, ECOFLEX_00_10, ECOFLEX_00_20,
        ECOFLEX_00_30, ECOFLEX_00_50,
    },
};

// =============================================================================
// Constants — probe deformation gradient (simple uniaxial stretch)
// =============================================================================

/// Probe stretch — `λ = 2.0` mirrors the Smooth-On `σ_100 = 100 %
/// engineering strain` data-sheet anchor (`ε = 1.0` ⇒ `λ = 1 + ε = 2`).
/// At this `λ`, simple-stretch `F = diag(λ, 1, 1)` carries `J = λ = 2`
/// (volumetric strain) — not what a traction-free tensile test
/// measures, but the closed form is scalar arithmetic with no inner
/// Newton, which is what makes the multi-material direct-eval gate
/// clean. Row 5's traction-free sweep covers the in-domain bracket at
/// the constitutive-surface layer.
const PROBE_LAMBDA: f64 = 2.0;

// =============================================================================
// Constants — tolerances (rows 5+11 closed-form anchor convention)
// =============================================================================

/// Closed-form rel-tol for `P_11` / `P_22` / ψ analytic vs observed.
/// Mirrors row 5's `REL_TOL = 1e-12` verbatim — same constitutive
/// surface, same closed-form-anchor protocol; pure-analytic probe
/// with no sparse-solver path, but `1e-12` admits any cross-platform
/// FMA-fusion drift in nalgebra's matrix-arithmetic path through
/// `Material::first_piola` (`self.mu * (f - f_inv_t) + self.lambda *
/// ln_j * f_inv_t`).
const REL_TOL: f64 = 1.0e-12;

/// Absolute floor in pascals. At `λ = 2.0`, the smallest material
/// (`ECOFLEX_00_10`, μ = 18 kPa) has `P_22 ≈ 49.9 kPa` — `1e-8 Pa`
/// is 13 orders below the smallest probed stress, catches any
/// genuine regression without flapping on round-off. Same precedent
/// as row 5.
const EPS_ABS_PA: f64 = 1.0e-8;

// =============================================================================
// Constants — PSI → Pa conversion (engineering-grade lookup)
// =============================================================================

/// Inch-pound-second `psi` to pascal conversion factor. `1 lbf/in² =
/// 4.448_221_615_2605 N / 0.000_645_16 m² = 6894.757_293_168_361 Pa`
/// exactly per NIST SP 811. Used for `σ_100` printout — Smooth-On's
/// data sheets tabulate `σ_100` in PSI; the row prints both the raw
/// PSI and the SI-converted Pa for engineering-grade lookup.
const PSI_TO_PA: f64 = 6_894.757_293_168_361;

// =============================================================================
// MaterialEntry — name + const + Smooth-On σ_100 from data sheet
// =============================================================================

/// One row of the silicone library: human-readable name (matches
/// Smooth-On product name verbatim), the F4 `pub const SiliconeMaterial`
/// reference, and Smooth-On's tabulated `σ_100` in PSI. The const
/// reference lives behind `&'static SiliconeMaterial` rather than a
/// copy so any drift in the F4 entry surfaces here at compile time.
struct MaterialEntry {
    name: &'static str,
    mat: &'static SiliconeMaterial,
    sigma_100_psi: f64,
}

/// The seven-material library, in source-PSI order. Mirrors F4's
/// `silicone_table::tests::ALL` array verbatim (same names, same
/// const references, same source-PSI ordering); `σ_100` values from
/// `silicone_table.rs`'s `# Table` section.
const LIBRARY: &[MaterialEntry] = &[
    MaterialEntry {
        name: "Ecoflex 00-10",
        mat: &ECOFLEX_00_10,
        sigma_100_psi: 8.0,
    },
    MaterialEntry {
        name: "Ecoflex 00-20",
        mat: &ECOFLEX_00_20,
        sigma_100_psi: 8.0,
    },
    MaterialEntry {
        name: "Ecoflex 00-30",
        mat: &ECOFLEX_00_30,
        sigma_100_psi: 10.0,
    },
    MaterialEntry {
        name: "Ecoflex 00-50",
        mat: &ECOFLEX_00_50,
        sigma_100_psi: 12.0,
    },
    MaterialEntry {
        name: "Dragon Skin 10A",
        mat: &DRAGON_SKIN_10A,
        sigma_100_psi: 22.0,
    },
    MaterialEntry {
        name: "Dragon Skin 20A",
        mat: &DRAGON_SKIN_20A,
        sigma_100_psi: 49.0,
    },
    MaterialEntry {
        name: "Dragon Skin 30A",
        mat: &DRAGON_SKIN_30A,
        sigma_100_psi: 86.0,
    },
];

// =============================================================================
// Closed-form analytic helpers — compressible NH at simple uniaxial stretch
// =============================================================================

/// `ψ(F = diag(λ, 1, 1))` for compressible NH. FMA chain matches
/// `NeoHookean::energy` in `neo_hookean.rs` so the closed-form vs
/// observed comparison is bit-exact at a given `(μ, λ_lame, λ)` —
/// the `1e-12` rel-tol gate is for cross-platform headroom, not for
/// round-off slack at the local FP path.
fn analytic_psi(lambda: f64, mu: f64, lambda_lame: f64) -> f64 {
    let i_1 = lambda.mul_add(lambda, 2.0);
    let ln_j = lambda.ln();
    let half_mu = 0.5 * mu;
    let half_lambda = 0.5 * lambda_lame;
    half_lambda.mul_add(ln_j * ln_j, half_mu.mul_add(i_1 - 3.0, -mu * ln_j))
}

/// `P_11(F = diag(λ, 1, 1))` for compressible NH:
/// `μ(λ − 1/λ) + Λ ln(λ) / λ`.
fn analytic_p11(lambda: f64, mu: f64, lambda_lame: f64) -> f64 {
    mu.mul_add(lambda - 1.0 / lambda, lambda_lame * lambda.ln() / lambda)
}

/// `P_22(F = diag(λ, 1, 1)) = P_33 = Λ ln(λ)` (axial-stretch identity).
fn analytic_p22(lambda: f64, lambda_lame: f64) -> f64 {
    lambda_lame * lambda.ln()
}

/// Construct `F = diag(λ, 1, 1)` for the constitutive eval.
fn deformation_gradient(lambda: f64) -> Matrix3<f64> {
    Matrix3::from_diagonal(&Vector3::new(lambda, 1.0, 1.0))
}

// =============================================================================
// MaterialRecord — per-entry computed state
// =============================================================================

#[derive(Debug, Clone)]
struct MaterialRecord {
    name: &'static str,
    mu_pa: f64,
    lambda_pa: f64,
    density_kg_per_m3: f64,
    sigma_100_psi: f64,
    sigma_100_pa: f64,
    p_11_observed: f64,
    p_11_analytic: f64,
    p_22_observed: f64,
    p_22_analytic: f64,
    p_33_observed: f64,
    psi_observed: f64,
    psi_analytic: f64,
    /// At `F = I`: every output should be `0u64` bit-exact; captured
    /// in the record so `verify_rest_config_zero` reads from one place.
    rest_p_11_bits: u64,
    rest_p_22_bits: u64,
    rest_psi_bits: u64,
}

fn build_records() -> Vec<MaterialRecord> {
    let f_probe = deformation_gradient(PROBE_LAMBDA);
    let identity = Matrix3::<f64>::identity();
    LIBRARY
        .iter()
        .map(|entry| {
            let nh = entry.mat.to_neo_hookean();
            let p_observed = nh.first_piola(&f_probe);
            let psi_observed = nh.energy(&f_probe);
            let rest_p = nh.first_piola(&identity);
            let rest_psi = nh.energy(&identity);
            MaterialRecord {
                name: entry.name,
                mu_pa: entry.mat.mu,
                lambda_pa: entry.mat.lambda,
                density_kg_per_m3: entry.mat.density,
                sigma_100_psi: entry.sigma_100_psi,
                sigma_100_pa: entry.sigma_100_psi * PSI_TO_PA,
                p_11_observed: p_observed[(0, 0)],
                p_11_analytic: analytic_p11(PROBE_LAMBDA, entry.mat.mu, entry.mat.lambda),
                p_22_observed: p_observed[(1, 1)],
                p_22_analytic: analytic_p22(PROBE_LAMBDA, entry.mat.lambda),
                p_33_observed: p_observed[(2, 2)],
                psi_observed,
                psi_analytic: analytic_psi(PROBE_LAMBDA, entry.mat.mu, entry.mat.lambda),
                rest_p_11_bits: rest_p[(0, 0)].to_bits(),
                rest_p_22_bits: rest_p[(1, 1)].to_bits(),
                rest_psi_bits: rest_psi.to_bits(),
            }
        })
        .collect()
}

// =============================================================================
// 1. verify_nu_invariant — `λ = 4μ` per F4 conversion at ν = 0.40
// =============================================================================

/// Per material, `λ_pa.to_bits() == (4 · μ_pa).to_bits()`. Mirrors
/// F4's `lambda_is_four_times_mu_at_nu_0_40` unit test — lifted here
/// as a cross-crate regression net so any future drift in the F4
/// conversion (e.g. ν changed, λ formula edited) fires loudly at
/// row 19's anchor pass.
fn verify_nu_invariant(records: &[MaterialRecord]) {
    for rec in records {
        assert_eq!(
            rec.lambda_pa.to_bits(),
            (4.0 * rec.mu_pa).to_bits(),
            "{}: λ = {} Pa drifted from 4μ = {} Pa (ν = 0.40 invariant)",
            rec.name,
            rec.lambda_pa,
            4.0 * rec.mu_pa,
        );
    }
}

// =============================================================================
// 2. verify_rest_config_zero — F = I gives `0u64` exact for every output
// =============================================================================

/// Per material, at `F = I`: `try_inverse(I) = I`, `F − F⁻ᵀ = 0`,
/// `ln(det I) = 0`, so every NH constitutive output is `0u64` bit-
/// exact. Pin the `to_bits() == 0u64` invariant directly across all
/// seven materials; any drift here is a regression in
/// `Material::first_piola` / `Material::energy` OR in F4's
/// `to_neo_hookean` const bridge.
fn verify_rest_config_zero(records: &[MaterialRecord]) {
    for rec in records {
        assert_eq!(
            rec.rest_p_11_bits, 0_u64,
            "{}: rest-config P_11 bits {:#018x} ≠ 0u64 (F = I should give P = 0 exact)",
            rec.name, rec.rest_p_11_bits,
        );
        assert_eq!(
            rec.rest_p_22_bits, 0_u64,
            "{}: rest-config P_22 bits {:#018x} ≠ 0u64",
            rec.name, rec.rest_p_22_bits,
        );
        assert_eq!(
            rec.rest_psi_bits, 0_u64,
            "{}: rest-config ψ bits {:#018x} ≠ 0u64",
            rec.name, rec.rest_psi_bits,
        );
    }
}

// =============================================================================
// 3. verify_closed_form_p11 — observed vs μ(λ − 1/λ) + Λ ln(λ)/λ
// =============================================================================

fn verify_closed_form_p11(records: &[MaterialRecord]) {
    for rec in records {
        assert_relative_eq!(
            rec.p_11_observed,
            rec.p_11_analytic,
            max_relative = REL_TOL,
            epsilon = EPS_ABS_PA,
        );
    }
}

// =============================================================================
// 4. verify_closed_form_p22 — observed vs Λ ln(λ); P_33 == P_22
// =============================================================================

fn verify_closed_form_p22(records: &[MaterialRecord]) {
    for rec in records {
        assert_relative_eq!(
            rec.p_22_observed,
            rec.p_22_analytic,
            max_relative = REL_TOL,
            epsilon = EPS_ABS_PA,
        );
        // Transverse symmetry: F = diag(λ, 1, 1) leaves the (2, 3)-
        // plane symmetric, so P_22 == P_33 in IEEE-754. Pin within
        // EPS_ABS_PA rather than `to_bits` strict — nalgebra's matrix
        // arithmetic visits (1, 1) and (2, 2) on different code paths
        // that may FMA-fuse differently across platforms.
        assert!(
            (rec.p_33_observed - rec.p_22_observed).abs() <= EPS_ABS_PA,
            "{}: P_33 = {:e} should equal P_22 = {:e} by transverse symmetry",
            rec.name,
            rec.p_33_observed,
            rec.p_22_observed,
        );
    }
}

// =============================================================================
// 5. verify_closed_form_psi — observed vs (μ/2)(I₁ − 3) − μ ln J + (Λ/2)(ln J)²
// =============================================================================

fn verify_closed_form_psi(records: &[MaterialRecord]) {
    for rec in records {
        assert_relative_eq!(
            rec.psi_observed,
            rec.psi_analytic,
            max_relative = REL_TOL,
            epsilon = EPS_ABS_PA,
        );
    }
}

// =============================================================================
// 6. verify_hardness_ordering — P_11 non-decreasing along source-PSI order
// =============================================================================

/// `P_11(λ = 2)` non-decreasing along the LIBRARY array, with the
/// Ecoflex 00-10 / 00-20 tie permitted (both at 8 PSI; F4's
/// `mu_is_non_decreasing_along_hardness_order` invariant lifted into
/// stress space). Every other adjacent pair is strictly increasing.
fn verify_hardness_ordering(records: &[MaterialRecord]) {
    for window in records.windows(2) {
        let lo = &window[0];
        let hi = &window[1];
        assert!(
            hi.p_11_observed >= lo.p_11_observed,
            "{}'s P_11 = {} Pa must be ≥ {}'s P_11 = {} Pa (hardness-ordering invariant)",
            hi.name,
            hi.p_11_observed,
            lo.name,
            lo.p_11_observed,
        );
    }
}

// =============================================================================
// 7. verify_captured_bits — per-material (P_11, P_22, ψ) self-pin
// =============================================================================

/// Captured `(P_11, P_22, ψ)` bits at `F = diag(2, 1, 1)` for each
/// of the seven materials. `to_bits()` strict equality (IV-1 dense
/// bit-equal tier — pure-analytic probe, no sparse-solver FMA-fusion
/// path that the IV-1 sparse-tier rel-tol would admit drift on).
///
/// Generated by running this row once at land-time and capturing the
/// hex output of `print_capture_block` (called from `main` when the
/// `CF_CAPTURE_BITS=1` env var is set).
const CAPTURED_BITS: [(u64, u64, u64); 7] = [
    // (P_11, P_22, ψ) per material at F = diag(2, 1, 1).
    // Index order matches LIBRARY order. Captured at land-time via
    // `CF_CAPTURE_BITS=1 cargo run -p example-sim-soft-silicone-material-table --release`
    // (sim-soft `dev` post-row-16-N+4 tip `1b295cf9`, rustc 1.95.0 on macOS arm64).
    (
        0x40e9_5e29_8d50_3413,
        0x40e8_5e53_1aa0_6826,
        0x40df_12ea_312b_01b2,
    ), // Ecoflex 00-10
    (
        0x40e9_5e29_8d50_3413,
        0x40e8_5e53_1aa0_6826,
        0x40df_12ea_312b_01b2,
    ), // Ecoflex 00-20
    (
        0x40f0_350c_532c_2145,
        0x40ef_2331_4cb0_8514,
        0x40e3_da4e_82f7_ebc0,
    ), // Ecoflex 00-30
    (
        0x40f3_bb03_dfb0_2880,
        0x40f2_f407_bf60_5101,
        0x40e8_2b27_ed5a_56a7,
    ), // Ecoflex 00-50
    (
        0x4101_f808_196e_24e2,
        0x4101_42d0_32dc_49c5,
        0x40f6_02bb_3829_2134,
    ), // Dragon Skin 10A
    (
        0x4113_e81d_0d1d_28dd,
        0x4113_1f5a_1a3a_51ba,
        0x4108_6265_ff78_dbff,
    ), // Dragon Skin 20A
    (
        0x4121_70bc_9127_23cd,
        0x4120_c0d9_224e_479a,
        0x4115_5d01_01cd_912b,
    ), // Dragon Skin 30A
];

fn verify_captured_bits(records: &[MaterialRecord]) {
    assert_eq!(
        records.len(),
        CAPTURED_BITS.len(),
        "LIBRARY ({}) and CAPTURED_BITS ({}) length mismatch — \
         re-bake CAPTURED_BITS to match LIBRARY",
        records.len(),
        CAPTURED_BITS.len(),
    );
    for (rec, &(p11_bits, p22_bits, psi_bits)) in records.iter().zip(CAPTURED_BITS.iter()) {
        assert_eq!(
            rec.p_11_observed.to_bits(),
            p11_bits,
            "{}: captured P_11 bits drift — got {:#018x} ({} Pa), expected {:#018x} ({} Pa)",
            rec.name,
            rec.p_11_observed.to_bits(),
            rec.p_11_observed,
            p11_bits,
            f64::from_bits(p11_bits),
        );
        assert_eq!(
            rec.p_22_observed.to_bits(),
            p22_bits,
            "{}: captured P_22 bits drift — got {:#018x} ({} Pa), expected {:#018x} ({} Pa)",
            rec.name,
            rec.p_22_observed.to_bits(),
            rec.p_22_observed,
            p22_bits,
            f64::from_bits(p22_bits),
        );
        assert_eq!(
            rec.psi_observed.to_bits(),
            psi_bits,
            "{}: captured ψ bits drift — got {:#018x} ({} J/m³), expected {:#018x} ({} J/m³)",
            rec.name,
            rec.psi_observed.to_bits(),
            rec.psi_observed,
            psi_bits,
            f64::from_bits(psi_bits),
        );
    }
}

/// One-shot capture helper — prints the `CAPTURED_BITS` block ready
/// to paste back into source. Activated only when `CF_CAPTURE_BITS=1`
/// is set; otherwise silent. Used at land-time + on intentional
/// re-bake (e.g. F4 const value edit); IV-1 failure-mode protocol
/// forbids re-bake to silence a drift assertion.
fn print_capture_block(records: &[MaterialRecord]) {
    println!("// Paste into CAPTURED_BITS — generated by CF_CAPTURE_BITS=1 run");
    println!(
        "const CAPTURED_BITS: [(u64, u64, u64); {}] = [",
        records.len()
    );
    for rec in records {
        println!(
            "    ({:#018x}, {:#018x}, {:#018x}), // {}",
            rec.p_11_observed.to_bits(),
            rec.p_22_observed.to_bits(),
            rec.psi_observed.to_bits(),
            rec.name,
        );
    }
    println!("];");
}

// =============================================================================
// JSON emit
// =============================================================================

/// Schema:
/// ```json
/// {
///   "probe": { "lambda": 2.0, "form": "F = diag(lambda, 1, 1) (simple uniaxial stretch)" },
///   "psi_to_pa": 6894.757293168361,
///   "materials": [
///     {
///       "name": "Ecoflex 00-10",
///       "mu_pa":            18000.0,
///       "lambda_pa":        72000.0,
///       "density_kg_per_m3":  1040.0,
///       "sigma_100_psi":         8.0,
///       "sigma_100_pa":      55158.0586547469,
///       "p_11_pa":           51953.07...,
///       "p_22_pa":           49906.61...,
///       "psi_j_per_m3":      31819.71...
///     },
///     ...
///   ]
/// }
/// ```
fn save_json(records: &[MaterialRecord], path: &Path) -> Result<()> {
    let materials_json: Vec<_> = records
        .iter()
        .map(|rec| {
            json!({
                "name":              rec.name,
                "mu_pa":             rec.mu_pa,
                "lambda_pa":         rec.lambda_pa,
                "density_kg_per_m3": rec.density_kg_per_m3,
                "sigma_100_psi":     rec.sigma_100_psi,
                "sigma_100_pa":      rec.sigma_100_pa,
                "p_11_pa":           rec.p_11_observed,
                "p_22_pa":           rec.p_22_observed,
                "psi_j_per_m3":      rec.psi_observed,
            })
        })
        .collect();
    let payload = json!({
        "probe": {
            "lambda": PROBE_LAMBDA,
            "form": "F = diag(lambda, 1, 1) (simple uniaxial stretch)",
        },
        "psi_to_pa": PSI_TO_PA,
        "materials": materials_json,
    });
    let pretty = serde_json::to_string_pretty(&payload)
        .with_context(|| format!("serialize silicone-table JSON at {}", path.display()))?;
    std::fs::write(path, pretty)
        .with_context(|| format!("write silicone-table JSON at {}", path.display()))?;
    Ok(())
}

// =============================================================================
// print_summary — museum-plaque stdout
// =============================================================================

fn print_summary(records: &[MaterialRecord], json_path: &Path) {
    println!("==== silicone-material-table ====");
    println!();
    println!("input  : 7 SiliconeMaterial consts from sim_soft::material::silicone_table (PR3 F4)");
    println!("         dispatched via SiliconeMaterial::to_neo_hookean() -> NeoHookean");
    println!("         probed at F = diag({PROBE_LAMBDA}, 1, 1) — simple uniaxial stretch (J = λ)");
    println!();
    println!("Anchor groups (all assertions exit-0 on success):");
    println!("  nu_invariant         : λ_pa.to_bits() == (4·μ_pa).to_bits()  per material");
    println!("  rest_config_zero     : at F = I, P_11/P_22/ψ all 0u64       per material");
    println!("  closed_form_p11      : observed vs μ(λ−1/λ) + Λ ln(λ)/λ      rel ≤ {REL_TOL:e}");
    println!("  closed_form_p22      : observed vs Λ ln(λ); P_33 == P_22    rel ≤ {REL_TOL:e}");
    println!(
        "  closed_form_psi      : observed vs (μ/2)(I₁−3) − μ lnJ + (Λ/2)(lnJ)²  rel ≤ {REL_TOL:e}"
    );
    println!("  hardness_ordering    : P_11 non-decreasing along source-PSI order");
    println!("  captured_bits        : per-material (P_11, P_22, ψ) to_bits self-pin");
    println!();
    println!(
        "{:<16}{:>9}{:>9}{:>9}{:>10}{:>10}{:>11}{:>13}",
        "Material", "μ kPa", "λ kPa", "ρ kg/m³", "σ_100 PSI", "σ_100 kPa", "P_11 kPa", "ψ kJ/m³"
    );
    println!("{}", "-".repeat(16 + 9 + 9 + 9 + 10 + 10 + 11 + 13));
    for rec in records {
        println!(
            "{:<16}{:>9.1}{:>9.1}{:>9.0}{:>10.0}{:>10.2}{:>11.2}{:>13.3}",
            rec.name,
            rec.mu_pa / 1000.0,
            rec.lambda_pa / 1000.0,
            rec.density_kg_per_m3,
            rec.sigma_100_psi,
            rec.sigma_100_pa / 1000.0,
            rec.p_11_observed / 1000.0,
            rec.psi_observed / 1000.0,
        );
    }
    println!();
    println!("Reference identities:");
    println!("  σ_100 small-strain (3μ)        = data-sheet linearization, F4 §Conversion");
    println!("  P_11 NH simple-stretch         ≈ 2.886 μ  at λ = 2, ν = 0.40  (this row's probe)");
    println!("  Δ ≈ 3.8 % below 3μ small-strain — finite-strain compressible-NH correction;");
    println!(
        "  Fork B post-cast modulus fit absorbs the constitutive gap + catalog-value uncertainty."
    );
    println!();
    println!("JSON   : {}", json_path.display());
    println!("         programmatic-consumption lookup; schema in src/main.rs::save_json doc.");
}

// =============================================================================
// main
// =============================================================================

fn main() -> Result<()> {
    let records = build_records();

    verify_nu_invariant(&records);
    verify_rest_config_zero(&records);
    verify_closed_form_p11(&records);
    verify_closed_form_p22(&records);
    verify_closed_form_psi(&records);
    verify_hardness_ordering(&records);

    // `CF_CAPTURE_BITS=1 cargo run -p ... --release` prints the
    // CAPTURED_BITS block and skips the captured-bits self-pin (the
    // pin would fail before the helper runs). Used at land-time and
    // on intentional re-bake; IV-1 protocol forbids re-bake to
    // silence a drift assertion.
    if std::env::var("CF_CAPTURE_BITS").is_ok() {
        print_capture_block(&records);
    } else {
        verify_captured_bits(&records);
    }

    let out_dir = Path::new(env!("CARGO_MANIFEST_DIR")).join("out");
    std::fs::create_dir_all(&out_dir)
        .with_context(|| format!("create out/ at {}", out_dir.display()))?;
    let json_path = out_dir.join("silicone_materials.json");
    save_json(&records, &json_path)?;

    print_summary(&records, &json_path);

    Ok(())
}
