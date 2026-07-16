//! material-table — engineering-grade lookup of Smooth-On
//! platinum-cure silicone Lamé pairs + density, with each entry's
//! compressible-Neo-Hookean stress + energy at the σ_100 anchor shown
//! alongside its data-sheet value. A demonstration, not a validator —
//! it computes and displays these readouts but asserts nothing; the
//! constitutive correctness is validated in the library (see below).
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
//! per `feedback_museum_plaque_readmes` + `feedback_visual_pass_collapses_for_json_rows`.
//!
//! Companion to row 5 (the [`neo_hookean`][r5] module): row 5 sweeps a
//! single material across `λ ∈ [0.15, 1.95]` under traction-free
//! uniaxial (transcendental `λ_t`); row 19 sweeps seven materials at
//! one fixed `F` under simple stretch (closed-form scalar — no inner
//! Newton). Row 19 DEMONSTRATES the production material library's
//! `to_neo_hookean()` const bridge — dispatching each entry through the
//! `Material`-trait surface and tabulating the result next to the
//! data-sheet `σ_100` anchor. The correctness of that dispatch (the
//! closed-form NH stress/energy the readouts embody) is validated in the
//! library, not here.
//!
//! [r5]: ../../stretch/stress-test
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
//! [st]: ../../../../sim/L0/soft/src/material/silicone_table.rs
//!
//! # Demonstration (not a validator — no self-gating asserts)
//!
//! This is an `example_kind = "demo"`: it iterates the seven silicone
//! entries, dispatches each via `to_neo_hookean()`, probes the resulting
//! `NeoHookean` at `F = diag(2, 1, 1)`, and emits the museum-plaque table
//! plus `out/silicone_materials.json` (the σ_100 datasheet reference
//! alongside the compressible-NH `P_11` — the "engineering-grade lookup up
//! to a known finite-strain correction" story). It does NOT gate on its
//! outputs; the
//! underlying correctness is validated in the library, so the example's
//! job is to SHOW, not to check:
//!
//! - **`NeoHookean` closed-form `first_piola` / `energy` value + `P_22 ==
//!   P_33` transverse symmetry + rest-config zero** → `neo_hookean.rs`'s
//!   `#[cfg(test)]` unit tests (material-parameter-independent, so they
//!   cover every entry in this table).
//! - **`λ = 4μ` (ν = 0.40) and the μ-non-decreasing-along-hardness
//!   ordering** → `silicone_table.rs`'s own `#[cfg(test)]` tests
//!   (`lambda_is_four_times_mu_at_nu_0_40` / `mu_is_non_decreasing_along_hardness_order`).
//!
//! The σ_100 datasheet values are decorative reference points (printed and
//! serialized, never asserted — the demo's contract is not "the table is
//! the data sheet bit-exact").

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
// MaterialRecord — per-entry computed state
// =============================================================================

/// Construct `F = diag(λ, 1, 1)` for the constitutive eval.
fn deformation_gradient(lambda: f64) -> Matrix3<f64> {
    Matrix3::from_diagonal(&Vector3::new(lambda, 1.0, 1.0))
}

#[derive(Debug, Clone)]
struct MaterialRecord {
    name: &'static str,
    mu_pa: f64,
    lambda_pa: f64,
    density_kg_per_m3: f64,
    sigma_100_psi: f64,
    sigma_100_pa: f64,
    /// Observed `P_11 = first_piola(diag(2,1,1))[(0,0)]`. The closed-form
    /// correctness of these outputs is validated in the lib
    /// (`neo_hookean.rs` unit tests); here they are demonstration readouts.
    p_11_observed: f64,
    p_22_observed: f64,
    psi_observed: f64,
}

fn build_records() -> Vec<MaterialRecord> {
    let f_probe = deformation_gradient(PROBE_LAMBDA);
    LIBRARY
        .iter()
        .map(|entry| {
            let nh = entry.mat.to_neo_hookean();
            let p_observed = nh.first_piola(&f_probe);
            MaterialRecord {
                name: entry.name,
                mu_pa: entry.mat.mu,
                lambda_pa: entry.mat.lambda,
                density_kg_per_m3: entry.mat.density,
                sigma_100_psi: entry.sigma_100_psi,
                sigma_100_pa: entry.sigma_100_psi * PSI_TO_PA,
                p_11_observed: p_observed[(0, 0)],
                p_22_observed: p_observed[(1, 1)],
                psi_observed: nh.energy(&f_probe),
            }
        })
        .collect()
}

// =============================================================================
// JSON emit
// =============================================================================

/// Schema (object-key order follows `serde_json::json!` — alphabetical):
/// ```json
/// {
///   "materials": [
///     {
///       "density_kg_per_m3": 1040.0,
///       "lambda_pa":         72000.0,
///       "mu_pa":             18000.0,
///       "name":              "Ecoflex 00-10",
///       "p_11_pa":           51953.29850015803,
///       "p_22_pa":           49906.59700031606,
///       "psi_j_per_m3":      31819.659250976234,
///       "sigma_100_pa":      55158.058345346886,
///       "sigma_100_psi":     8.0
///     },
///     ...
///   ],
///   "probe": { "form": "F = diag(lambda, 1, 1) (simple uniaxial stretch)", "lambda": 2.0 },
///   "psi_to_pa": 6894.757293168361
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
    println!("==== material_table ====");
    println!();
    println!("input  : 7 SiliconeMaterial consts from sim_soft::material::silicone_table (PR3 F4)");
    println!("         dispatched via SiliconeMaterial::to_neo_hookean() -> NeoHookean");
    println!("         probed at F = diag({PROBE_LAMBDA}, 1, 1) — simple uniaxial stretch (J = λ)");
    println!();
    println!("demo (no self-gating asserts — the table below IS the artifact):");
    println!("  the NeoHookean closed-form correctness these outputs demonstrate is validated in");
    println!("  the lib (`neo_hookean.rs` first_piola/energy/symmetry unit tests); the `λ = 4μ`");
    println!("  and hardness-ordering invariants live in `silicone_table.rs`'s own tests.");
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
    println!(
        "  σ_100 ≈ 3μ                     small-strain incompressible-NH identity (F4 §Conversion)"
    );
    println!("  P_11 NH simple-stretch         ≈ 2.886 μ  at λ = 2, ν = 0.40  (this row's probe)");
    println!(
        "  Δ ≈ 3.8 % below 3μ            finite-strain compressible-NH simple-stretch correction;"
    );
    println!(
        "  Fork B post-cast modulus fit absorbs the constitutive gap + catalog-value uncertainty."
    );
    println!();
    println!("JSON   : {}", json_path.display());
    println!(
        "         programmatic-consumption lookup; schema in src/material_table.rs::save_json doc."
    );
}

// =============================================================================
// main
// =============================================================================

pub fn run() -> Result<()> {
    let records = build_records();

    let out_dir = Path::new(env!("CARGO_MANIFEST_DIR")).join("out");
    std::fs::create_dir_all(&out_dir)
        .with_context(|| format!("create out/ at {}", out_dir.display()))?;
    let json_path = out_dir.join("silicone_materials.json");
    save_json(&records, &json_path)?;

    print_summary(&records, &json_path);

    Ok(())
}
