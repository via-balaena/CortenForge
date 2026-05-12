//! Layered silicone device v1 cast — Stage 2 F4 example wrapper.
//!
//! Builds a 3-layer [`CastSpec`] against the layered-silicone-device
//! v1.0 geometry (capsule plug per the casting roadmap Q2
//! resolution, 6 / 10 / 14 mm cumulative shells, Ecoflex 00-30 for
//! the inner and outer layers, Dragon Skin 10A for the middle, to
//! match the row 25 sim recipe), and runs the full Track F.2 export
//! pipeline end-to-end:
//!
//! - `CastSpec::export_molds` writes 3 mold cup STLs + 1 plug STL
//!   into the example's `out/` directory after gating each layer's
//!   pour mass against the 2 lb-per-pour budget
//!   ([`cf_cast::DEFAULT_MASS_BUDGET_KG`]) and the F4 printability
//!   gate at each mesh write.
//! - `CastSpec::write_procedure` writes the workshop-Markdown
//!   procedure to `out/procedure.md` after re-applying the same
//!   budget gate, so the generated procedure's "every layer within
//!   budget" claim is factually correct on every successful run.
//!
//! Stage 2 ships a capsule-plug-only example because the v1.0
//! workflow (per Q2) defaults to the capsule fallback when the
//! user's scanned reference geometry isn't loaded; the swap-in
//! pattern for a scan-derived plug via
//! `mesh_sdf::SignedDistanceField` + `mesh_io::load_stl` is
//! documented in the module body below and lands as a follow-up
//! once the user produces an iter-1 scan asset.
//!
//! Sanitization: per the layered-silicone-device memo's tracked-file
//! directive, the cavity geometry is referred to as
//! "scanned reference geometry" or the capsule placeholder; no
//! anatomical references appear in this crate.

use anyhow::{Context, Result};
use cf_cast::{CastLayer, CastSpec, DEFAULT_MASS_BUDGET_KG, MoldingMaterial};
use cf_design::Solid;
use nalgebra::Vector3;
use std::path::{Path, PathBuf};

// ============================================================
// Geometry constants — capsule-plug + cumulative-shell offsets.
// ============================================================

/// Capsule plug radius in meters. 12 mm matches the Q2 fallback
/// envelope — large enough that the displaced cavity volume is
/// non-trivial, small enough that all three offset shells fit
/// inside the bounding region without thin-wall warnings.
const PLUG_RADIUS_M: f64 = 0.012;

/// Capsule plug cylindrical half-height in meters. Combined with
/// `PLUG_RADIUS_M = 0.012`, the capsule's total z-extent is
/// `2 · (PLUG_HALF_HEIGHT_M + PLUG_RADIUS_M) = 64 mm`.
const PLUG_HALF_HEIGHT_M: f64 = 0.020;

/// Innermost shell wall thickness in meters (Ecoflex inner contact
/// surface). Matches row 25's `LAYER_INNER`.
const LAYER_INNER_M: f64 = 0.006;

/// Middle shell cumulative outer offset in meters (inner shell
/// thickness + middle composite shell thickness). Matches row 25's
/// `LAYER_MIDDLE_OUTER` — gives a 4 mm middle layer thickness on top
/// of the 6 mm inner.
const LAYER_MIDDLE_OUTER_M: f64 = 0.010;

/// Outermost shell cumulative outer offset in meters — the full
/// device wall thickness. Matches row 25's `LAYER_OUTER`; gives a
/// 4 mm outer layer thickness on top of the 10 mm cumulative inner +
/// middle.
const LAYER_OUTER_M: f64 = 0.014;

/// Bounding-region half-extent in xy (meters). Equals the outer
/// shell's radial extent (`PLUG_RADIUS_M + LAYER_OUTER_M = 26 mm`)
/// plus ~9 mm of mold wall + clearance. The clip cuboid above
/// `body.z_max` opens the cup's top at FDM-print time.
const BOUNDING_HALF_XY_M: f64 = 0.035;

/// Bounding-region half-extent in z (meters). Equals the outer
/// shell's z-extent (`PLUG_HALF_HEIGHT_M + PLUG_RADIUS_M +
/// LAYER_OUTER_M = 46 mm`) plus ~9 mm of mold floor headroom; the
/// top is clipped above the body so cup ceiling height doesn't
/// matter.
const BOUNDING_HALF_Z_M: f64 = 0.055;

/// Marching-cubes sampling cell size in meters. 2 mm is the
/// production-fidelity value Stage 1's Eyes-on-pixels pass
/// confirmed; finer cells (1 mm) would improve surface smoothness
/// at the cost of ~10× wall time in the F4 printability check.
const MESH_CELL_SIZE_M: f64 = 0.002;

/// Silicone material densities per the Smooth-On TDS — Ecoflex
/// 00-30 = 1070 kg/m³, Dragon Skin 10A = 1070 kg/m³. Both anchor
/// names match the sim-soft `silicone_table.rs` anchor identifiers
/// so the F3 cure-protocol lookup resolves to the matching
/// pot-life + cure-time values.
const ECOFLEX_00_30_DENSITY_KG_M3: f64 = 1070.0;
const DRAGON_SKIN_10A_DENSITY_KG_M3: f64 = 1070.0;

// ============================================================
// Spec construction.
// ============================================================

/// Build the v1 [`CastSpec`] — 3-layer device + capsule plug.
///
/// Layer ordering is innermost-first (Stage 2 F1 convention). Each
/// `CastLayer.body` is the cumulative outer-surface positive after
/// that pour cures:
/// - `layers[0].body` — innermost shell only
///   (`capsule.offset(LAYER_INNER) ∖ capsule`)
/// - `layers[1].body` — inner + middle composite fused
///   (`capsule.offset(LAYER_MIDDLE_OUTER) ∖ capsule`)
/// - `layers[2].body` — full device
///   (`capsule.offset(LAYER_OUTER) ∖ capsule`)
///
/// The shared printed plug is the same capsule the shells offset
/// from; it physically shapes the innermost layer's inner cavity at
/// cast-time (subsequent layers cast around the previously cured
/// layer, not the printed plug).
///
/// Scan-derived swap-in (for iter-2+): replace the capsule with
///
/// ```ignore
/// use mesh_io::load_stl;
/// use mesh_sdf::SignedDistanceField;
/// let scan_mesh = load_stl("scan.stl")?;
/// let scan_sdf = SignedDistanceField::from_mesh(&scan_mesh, ...);
/// let cavity = Solid::from_sdf(scan_sdf, scan_bounds);
/// ```
///
/// and build `cavity.offset(LAYER_INNER) ∖ cavity` etc. exactly as
/// below — the rest of the wrapper is geometry-agnostic.
fn build_spec() -> CastSpec {
    let plug = Solid::capsule(PLUG_RADIUS_M, PLUG_HALF_HEIGHT_M);

    let layer_inner_body = plug.clone().offset(LAYER_INNER_M).subtract(plug.clone());
    let layer_middle_body = plug
        .clone()
        .offset(LAYER_MIDDLE_OUTER_M)
        .subtract(plug.clone());
    let layer_outer_body = plug.clone().offset(LAYER_OUTER_M).subtract(plug.clone());

    let ecoflex_00_30 = MoldingMaterial {
        display_name: "Ecoflex 00-30".to_string(),
        density_kg_m3: ECOFLEX_00_30_DENSITY_KG_M3,
        anchor_key: Some("ECOFLEX_00_30"),
    };
    let dragon_skin_10a = MoldingMaterial {
        display_name: "Dragon Skin 10A".to_string(),
        density_kg_m3: DRAGON_SKIN_10A_DENSITY_KG_M3,
        anchor_key: Some("DRAGON_SKIN_10A"),
    };

    let bounding_region = Solid::cuboid(Vector3::new(
        BOUNDING_HALF_XY_M,
        BOUNDING_HALF_XY_M,
        BOUNDING_HALF_Z_M,
    ));

    // Per the casting roadmap Q1 resolution, the F4 printability
    // gate uses `PrinterConfig::fdm_default` — the same baseline
    // the cf-cast Stage 1 + 2 fixtures validate against.
    let printer_config = mesh_printability::PrinterConfig::fdm_default();

    CastSpec {
        layers: vec![
            CastLayer {
                body: layer_inner_body,
                material: ecoflex_00_30.clone(),
            },
            CastLayer {
                body: layer_middle_body,
                material: dragon_skin_10a,
            },
            CastLayer {
                body: layer_outer_body,
                material: ecoflex_00_30,
            },
        ],
        plug,
        bounding_region,
        mesh_cell_size_m: MESH_CELL_SIZE_M,
        printer_config,
        mass_budget_kg: DEFAULT_MASS_BUDGET_KG,
    }
}

// ============================================================
// Entry point.
// ============================================================

fn main() -> Result<()> {
    let out_dir: PathBuf = Path::new(env!("CARGO_MANIFEST_DIR")).join("out");

    let spec = build_spec();

    // `export_molds` creates `out_dir` internally, writes 3 mold
    // STLs + 1 plug STL, and returns the report carrying per-layer
    // paths + pour-volume summaries.
    let report = spec
        .export_molds(&out_dir)
        .context("export_molds (3 mold STLs + 1 plug STL)")?;

    // Procedure markdown goes into the same `out_dir` that
    // `export_molds` just created.
    let procedure_path = out_dir.join("procedure.md");
    spec.write_procedure(&procedure_path)
        .context("write_procedure (Markdown)")?;

    // Stdout summary — friendly enough to read at a glance after
    // `cargo run --release -p example-cast-layered-silicone-device-v1`.
    println!("Layered silicone device v1 cast artifacts:");
    println!();
    println!("Output directory: {}", out_dir.display());
    println!();
    let mut total_mass_g = 0.0;
    for mold in &report.molds {
        let mass_g = mold.pour_volume.pour_mass_kg * 1000.0;
        total_mass_g += mass_g;
        println!(
            "  layer {} ({}): {:.2} g  →  {}",
            mold.layer_index,
            mold.material_display_name,
            mass_g,
            mold.path.display()
        );
    }
    println!("  plug  →  {}", report.plug_path.display());
    println!("  procedure  →  {}", procedure_path.display());
    println!();
    println!(
        "Total silicone mass: {:.2} g across {} layers (per-pour budget {:.2} g).",
        total_mass_g,
        report.molds.len(),
        DEFAULT_MASS_BUDGET_KG * 1000.0,
    );

    Ok(())
}
