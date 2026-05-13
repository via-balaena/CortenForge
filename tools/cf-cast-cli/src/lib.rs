//! cf-cast-cli — scan → cast bridge.
//!
//! Reads a `cast.toml` describing a multi-layer silicone-device cast,
//! resolves the referenced cf-scan-prep output (`*.cleaned.stl` and
//! `*.cleaned.prep.toml`), auto-derives the v2.1 [`cf_cast::CastSpec`]
//! and [`cf_cast::Ribbon`] from scan SDF geometry, then writes
//! per-layer mold pieces + plugs + a procedure markdown to disk via
//! [`cf_cast::CastSpec::export_molds_v2`] and
//! [`cf_cast::CastSpec::write_procedure_v2`].
//!
//! # Pipeline
//!
//! 1. Parse `cast.toml` → [`config::CastConfig`].
//! 2. Resolve scan paths (relative to the `cast.toml` directory).
//! 3. Load cleaned STL → `IndexedMesh` →
//!    [`mesh_sdf::SignedDistanceField`].
//! 4. Parse `.prep.toml` → centerline polyline.
//! 5. Compute scan AABB; pad by `bounding_margin_m` per axis →
//!    bounding cuboid.
//! 6. Derive plug + per-layer body solids from a shared `Arc`-backed
//!    scan SDF (one SDF allocation, N+1 cf-design references):
//!    - `plug` = `Solid::from_sdf(scan_sdf, aabb)`  (Option A: scan
//!      IS the inner cavity surface)
//!    - `layer[N].body` =
//!      `Solid::from_sdf(scan_sdf, aabb).offset(sum t[0..=N])`
//! 7. Build [`cf_cast::Ribbon`] from the parsed centerline + the
//!    config's split-normal + registration / pour-gate / plug-pin
//!    overrides.
//! 8. Invoke [`cf_cast::CastSpec::export_molds_v2`] +
//!    [`cf_cast::CastSpec::write_procedure_v2`].
//!
//! # Plug-derivation choice (Option A)
//!
//! Per `project_scan_to_cast_bridge_design.md` + 2026-05-13 design
//! call, the plug equals the scan with no inset. The TOML
//! `[[layers]] thickness_m` value therefore equals the cured-silicone
//! shell thickness on top of the scan surface (no compression bias).
//! Workshop users wanting a compressive inner-surface fit can iterate
//! to a future `[scan].inner_clearance_m` override.

mod config;
mod derive;
mod prep;
mod scan;

use std::fs;
use std::path::{Path, PathBuf};

use anyhow::{Context, Result, bail};

pub use config::{
    CastConfig, CastDefaults, LayerConfig, PlugPinConfig, PourGateConfig, ScanConfig,
};
pub use derive::{
    DerivedSpec, density_for_anchor, derive_spec_and_ribbon, display_name_for_anchor,
};
pub use prep::parse_centerline_from_prep_toml;
pub use scan::{SharedScanSdf, load_scan_sdf};

/// Run the cf-cast-cli pipeline end-to-end.
///
/// `cast_toml_path` is the path to the user-edited cast TOML.
/// `output_dir_override` is the CLI `--output-dir` flag's value; if
/// `Some`, it takes precedence over the cast TOML's `cast.output_dir`.
/// Otherwise the output dir is resolved as
/// `cast_toml_path.parent() / cast.output_dir` (defaults to `"out"`).
///
/// # Errors
///
/// Returns an error chain (via `anyhow::Context`) for any of:
///
/// - cast TOML read / parse failure
/// - scan STL or `.prep.toml` not found / unreadable / malformed
/// - empty layers, non-finite thickness, unresolvable material
/// - non-normalizable split-normal, empty centerline
/// - export pipeline failure (F4 gate trip, mass-budget overrun)
pub fn run(cast_toml_path: &Path, output_dir_override: Option<&Path>) -> Result<RunReport> {
    let cast_toml_dir = cast_toml_path
        .parent()
        .map(Path::to_path_buf)
        .unwrap_or_else(|| PathBuf::from("."));

    let cast_toml_text = fs::read_to_string(cast_toml_path)
        .with_context(|| format!("read cast TOML at {}", cast_toml_path.display()))?;
    let config = config::CastConfig::from_toml_str(&cast_toml_text)
        .with_context(|| format!("parse cast TOML at {}", cast_toml_path.display()))?;
    config.validate().context("cast TOML semantic validation")?;

    let scan_stl_path = resolve_relative(&cast_toml_dir, &config.scan.cleaned_stl);
    let prep_toml_path = resolve_relative(&cast_toml_dir, &config.scan.prep_toml);

    let loaded_scan = scan::load_scan_sdf(&scan_stl_path)
        .with_context(|| format!("load scan SDF from {}", scan_stl_path.display()))?;

    let prep_text = fs::read_to_string(&prep_toml_path)
        .with_context(|| format!("read prep TOML at {}", prep_toml_path.display()))?;
    let centerline = prep::parse_centerline_from_prep_toml(&prep_text)
        .with_context(|| format!("parse centerline from {}", prep_toml_path.display()))?;
    if centerline.is_empty() {
        bail!(
            "centerline polyline in {} is empty — cf-scan-prep emits the [centerline] block only when a centerline is present; ensure the Cap panel was applied + centerline polyline was computed before saving",
            prep_toml_path.display()
        );
    }

    let derived =
        derive::derive_spec_and_ribbon(&config, &loaded_scan.sdf, loaded_scan.aabb, &centerline)
            .context("derive CastSpec + Ribbon from scan + cast TOML")?;
    let DerivedSpec { spec, ribbon } = derived;

    let out_dir = match output_dir_override {
        Some(p) => cast_toml_dir.join(p),
        None => cast_toml_dir.join(&config.cast.output_dir),
    };

    let report = spec
        .export_molds_v2(&ribbon, &out_dir)
        .context("export_molds_v2 (2 pieces + 1 plug per layer)")?;
    let procedure_path = out_dir.join("procedure.md");
    spec.write_procedure_v2(&ribbon, &procedure_path)
        .context("write_procedure_v2")?;

    Ok(RunReport {
        out_dir,
        procedure_path,
        layer_count: report.layers.len(),
        total_mass_g: report
            .layers
            .iter()
            .map(|l| l.pour_volume.pour_mass_kg * 1000.0)
            .sum(),
        v2: report,
        arc_length_mm: ribbon.arc_length() * 1000.0,
        max_tangent_rotation_deg: ribbon.max_tangent_rotation_rad().to_degrees(),
    })
}

/// Console-printable summary of a successful [`run`].
#[derive(Debug)]
pub struct RunReport {
    /// Resolved output directory (where mold pieces, plugs, and
    /// procedure.md were written).
    pub out_dir: PathBuf,
    /// Path to the written procedure.md.
    pub procedure_path: PathBuf,
    /// Number of cast layers (parallel to `v2.layers.len()`).
    pub layer_count: usize,
    /// Sum of `pour_volume.pour_mass_kg * 1000` across all layers
    /// (grams), for the CLI's stdout summary line.
    pub total_mass_g: f64,
    /// Full cf-cast v2 export report.
    pub v2: cf_cast::V2MoldExportReport,
    /// Centerline arc length in millimeters (for the stdout summary).
    pub arc_length_mm: f64,
    /// Max inter-segment tangent rotation in degrees (for the stdout
    /// summary).
    pub max_tangent_rotation_deg: f64,
}

/// Resolve a path field from the TOML against the cast TOML's
/// directory. Absolute paths pass through unchanged; relative paths
/// are joined to the cast TOML's parent dir.
pub fn resolve_relative(cast_toml_dir: &Path, p: &Path) -> PathBuf {
    if p.is_absolute() {
        p.to_path_buf()
    } else {
        cast_toml_dir.join(p)
    }
}

pub use cf_cast::V2MoldExportReport;
