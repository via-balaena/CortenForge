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
//!    flood-fill-signed [`mesh_sdf::Signed<TriMeshDistance,
//!    FloodFillSign>`].
//! 4. Parse `.prep.toml` → centerline polyline.
//! 5. Compute scan AABB; grow outermost layer body outward by
//!    `wall_thickness_m` → contour-following bounding region.
//! 6. Derive plug + per-layer body solids from a shared `Arc`-backed
//!    scan SDF (one SDF allocation, N+1 cf-design references). With
//!    `cavity_inset_m` lifted from `.design.toml` (0 if no design
//!    source):
//!    - `plug` = `Solid::from_sdf(scan_sdf, aabb).offset(-cavity_inset_m)`
//!    - `layer[N].body` =
//!      `Solid::from_sdf(scan_sdf, aabb).offset(sum t[0..=N] - cavity_inset_m)`
//! 7. Build [`cf_cast::Ribbon`] from the parsed centerline + the
//!    config's split-normal + dowel-hole / pour-gate / plug-pin
//!    overrides.
//! 8. Invoke [`cf_cast::CastSpec::export_molds_v2`] +
//!    [`cf_cast::CastSpec::write_procedure_v2`].
//! 9. If `cavity_inset_m > 0`, post-process the procedure.md to
//!    surface the press-fit reservation (see the
//!    `procedure_post` private module).
//!
//! # Plug-derivation choice (Option A.1)
//!
//! Per `project_scan_to_cast_bridge_design.md` + slice-9.6 update,
//! the plug equals the scan shrunk inward by `cf-device-design`'s
//! `cavity.inset_m`, baking the press-fit reservation into the
//! mold geometry. When no design source is present (inline
//! `[[layers]]`), `cavity_inset_m` defaults to `0.0`, recovering the
//! original Option-A behavior (plug == scan literal) bit-for-bit.

mod config;
mod derive;
pub mod design_ref;
mod prep;
mod procedure_post;
mod scan;

use std::fs;
use std::path::{Path, PathBuf};

use anyhow::{Context, Result, bail};

pub use cf_cast::{CastMode, PartId, PartSelection, PieceSide};
pub use config::{
    CanalConfig, CastConfig, CastDefaults, DesignSourceConfig, LayerConfig, PlugPinConfig,
    PourGateConfig, RingConfig, ScanConfig, ShellTextureConfig,
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

    run_with_config(config, &cast_toml_dir, output_dir_override)
}

/// The derived inputs shared by [`run_with_config`] +
/// [`run_selected_with_config`]: the lifted config + the [`CastSpec`] /
/// [`Ribbon`] / output dir / cavity inset, ready for either export.
struct Prepared {
    config: CastConfig,
    spec: cf_cast::CastSpec,
    ribbon: cf_cast::Ribbon,
    out_dir: PathBuf,
    cavity_inset_m: f64,
}

/// Front half of the cast pipeline shared by the full + selective exports:
/// validate, lift the design layers, load the scan SDF + centerline + caps,
/// derive the [`CastSpec`] + [`Ribbon`], and resolve the output dir.
///
/// # Errors
/// Layer-source validation, design-TOML load, scan/prep load, or ribbon
/// derivation failures.
fn prepare_cast(
    mut config: CastConfig,
    base_dir: &Path,
    output_dir_override: Option<&Path>,
) -> Result<Prepared> {
    // Slice 9 — interleaved validation: first the cross-field gate
    // (`[design]` ↔ `[[layers]]` mutual exclusion), then the
    // design.toml lift if applicable, then the per-layer + numeric
    // gate on the populated layers. The split avoids the no-op
    // contradiction where the gate wants `layers.is_empty()` pre-lift
    // and per-layer checks want non-empty post-lift.
    config
        .validate_layer_source()
        .context("cast TOML semantic validation (layer source)")?;
    // Slice 9.6 — the inset is lifted from the design.toml's
    // `[cavity].inset_m` when a design source is in play, and defaults
    // to 0.0 for the inline-layers path. `derive_spec_and_ribbon`
    // applies it to the plug + every layer outer surface, baking the
    // press-fit reservation into the mold geometry.
    let mut cavity_inset_m: f64 = 0.0;
    if let Some(design_cfg) = &config.design {
        let design_path = resolve_relative(base_dir, &design_cfg.path);
        let design = design_ref::load_design_ref(&design_path)
            .with_context(|| format!("load design TOML at {}", design_path.display()))?;
        config.layers = derive_layers_from_design(&design);
        cavity_inset_m = design.cavity.inset_m;
        println!(
            "loaded design from {} ({} layers, cavity inset {:.2} mm, schema v{})",
            design_path.display(),
            config.layers.len(),
            design.cavity.inset_m * 1e3,
            design.device_design.schema_version,
        );
    }
    config
        .validate_after_layer_source()
        .context("cast TOML semantic validation (post-layer-source)")?;

    let scan_stl_path = resolve_relative(base_dir, &config.scan.cleaned_stl);
    let prep_toml_path = resolve_relative(base_dir, &config.scan.prep_toml);

    // Flood-fill bounds must enclose every point `derive_spec_and_ribbon`
    // queries — the consumer-side `sdf_bounds = scan_aabb.expanded(
    // cumulative_thickness + wall_thickness_m)`. Compute the same
    // padding here so the SDF built in `load_scan_sdf` covers the same
    // domain the mesher walks downstream.
    let cumulative_thickness: f64 = config.layers.iter().map(|l| l.thickness_m).sum();
    let bounds_padding_m = cumulative_thickness + config.cast.wall_thickness_m;
    let loaded_scan = scan::load_scan_sdf(
        &scan_stl_path,
        bounds_padding_m,
        config.cast.mesh_cell_size_m,
    )
    .with_context(|| format!("load scan SDF from {}", scan_stl_path.display()))?;

    let prep_text = fs::read_to_string(&prep_toml_path)
        .with_context(|| format!("read prep TOML at {}", prep_toml_path.display()))?;
    let centerline = prep::parse_centerline_from_prep_toml(&prep_text)
        .with_context(|| format!("parse centerline from {}", prep_toml_path.display()))?;
    if centerline.len() < 2 {
        bail!(
            "centerline polyline in {} has fewer than 2 points (got {}) — cf_cast::Ribbon::new requires at least 2; ensure the Cap panel was applied + the centerline polyline was computed before saving",
            prep_toml_path.display(),
            centerline.len()
        );
    }
    // Lift the `[caps]` block out of the same .prep.toml. Empty when
    // the block is absent or every loop is excluded — the no-caps
    // fast path inside `pinned_floor_shell` then degenerates to the
    // pre-pinned-floor uniform offset, so older scans (cf-scan-prep
    // pre-PR-246) and excluded-cap scans keep producing the same
    // molds they always did.
    let cap_planes = cf_cap_planes::parse_cap_planes(&prep_text)
        .with_context(|| format!("parse cap planes from {}", prep_toml_path.display()))?;
    if !cap_planes.is_empty() {
        println!(
            "loaded {n} cap plane(s) from {} (candidate-A pinned floor enabled)",
            prep_toml_path.display(),
            n = cap_planes.len(),
        );
    }

    let derived = derive::derive_spec_and_ribbon(
        &config,
        &loaded_scan.sdf,
        loaded_scan.aabb,
        &centerline,
        cavity_inset_m,
        &cap_planes,
    )
    .context("derive CastSpec + Ribbon from scan + cast TOML")?;
    let DerivedSpec { spec, ribbon } = derived;

    let out_dir = match output_dir_override {
        Some(p) => base_dir.join(p),
        None => base_dir.join(&config.cast.output_dir),
    };

    Ok(Prepared {
        config,
        spec,
        ribbon,
        out_dir,
        cavity_inset_m,
    })
}

/// Run the cast pipeline from an already-parsed [`CastConfig`].
///
/// The typed entry point: programmatic consumers (e.g. `cf-studio-engine`)
/// build a [`CastConfig`] in code and call this directly, with no
/// `cast.toml` to author + re-parse. [`run`] is the thin file wrapper over
/// it (read the TOML → `CastConfig::from_toml_str` → here).
///
/// `base_dir` is the directory the config's relative paths
/// (`scan.cleaned_stl`, `scan.prep_toml`, `design.path`, `cast.output_dir`)
/// resolve against — for [`run`] that is the cast TOML's parent directory.
///
/// # Errors
/// Every failure mode after the TOML parse: layer-source validation,
/// design-TOML load, scan/prep load, ribbon derivation, mold export, and
/// procedure write.
pub fn run_with_config(
    config: CastConfig,
    base_dir: &Path,
    output_dir_override: Option<&Path>,
) -> Result<RunReport> {
    let Prepared {
        config,
        spec,
        ribbon,
        out_dir,
        cavity_inset_m,
    } = prepare_cast(config, base_dir, output_dir_override)?;

    // Slice 9.7 — orchestration progress for workshop runs (large
    // scans at sub-5 mm cells take minutes; without this the run is
    // a black box between "loaded design" and the final write).
    eprintln!(
        "[cf-cast-cli] exporting {layer_count} layer(s) × (2 pieces + 1 plug) = {stl_count} STLs at mesh_cell_size_m={cell_size}…",
        layer_count = config.layers.len(),
        stl_count = config.layers.len() * 3,
        cell_size = config.cast.mesh_cell_size_m,
    );
    let t_export = std::time::Instant::now();
    let report = spec
        .export_molds_v2(&ribbon, &out_dir)
        .context("export_molds_v2 (2 pieces + 1 plug per layer)")?;
    eprintln!(
        "[cf-cast-cli] export_molds_v2 complete in {:.1}s",
        t_export.elapsed().as_secs_f64()
    );
    let procedure_path = out_dir.join("procedure.md");
    eprintln!("[cf-cast-cli] writing procedure.md…");
    spec.write_procedure_v2(&ribbon, &procedure_path)
        .context("write_procedure_v2")?;
    // Slice 9.6c — splice `## Press-Fit Reservation` into the
    // procedure markdown when the design's cavity inset is non-zero.
    // No-op for inline-layers casts (inset = 0.0) so pre-slice-9.6
    // procedure output is preserved bit-for-bit.
    procedure_post::inject_press_fit_section(&procedure_path, cavity_inset_m)
        .context("post-process procedure.md to surface press-fit reservation")?;
    if cavity_inset_m > 0.0 {
        eprintln!(
            "[cf-cast-cli] press-fit reservation section injected ({:.2} mm)",
            cavity_inset_m * 1e3
        );
    }
    // Slice 9.5 — splice `## Slacker Recipe` into the procedure
    // markdown when at least one layer has a non-zero Slacker
    // fraction. No-op when no layer uses Slacker. The recipe table
    // is built from `config.layers` (slacker_fraction) and
    // `report.layers[i].pour_volume.pour_mass_kg` (base mass).
    //
    // `derive_layers_from_design` upstream guarantees the 1:1 layer
    // correspondence with `report.layers`. The `debug_assert_eq!`
    // catches silent truncation in tests if that invariant ever drifts;
    // release builds short-circuit to the zip.
    debug_assert_eq!(
        config.layers.len(),
        report.layers.len(),
        "config.layers and report.layers must be 1:1 (else the slacker-recipe zip silently truncates)"
    );
    let slacker_recipes: Vec<procedure_post::SlackerLayerRecipe> = config
        .layers
        .iter()
        .zip(report.layers.iter())
        .map(
            |(layer_cfg, mold_artifact)| procedure_post::SlackerLayerRecipe {
                display_name: mold_artifact.material_display_name.clone(),
                pour_mass_kg: mold_artifact.pour_volume.pour_mass_kg,
                slacker_fraction: layer_cfg.slacker_fraction,
            },
        )
        .collect();
    let slacker_layer_count = slacker_recipes
        .iter()
        .filter(|r| r.slacker_fraction.is_some())
        .count();
    procedure_post::inject_slacker_recipe_section(&procedure_path, &slacker_recipes)
        .context("post-process procedure.md to surface slacker recipe")?;
    if slacker_layer_count > 0 {
        eprintln!(
            "[cf-cast-cli] slacker recipe section injected ({slacker_layer_count} of {total} layer(s) use Slacker)",
            total = config.layers.len()
        );
    }
    eprintln!("[cf-cast-cli] done — {}", procedure_path.display());

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

/// Run the cast pipeline but export ONLY `selection`'s parts — the
/// time-saving partial sibling of [`run_with_config`].
///
/// Reuses the same front half (validate / derive) and still writes a
/// complete `procedure.md` (whole-device instructions) and per-layer pour
/// masses for **every** layer, so a frontend's pour plan stays complete
/// even when only a subset of molds was generated. Unselected pieces are
/// never meshed (the time saving).
///
/// # Errors
/// As [`run_with_config`], plus selective-export meshing failures.
pub fn run_selected_with_config(
    config: CastConfig,
    base_dir: &Path,
    output_dir_override: Option<&Path>,
    selection: &cf_cast::PartSelection,
    cast_mode: cf_cast::CastMode,
) -> Result<SelectedRunReport> {
    let Prepared {
        config,
        spec,
        ribbon,
        out_dir,
        cavity_inset_m,
    } = prepare_cast(config, base_dir, output_dir_override)?;

    eprintln!("[cf-cast-cli] exporting selected parts…");
    let t_export = std::time::Instant::now();
    let report = spec
        .export_selected(&ribbon, &out_dir, selection)
        .context("export_selected (per-piece partial cast)")?;
    eprintln!(
        "[cf-cast-cli] export_selected wrote {n} STL(s) in {:.1}s",
        t_export.elapsed().as_secs_f64(),
        n = report.written.len(),
    );

    // procedure.md is whole-device instruction prose (spec/ribbon based), so
    // write it regardless of which parts were generated, with the same
    // press-fit + slacker injections the full export applies. The pour
    // volumes are computed for every layer (no marching cubes), so the
    // injections + masses below are complete on a partial run.
    let procedure_path = out_dir.join("procedure.md");
    spec.write_procedure_v2_for_mode(&ribbon, &procedure_path, cast_mode)
        .context("write_procedure_v2_for_mode")?;
    procedure_post::inject_press_fit_section(&procedure_path, cavity_inset_m)
        .context("post-process procedure.md to surface press-fit reservation")?;
    debug_assert_eq!(
        config.layers.len(),
        report.layer_pours.len(),
        "config.layers and report.layer_pours must be 1:1"
    );
    let slacker_recipes: Vec<procedure_post::SlackerLayerRecipe> = config
        .layers
        .iter()
        .zip(report.layer_pours.iter())
        .map(|(layer_cfg, pour)| procedure_post::SlackerLayerRecipe {
            display_name: pour.material_display_name.clone(),
            pour_mass_kg: pour.pour_mass_kg,
            slacker_fraction: layer_cfg.slacker_fraction,
        })
        .collect();
    procedure_post::inject_slacker_recipe_section(&procedure_path, &slacker_recipes)
        .context("post-process procedure.md to surface slacker recipe")?;
    eprintln!("[cf-cast-cli] done — {}", procedure_path.display());

    Ok(SelectedRunReport {
        out_dir,
        procedure_path,
        total_mass_g: report
            .layer_pours
            .iter()
            .map(|p| p.pour_mass_kg * 1000.0)
            .sum(),
        layer_pour_masses_kg: report.layer_pours.iter().map(|p| p.pour_mass_kg).collect(),
        written: report.written,
    })
}

/// Summary of a successful [`run_selected_with_config`] — the partial-cast
/// sibling of [`RunReport`]. Carries the written paths + the per-layer pour
/// masses for every layer (so the pour plan survives a partial run).
#[derive(Debug)]
pub struct SelectedRunReport {
    /// Resolved output directory (where the selected STLs + procedure.md
    /// were written).
    pub out_dir: PathBuf,
    /// Path to the written procedure.md.
    pub procedure_path: PathBuf,
    /// Sum of `pour_mass_kg * 1000` across all layers (grams).
    pub total_mass_g: f64,
    /// Per-layer pour masses (kg), innermost-first, for **every** layer
    /// (1:1 with the design) — so a partial run still drives a complete
    /// pour plan.
    pub layer_pour_masses_kg: Vec<f64>,
    /// Filesystem paths of the STLs actually written this run.
    pub written: Vec<PathBuf>,
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

/// Slice 9 — lift a [`design_ref::DesignRef`]'s layer stack into
/// cf-cast-cli's [`LayerConfig`] vector. The mapping is direct:
///
/// | design.toml field             | cast.toml LayerConfig field       |
/// |---|---|
/// | `thickness_m`                  | `thickness_m`                     |
/// | `material_anchor_key`          | `material`                        |
/// | `slacker_fraction`             | (informational — not consumed by cf-cast geometry today; future enhancement could surface it in `procedure.md` per-layer recipe) |
/// | (none)                         | `density_kg_m3 = None` (auto-lookup via `density_for_anchor`) |
/// | (none)                         | `display_name = None`             |
/// | `visible`                      | (ignored — viewport-only concern) |
///
/// `cavity.inset_m` is NOT a `LayerConfig` field — it's read directly
/// in [`run`] and passed to [`derive_spec_and_ribbon`] as a top-level
/// parameter (slice 9.6 — Option A.1: plug + every layer outer
/// surface are shifted inward by `cavity.inset_m`, baking the
/// press-fit reservation into the mold geometry).
pub fn derive_layers_from_design(design: &design_ref::DesignRef) -> Vec<LayerConfig> {
    design
        .layers
        .iter()
        .map(|l| LayerConfig {
            thickness_m: l.thickness_m,
            material: l.material_anchor_key.clone(),
            density_kg_m3: None,
            display_name: None,
            // Slice 9.5 — lift slacker fraction only when non-zero,
            // matching the "Slacker Recipe" section's opt-in
            // semantics (no section emitted when no layer uses it).
            slacker_fraction: if l.slacker_fraction > 0.0 {
                Some(l.slacker_fraction)
            } else {
                None
            },
        })
        .collect()
}

pub use cf_cast::V2MoldExportReport;
