//! cf-device-design — layered-silicone-device design + testing suite
//! (`docs/ENGINEERING_SUITE_DESIGN.md`).
//!
//! Slices shipped:
//! - Slice 2: crate scaffold + STL/prep load + centerline overlay +
//!   Scan Info panel.
//! - Slice 3: decimated scan proxy (`EnvelopeProxyMesh`) +
//!   per-vertex radial directions.
//! - Slice 4: Cavity panel — scan-derived inner void, dialed inward
//!   by a uniform inset.
//! - Slice 5: Layers panel — 1–6 concentric silicone layers, each
//!   with its own thickness slider + material + visibility toggle.
//!   Surfaces render as solid depth-tested Bevy meshes (cavity + one
//!   per layer); the outermost layer's outer surface IS the device's
//!   outer skin (no separate "outer envelope" concept).
//! - Slice 6: Validations panel — per-layer pour volume + mass
//!   graded against the 2 lb single-pour budget, cavity self-
//!   intersection check, minimum-castable-wall check. All derived,
//!   read-only; computed each frame by `compute_validations`.
//! - Slice 6.5: per-layer Slacker recipe — each layer picks a base
//!   silicone + a Slacker ratio from Smooth-On's TB curves (`mod
//!   slacker`), and the Layers panel reads back the effective Shore
//!   hardness, tack, and the mix in grams.
//!
//! Insertion-sim (FEM) lives in the sibling `tools/cf-sim-research/`
//! binary as of the sim-decouple refactor (Phases 1-4); this crate is
//! CAD-only. See `docs/SIM_DECOUPLE_REFACTOR_PLAN.md` for the arc.
//!
//! `docs/ENGINEERING_SUITE_DESIGN.md` predates the build and is stale
//! — trust the code.

use std::path::{Path, PathBuf};

use anyhow::{Context, Result};
use bevy::input::mouse::{AccumulatedMouseMotion, AccumulatedMouseScroll};
use bevy::pbr::ExtendedMaterial;
use bevy::prelude::*;
use bevy_egui::{EguiContexts, EguiPlugin, EguiPrimaryContextPass, egui};
use cf_bevy_common::mesh::triangle_mesh_flat_shaded;
use cf_bevy_common::prelude::*;
use cf_device_types::{
    CavityState, Centerline, LAYER_COUNT_MAX, LAYER_MATERIALS, LAYER_SURFACE_PALETTE, LayerSpec,
    LayersState, ScanFilePath, ScanInfo, ScanMesh, ScanMeshVisible, design_toml, material_density,
    resolve_slacker_fraction, slacker,
};
use cf_viewer::{RenderScale, compute_render_scale, scale_aabb, setup_camera_and_lighting};

use cf_device_geometry::bevy_mesh::build_bevy_mesh_from_indexed;
use cf_device_geometry::cavity::{CavityEntity, spawn_cavity_mesh};
use cf_device_geometry::clip_plane::{self, ClipPlaneExt, ClipPlaneMaterial};
use cf_device_geometry::sdf_layers;
use clap::Parser;
use mesh_io::load_stl;
use mesh_types::{Bounded, IndexedMesh, Point3};
use serde::Deserialize;

/// Cast-frame demolding-axis convention: `+Z` is up. Inherited from
/// cf-scan-prep + cf-cast — every CortenForge cast tool assumes the
/// `UpAxis::PlusZ` swap from physics frame to Bevy frame.
const DEVICE_UP_AXIS: UpAxis = UpAxis::PlusZ;

#[derive(Parser, Debug)]
#[command(
    name = "cf-device-design",
    about = "Layered-silicone-device design + testing suite (composes a cleaned scan into a device design: cavity + concentric silicone layers)",
    version
)]
struct Cli {
    /// Path to the cleaned scan STL (cf-scan-prep output;
    /// `<stem>.cleaned.stl`).
    cleaned_stl: PathBuf,

    /// Optional path to a previously-saved design TOML to reopen +
    /// iterate on. When supplied, the suite pre-populates panels from
    /// the file; absent, the suite auto-resolves
    /// `<cleaned-stl-stem>.design.toml` next to the STL and loads it
    /// if it exists (else panels start at defaults). Wired in slice 8.
    #[arg(long, value_name = "PATH")]
    design: Option<PathBuf>,

    /// Optional path to the cf-scan-prep `.prep.toml` companion file.
    /// Defaults to `<cleaned_stl_stem>.prep.toml` next to the cleaned
    /// STL. The `[centerline].points_m` block drives the on-screen
    /// centerline overlay; absent, the overlay does not render. The
    /// centerline no longer participates in cavity / layer surface
    /// shaping under the slice-9 SDF path — those surfaces come from
    /// uniform-offset isosurfaces of the cleaned scan's SDF directly.
    #[arg(long, value_name = "PATH")]
    prep_toml: Option<PathBuf>,
}

// Device-design state types (`ScanMesh`, `ScanFilePath`,
// `ScanMeshVisible`, `ScanInfo`, `Centerline`, `CavityState`,
// `LayerSpec`, `LayersState`) plus the silicone catalog
// (`LAYER_MATERIALS`, `material_density`, `LAYER_COUNT_MAX`,
// `CAVITY_DEFAULT_INSET_M`) and the `slacker` recipe module live in
// `cf-device-types` (lifted out of this binary per
// `docs/SIM_DECOUPLE_REFACTOR_PLAN.md` §3 A1 Phase 1). Imported above.

// ----- .prep.toml parsing (subset; centerline only — caps lifted to
//       cf-cap-planes via `cf_cap_planes::parse_cap_planes`) ---

#[derive(Deserialize)]
struct PrepTomlSubset {
    centerline: Option<CenterlineBlock>,
}

#[derive(Deserialize)]
struct CenterlineBlock {
    points_m: Vec<[f64; 3]>,
}

/// Parse a `.prep.toml` string and return the centerline polyline.
/// Returns an empty `Vec` if the `[centerline]` block is absent or
/// empty — caller treats both as "no centerline available," same
/// posture as cf-cast-cli's prep parser.
fn parse_centerline(text: &str) -> Result<Vec<Point3<f64>>> {
    let subset: PrepTomlSubset = toml::from_str(text)?;
    Ok(subset
        .centerline
        .map(|c| c.points_m)
        .unwrap_or_default()
        .into_iter()
        .map(|[x, y, z]| Point3::new(x, y, z))
        .collect())
}

/// Resolve the `.prep.toml` path from CLI args. Honors `--prep-toml
/// <path>` when supplied; else falls back to `<cleaned_stl_stem>
/// .prep.toml` next to the cleaned STL. Returns `None` if no
/// inferred path exists (extremely rare: bare filename with no
/// parent).
fn resolve_prep_toml_path(cli: &Cli) -> Option<PathBuf> {
    if let Some(p) = &cli.prep_toml {
        return Some(p.clone());
    }
    // cf-scan-prep writes `<stem>.prep.toml` alongside
    // `<stem>.cleaned.stl`. Strip `.cleaned` from the cleaned STL's
    // stem to recover the original stem.
    let parent = cli.cleaned_stl.parent()?;
    let stem = cli.cleaned_stl.file_stem()?.to_str()?;
    let base_stem = stem.strip_suffix(".cleaned").unwrap_or(stem);
    Some(parent.join(format!("{base_stem}.prep.toml")))
}

/// Compute the centerline polyline's total arc length in meters
/// (sum of consecutive segment distances). Zero for a polyline with
/// fewer than 2 points.
fn centerline_arc_length_m(points: &[Point3<f64>]) -> f64 {
    points
        .windows(2)
        .map(|pair| (pair[1] - pair[0]).norm())
        .sum()
}

// ----- Bevy app -----------------------------------------------------

#[derive(Component)]
struct ScanMeshEntity;

fn main() -> Result<()> {
    let cli = Cli::parse();
    let scan_mesh = load_stl(&cli.cleaned_stl)
        .with_context(|| format!("load cleaned STL {}", cli.cleaned_stl.display()))?;

    let prep_toml_path = resolve_prep_toml_path(&cli);
    let (centerline_points, cap_planes) = if let Some(prep_path) = &prep_toml_path {
        match std::fs::read_to_string(prep_path) {
            Ok(text) => {
                let centerline = parse_centerline(&text).with_context(|| {
                    format!("parse `.prep.toml` centerline at {}", prep_path.display())
                })?;
                let caps = cf_cap_planes::parse_cap_planes(&text).with_context(|| {
                    format!("parse `.prep.toml` caps at {}", prep_path.display())
                })?;
                (centerline, caps)
            }
            Err(e) => {
                eprintln!(
                    "warning: could not read {} ({e:#}); centerline overlay + cap planes disabled",
                    prep_path.display()
                );
                (Vec::new(), Vec::new())
            }
        }
    } else {
        (Vec::new(), Vec::new())
    };

    let scan_info = build_scan_info(&cli.cleaned_stl, &scan_mesh, &centerline_points);
    println!(
        "loaded {} vertices, {} faces; bbox diagonal = {:.4} m; \
         centerline = {} points / {:.4} m arc; cap planes = {}",
        scan_info.vertex_count,
        scan_info.face_count,
        scan_info.bbox_diagonal_m,
        scan_info.centerline_point_count,
        scan_info.centerline_arc_length_m,
        cap_planes.len(),
    );

    // Slice 8 — resolve the design-TOML path and load it if it exists.
    // The CLI's `--design` wins; else fall back to
    // `<cleaned-stl-stem-minus-.cleaned>.design.toml`. A missing file
    // is *not* an error (first-time use); a parse / validation error
    // IS surfaced (the file exists but is broken — user wants to know
    // before silently dropping their design).
    let design_toml_path =
        design_toml::resolve_design_toml_path(&cli.cleaned_stl, cli.design.as_deref());
    let loaded_design = match &design_toml_path {
        Some(p) if p.exists() => match design_toml::load_design_toml(p) {
            Ok(d) => {
                println!(
                    "loaded design from {} ({} layers, cavity inset {:.2} mm)",
                    p.display(),
                    d.layers.len(),
                    d.cavity.inset_m * 1e3,
                );
                Some(d)
            }
            Err(e) => {
                return Err(e.context(format!("load design.toml at {}", p.display())));
            }
        },
        _ => None,
    };

    // Slice 9 — build the cached scan SDF + grid that feeds the
    // per-layer iso-extraction pipeline (`mod sdf_layers`).
    // One-time cost at startup: ~324 ms on the iter-1 sock fixture
    // (dec-2500 @ 5 mm); the slider-tick MC pulls from this cache
    // rather than re-sampling the SDF per layer. See the spec at
    // `docs/CF_DEVICE_DESIGN_SDF_LAYERS_SPEC.md` for the perf table
    // that pinned the constants. Sub-leaves 3/4 swap the cavity +
    // per-layer surface previews from `EnvelopeProxyMesh` to the
    // marching-cubes extraction this cache powers.
    let sdf_build_start = std::time::Instant::now();
    let cached_scan_sdf = sdf_layers::build_cached_scan_sdf(
        &scan_mesh,
        &cap_planes,
        sdf_layers::LAYER_PREVIEW_CELL_SIZE_M,
        sdf_layers::LAYER_GRID_MARGIN_M,
    )
    .with_context(|| format!("build cached scan SDF from {}", cli.cleaned_stl.display(),))?;
    let sdf_build_ms = sdf_build_start.elapsed().as_secs_f64() * 1e3;
    let (gx, gy, gz) = cached_scan_sdf.closed_grid.dimensions();
    println!(
        "cached scan SDF: {gx}×{gy}×{gz} grid cells ({} k), \
         min sdf {:.3} mm, fill {sdf_build_ms:.0} ms (caps = {})",
        gx * gy * gz / 1_000,
        cached_scan_sdf.min_sdf_value * 1e3,
        cap_planes.len(),
    );

    run_render_app(
        scan_mesh,
        scan_info,
        centerline_points,
        cli.cleaned_stl.clone(),
        design_toml_path,
        loaded_design,
        cached_scan_sdf,
        sdf_layers::CapPlanes { planes: cap_planes },
    );
    Ok(())
}

fn build_scan_info(path: &Path, mesh: &IndexedMesh, centerline_points: &[Point3<f64>]) -> ScanInfo {
    let aabb = mesh.aabb();
    let extents_mm = [
        (aabb.max.x - aabb.min.x) * 1000.0,
        (aabb.max.y - aabb.min.y) * 1000.0,
        (aabb.max.z - aabb.min.z) * 1000.0,
    ];
    ScanInfo {
        file_label: path
            .file_name()
            .and_then(|s| s.to_str())
            .unwrap_or("(unknown)")
            .to_string(),
        vertex_count: mesh.vertices.len(),
        face_count: mesh.faces.len(),
        aabb_mm_extents: extents_mm,
        bbox_diagonal_m: aabb.diagonal(),
        centerline_arc_length_m: centerline_arc_length_m(centerline_points),
        centerline_point_count: centerline_points.len(),
    }
}

// `CAVITY_DEFAULT_INSET_M` lives in `cf-device-types` alongside
// `CavityState::default_for_scan()`.

// ============================================================
// Geometric validations (slice 6)
// ============================================================
//
// Cheap, derived-from-state checks surfaced in the Validations
// panel: per-layer pour volume + mass against the 2 lb single-pour
// budget, cavity self-intersection, and minimum castable wall
// thickness. Computed fresh each frame from the cached scan SDF +
// `CavityState` + `LayersState` — `compute_validations` is a pure
// function so the whole check set is unit-testable without Bevy.
//
// Pour volume uses the divergence-theorem signed mesh volume on the
// per-layer MC iso-extracted shells (not cf-cast's SDF voxel
// integration): the closed-form mesh volume is both dependency-free
// and more accurate than coarse-voxel counting.

/// Per-silicone single-pour mass budget (kg) for the layered-
/// silicone-device v1 cast — 2 lb via NIST's exact pound-to-kg
/// conversion (1 lb = 0.453_592_37 kg). Mirrors cf-cast's
/// `pour_volume::DEFAULT_MASS_BUDGET_KG` by-value (no cf-cast dep,
/// same standalone-catalog posture as [`LAYER_MATERIALS`]).
const MASS_BUDGET_KG: f64 = 0.907_184_74;

/// Fraction of [`MASS_BUDGET_KG`] below which a single pour grades
/// green; between this fraction and 1.0× the budget grades yellow;
/// above 1.0× grades red. 0.8 reserves a 20 % working margin before
/// the budget becomes a hard concern.
const BUDGET_GREEN_FRACTION: f64 = 0.8;

/// Minimum castable layer thickness (meters). Below ~2 mm a poured
/// silicone wall is fragile and tears under insertion stretch (cf.
/// Ecoflex 00-30's ~2 mm castability threshold, cited in
/// [`cf_device_types::CAVITY_DEFAULT_INSET_M`]). The per-layer thickness slider
/// floor is 1 mm — so sub-2 mm layers are reachable, and the
/// min-wall check flags them.
const MIN_CASTABLE_THICKNESS_M: f64 = 0.002;

/// Mass-budget severity grade for one cast pour (or the device
/// total), graded against [`MASS_BUDGET_KG`]. Variant declaration
/// order is the severity order — `Ord` is derived so the worst
/// per-layer status is `layers.iter().map(...).max()`.
#[derive(Debug, Clone, Copy, PartialEq, Eq, PartialOrd, Ord)]
enum BudgetStatus {
    /// Comfortably under budget (< [`BUDGET_GREEN_FRACTION`]× the
    /// budget).
    Green,
    /// Approaching the budget ([`BUDGET_GREEN_FRACTION`]×..=1.0× the
    /// budget).
    Yellow,
    /// Over budget (> 1.0× the budget) — not pourable in one pour.
    Red,
}

/// Grade a single pour mass (kg) against [`MASS_BUDGET_KG`].
fn grade_budget(mass_kg: f64) -> BudgetStatus {
    if mass_kg > MASS_BUDGET_KG {
        BudgetStatus::Red
    } else if mass_kg >= MASS_BUDGET_KG * BUDGET_GREEN_FRACTION {
        BudgetStatus::Yellow
    } else {
        BudgetStatus::Green
    }
}

/// Per-layer validation readout — one entry per cast layer.
#[derive(Debug, Clone, PartialEq)]
struct LayerValidation {
    /// Index into [`LayersState::layers`], innermost-first.
    layer_index: usize,
    /// Shell volume (m³) — the new silicone this layer's pour adds,
    /// = V(this layer's outer surface) − V(its inner surface). The
    /// inner surface is the cavity surface for layer 0, or the
    /// prior layer's outer surface for layer `i > 0`.
    shell_volume_m3: f64,
    /// Pour mass (kg) = `shell_volume_m3` × the layer material's
    /// bulk density (from [`material_density`]).
    pour_mass_kg: f64,
    /// Mass-budget grade for this single pour.
    status: BudgetStatus,
    /// Whether this layer's thickness is at or above
    /// [`MIN_CASTABLE_THICKNESS_M`].
    thickness_castable: bool,
}

/// Whole-device validation readout, computed fresh each frame by
/// [`compute_validations`] from the cached scan SDF + cavity +
/// layer state (slice 9 sub-leaf 5; per-vertex `EnvelopeProxyMesh`
/// path retired).
#[derive(Debug, Clone, PartialEq)]
struct DeviceValidations {
    /// One entry per layer, innermost-first.
    layers: Vec<LayerValidation>,
    /// Sum of every layer's pour mass (kg). Informational: same-
    /// material layers may be combined into one pour or each poured
    /// separately — the per-layer [`LayerValidation::status`] is the
    /// pour-by-pour budget constraint, this total is context.
    total_mass_kg: f64,
    /// Worst per-layer budget status — drives the panel's headline
    /// color. [`BudgetStatus::Green`] for an empty layer stack.
    worst_status: BudgetStatus,
    /// Whether the cavity mesh collapses at the current inset:
    /// `true` when `cavity.inset_m ≥ -cached_sdf.min_sdf_value` (the
    /// requested inset exceeds the maximum interior SDF depth, so
    /// the marching-cubes extraction at iso = -inset is empty). SDF
    /// analog of the prior `cavity.inset_m ≥ proxy.min_radial_distance_m`
    /// check. Always applicable (no centerline-absent fallback —
    /// the SDF carries its own collapse depth).
    cavity_self_intersects: bool,
    /// The inset (m) at which the cavity first collapses =
    /// `-cached_sdf.min_sdf_value`. Always finite (the cached grid
    /// always has at least one interior cell for a non-empty scan).
    cavity_collapse_inset_m: f64,
    /// `true` when every layer is at or above
    /// [`MIN_CASTABLE_THICKNESS_M`].
    min_wall_ok: bool,
}

/// Absolute enclosed volume (m³) of an `IndexedMesh` (the
/// marching-cubes output of [`sdf_layers::extract_layer_surface`])
/// via the divergence theorem about the world origin:
/// `⅙·|Σ_faces a·(b×c)|` over the triangles, with `a, b, c` the
/// face's vertices.
///
/// Pure geometry in physics-frame meters — no render-scale / UpAxis
/// mapping (those are display-only concerns). The mesh is consumed
/// as-extracted: no displacement step (the iso surface naturally
/// sits where it should).
///
/// Origin-independent for CLOSED meshes by the divergence theorem
/// (internal contributions cancel). The pinned-floor scope-C arc's
/// sub-leaf 2 rewrite produces closed pinned-floor shells; the
/// cap-centroid-origin trick the cavity-mouth arc shipped is no longer
/// needed and was removed here (sub-leaf 3). See the
/// `signed_volume_closed_mesh_translation_invariant` test for the
/// regression gate.
///
/// The `abs` is load-bearing: mesh-offset's marching-cubes table
/// produces INWARD-winding triangles for our SDF convention
/// (negative inside, positive outside), so the raw divergence
/// integral is negative for a closed iso surface. Stripping the
/// sign lets downstream callers (`compute_validations`) treat the
/// result as the geometric "amount enclosed" without coupling to MC's
/// winding choice. The shell-volume difference in
/// `compute_validations` is then itself `.abs()`'d for the same
/// floating-point-sign-noise reason (a near-closed cleaned scan
/// can leave a tiny sign asymmetry between outer and inner).
fn signed_volume_m3(mesh: &IndexedMesh) -> f64 {
    let mut six_volume = 0.0_f64;
    for face in &mesh.faces {
        let a = mesh.vertices[face[0] as usize].coords;
        let b = mesh.vertices[face[1] as usize].coords;
        let c = mesh.vertices[face[2] as usize].coords;
        six_volume += a.dot(&b.cross(&c));
    }
    (six_volume / 6.0).abs()
}

/// Compute the whole-device validation readout from the cached scan
/// SDF plus the current cavity + layer state. Pure function — the
/// Validations panel calls this each frame; the tests call it
/// directly.
///
/// Shell volumes: layer 0's inner surface is the cavity surface (at
/// iso `−inset_m`); layer `i > 0`'s inner surface is layer `i − 1`'s
/// outer surface. Layer `i`'s outer surface is at iso
/// `cumulative_thickness − inset_m`. A pour's shell volume is
/// `V(outer) − V(inner)` — the new material that pour adds.
///
/// Per-tick cost: one marching-cubes extraction per layer + one for
/// the cavity (sub-millisecond each on the cached 5 mm grid), then
/// six divergence-theorem volume reductions. Cheap enough to run
/// every frame the panel renders; matches the spec's
/// "extract-many" contract.
fn compute_validations(
    cached_sdf: &sdf_layers::CachedScanSdf,
    cap_planes: &sdf_layers::CapPlanes,
    cavity: &CavityState,
    layers: &LayersState,
) -> DeviceValidations {
    // Clamp the iso to the cached grid's own envelope so a wide slider never
    // trips the `assert!` in `extract_layer_surface` — same clamp shape as
    // `update_layer_meshes` (slice 9 sub-leaf 4). Ask the cache rather than
    // rebuilding `margin - cell` here: the envelope belongs to the grid that
    // was actually built, not to `LAYER_GRID_MARGIN_M`.
    let max_iso = cached_sdf.max_extractable_iso();
    let clamp = |iso: f64| iso.clamp(-max_iso, max_iso);

    // Closed pinned-floor shells (sub-leaf 2) → signed_volume_m3 is
    // origin-invariant by the divergence theorem; no per-cap origin
    // selection needed (sub-leaf 3 deleted primary_cap_origin).

    // The cavity surface is layer 0's inner surface.
    let cavity_mesh =
        sdf_layers::extract_layer_surface(cached_sdf, &cap_planes.planes, clamp(-cavity.inset_m));
    let mut prev_inner_volume = signed_volume_m3(&cavity_mesh);
    let mut cumulative_thickness_m = 0.0_f64;
    let mut layer_validations = Vec::with_capacity(layers.layers.len());
    let mut total_mass_kg = 0.0_f64;
    let mut worst_status = BudgetStatus::Green;
    let mut min_wall_ok = true;

    for (layer_index, layer) in layers.layers.iter().enumerate() {
        cumulative_thickness_m += layer.thickness_m;
        let outer_iso = clamp(cumulative_thickness_m - cavity.inset_m);
        let outer_mesh =
            sdf_layers::extract_layer_surface(cached_sdf, &cap_planes.planes, outer_iso);
        let outer_volume = signed_volume_m3(&outer_mesh);
        // The outer surface always encloses more volume than the
        // inner; `abs` guards against floating-point sign noise on
        // a near-closed cleaned scan, not a real sign ambiguity.
        let shell_volume_m3 = (outer_volume - prev_inner_volume).abs();
        prev_inner_volume = outer_volume;

        let pour_mass_kg = shell_volume_m3 * material_density(layer.material_anchor_key);
        total_mass_kg += pour_mass_kg;
        let status = grade_budget(pour_mass_kg);
        worst_status = worst_status.max(status);
        let thickness_castable = layer.thickness_m >= MIN_CASTABLE_THICKNESS_M;
        if !thickness_castable {
            min_wall_ok = false;
        }

        layer_validations.push(LayerValidation {
            layer_index,
            shell_volume_m3,
            pour_mass_kg,
            status,
            thickness_castable,
        });
    }

    // SDF-side cavity collapse: when the requested inset exceeds the
    // deepest interior SDF magnitude, the iso lies past every grid
    // cell value → marching cubes returns an empty mesh. Equivalent
    // gate the per-vertex path used `proxy.min_radial_distance_m` for.
    let cavity_collapse_inset_m = -cached_sdf.min_sdf_value;
    DeviceValidations {
        layers: layer_validations,
        total_mass_kg,
        worst_status,
        cavity_self_intersects: cavity.inset_m >= cavity_collapse_inset_m,
        cavity_collapse_inset_m,
        min_wall_ok,
    }
}

// ============================================================
// Mesh-based surface rendering (slice 5 polish 2)
// ============================================================
//
// Each surface (the cavity + one per layer) is a Bevy entity with
// a triangle Mesh asset + opaque StandardMaterial. Depth-test does
// the right thing — near fragments occlude far fragments — so the
// user no longer sees the far side rendering through the near side
// (the gizmo-wireframe artifact that slice 5 polish 2 replaced).
//
// The mesh asset is regenerated when its source state changes
// (`is_changed()` check on the relevant resource). Re-meshing one
// surface is sub-millisecond on the iter-1 decimated proxy
// (1500 faces).

/// Marker component for a per-layer outer-surface mesh entity.
/// Bevy spawns / despawns one entity per layer whenever the layer
/// stack changes. The layer index isn't stored on the component
/// because the despawn-all-then-respawn update pattern doesn't
/// need it.
#[derive(Component)]
struct LayerSurfaceEntity;

/// Snapshot of every input `update_layer_meshes` reads, kept in the
/// system's `Local<>` so the per-frame check is "did the values
/// actually change?" rather than "did Bevy's `Res::is_changed()`
/// tick?". The latter is the deref-mut-on-access footgun
/// (`device_design_panel` holds both `CavityState` + `LayersState` as
/// `ResMut<>`, so their change ticks fire every frame regardless of
/// whether a value moved); the former is the documented escape hatch
/// per [[project-bevy-is-changed-footgun]].
#[derive(Debug, Clone, PartialEq)]
struct LayerMeshKey {
    cavity: CavityState,
    layers: LayersState,
}

/// Translucency for the per-layer surface shells so outer layers don't
/// fully occlude the inner geometry + cavity. 0.35 keeps each layer
/// readable as a translucent band of silicone without losing what's
/// behind it.
const LAYER_SLAB_ALPHA: f32 = 0.35;

/// Snapshot of every input `update_cavity_mesh` reads, kept in the
/// system's `Local<>` so the per-frame check is "did the values
/// actually change?" rather than "did Bevy's `Res::is_changed()`
/// tick?" — same posture as [`LayerMeshKey`].
#[derive(Debug, Clone, PartialEq)]
struct CavityMeshKey {
    cavity: CavityState,
}

/// Regenerate the cavity mesh asset from the cached scan SDF at
/// `iso = -cavity.inset_m`. Smooth MC iso-surface, fitting the scan
/// exactly perpendicular to its source (no apex-nipple artifacts).
///
/// Change-detection mirrors [`update_layer_meshes`]: snapshot-and-
/// compare via `Local<Option<CavityMeshKey>>` so a Bevy `ResMut<T>`
/// deref-mut-on-access tick doesn't rebuild every frame.
#[allow(clippy::needless_pass_by_value, clippy::too_many_arguments)]
fn update_cavity_mesh(
    state: Res<CavityState>,
    cached_sdf: Res<sdf_layers::CachedScanSdf>,
    cap_planes: Res<sdf_layers::CapPlanes>,
    up: Res<UpAxis>,
    render_scale: Res<RenderScale>,
    mut last_key: Local<Option<CavityMeshKey>>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut q: Query<(&mut Mesh3d, &mut Visibility), With<CavityEntity>>,
) {
    let current_key = CavityMeshKey { cavity: *state };
    let changed = last_key.as_ref().is_none_or(|prev| prev != &current_key);
    *last_key = Some(current_key);
    if !changed {
        return;
    }
    let cavity_indexed =
        sdf_layers::extract_layer_surface(&cached_sdf, &cap_planes.planes, -state.inset_m);
    let mesh_asset = meshes.add(build_bevy_mesh_from_indexed(
        &cavity_indexed,
        *up,
        render_scale.0,
    ));
    for (mut mesh_handle, mut visibility) in &mut q {
        mesh_handle.0 = mesh_asset.clone();
        *visibility = if state.visible {
            Visibility::Visible
        } else {
            Visibility::Hidden
        };
    }
}

/// Synchronize the per-layer mesh entities with `LayersState`. One
/// entity per layer, each at its OUTER-surface offset = cumulative
/// `sum(thickness[0..=i]) - cavity.inset_m` from the scan surface.
/// Layer N-1's outer surface is the device's outer skin (no
/// separate "outer envelope" concept). Each entity's visibility
/// tracks its own `LayerSpec::visible` flag.
///
/// Surfaces come from [`sdf_layers::extract_layer_surface`] at
/// `iso = cumulative_thickness - cavity.inset_m` — exact perpendicular
/// to the scan by construction (no apex-nipple artifacts; the
/// SDF-based path that retired the per-vertex radial displacement
/// in slice 9).
///
/// Change-detection is snapshot-and-compare via `Local<>` (same
/// posture as [`update_cavity_mesh`]), not `Res::is_changed()` —
/// `device_design_panel` holds `CavityState` + `LayersState` as
/// `ResMut<>` so their ticks fire every frame regardless of value
/// motion per [[project-bevy-is-changed-footgun]].
///
/// **Envelope note** ([`sdf_layers::LAYER_GRID_MARGIN_M`] = 40 mm):
/// cumulative thickness theoretically reaches 6 layers × 20 mm =
/// 120 mm via the existing per-layer thickness slider — past the
/// cached grid's envelope. The current per-layer slider defaults
/// keep typical use well inside 40 mm; release builds silently
/// clip to the grid bounds if a user dials past, and debug builds
/// trip the `debug_assert!` in
/// [`sdf_layers::extract_layer_surface`]. A future arc may tighten
/// the slider OR grow the grid — for iter-1 the 40 mm envelope is
/// the load-bearing constraint.
#[allow(clippy::needless_pass_by_value, clippy::too_many_arguments)]
fn update_layer_meshes(
    layers: Res<LayersState>,
    cavity: Res<CavityState>,
    cached_sdf: Res<sdf_layers::CachedScanSdf>,
    cap_planes: Res<sdf_layers::CapPlanes>,
    up: Res<UpAxis>,
    render_scale: Res<RenderScale>,
    mut last_key: Local<Option<LayerMeshKey>>,
    mut commands: Commands,
    mut meshes: ResMut<Assets<Mesh>>,
    mut materials: ResMut<Assets<ClipPlaneMaterial>>,
    existing: Query<Entity, With<LayerSurfaceEntity>>,
) {
    let current_key = LayerMeshKey {
        cavity: *cavity,
        layers: layers.clone(),
    };
    let changed = last_key.as_ref().is_none_or(|prev| prev != &current_key);
    *last_key = Some(current_key);
    if !changed {
        return;
    }
    for entity in &existing {
        commands.entity(entity).despawn();
    }
    if layers.layers.is_empty() {
        return;
    }
    let mut cumulative_thickness_m = 0.0_f64;
    for (i, layer) in layers.layers.iter().enumerate() {
        cumulative_thickness_m += layer.thickness_m;
        let offset_m = cumulative_thickness_m - cavity.inset_m;
        // Clamp the iso value to the cached grid's envelope so a wide
        // slider configuration doesn't trip the debug-assert in
        // [`sdf_layers::extract_layer_surface`]. Clamping AT the
        // envelope (not just BELOW it) lets MC pull whatever surface
        // is visible inside the cached grid; the alternative
        // (silently dropping the layer) would leave the user without
        // a visual cue that the slider exceeded the envelope. A
        // future arc should either widen `LAYER_GRID_MARGIN_M` or
        // tighten the slider once iter-1 / iter-2 surfaces a real
        // case.
        // The cache owns its envelope (`margin - cell` of the grid actually
        // built); clamping to it satisfies `extract_layer_surface`'s `assert!`
        // by construction, including at saturation (the assert is `<=`).
        let max_iso = cached_sdf.max_extractable_iso();
        let safe_offset_m = offset_m.clamp(-max_iso, max_iso);
        let layer_indexed =
            sdf_layers::extract_layer_surface(&cached_sdf, &cap_planes.planes, safe_offset_m);
        let mesh = meshes.add(build_bevy_mesh_from_indexed(
            &layer_indexed,
            *up,
            render_scale.0,
        ));
        let (r, g, b) = LAYER_SURFACE_PALETTE[i % LAYER_SURFACE_PALETTE.len()];
        // `AlphaMode::Blend` + `LAYER_SLAB_ALPHA` so outer layers don't
        // fully occlude the inner geometry + cavity.
        let material = materials.add(ExtendedMaterial {
            base: StandardMaterial {
                base_color: Color::srgba(r, g, b, LAYER_SLAB_ALPHA),
                alpha_mode: AlphaMode::Blend,
                double_sided: true,
                cull_mode: None,
                ..default()
            },
            extension: ClipPlaneExt::default(),
        });
        let visibility = if layer.visible {
            Visibility::Visible
        } else {
            Visibility::Hidden
        };
        commands.spawn((
            Mesh3d(mesh),
            MeshMaterial3d(material),
            visibility,
            LayerSurfaceEntity,
        ));
    }
}

/// Apply the [`ScanMeshVisible`] toggle to the scan-mesh entity's
/// `Visibility` each frame. Cheap (one component write per frame
/// when the toggle changes; Bevy short-circuits Visibility writes
/// that don't change the value).
#[allow(clippy::needless_pass_by_value)] // Bevy systems take resources by value.
fn apply_scan_mesh_visibility(
    visible: Res<ScanMeshVisible>,
    mut q: Query<&mut Visibility, With<ScanMeshEntity>>,
) {
    for mut v in &mut q {
        *v = if visible.0 {
            Visibility::Visible
        } else {
            Visibility::Hidden
        };
    }
}

#[allow(clippy::needless_pass_by_value)] // Bevy systems take resources by value.
fn setup_render_scene(
    mut commands: Commands,
    scan: Res<ScanMesh>,
    up: Res<UpAxis>,
    render_scale: Res<RenderScale>,
    mut meshes: ResMut<Assets<Mesh>>,
    mut clip_materials: ResMut<Assets<ClipPlaneMaterial>>,
) {
    let scale = render_scale.0;
    let raw_aabb = scan.0.aabb();
    let scaled_aabb = scale_aabb(&raw_aabb, scale);
    setup_camera_and_lighting(&mut commands, &scaled_aabb, *up);
    let entity_transform = Transform::from_scale(Vec3::splat(scale));
    // Local inline of `cf_viewer::spawn_face_mesh` that mounts the
    // `ClipPlaneMaterial` extended material instead of the stock
    // `StandardMaterial` — keeps cf-viewer generic (per spec sub-leaf 3
    // open risk #3) and gives the scan mesh the same clip-plane
    // behavior as the cavity + per-layer shells. Same material shape
    // as cf-viewer's helper: opaque grey, double-sided, cull-mode
    // None.
    let bevy_mesh = triangle_mesh_flat_shaded(&scan.0, None, *up);
    let material = clip_materials.add(ExtendedMaterial {
        base: StandardMaterial {
            base_color: Color::srgb(0.70, 0.72, 0.78),
            metallic: 0.10,
            perceptual_roughness: 0.6,
            double_sided: true,
            cull_mode: None,
            ..default()
        },
        extension: ClipPlaneExt::default(),
    });
    commands.spawn((
        Mesh3d(meshes.add(bevy_mesh)),
        MeshMaterial3d(material),
        entity_transform,
        ScanMeshEntity,
    ));
}

/// Draw the always-on viewport reference overlays each frame:
///
/// 1. **Three colored axis arrows** at world origin (cast-frame
///    convention: X = red, Y = green, Z = blue). Sized to the
///    rendered scan's diagonal.
/// 2. **Centerline polyline overlay** (cyan), if a centerline was
///    parsed from `.prep.toml`. Each polyline point is projected
///    through the same `UpAxis::PlusZ` swap + `render_scale` lift
///    the scan mesh entity uses, so the centerline tracks the mesh
///    exactly.
#[allow(clippy::needless_pass_by_value)] // Bevy systems take resources by value.
fn draw_reference_overlays(
    centerline: Res<Centerline>,
    up: Res<UpAxis>,
    render_scale: Res<RenderScale>,
    scan: Res<ScanMesh>,
    mut gizmos: Gizmos,
) {
    #[allow(clippy::cast_possible_truncation)] // f64 -> f32 for Bevy gizmo length.
    let arrow_length = (scan.0.aabb().diagonal() * 0.6 * f64::from(render_scale.0)) as f32;
    gizmos.arrow(
        Vec3::ZERO,
        Vec3::X * arrow_length,
        Color::srgb(1.0, 0.30, 0.30),
    );
    gizmos.arrow(
        Vec3::ZERO,
        Vec3::Z * arrow_length,
        Color::srgb(0.30, 1.0, 0.30),
    );
    gizmos.arrow(
        Vec3::ZERO,
        Vec3::Y * arrow_length,
        Color::srgb(0.40, 0.60, 1.00),
    );

    if centerline.points_m.is_empty() {
        return;
    }
    let cyan = Color::srgb(0.25, 0.85, 1.0);
    let positions: Vec<Vec3> = centerline
        .points_m
        .iter()
        .map(|p| Vec3::from_array(up.to_bevy_point(p)) * render_scale.0)
        .collect();
    for window in positions.windows(2) {
        gizmos.line(window[0], window[1], cyan);
    }
}

/// Render the Cavity egui section. The cavity is the scan-derived
/// inner void the appendage slides into; its surface sits `inset_m`
/// inside the scan along the per-vertex radial direction, so the
/// silicone skin between the cavity and the scan stretches over the
/// appendage for a snug fit.
///
/// Slice 4 v1 ships uniform-inset only. A later slice may add the
/// insertable-length clip + tangent-perpendicular end cap.
fn render_cavity_section(ui: &mut egui::Ui, state: &mut CavityState) {
    egui::CollapsingHeader::new("Cavity")
        .default_open(true)
        .show(ui, |ui| {
            ui.checkbox(&mut state.visible, "Show cavity");
            let (min_m, max_m) = CavityState::inset_slider_range_m();
            let mut inset_mm = state.inset_m * 1000.0;
            let min_mm = min_m * 1000.0;
            let max_mm = max_m * 1000.0;
            if ui
                .add(egui::Slider::new(&mut inset_mm, min_mm..=max_mm).text("inset (mm)"))
                .changed()
            {
                state.inset_m = inset_mm * 0.001;
            }
            ui.label("(silicone skin stretches inset_m over appendage)");
            // The slider's max value (8 mm) is the workshop-validated
            // envelope; see `CAVITY_INSET_SLIDER_MAX_M` docstring for
            // the rationale.
            ui.label(format!(
                "(capped at {:.0} mm — workshop envelope)",
                cf_device_types::CAVITY_INSET_SLIDER_MAX_M * 1e3,
            ));
        });
}

/// Render the Layers egui section. Surfaces the ordered layer
/// stack innermost-first. Every layer carries its own controls:
/// a visibility checkbox, a thickness slider, a material dropdown,
/// a Slacker recipe control, and a remove button (hidden when only
/// one layer remains). A "+ Add layer" button appends a new
/// outermost layer up to [`LAYER_COUNT_MAX`].
///
/// The device wall total is the sum of layer thicknesses; the
/// outer skin sits at `sum(thickness) - cavity.inset_m` radially
/// from the scan. Those derived figures are shown as read-only
/// labels at the bottom of the section.
///
/// `validations` is the shared [`DeviceValidations`] — the Slacker
/// control reads each layer's pour mass from it for the mix-in-
/// grams readout. A layer added this frame is not yet in
/// `validations.layers` (it is computed once at the top of the
/// panel); the control shows a `—` placeholder for that one frame.
///
/// [`update_layer_meshes`] regenerates the per-layer surface mesh
/// entities whenever `LayersState` or `CavityState` mutates.
fn render_layers_section(
    ui: &mut egui::Ui,
    layers: &mut LayersState,
    cavity: &CavityState,
    validations: &DeviceValidations,
) {
    egui::CollapsingHeader::new("Layers")
        .default_open(true)
        .show(ui, |ui| {
            let n = layers.layers.len();
            let (t_floor_m, t_ceiling_m) = LayerSpec::thickness_slider_range_m();
            let t_floor_mm = t_floor_m * 1000.0;
            let t_ceiling_mm = t_ceiling_m * 1000.0;

            // Track which layer the user requested removal of (if
            // any). egui's immediate-mode loop can't mutate the Vec
            // while iterating, so we defer until after the loop.
            let mut remove_index: Option<usize> = None;
            for i in 0..n {
                ui.group(|ui| {
                    ui.label(format!("Layer {} ({})", i, layer_position_label(i, n)));
                    ui.checkbox(&mut layers.layers[i].visible, "Show layer");
                    let mut t_mm = layers.layers[i].thickness_m * 1000.0;
                    let slider_text = if i == 0 {
                        "thickness from cavity (mm)"
                    } else {
                        "Δ from prior layer (mm)"
                    };
                    if ui
                        .add(
                            egui::Slider::new(&mut t_mm, t_floor_mm..=t_ceiling_mm)
                                .text(slider_text),
                        )
                        .changed()
                    {
                        layers.layers[i].thickness_m = t_mm * 0.001;
                    }
                    render_material_dropdown(ui, i, &mut layers.layers[i].material_anchor_key);
                    let pour_mass_kg = validations.layers.get(i).map(|lv| lv.pour_mass_kg);
                    render_slacker_control(ui, i, &mut layers.layers[i], pour_mass_kg);
                    if n > 1 && ui.button("Remove layer").clicked() {
                        remove_index = Some(i);
                    }
                });
                ui.add_space(2.0);
            }
            if let Some(idx) = remove_index {
                layers.layers.remove(idx);
            }

            if layers.layers.len() < LAYER_COUNT_MAX && ui.button("+ Add layer").clicked() {
                // Append at outermost position with 3 mm default.
                // New layer starts visible, no Slacker; user can
                // toggle via the per-layer "Show layer" checkbox.
                layers.layers.push(LayerSpec {
                    thickness_m: 0.003,
                    material_anchor_key: "ECOFLEX_00_30",
                    slacker_fraction: 0.0,
                    visible: true,
                });
            }

            // Derived geometric readouts: the device's outer skin
            // is the outermost layer's outer surface, at offset
            // `sum(thickness) - cavity.inset` radially from the
            // scan surface.
            let sum_layers_m: f64 = layers.layers.iter().map(|l| l.thickness_m).sum();
            let derived_outer_offset_m = sum_layers_m - cavity.inset_m;
            ui.separator();
            ui.label(format!(
                "Wall total: {:.1} mm  (sum of layers)",
                sum_layers_m * 1000.0,
            ));
            ui.label(format!(
                "  outer skin: +{:.1} mm from scan",
                derived_outer_offset_m * 1000.0,
            ));
            ui.label(format!(
                "  cavity: -{:.1} mm from scan",
                cavity.inset_m * 1000.0,
            ));
        });
}

/// Human-readable position label for a layer in the stack
/// ("single", "innermost", "middle", "outermost"). The innermost
/// layer touches the cavity surface; the outermost layer's outer
/// surface is the device's outer skin.
fn layer_position_label(i: usize, n: usize) -> &'static str {
    if n == 1 {
        "single"
    } else if i == 0 {
        "innermost"
    } else if i == n - 1 {
        "outermost"
    } else {
        "middle"
    }
}

/// Material-selection ComboBox for one layer. `id_source = i`
/// distinguishes multiple dropdowns in the same parent UI.
fn render_material_dropdown(ui: &mut egui::Ui, layer_index: usize, selected: &mut &'static str) {
    let current_label = LAYER_MATERIALS
        .iter()
        .find(|(key, _, _)| *key == *selected)
        .map(|(_, label, _)| *label)
        .unwrap_or("(unknown)");
    egui::ComboBox::from_id_salt(("layer-material", layer_index))
        .selected_text(current_label)
        .show_ui(ui, |ui| {
            for (key, label, _) in LAYER_MATERIALS {
                ui.selectable_value(selected, key, *label);
            }
        });
}

// `resolve_slacker_fraction` lives in `cf-device-types`'s `slacker`
// module alongside the TB curve data.

/// Part A / Part B / Slacker masses (grams) to weigh out for one
/// layer's pour. Returns `(part_ab_g, slacker_g)` where `part_ab_g`
/// is the mass of *each* of Part A and Part B (the base silicone is
/// 1:1 A:B by weight).
///
/// `pour_mass_kg` is the layer's shell mass from
/// [`compute_validations`]; `slacker_fraction` is Slacker mass as a
/// fraction of the base A+B mass, so the cured mix obeys
/// `pour_mass = 2·part_ab·(1 + slacker_fraction)`.
///
/// Approximation: `pour_mass_kg` is `shell_volume × base density` —
/// this treats the cured base+Slacker mix density as ≈ the base
/// silicone's (the Slacker TB publishes no density). Close enough
/// for a bench weigh-out, where the user adds a waste margin
/// regardless.
fn slacker_recipe_grams(pour_mass_kg: f64, slacker_fraction: f64) -> (f64, f64) {
    let pour_mass_g = pour_mass_kg * 1000.0;
    let part_ab_g = pour_mass_g / (2.0 * (1.0 + slacker_fraction));
    let slacker_g = slacker_fraction * pour_mass_g / (1.0 + slacker_fraction);
    (part_ab_g, slacker_g)
}

/// Dropdown label for one Slacker-curve point: the native (`0.0`)
/// point reads `No Slacker — Shore 00-30`; every other point reads
/// `+25% Slacker — Shore 000-40 (slight-to-very tack)` (percentage,
/// effective Shore hardness, cured tack).
fn slacker_point_label(point: &slacker::Point) -> String {
    if point.slacker_fraction > 0.0 {
        format!(
            "+{:.0}% Slacker — Shore {} ({})",
            point.slacker_fraction * 100.0,
            point.hardness,
            point.tack,
        )
    } else {
        format!("No Slacker — Shore {}", point.hardness)
    }
}

/// Render one layer's Slacker recipe control. For a base silicone
/// with TB data this is a dropdown of the curve's Slacker ratios
/// (each labelled with its effective Shore hardness + tack) plus a
/// mix-in-grams readout for the selected ratio; for a base without
/// data it is a disabled note explaining why.
///
/// `pour_mass_kg` is the layer's shell mass (from the shared
/// [`DeviceValidations`]), or `None` for a layer added this frame
/// that the validation pass has not caught up to yet.
fn render_slacker_control(
    ui: &mut egui::Ui,
    layer_index: usize,
    layer: &mut LayerSpec,
    pour_mass_kg: Option<f64>,
) {
    // Snap the stored fraction to a value valid for the current
    // base material before rendering — guards a base-material
    // change that orphaned it.
    layer.slacker_fraction =
        resolve_slacker_fraction(layer.material_anchor_key, layer.slacker_fraction);

    match slacker::support(layer.material_anchor_key) {
        slacker::Support::Curve(curve) => {
            let selected_label = curve
                .iter()
                .find(|p| (p.slacker_fraction - layer.slacker_fraction).abs() < f64::EPSILON)
                .map_or_else(|| "No Slacker".to_string(), slacker_point_label);
            egui::ComboBox::from_id_salt(("layer-slacker", layer_index))
                .selected_text(selected_label)
                .show_ui(ui, |ui| {
                    for p in curve {
                        ui.selectable_value(
                            &mut layer.slacker_fraction,
                            p.slacker_fraction,
                            slacker_point_label(p),
                        );
                    }
                });
            // Mix-in-grams readout for the selected ratio.
            match pour_mass_kg {
                Some(mass_kg) => {
                    let (part_ab_g, slacker_g) =
                        slacker_recipe_grams(mass_kg, layer.slacker_fraction);
                    if layer.slacker_fraction > 0.0 {
                        ui.label(format!(
                            "  Mix: {part_ab_g:.1} g A + {part_ab_g:.1} g B + {slacker_g:.1} g Slacker",
                        ));
                    } else {
                        ui.label(format!("  Mix: {part_ab_g:.1} g A + {part_ab_g:.1} g B"));
                    }
                }
                None => {
                    ui.label("  Mix: —");
                }
            }
        }
        slacker::Support::NotRecommended => {
            ui.add_enabled(
                false,
                egui::Label::new("Slacker: not recommended for this silicone"),
            );
        }
        slacker::Support::NoData => {
            ui.add_enabled(
                false,
                egui::Label::new("Slacker: no published data for this silicone"),
            );
        }
    }
}

/// Block orbit-camera input when the pointer is over the egui side
/// panel — prevents the sidebar from accidentally rotating the
/// camera when the user scrolls a slider list. Mirror of
/// cf-scan-prep's matching system.
#[allow(clippy::needless_pass_by_value)] // Bevy systems take resources by value.
fn block_orbit_input_when_over_egui(
    mut contexts: EguiContexts,
    mut motion: ResMut<AccumulatedMouseMotion>,
    mut scroll: ResMut<AccumulatedMouseScroll>,
) -> bevy::ecs::error::Result {
    let ctx = contexts.ctx_mut()?;
    if ctx.wants_pointer_input() || ctx.is_pointer_over_area() {
        motion.delta = Vec2::ZERO;
        scroll.delta = Vec2::ZERO;
    }
    Ok(())
}

#[allow(clippy::needless_pass_by_value)] // Bevy systems take resources by value.
fn exit_on_esc(keys: Res<ButtonInput<KeyCode>>, mut exit: MessageWriter<AppExit>) {
    if keys.just_pressed(KeyCode::Escape) {
        exit.write(AppExit::Success);
    }
}

/// Right-side egui sidebar carrying the design-suite panels:
/// Scan Info + Cavity + Layers + Validations + Clip-plane + Save-Open.
///
/// [`compute_validations`] runs once at the top of the panel, from
/// the current resource state, and the resulting [`DeviceValidations`]
/// is shared by two consumers: the Layers section (each layer's
/// pour mass drives its Slacker mix-in-grams readout) and the
/// Validations section. A slider edit this frame lands in those
/// readouts one frame later — imperceptible at frame rate, and it
/// keeps both sections reading a single consistent computation
/// rather than each re-deriving it.
// 11 system parameters; over the default clippy cap. The alternative
// is bundling resources into a SystemParam struct, which for this
// single all-panels driver is more ceremony than the cap is worth —
// every parameter is a Bevy resource consumed by a single section of
// the panel.
#[allow(clippy::needless_pass_by_value, clippy::too_many_arguments)]
fn device_design_panel(
    mut contexts: EguiContexts,
    info: Res<ScanInfo>,
    cached_sdf: Res<sdf_layers::CachedScanSdf>,
    cap_planes: Res<sdf_layers::CapPlanes>,
    scan_path: Res<ScanFilePath>,
    centerline: Res<Centerline>,
    mut scan_visible: ResMut<ScanMeshVisible>,
    mut cavity: ResMut<CavityState>,
    mut layers: ResMut<LayersState>,
    mut clip_state: ResMut<clip_plane::ClipPlaneState>,
    mut save_state: ResMut<SaveState>,
) -> bevy::ecs::error::Result {
    let ctx = contexts.ctx_mut()?;
    let validations = compute_validations(&cached_sdf, &cap_planes, &cavity, &layers);
    let centerline_available = centerline.points_m.len() >= 2;
    egui::SidePanel::right("cf-device-design-panels")
        .resizable(false)
        .default_width(320.0)
        .show(ctx, |ui| {
            egui::ScrollArea::vertical()
                .auto_shrink([false, true])
                .show(ui, |ui| {
                    render_scan_info_section(ui, &info, &mut scan_visible);
                    render_cavity_section(ui, &mut cavity);
                    render_layers_section(ui, &mut layers, &cavity, &validations);
                    render_validations_section(ui, &validations);
                    ui.separator();
                    clip_plane::render_clip_plane_section(
                        ui,
                        &mut clip_state,
                        centerline_available,
                    );
                    ui.separator();
                    render_save_open_section(ui, &mut save_state, &cavity, &layers, &scan_path.0);
                    render_panel_stubs(ui);
                });
        });
    Ok(())
}

fn render_scan_info_section(
    ui: &mut egui::Ui,
    info: &ScanInfo,
    scan_visible: &mut ScanMeshVisible,
) {
    egui::CollapsingHeader::new("Scan Info")
        .default_open(true)
        .show(ui, |ui| {
            ui.checkbox(&mut scan_visible.0, "Show scan mesh");
            ui.label(format!("File:  {}", info.file_label));
            ui.label(format!("Vertices: {}", human_count(info.vertex_count)));
            ui.label(format!("Faces: {}", human_count(info.face_count)));
            ui.label("Raw AABB:");
            ui.label(format!("  W: {:.1} mm", info.aabb_mm_extents[0]));
            ui.label(format!("  D: {:.1} mm", info.aabb_mm_extents[1]));
            ui.label(format!("  H: {:.1} mm", info.aabb_mm_extents[2]));
            ui.label(format!("Diagonal: {:.1} mm", info.bbox_diagonal_m * 1000.0));
            ui.separator();
            if info.centerline_point_count == 0 {
                ui.label("Centerline: not available");
                ui.label("  Reopen the scan in cf-scan-prep,");
                ui.label("  apply [Cap] + centerline,");
                ui.label("  and re-save to enable.");
            } else {
                ui.label(format!(
                    "Centerline: {} points",
                    info.centerline_point_count
                ));
                ui.label(format!(
                    "  arc length: {:.1} mm",
                    info.centerline_arc_length_m * 1000.0
                ));
            }
        });
}

/// Stub-section placeholders for panels arriving in later slices.
/// Save/Open took the previous stub; nothing left to render today.
/// The function remains so a future slice (Features / Texture) can
/// hang its own placeholder back on it.
fn render_panel_stubs(ui: &mut egui::Ui) {
    let _ = ui;
}

/// Slice 8 — Save state for the Save/Open panel. Tracks the resolved
/// design.toml path + last-save status (success message or error)
/// for the panel readout.
#[derive(Resource, Debug, Default)]
struct SaveState {
    /// Where `[Save]` writes to — resolved from the cleaned-STL path
    /// or the `--design` flag at startup. `None` only in the
    /// bare-filename-with-no-parent edge case (where the panel
    /// disables `[Save]`).
    path: Option<PathBuf>,
    /// `Some(msg)` after a save attempt — green on success, red on
    /// error. Cleared when the user clicks `[Save]` again.
    last_message: Option<SaveMessage>,
}

#[derive(Debug, Clone)]
enum SaveMessage {
    /// `[Save]` succeeded — path it wrote to.
    Ok(String),
    /// `[Save]` failed — error chain.
    Err(String),
}

impl SaveState {
    fn new(path: Option<PathBuf>) -> Self {
        Self {
            path,
            last_message: None,
        }
    }
}

/// Slice 8 — Save / Open panel section. Clicking `[💾 Save Design]`
/// writes the current `(CavityState, LayersState)` to the resolved
/// `.design.toml` path, atomically (tmp + rename). Open is implicit:
/// re-launch with `--design <PATH>` (or the auto-default) — no in-app
/// open dialog yet (would need a `rfd`-class dep + a "reload mid-
/// session" path; deferred).
#[allow(clippy::needless_pass_by_value)]
fn render_save_open_section(
    ui: &mut egui::Ui,
    save_state: &mut SaveState,
    cavity: &CavityState,
    layers: &LayersState,
    cleaned_stl: &Path,
) {
    egui::CollapsingHeader::new("Save / Open")
        .default_open(false)
        .show(ui, |ui| {
            match &save_state.path {
                Some(p) => {
                    ui.label(format!("Path: {}", p.display()));
                    if ui.button("💾 Save Design").clicked() {
                        let design = design_toml::build_design_toml(cleaned_stl, cavity, layers);
                        save_state.last_message =
                            Some(match design_toml::save_design_toml(&design, p) {
                                Ok(()) => SaveMessage::Ok(format!(
                                    "saved {} layers · cavity {:.2} mm",
                                    layers.layers.len(),
                                    cavity.inset_m * 1e3,
                                )),
                                Err(e) => SaveMessage::Err(format!("{e:#}")),
                            });
                    }
                }
                None => {
                    ui.label(
                        "(no parent directory to write into — \
                         supply an absolute --design path on launch)",
                    );
                }
            }
            ui.label("Open: re-launch with `--design <path>`");
            if let Some(msg) = &save_state.last_message {
                match msg {
                    SaveMessage::Ok(m) => {
                        ui.colored_label(egui::Color32::from_rgb(90, 200, 110), format!("✓ {m}"));
                    }
                    SaveMessage::Err(m) => {
                        ui.colored_label(egui::Color32::from_rgb(225, 90, 80), format!("✗ {m}"));
                    }
                }
            }
        });
}

/// egui color for a [`BudgetStatus`] — green / amber / red.
fn budget_color(status: BudgetStatus) -> egui::Color32 {
    match status {
        BudgetStatus::Green => egui::Color32::from_rgb(90, 200, 110),
        BudgetStatus::Yellow => egui::Color32::from_rgb(230, 180, 60),
        BudgetStatus::Red => egui::Color32::from_rgb(225, 90, 80),
    }
}

/// Render the Validations egui section — per-layer pour volume +
/// mass graded against the 2 lb single-pour budget, plus the cavity
/// self-intersection and minimum-castable-wall checks.
///
/// All figures are derived; the section is read-only. The headline
/// color is the worst per-layer budget status. See
/// [`compute_validations`] for the geometry behind each number.
fn render_validations_section(ui: &mut egui::Ui, v: &DeviceValidations) {
    egui::CollapsingHeader::new("Validations")
        .default_open(true)
        .show(ui, |ui| {
            let n = v.layers.len();
            ui.colored_label(
                budget_color(v.worst_status),
                format!(
                    "Pour mass — budget {:.0} g / 2 lb per pour",
                    MASS_BUDGET_KG * 1000.0,
                ),
            );
            for lv in &v.layers {
                ui.colored_label(
                    budget_color(lv.status),
                    format!(
                        "  Layer {} ({}): {:.1} cm³ · {:.1} g",
                        lv.layer_index,
                        layer_position_label(lv.layer_index, n),
                        lv.shell_volume_m3 * 1.0e6,
                        lv.pour_mass_kg * 1000.0,
                    ),
                );
            }
            ui.label(format!(
                "Total: {:.1} g across {} pour{}",
                v.total_mass_kg * 1000.0,
                n,
                if n == 1 { "" } else { "s" },
            ));

            ui.separator();

            // Cavity self-intersection.
            if v.cavity_collapse_inset_m.is_infinite() {
                ui.label("Cavity: n/a (no centerline)");
            } else if v.cavity_self_intersects {
                ui.colored_label(
                    budget_color(BudgetStatus::Red),
                    format!(
                        "Cavity self-intersects (inset ≥ {:.1} mm)",
                        v.cavity_collapse_inset_m * 1000.0,
                    ),
                );
            } else {
                ui.colored_label(
                    budget_color(BudgetStatus::Green),
                    format!(
                        "Cavity: OK (collapses at inset ≥ {:.1} mm)",
                        v.cavity_collapse_inset_m * 1000.0,
                    ),
                );
            }

            // Minimum castable wall thickness.
            if v.min_wall_ok {
                ui.colored_label(
                    budget_color(BudgetStatus::Green),
                    format!(
                        "Min wall: OK (every layer ≥ {:.1} mm)",
                        MIN_CASTABLE_THICKNESS_M * 1000.0,
                    ),
                );
            } else {
                for lv in v.layers.iter().filter(|lv| !lv.thickness_castable) {
                    ui.colored_label(
                        budget_color(BudgetStatus::Red),
                        format!(
                            "Layer {} below {:.1} mm castability floor",
                            lv.layer_index,
                            MIN_CASTABLE_THICKNESS_M * 1000.0,
                        ),
                    );
                }
            }
        });
}

/// Truncate large counts to k / M suffixes for compact panel
/// display. Matches cf-scan-prep's formatter.
fn human_count(n: usize) -> String {
    if n >= 1_000_000 {
        #[allow(clippy::cast_precision_loss)]
        let m = n as f64 / 1_000_000.0;
        format!("{m:.2}M")
    } else if n >= 1_000 {
        #[allow(clippy::cast_precision_loss)]
        let k = n as f64 / 1_000.0;
        format!("{k:.1}k")
    } else {
        n.to_string()
    }
}

#[allow(clippy::too_many_arguments)]
fn run_render_app(
    scan_mesh: IndexedMesh,
    scan_info: ScanInfo,
    centerline_points: Vec<Point3<f64>>,
    cleaned_stl_path: PathBuf,
    design_toml_path: Option<PathBuf>,
    loaded_design: Option<design_toml::DesignToml>,
    cached_scan_sdf: sdf_layers::CachedScanSdf,
    cap_planes: sdf_layers::CapPlanes,
) {
    #[allow(clippy::cast_possible_truncation)] // f64 → f32 is intentional for Bevy.
    let raw_diagonal = scan_mesh.aabb().diagonal() as f32;
    let render_scale = compute_render_scale(raw_diagonal);
    // Slice 8 — start from defaults, then apply the loaded design if
    // any. `apply_design_toml` errors only on catalog lookup, already
    // gated by `validate_design_toml` at `load_design_toml`. A failure
    // here means the catalog regressed between load and apply — fall
    // back to defaults + log, don't crash the GUI.
    let mut cavity = CavityState::default_for_scan();
    let mut layers = LayersState::default_for_scan();
    if let Some(d) = &loaded_design {
        if let Err(e) = design_toml::apply_design_toml(d, &mut cavity, &mut layers) {
            eprintln!(
                "warning: apply_design_toml failed on a validated load ({e:#}); \
                 starting with defaults"
            );
            cavity = CavityState::default_for_scan();
            layers = LayersState::default_for_scan();
        }
    }
    let save_state = SaveState::new(design_toml_path);

    App::new()
        .add_plugins(DefaultPlugins.set(WindowPlugin {
            primary_window: Some(Window {
                title: "cf-device-design".into(),
                ..default()
            }),
            ..default()
        }))
        .add_plugins(EguiPlugin::default())
        .add_plugins(OrbitCameraPlugin)
        .add_plugins(clip_plane::ClipPlanePlugin)
        .insert_resource(ClearColor(Color::srgb(0.10, 0.10, 0.12)))
        .insert_resource(DEVICE_UP_AXIS)
        .insert_resource(RenderScale(render_scale))
        .insert_resource(ScanMesh(scan_mesh))
        .insert_resource(ScanFilePath(cleaned_stl_path))
        .insert_resource(scan_info)
        .insert_resource(Centerline {
            points_m: centerline_points,
        })
        .insert_resource(cached_scan_sdf)
        .insert_resource(cap_planes)
        .insert_resource(cavity)
        .insert_resource(layers)
        .insert_resource(save_state)
        .insert_resource(ScanMeshVisible::default())
        .add_systems(Startup, (setup_render_scene, spawn_cavity_mesh).chain())
        .add_systems(
            Update,
            (
                block_orbit_input_when_over_egui.before(orbit_camera_input),
                draw_reference_overlays,
                update_cavity_mesh,
                update_layer_meshes,
                apply_scan_mesh_visibility,
                exit_on_esc,
            ),
        )
        .add_systems(EguiPrimaryContextPass, device_design_panel)
        .run();
}

// ============================================================
// Tests
// ============================================================

#[cfg(test)]
mod tests {
    // `unwrap()` + `expect()` are denied at the crate level for
    // production safety; allow them inside tests so assertions can
    // pull values out of `Option` / `Result` returns without
    // multi-line `match` ceremony.
    #![allow(clippy::unwrap_used, clippy::expect_used)]

    use super::*;

    fn unit_cube_mesh() -> IndexedMesh {
        let mut m = IndexedMesh::new();
        let h = 0.05;
        for (x, y, z) in [
            (-h, -h, -h),
            (h, -h, -h),
            (h, h, -h),
            (-h, h, -h),
            (-h, -h, h),
            (h, -h, h),
            (h, h, h),
            (-h, h, h),
        ] {
            m.vertices.push(Point3::new(x, y, z));
        }
        for f in [
            [0, 2, 1],
            [0, 3, 2],
            [4, 5, 6],
            [4, 6, 7],
            [0, 1, 5],
            [0, 5, 4],
            [2, 3, 7],
            [2, 7, 6],
            [0, 4, 7],
            [0, 7, 3],
            [1, 2, 6],
            [1, 6, 5],
        ] {
            m.faces.push(f);
        }
        m
    }

    #[test]
    fn parses_prep_toml_centerline_when_present() {
        let text = r#"
[scan_prep]
source_stl = "raw.stl"
tool_version = "0.0.0-test"

[centerline]
points_m = [
    [0.0, 0.0, 0.0],
    [0.0, 0.0, 0.1],
    [0.0, 0.0, 0.2],
]
algorithm = "cross_section_centroids"
"#;
        let pts = parse_centerline(text).unwrap();
        assert_eq!(pts.len(), 3);
        assert!((pts[2].z - 0.2).abs() < 1e-12);
    }

    #[test]
    fn parse_centerline_absent_block_returns_empty() {
        let text = r#"
[scan_prep]
source_stl = "raw.stl"
"#;
        let pts = parse_centerline(text).unwrap();
        assert!(pts.is_empty());
    }

    #[test]
    fn parse_centerline_extra_fields_tolerated() {
        // Forward-compatibility: schema additions in cf-scan-prep
        // don't break cf-device-design's loader.
        let text = r#"
[scan_prep]
source_stl = "raw.stl"
future_field = 42

[centerline]
points_m = [[0.0, 0.0, 0.0], [1.0, 0.0, 0.0]]
algorithm = "cross_section_centroids"
another_future_field = "foo"
"#;
        let pts = parse_centerline(text).unwrap();
        assert_eq!(pts.len(), 2);
    }

    // parse_cap_planes_* tests lifted to cf-cap-planes alongside the
    // parser itself; see `design/cf-cap-planes/src/lib.rs`'s test module.

    #[test]
    fn centerline_arc_length_sums_segment_distances() {
        let pts = vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
            Point3::new(1.0, 1.0, 1.0),
        ];
        let arc = centerline_arc_length_m(&pts);
        assert!((arc - 3.0).abs() < 1e-12, "arc = {arc}");
    }

    #[test]
    fn centerline_arc_length_zero_for_short_polylines() {
        assert_eq!(centerline_arc_length_m(&[]), 0.0);
        assert_eq!(centerline_arc_length_m(&[Point3::origin()]), 0.0);
    }

    #[test]
    fn build_scan_info_populates_all_fields() {
        let mesh = unit_cube_mesh();
        let centerline = vec![Point3::new(0.0, 0.0, 0.0), Point3::new(0.0, 0.0, 0.05)];
        let info = build_scan_info(Path::new("/scans/test.cleaned.stl"), &mesh, &centerline);
        assert_eq!(info.file_label, "test.cleaned.stl");
        assert_eq!(info.vertex_count, 8);
        assert_eq!(info.face_count, 12);
        assert!((info.aabb_mm_extents[0] - 100.0).abs() < 1e-9);
        assert_eq!(info.centerline_point_count, 2);
        assert!((info.centerline_arc_length_m - 0.05).abs() < 1e-12);
    }

    #[test]
    fn resolve_prep_toml_strips_cleaned_suffix_from_stem() {
        let cli = Cli {
            cleaned_stl: PathBuf::from("/scans/sock.cleaned.stl"),
            design: None,
            prep_toml: None,
        };
        let p = resolve_prep_toml_path(&cli);
        assert_eq!(p, Some(PathBuf::from("/scans/sock.prep.toml")));
    }

    #[test]
    fn resolve_prep_toml_honors_explicit_flag() {
        let cli = Cli {
            cleaned_stl: PathBuf::from("/scans/sock.cleaned.stl"),
            design: None,
            prep_toml: Some(PathBuf::from("/elsewhere/custom.prep.toml")),
        };
        let p = resolve_prep_toml_path(&cli);
        assert_eq!(p, Some(PathBuf::from("/elsewhere/custom.prep.toml")));
    }

    #[test]
    fn resolve_prep_toml_handles_stem_without_cleaned_suffix() {
        // Defensive — user supplies a non-cleaned STL; we still try
        // the `<stem>.prep.toml` sibling.
        let cli = Cli {
            cleaned_stl: PathBuf::from("/scans/raw.stl"),
            design: None,
            prep_toml: None,
        };
        let p = resolve_prep_toml_path(&cli);
        assert_eq!(p, Some(PathBuf::from("/scans/raw.prep.toml")));
    }

    #[test]
    fn human_count_formats_orders_of_magnitude() {
        assert_eq!(human_count(0), "0");
        assert_eq!(human_count(500), "500");
        assert_eq!(human_count(1_500), "1.5k");
        assert_eq!(human_count(200_000), "200.0k");
        assert_eq!(human_count(3_350_000), "3.35M");
    }

    #[test]
    fn device_up_axis_is_plus_z() {
        // Pinned: every cast tool in CortenForge uses +Z up. Drifting
        // away would mismatch cf-scan-prep and cf-cast.
        assert_eq!(DEVICE_UP_AXIS, UpAxis::PlusZ);
    }

    fn approx_eq(a: f64, b: f64, eps: f64) -> bool {
        (a - b).abs() < eps
    }

    // ----- Slice 4 v1 — cavity state -------------------------------

    #[test]
    fn cavity_default_inset_is_three_mm() {
        // 3 mm = minimum-acceptable buildable design (≥ Ecoflex's
        // ~2 mm castability threshold + ~10 % radial pre-strain).
        let state = CavityState::default_for_scan();
        assert!(approx_eq(state.inset_m, 0.003, 1e-12));
    }

    #[test]
    fn cavity_inset_slider_range_zero_to_eight_mm() {
        // Sentinel test pinning the workshop cavity-inset envelope.
        // The cap value lives in `CAVITY_INSET_SLIDER_MAX_M`; see
        // its docstring + `CavityState::inset_slider_range_m` for
        // the workshop-envelope rationale + a pointer to the
        // sim-research convergence-envelope history.
        let (min_m, max_m) = CavityState::inset_slider_range_m();
        assert!(approx_eq(min_m, 0.0, 1e-12));
        assert!(approx_eq(
            max_m,
            cf_device_types::CAVITY_INSET_SLIDER_MAX_M,
            1e-12,
        ));
        // Pin the const at 8 mm so a future bound-edit notices.
        assert!(approx_eq(
            cf_device_types::CAVITY_INSET_SLIDER_MAX_M,
            0.008,
            1e-12,
        ));
    }

    #[test]
    fn default_surfaces_are_visible() {
        // Sanity: launching the app shows the cavity + every
        // default layer. If we ever flip a default to "hidden,"
        // the user would see a near-empty viewport on launch —
        // worth a regression pin.
        let cv = CavityState::default_for_scan();
        let ls = LayersState::default_for_scan();
        assert!(cv.visible);
        assert!(ls.layers.iter().all(|l| l.visible));
    }

    // ----- Slice 5 v1 — layers state -------------------------------

    #[test]
    fn layers_default_is_single_visible_ecoflex_layer() {
        let layers = LayersState::default_for_scan();
        assert_eq!(layers.layers.len(), 1);
        assert_eq!(layers.layers[0].material_anchor_key, "ECOFLEX_00_30");
        assert!(approx_eq(layers.layers[0].thickness_m, 0.005, 1e-12));
        assert!(layers.layers[0].visible);
    }

    #[test]
    fn layer_count_max_is_six() {
        // Workshop-cap pin: more than 6 layers makes the panel
        // unwieldy and the per-frame per-layer surface-mesh
        // regeneration cost climb. Pinning so a future "let me have
        // 100 layers" accident is caught.
        assert_eq!(LAYER_COUNT_MAX, 6);
    }

    #[test]
    fn layer_material_catalog_covers_all_silicone_anchor_keys() {
        // Mirrors cf-cast's cure-table anchor keys; verify count +
        // each entry name to catch typos / drift from cf-cast's
        // table. If cf-cast ever adds / removes a silicone, this
        // test fails and we update both sides.
        let keys: Vec<&str> = LAYER_MATERIALS.iter().map(|(k, _, _)| *k).collect();
        assert_eq!(keys.len(), 8);
        assert!(keys.contains(&"ECOFLEX_00_10"));
        assert!(keys.contains(&"ECOFLEX_00_20"));
        assert!(keys.contains(&"ECOFLEX_00_30"));
        assert!(keys.contains(&"ECOFLEX_00_50"));
        assert!(keys.contains(&"DRAGON_SKIN_10A"));
        assert!(keys.contains(&"DRAGON_SKIN_15"));
        assert!(keys.contains(&"DRAGON_SKIN_20A"));
        assert!(keys.contains(&"DRAGON_SKIN_30A"));
    }

    #[test]
    fn layer_material_densities_mirror_silicone_table() {
        // Densities mirror `sim-soft`'s `silicone_table.rs` anchor
        // values by-name. If sim-soft revises an anchor density, this
        // pin fails and we update the catalog to match.
        assert!(approx_eq(material_density("ECOFLEX_00_10"), 1040.0, 1e-9));
        assert!(approx_eq(material_density("ECOFLEX_00_20"), 1070.0, 1e-9));
        assert!(approx_eq(material_density("ECOFLEX_00_30"), 1070.0, 1e-9));
        assert!(approx_eq(material_density("ECOFLEX_00_50"), 1070.0, 1e-9));
        assert!(approx_eq(material_density("DRAGON_SKIN_10A"), 1070.0, 1e-9));
        assert!(approx_eq(material_density("DRAGON_SKIN_15"), 1070.0, 1e-9));
        assert!(approx_eq(material_density("DRAGON_SKIN_20A"), 1080.0, 1e-9));
        assert!(approx_eq(material_density("DRAGON_SKIN_30A"), 1080.0, 1e-9));
        // Every catalog density lands in the silicone band.
        for (key, _, density) in LAYER_MATERIALS {
            assert!(
                (1040.0..=1080.0).contains(density),
                "{key}: density {density} kg/m³ outside silicone band",
            );
        }
        // Unknown key → defensive median fallback.
        assert!(approx_eq(material_density("NOT_A_REAL_KEY"), 1070.0, 1e-9));
    }

    #[test]
    fn layer_position_labels() {
        assert_eq!(layer_position_label(0, 1), "single");
        assert_eq!(layer_position_label(0, 3), "innermost");
        assert_eq!(layer_position_label(1, 3), "middle");
        assert_eq!(layer_position_label(2, 3), "outermost");
        assert_eq!(layer_position_label(5, 6), "outermost");
    }

    #[test]
    fn layer_thickness_slider_range_one_to_twenty_mm() {
        let (min_m, max_m) = LayerSpec::thickness_slider_range_m();
        assert!(approx_eq(min_m, 0.001, 1e-12));
        assert!(approx_eq(max_m, 0.020, 1e-12));
    }

    #[test]
    fn default_scan_mesh_visible() {
        // Sanity pin: the scan mesh is shown by default. The user
        // can hide it via the Scan Info panel checkbox to inspect
        // the cavity, which sits INSIDE the scan and is occluded by
        // it when both are drawn.
        assert!(ScanMeshVisible::default().0);
    }

    /// `Aabb` re-export from `mesh_types` (transitive from
    /// `cf_geometry`) is the canonical aabb type the suite consumes
    /// for the scan AABB readouts + render-scale sizing. Pin
    /// compile-time visibility so a future workspace dep shuffle
    /// doesn't silently break the loader.
    #[test]
    fn aabb_canonical_type_in_scope() {
        use mesh_types::Aabb;
        let a = Aabb::new(Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 1.0, 1.0));
        assert!((a.diagonal() - (3.0_f64).sqrt()).abs() < 1e-12);
    }

    // ----- Slice 6 — geometric validations -------------------------

    #[test]
    fn signed_volume_recovers_unit_cube_volume() {
        // The unit cube fixture is 0.1 m on a side → 1e-3 m³.
        // Divergence-theorem volume on the cube mesh directly (no
        // displacement) recovers the analytical volume — positive
        // because cf-scan-prep meshes wind CCW / outward (the fixture
        // mirrors that convention).
        let vol = signed_volume_m3(&unit_cube_mesh());
        assert!(
            approx_eq(vol, 1.0e-3, 1e-9),
            "unit-cube signed volume = {vol}, expected 1e-3",
        );
    }

    #[test]
    fn signed_volume_closed_mesh_translation_invariant() {
        // For a CLOSED mesh the divergence integral is independent of
        // the choice of integration origin — internal tetrahedra
        // cancel. The pinned-floor scope-C arc's sub-leaf 3 dropped
        // the `origin` parameter; this test pins the equivalent
        // property by translating the mesh in space and confirming
        // the volume stays bit-identical.
        let mesh = unit_cube_mesh();
        let vol_orig = signed_volume_m3(&mesh);
        let shifted = IndexedMesh {
            vertices: mesh
                .vertices
                .iter()
                .map(|v| Point3::new(v.x + 1.0, v.y + 2.0, v.z + 3.0))
                .collect(),
            faces: mesh.faces.clone(),
        };
        let vol_shifted = signed_volume_m3(&shifted);
        assert!(
            approx_eq(vol_orig, vol_shifted, 1e-9),
            "closed-mesh volume must be translation-invariant: \
             orig {vol_orig} vs shifted {vol_shifted}",
        );
    }

    // Three primary_cap_origin_* tests + two open-mesh
    // signed_volume_* tests were deleted alongside the
    // primary_cap_origin function + the origin parameter (sub-leaf 3).
    // Closed pinned-floor shells (sub-leaf 2) are origin-invariant by
    // the divergence theorem, so the cap-centroid-origin trick the
    // cavity-mouth arc shipped is no longer needed.

    #[test]
    fn signed_volume_grows_with_outward_offset_extracted_mesh() {
        // Extract two iso surfaces from the same SDF cache (small
        // cube fixture, off-grid margin to dodge the mesh-sdf
        // sign-tie at face planes — same posture as sdf_layers'
        // own tests). The outward iso should enclose more volume
        // than the inward iso. This is the property
        // `compute_validations` relies on for monotonic shell
        // volumes.
        let cube = unit_cube_mesh();
        let cache =
            sdf_layers::build_cached_scan_sdf(&cube, &[], 0.005, 0.043).expect("build cache");
        let inner = sdf_layers::extract_layer_surface(&cache, &[], -0.005);
        let outer = sdf_layers::extract_layer_surface(&cache, &[], 0.005);
        let v_inner = signed_volume_m3(&inner);
        let v_outer = signed_volume_m3(&outer);
        assert!(
            v_inner < v_outer,
            "inner ({v_inner}) should enclose less volume than outer ({v_outer})",
        );
    }

    #[test]
    fn grade_budget_thresholds() {
        // Green below 0.8× budget, yellow in [0.8×, 1.0×], red above.
        assert_eq!(grade_budget(0.1), BudgetStatus::Green);
        assert_eq!(grade_budget(MASS_BUDGET_KG * 0.5), BudgetStatus::Green);
        assert_eq!(grade_budget(MASS_BUDGET_KG * 0.8), BudgetStatus::Yellow);
        assert_eq!(grade_budget(MASS_BUDGET_KG * 0.99), BudgetStatus::Yellow);
        assert_eq!(grade_budget(MASS_BUDGET_KG), BudgetStatus::Yellow);
        assert_eq!(grade_budget(MASS_BUDGET_KG * 1.01), BudgetStatus::Red);
    }

    #[test]
    fn budget_status_orders_by_severity() {
        // `worst_status` relies on the derived `Ord` matching the
        // severity order Green < Yellow < Red.
        assert!(BudgetStatus::Green < BudgetStatus::Yellow);
        assert!(BudgetStatus::Yellow < BudgetStatus::Red);
        assert_eq!(
            [BudgetStatus::Green, BudgetStatus::Red, BudgetStatus::Yellow]
                .into_iter()
                .max(),
            Some(BudgetStatus::Red),
        );
    }

    /// Build a cached scan SDF from the unit-cube fixture suitable
    /// for the slice-6 validation tests. Cell pitch 5 mm, off-grid
    /// margin (`MARGIN_OFFSET_M` defined inline at 0.043 m) to dodge
    /// `mesh-sdf::compute_sign`'s tie at axis-aligned grid points.
    fn cube_cached_sdf() -> sdf_layers::CachedScanSdf {
        sdf_layers::build_cached_scan_sdf(&unit_cube_mesh(), &[], 0.005, 0.043)
            .expect("build cached scan SDF for unit cube fixture")
    }

    #[test]
    fn compute_validations_one_entry_per_layer_with_positive_shells() {
        // Two-layer stack on the cube SDF: one validation per layer,
        // every shell volume + mass strictly positive, and the total
        // mass is the sum of the per-layer pour masses.
        let cache = cube_cached_sdf();
        let cavity = CavityState {
            inset_m: 0.003,
            visible: true,
        };
        let layers = LayersState {
            layers: vec![
                LayerSpec {
                    thickness_m: 0.005,
                    material_anchor_key: "ECOFLEX_00_30",
                    slacker_fraction: 0.0,
                    visible: true,
                },
                LayerSpec {
                    thickness_m: 0.004,
                    material_anchor_key: "DRAGON_SKIN_20A",
                    slacker_fraction: 0.0,
                    visible: true,
                },
            ],
        };
        let v = compute_validations(&cache, &sdf_layers::CapPlanes::default(), &cavity, &layers);
        assert_eq!(v.layers.len(), 2);
        for lv in &v.layers {
            assert!(lv.shell_volume_m3 > 0.0, "shell volume must be positive");
            assert!(lv.pour_mass_kg > 0.0, "pour mass must be positive");
        }
        let summed: f64 = v.layers.iter().map(|lv| lv.pour_mass_kg).sum();
        assert!(
            approx_eq(v.total_mass_kg, summed, 1e-12),
            "total {} != sum of layers {}",
            v.total_mass_kg,
            summed,
        );
        // Pour mass uses the per-layer material density.
        let l1 = &v.layers[1];
        assert!(approx_eq(
            l1.pour_mass_kg,
            l1.shell_volume_m3 * material_density("DRAGON_SKIN_20A"),
            1e-12,
        ));
    }

    #[test]
    fn compute_validations_flags_cavity_self_intersection() {
        // Below the cached SDF's max-interior-depth the cavity is
        // fine; at or above it the cavity MC mesh is empty and the
        // self-intersection flag fires. SDF analog of the prior
        // `proxy.min_radial_distance_m` check.
        let cache = cube_cached_sdf();
        let collapse = -cache.min_sdf_value;
        assert!(collapse.is_finite() && collapse > 0.0);
        let layers = LayersState::default_for_scan();

        let safe = compute_validations(
            &cache,
            &sdf_layers::CapPlanes::default(),
            &CavityState {
                inset_m: collapse * 0.5,
                visible: true,
            },
            &layers,
        );
        assert!(!safe.cavity_self_intersects);
        assert!(approx_eq(safe.cavity_collapse_inset_m, collapse, 1e-12));

        let collapsed = compute_validations(
            &cache,
            &sdf_layers::CapPlanes::default(),
            &CavityState {
                inset_m: collapse * 1.5,
                visible: true,
            },
            &layers,
        );
        assert!(collapsed.cavity_self_intersects);
    }

    #[test]
    fn compute_validations_min_wall_flags_sub_floor_layers() {
        let cache = cube_cached_sdf();
        let cavity = CavityState {
            inset_m: 0.003,
            visible: true,
        };
        // A 1 mm layer is below the 2 mm castability floor.
        let thin = LayersState {
            layers: vec![LayerSpec {
                thickness_m: 0.001,
                material_anchor_key: "ECOFLEX_00_30",
                slacker_fraction: 0.0,
                visible: true,
            }],
        };
        let v_thin = compute_validations(&cache, &sdf_layers::CapPlanes::default(), &cavity, &thin);
        assert!(!v_thin.min_wall_ok);
        assert!(!v_thin.layers[0].thickness_castable);

        // A 3 mm layer clears the floor.
        let thick = LayersState {
            layers: vec![LayerSpec {
                thickness_m: 0.003,
                material_anchor_key: "ECOFLEX_00_30",
                slacker_fraction: 0.0,
                visible: true,
            }],
        };
        let v_thick =
            compute_validations(&cache, &sdf_layers::CapPlanes::default(), &cavity, &thick);
        assert!(v_thick.min_wall_ok);
        assert!(v_thick.layers[0].thickness_castable);
    }

    #[test]
    fn compute_validations_worst_status_is_the_max_layer_status() {
        // `worst_status` must equal the most-severe per-layer grade.
        let cache = cube_cached_sdf();
        let cavity = CavityState {
            inset_m: 0.003,
            visible: true,
        };
        let layers = LayersState {
            layers: vec![
                LayerSpec {
                    thickness_m: 0.005,
                    material_anchor_key: "ECOFLEX_00_30",
                    slacker_fraction: 0.0,
                    visible: true,
                },
                LayerSpec {
                    thickness_m: 0.012,
                    material_anchor_key: "DRAGON_SKIN_30A",
                    slacker_fraction: 0.0,
                    visible: true,
                },
            ],
        };
        let v = compute_validations(&cache, &sdf_layers::CapPlanes::default(), &cavity, &layers);
        let expected = v
            .layers
            .iter()
            .map(|lv| lv.status)
            .max()
            .unwrap_or(BudgetStatus::Green);
        assert_eq!(v.worst_status, expected);
    }

    #[test]
    fn mass_budget_is_two_pounds_to_kg_exact() {
        // Mirrors cf-cast's `DEFAULT_MASS_BUDGET_KG` by-value: 2 lb
        // via the NIST exact conversion 1 lb = 0.453_592_37 kg.
        let lb_to_kg = 0.453_592_37_f64;
        assert!((MASS_BUDGET_KG - (lb_to_kg + lb_to_kg)).abs() < f64::EPSILON);
    }

    // ----- Slice 6.5 — per-layer Slacker recipe --------------------

    #[test]
    fn default_layer_has_no_slacker() {
        // A fresh layer is plain base silicone — no Slacker.
        let layers = LayersState::default_for_scan();
        assert!(approx_eq(layers.layers[0].slacker_fraction, 0.0, 1e-12));
    }

    #[test]
    fn slacker_recipe_grams_splits_pour_mass_by_ratio() {
        // No Slacker: the pour is pure base silicone, 1:1 A:B.
        let (ab, slk) = slacker_recipe_grams(0.200, 0.0);
        assert!(approx_eq(ab, 100.0, 1e-9), "part A/B = {ab}");
        assert!(approx_eq(slk, 0.0, 1e-9), "slacker = {slk}");

        // 50 % Slacker on a 300 g pour → 100 g A + 100 g B + 100 g
        // Slacker (the TB's "100A + 100B + 100 Slacker" row).
        let (ab, slk) = slacker_recipe_grams(0.300, 0.50);
        assert!(approx_eq(ab, 100.0, 1e-9), "part A/B = {ab}");
        assert!(approx_eq(slk, 100.0, 1e-9), "slacker = {slk}");

        // 25 % Slacker: A + B + Slacker must sum back to the pour
        // mass, and Slacker must be 0.25 × the base A+B mass.
        let (ab, slk) = slacker_recipe_grams(0.250, 0.25);
        assert!(approx_eq(2.0 * ab + slk, 250.0, 1e-9));
        assert!(approx_eq(slk, 0.25 * 2.0 * ab, 1e-9));
    }

    #[test]
    fn slacker_point_label_distinguishes_native_from_slacker_points() {
        // The native (0.0) point reads "No Slacker"; a Slacker
        // point carries its percentage, hardness, and tack.
        let slacker::Support::Curve(curve) = slacker::support("ECOFLEX_00_30") else {
            unreachable!("Ecoflex 00-30 has a Slacker curve");
        };
        assert_eq!(slacker_point_label(&curve[0]), "No Slacker — Shore 00-30",);
        assert_eq!(
            slacker_point_label(&curve[1]),
            "+25% Slacker — Shore 000-40 (slight-to-very tack)",
        );
    }
}
