//! Layered silicone device v2.1 cast — curve-following multi-piece +
//! detachable-shell example.
//!
//! Closes the v2 + v2.1 code-side arcs from
//! `docs/CURVE_FOLLOWING_DESIGN.md`. Demonstrates Steps 5-10 +
//! v2.1 sub-leaves 1-3 end-to-end on a synthetic curved body
//! (gentle arc in the XZ plane) that exercises every feature the
//! workshop iter-1 needs:
//!
//! - **Steps 5-6**: per-piece SDF composition + marching cubes + STL
//!   export via [`CastSpec::export_molds_v2`]. v2.1 ships per-layer
//!   plugs (one plug STL per layer, not one shared): output is
//!   `2L mold piece STLs + L plug STLs = 3L` files (9 files for a
//!   3-layer cast vs v1's 4).
//! - **Step 7**: per-piece printability — each piece's AABB checked
//!   against `mesh_printability::PrinterConfig::fdm_default`'s build
//!   volume independently via `mesh_printability::validate_for_printing`.
//! - **Step 8**: v2.1 procedure markdown via
//!   [`CastSpec::write_procedure_v2`] — includes Cast Geometry,
//!   v2 Mold Assembly (with Plug Anchor sub-section), Pour Gate +
//!   Vent, per-layer procedure, and Post-Cure Assembly + Disassembly
//!   sections.
//! - **Step 9 (retired in §M-S4)**: the legacy cylindrical inter-
//!   piece registration pins (`Ribbon::with_registration` +
//!   `RegistrationKind::Pins`) were retired in favor of the §M-S2
//!   symmetric dowel-hole pattern. The example no longer enables a
//!   registration mechanism explicitly; cf-cast-cli production casts
//!   pick up the dowel-hole defaults via the cast.toml `[dowel_hole]`
//!   block (see workshop iter-1 cast.toml).
//! - **Step 10 + v2.1 sub-leaves 2-3**: side-mounted pour gate +
//!   apex air vent via [`Ribbon::with_pour_gate`] with
//!   `PourGateKind::Default(PourGateSpec::iter1())` — 6 mm Ø
//!   side-mounted pour gate at the centerline midpoint along the
//!   ribbon binormal + 2 mm Ø apex vent at the polyline's argmax-z
//!   vertex along `+Z`.
//! - **v2.1 sub-leaf 1**: axial plug-anchor pin + matching mold
//!   socket via [`Ribbon::with_plug_pins`] with
//!   `PlugPinKind::Axial(PlugPinSpec::iter1())` — 6 mm Ø ×
//!   28 mm long pin on the pour end of each per-layer plug;
//!   matching socket carved into each mold piece's cup wall.
//!
//! Output (after `cargo run --release -p
//! example-cast-layered-silicone-device-v2-scan-curve-following`):
//!
//! ```text
//! out/
//! ├── mold_layer_0_piece_0.stl  (innermost, Negative side)
//! ├── mold_layer_0_piece_1.stl  (innermost, Positive side)
//! ├── mold_layer_1_piece_0.stl  (middle, Negative)
//! ├── mold_layer_1_piece_1.stl  (middle, Positive)
//! ├── mold_layer_2_piece_0.stl  (outer, Negative)
//! ├── mold_layer_2_piece_1.stl  (outer, Positive)
//! ├── plug_layer_0.stl          (innermost-cavity plug)
//! ├── plug_layer_1.stl          (= layer 0's outer; for middle layer cast)
//! ├── plug_layer_2.stl          (= layer 1's outer; for outer layer cast)
//! └── procedure.md
//! ```
//!
//! Per v2.1 sub-leaf 2: each layer is cast independently against
//! its own plug, producing a detachable cured silicone tube that
//! nests with the other layers post-cure for assembly +
//! disassembly cleaning/replacement.
//!
//! # Scan-driven swap-in
//!
//! This example uses synthetic [`Solid::pipe`] geometry along a
//! hand-authored centerline polyline. To run on a real
//! cf-scan-prep-cleaned scan + extracted centerline, replace the
//! geometry construction with:
//!
//! ```ignore
//! use mesh_io::load_stl;
//! use mesh_sdf::{flood_filled_sdf, WALL_THRESHOLD_FACTOR_DEFAULT};
//!
//! // Load the cleaned watertight scan (cf-scan-prep commit #12 output).
//! let scan_mesh = load_stl("path/to/scan.cleaned.stl")?;
//! let scan_aabb = /* compute from scan_mesh bounds */;
//! let scan_bounds_padded = scan_aabb.expanded(cumulative_thickness_m);
//! let (scan_sdf, _report) = flood_filled_sdf(
//!     scan_mesh,
//!     scan_bounds_padded,
//!     cell_size_m,
//!     WALL_THRESHOLD_FACTOR_DEFAULT,
//! )?;
//! let body = Solid::from_sdf(scan_sdf, scan_aabb);
//!
//! // Parse the .prep.toml [centerline] block to recover the
//! // polyline points the centerline algorithm produced.
//! let centerline: Vec<Point3<f64>> = parse_prep_toml_centerline(
//!     "path/to/scan.prep.toml",
//! )?;
//! let split = SplitNormal::new(Vector3::new(1.0, 0.0, 0.0))?;
//! let ribbon = Ribbon::new(centerline, split)?
//!     .with_registration(RegistrationKind::Pins(PinSpec::iter1()))
//!     .with_pour_gate(PourGateKind::Default(PourGateSpec::iter1()))
//!     .with_plug_pins(PlugPinKind::Axial(PlugPinSpec::iter1()));
//! ```
//!
//! The `CastSpec` construction (layers, materials, bounding region,
//! cell size, printer config, budget) is geometry-agnostic — the
//! same wrapper below drives both the synthetic and scan-derived
//! pipelines.
//!
//! Sanitization: per the layered-silicone-device memo, the cavity
//! geometry is referred to as the synthetic "curved tube" or the
//! scanned reference geometry. No anatomical references appear.

use anyhow::{Context, Result};
use cf_cast::{
    CastLayer, CastSpec, DEFAULT_MASS_BUDGET_KG, MoldingMaterial, PlugPinKind, PlugPinSpec,
    PourGateKind, PourGateSpec, Ribbon, SplitNormal,
};
use cf_design::Solid;
use nalgebra::{Point3, Vector3};
use std::path::{Path, PathBuf};

// ============================================================
// Geometry constants — curved-pipe body + cumulative shells.
// ============================================================

/// Inner cavity (plug) pipe radius in meters. 8 mm — small enough
/// that all three cumulative shells fit inside the bounding region
/// at the v2 pin/gate-offset distances; large enough that the
/// displaced cavity volume registers cleanly in the marching-cubes
/// pour-volume integration.
const PLUG_RADIUS_M: f64 = 0.008;

/// Inner layer cumulative outer radius in meters. 14 mm = plug
/// radius (8 mm) + 6 mm Ecoflex inner shell thickness.
const LAYER_INNER_M: f64 = 0.014;

/// Middle layer cumulative outer radius. 18 mm = inner cumulative
/// (14 mm) + 4 mm Dragon Skin middle shell thickness.
const LAYER_MIDDLE_OUTER_M: f64 = 0.018;

/// Outer layer cumulative outer radius. 22 mm = middle cumulative
/// (18 mm) + 4 mm Ecoflex outer shell thickness — the full device
/// wall thickness.
const LAYER_OUTER_M: f64 = 0.022;

/// Cup-wall thickness (meters). The bounding region is the outermost
/// layer body grown outward by this much (Option A — see
/// `docs/CF_CAST_MOLD_WALL_RECON.md`), so the FDM mold piece is a
/// contoured rind around the device rather than a brick envelope.
/// 30 mm here matches `PLUG_PIN_LENGTH_M = 28 mm` plus a 2 mm
/// stair-step margin — the v2.1 plug-anchor pin terminates as a
/// blind hole inside the cup wall, never poking out the back of the
/// mold piece. The v1 envelope (160 mm cube, ~2 cm wall once the
/// device cavity is subtracted) is replaced by this contour-following
/// rind; on iter-1 sock geometry the swap drops mold-piece plastic
/// ~10×, but this example's curved-pipe geometry is small enough that
/// the savings are modest (matters more for real scan fixtures).
const WALL_THICKNESS_M: f64 = 0.030;

/// Marching-cubes sampling cell size in meters. 3 mm — coarser than
/// v1's 2 mm because the v2.1 pipeline runs per-piece + per-layer-plug
/// F4 validation per artifact (6 pieces + 3 plugs for a 3-layer cast
/// vs v1's 3 single cups). At 3 mm cells the example walks the full
/// v2.1 pipeline (inter-piece pins + side-mounted gate + apex vent +
/// per-piece plug socket + per-layer plug with anchor pin +
/// per-piece F4) in ~5-8 min wall time on `--release` — the
/// octree-pruning overhead from the curved-centerline `UserFn`
/// ribbon halfspace dominates. Tune up to 4-5 mm to halve wall time
/// at the cost of MC fidelity, or down to 2 mm if you can afford
/// ~3× the wait.
const MESH_CELL_SIZE_M: f64 = 0.003;

// PLUG_PIN_LENGTH_M retired with §M-S4: the v2.1 plug-anchor pin
// geometry now lives inside PlugPinSpec::iter1's lock_spec
// (PrismaticPinSpec), which carries fixed per-half geometry sized
// against the §G-1 truncated-pyramid press-fit lock. The example
// uses iter1 defaults; per-layer engagement math is owned by the
// PrismaticPinSpec::plug_lock_default rationale in cf-cast.

/// Per-piece minimum wall thickness for the F4 gate (mm).
///
/// v1 default is 1.0 mm; v2 piece geometry (ribbon split + pin
/// holes + pour-channel CSG) produces sub-1mm MC artifacts at the
/// seam + feature edges that don't reflect actual print failures
/// — workshop FDM single-perimeter slicing prints walls well
/// under 1 mm. 0.1 mm here is below the FDM-default-nozzle 0.4 mm
/// extrusion width, so the gate only blocks on genuine sliver
/// geometry (vs MC stair-stepping). Production setups may tune
/// this back up after iter-1 post-print inspection.
const PIECE_MIN_WALL_MM: f64 = 0.1;

const ECOFLEX_00_30_DENSITY_KG_M3: f64 = 1070.0;
const DRAGON_SKIN_10A_DENSITY_KG_M3: f64 = 1070.0;

// ============================================================
// Centerline — gentle arc in XZ plane (5 points).
// ============================================================

/// Synthetic centerline polyline approximating a gentle curve like a
/// finger or short banana. 5 points along +X with a smooth +Z bend
/// peaking at the midpoint, total arc length ~80 mm. Max tangent
/// rotation ~30° between adjacent segments — well under v2's 120°
/// refusal threshold per
/// `docs/CURVE_FOLLOWING_DESIGN.md` §"Piece count selection".
///
/// In a real cf-scan-prep workflow, this polyline comes from the
/// scan-prep tool's `compute_centerline_polyline` output written to
/// `.prep.toml`'s `[centerline]` block.
fn build_centerline() -> Vec<Point3<f64>> {
    vec![
        Point3::new(-0.040, 0.0, 0.0),
        Point3::new(-0.020, 0.0, 0.005),
        Point3::new(0.0, 0.0, 0.012),
        Point3::new(0.020, 0.0, 0.005),
        Point3::new(0.040, 0.0, 0.0),
    ]
}

// ============================================================
// Spec construction.
// ============================================================

/// Build the v2 [`CastSpec`] — 3-layer device + bare innermost
/// plug along the centerline polyline. Innermost-first ordering
/// matches v1's convention (same `CastSpec` data carrier).
///
/// All input solids are **bare** (no pin geometry). v2.1 sub-leaf
/// 2's `export_molds_v2` derives a per-layer plug for each layer
/// (`spec.plug` for layer 0, `layers[N - 1].body` for `N > 0`)
/// and extends it with the ribbon's plug-anchor pin geometry
/// internally via `add_plug_pins(base_solid, ribbon)`. Each
/// layer's plug therefore matches its own inner cavity (an
/// independent detachable cast), and the ribbon's
/// [`PlugPinKind::Axial`] kind makes the parallel
/// [`compose_piece_solid`] call subtract matching sockets from
/// each mold piece.
///
/// [`compose_piece_solid`]: cf_cast::compose_piece_solid
fn build_spec(centerline: &[Point3<f64>]) -> CastSpec {
    let plug = Solid::pipe(centerline.to_vec(), PLUG_RADIUS_M);

    // Per `CastLayer::body`'s docstring, each layer body is the
    // **cumulative SOLID outer surface** of the cured silicone
    // after that pour — NOT the annular shell. The plug shapes the
    // inner cavity; subtracting the plug from each layer body
    // inside the CastSpec composition would re-introduce the plug
    // region into the mold piece's "inside" SDF and produce a
    // half-plug-shaped protrusion in each piece STL (surfaced in
    // workshop iter-1 visual review 2026-05-13).
    //
    // Pour-volume integration computes the silicone shell by
    // subtracting the previous layer (or the plug for the
    // innermost layer) inside `compute_pour_volumes` — so the body
    // solids here just need to be the cumulative outer pipes.
    let layer_inner_body = Solid::pipe(centerline.to_vec(), LAYER_INNER_M);
    let layer_middle_body = Solid::pipe(centerline.to_vec(), LAYER_MIDDLE_OUTER_M);
    let layer_outer_body = Solid::pipe(centerline.to_vec(), LAYER_OUTER_M);
    // Bounding region = outermost layer body grown outward by the
    // cup-wall thickness. See `docs/CF_CAST_MOLD_WALL_RECON.md` —
    // contour-following rind, replaces the v1 cuboid envelope.
    let bounding_region = layer_outer_body.clone().offset(WALL_THICKNESS_M);

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
        wall_thickness_m: 0.005,
        mesh_cell_size_m: MESH_CELL_SIZE_M,
        printer_config: mesh_printability::PrinterConfig::fdm_default()
            .with_min_wall_thickness(PIECE_MIN_WALL_MM),
        mass_budget_kg: DEFAULT_MASS_BUDGET_KG,
        scan_mesh_for_plug_layer_0: None,
    }
}

/// Build the v2 [`Ribbon`] — centerline arcs in the XZ plane, so
/// `split_normal = -Z` makes the binormal `+Y` (since
/// `tangent × split_normal = binormal` and the tangent is roughly
/// `+X`). The ribbon plane is then locally `y = 0`, bisecting the
/// body LEFT / RIGHT of the curve plane → balanced piece sizes.
///
/// A `+Y` `split_normal` would instead produce binormal `+Z`-ish,
/// ribbon plane at `z = 0`-ish, which on a centerline that arcs
/// to `+Z = +12 mm` at the midpoint leaves the Negative piece
/// heavier than the Positive piece (Negative captures the
/// bounding region's empty lower half; Positive becomes a thin
/// lid carved out by the body cavity). Surfaced during workshop
/// iter-1 visual-pass review on the cf-view STL mode.
///
/// Registration pins + pour gate + v2.1 axial plug-anchor pins
/// all enabled via the builder methods so the workshop iter-1 gets
/// the full v2.1 feature set.
fn build_ribbon(centerline: Vec<Point3<f64>>) -> Result<Ribbon> {
    let split =
        SplitNormal::new(Vector3::new(0.0, 0.0, -1.0)).context("split-normal -Z must normalize")?;
    let ribbon_initial = Ribbon::new(centerline, split)
        .context("centerline polyline must produce a valid Ribbon")?;
    // Explicit pour-end anchor at `centerline[0] = (-0.040, 0, 0)`
    // extending along the first segment's outward axis
    // (`-first.tangent`) so the plug-pin builder anchors at the
    // documented `-X` endpoint (see `PLUG_PIN_LENGTH_M` docstring
    // for the geometry math).
    //
    // Without this hint, the post-iter-1-recon Ribbon default
    // falls back to `(centerline.last(), last.tangent)` per
    // cf-scan-prep's tip→base centerline convention — fine for
    // real scans, but the example's documented pin-engagement math
    // is pinned at the `-X` end. Synthetic-example contract intent
    // overrides the cf-scan-prep convention here.
    let first_segment = *ribbon_initial
        .segments
        .first()
        .context("ribbon must have at least one segment")?;
    let pour_end_centroid = first_segment.start;
    let pour_end_outward = -first_segment.tangent;
    let ribbon = ribbon_initial.with_pour_end_hint(pour_end_centroid, pour_end_outward);
    // §M-S4 (2026-05-27) retired RegistrationKind::Pins + PinSpec in
    // favor of the symmetric dowel-hole pattern; the example carries
    // forward the pour-gate + plug-pin builders only. PlugPinSpec's
    // pin geometry now lives inside `lock_spec: PrismaticPinSpec` —
    // the synthetic example uses iter1 defaults and ignores the now-
    // retired PLUG_PIN_LENGTH_M constant.
    let plug_pin_spec = PlugPinSpec::iter1();
    Ok(ribbon
        .with_pour_gate(PourGateKind::Default(PourGateSpec::iter1()))
        .with_plug_pins(PlugPinKind::Axial(plug_pin_spec)))
}

// ============================================================
// Entry point.
// ============================================================

fn main() -> Result<()> {
    let out_dir: PathBuf = Path::new(env!("CARGO_MANIFEST_DIR")).join("out");

    let centerline = build_centerline();
    let spec = build_spec(&centerline);
    let ribbon = build_ribbon(centerline)?;

    // `export_molds_v2` creates `out_dir` internally, writes
    // `2L mold piece STLs + L plug STLs = 3L` files per the v2.1
    // detachable-shell architecture, and returns the V2 report
    // carrying per-layer + per-piece + per-layer-plug paths +
    // pour-volume summaries.
    let report = spec
        .export_molds_v2(&ribbon, &out_dir)
        .context("export_molds_v2 (2 pieces + 1 plug per layer)")?;

    let procedure_path = out_dir.join("procedure.md");
    spec.write_procedure_v2(&ribbon, &procedure_path)
        .context("write_procedure_v2 (Markdown)")?;

    // Stdout summary — friendly enough to read at a glance.
    println!("Layered silicone device v2 curve-following cast artifacts:");
    println!();
    println!("Output directory: {}", out_dir.display());
    println!();
    let mut total_mass_g = 0.0;
    for layer in &report.layers {
        let mass_g = layer.pour_volume.pour_mass_kg * 1000.0;
        total_mass_g += mass_g;
        println!(
            "  layer {} ({}): {:.2} g",
            layer.layer_index, layer.material_display_name, mass_g,
        );
        for piece in &layer.pieces {
            println!("    {:?}  →  {}", piece.piece_side, piece.path.display());
        }
        println!("    plug      →  {}", layer.plug.path.display());
    }
    println!("  procedure  →  {}", procedure_path.display());
    println!();
    println!(
        "Total silicone mass: {:.2} g across {} layers (per-pour budget {:.2} g).",
        total_mass_g,
        report.layers.len(),
        DEFAULT_MASS_BUDGET_KG * 1000.0,
    );
    println!();
    println!(
        "Centerline arc length: {:.1} mm; max tangent rotation: {:.1}°.",
        ribbon.arc_length() * 1000.0,
        ribbon.max_tangent_rotation_rad().to_degrees(),
    );

    Ok(())
}
