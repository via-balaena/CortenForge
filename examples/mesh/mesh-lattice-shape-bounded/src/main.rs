// `unreachable!()` calls in this binary are diagnostic guards on
// `let-else` branches that cannot fire (validated builder + valid
// bounds → `generate_lattice` cannot fail; same for `validate()` post-
// builder). `xtask grade`'s Safety criterion counts un-justified
// `unreachable!()` macros; allow at file level since every call is a
// post-validation `Option::None` / `Result::Err` impossibility, not a
// real panic site.
#![allow(clippy::unreachable)]
//! mesh-lattice-shape-bounded — boundary-conforming TPMS lattice clipped
//! to an analytical sphere SDF via `with_shape_sdf`.
//!
//! Spec: `mesh/MESH_V1_EXAMPLES_SCOPE.md` §5.8. Fixture: 30 mm cube
//! centered at origin (`min = (-15, -15, -15)`, `max = (15, 15, 15)`)
//! at cell size 10 mm and resolution 15, with sphere SDF of radius
//! 12 mm at origin (`Arc::new(|p| p.coords.norm() - 12.0)`) clipping
//! the gyroid output. The bbox is an integer multiple of the cell, so
//! `total_resolution = ceil((30 / 10) × 15) = 45` and the
//! marching-cubes voxel size is exactly `30 / 45 = 2/3 ≈ 0.667 mm`
//! (per `mesh-lattice/src/generate.rs:445-448`). Every output vertex
//! of the SDF-trimmed lattice satisfies
//! `(v - origin).norm() < 12 + 2/3 + 1.0 ≈ 13.667`, where the cushion
//! 1.0 mm absorbs marching-cubes interpolation across cells straddling
//! the sphere boundary.
//!
//! Boundary-conforming counterpart to §5.5 `mesh-lattice-tpms-gyroid`
//! (same gyroid TPMS path, but the lattice is trimmed to a
//! mathematical shape rather than filling the full bbox);
//! complementary to §5.9 `mesh-lattice-mesh-bounded-infill` (the
//! mesh-bounded composite path via `generate_infill`).

use std::path::Path;
use std::sync::Arc;

use anyhow::Result;
use approx::assert_relative_eq;
use mesh_io::save_ply;
use mesh_lattice::{LatticeParams, LatticeResult, generate_lattice};
use mesh_types::Point3;

// =============================================================================
// Fixture constants
// =============================================================================

/// 30 mm³ cube centered at origin: `min = (-15, -15, -15)` /
/// `max = (15, 15, 15)`.
const BBOX_MIN: f64 = -15.0;
const BBOX_MAX: f64 = 15.0;

/// Unit cell edge length in mm. `30 / 10 = 3` cells per axis.
const CELL_SIZE: f64 = 10.0;

/// Samples per cell along each axis.
/// `total_resolution = ceil((30 / 10) × 15) = 45` ⇒ MC voxel size
/// `30 / 45 = 2/3 ≈ 0.667 mm` exactly.
const RESOLUTION: usize = 15;

/// Density target for the gyroid wall fraction. Mirrors the in-tree
/// `test_gyroid_lattice_with_shape_sdf` precedent at
/// `mesh-lattice/src/generate.rs:782`.
const DENSITY: f64 = 0.3;

/// Sphere SDF radius in mm. Convention per `params.rs:415-417`:
/// `is_outside_shape(p) == sdf(p) > 0.0`, where
/// `sdf(p) = norm(p) - radius`.
const SPHERE_RADIUS: f64 = 12.0;

/// Edge-case sphere radius (1 mm) — proves the SDF clip is real (the
/// trimmed result has sharply fewer vertices than the radius-12 trim).
const TINY_RADIUS: f64 = 1.0;

/// `bbox_size / cell_size = 30 / 10 = 3`; cells-per-axis cubed.
const EXPECTED_CELL_COUNT: usize = 27;

/// MC voxel size: `cell_size / resolution = 10 / 15 = 2/3`.
const VOXEL_SIZE: f64 = 2.0 / 3.0;

/// Per-vertex distance bound cushion in mm — absorbs marching-cubes
/// interpolation wobble across boundary-straddling cells.
const DISTANCE_CUSHION: f64 = 1.0;

/// Per-vertex distance-to-origin bound:
/// `sphere_radius + voxel_size + cushion`.
const DISTANCE_BOUND: f64 = SPHERE_RADIUS + VOXEL_SIZE + DISTANCE_CUSHION;

/// Tight tolerance for FP-exact assertions on integer-spatial inputs
/// (`cell_size`, density, etc.).
const TIGHT_TOL: f64 = 1e-12;

/// Toggle for the un-trimmed comparison artifact at
/// `out/sphere_gyroid_full.ply` (in addition to the trimmed
/// `out/sphere_gyroid.ply`). Per spec §5.8 line 751: "for reviewer
/// clarity."
const WRITE_COMPARISON: bool = true;

// =============================================================================
// sphere_sdf — analytical shape closure factory
// =============================================================================

/// Builds an `Arc<dyn Fn(Point3<f64>) -> f64 + Send + Sync>` shape SDF
/// for a sphere of `radius` centered at origin:
/// `sdf(p) = norm(p) - radius`. Convention: `> 0` ⇒ outside,
/// `< 0` ⇒ inside, `== 0` ⇒ on surface (per
/// `mesh-lattice/src/params.rs:415-417`).
fn sphere_sdf(radius: f64) -> Arc<dyn Fn(Point3<f64>) -> f64 + Send + Sync> {
    Arc::new(move |p| p.coords.norm() - radius)
}

// =============================================================================
// verify_is_outside_shape_predicate — 4 spatial probes + no-SDF default
// =============================================================================

/// `is_outside_shape(p) == sdf(p) > 0.0` per `params.rs:415-417`.
/// Probes the convention at four known points against the radius-12
/// sphere SDF (origin / `r=11` interior / `r=13` exterior / `r=20` far
/// exterior), then verifies the no-SDF default returns `false` for any
/// point (the underlying `is_some_and` short-circuits to `false` when
/// `shape_sdf` is `None`).
fn verify_is_outside_shape_predicate(
    params_with_sdf: &LatticeParams,
    params_without_sdf: &LatticeParams,
) {
    // (1) origin: sdf = -12 ⇒ inside ⇒ false.
    assert!(
        !params_with_sdf.is_outside_shape(Point3::new(0.0, 0.0, 0.0)),
        "origin (sdf = -12) must read as inside the radius-12 sphere",
    );

    // (2) (11, 0, 0): sdf = -1 ⇒ interior ⇒ false.
    assert!(
        !params_with_sdf.is_outside_shape(Point3::new(11.0, 0.0, 0.0)),
        "(11, 0, 0) (sdf = -1) must read as inside the radius-12 sphere",
    );

    // (3) (13, 0, 0): sdf = +1 ⇒ exterior ⇒ true.
    assert!(
        params_with_sdf.is_outside_shape(Point3::new(13.0, 0.0, 0.0)),
        "(13, 0, 0) (sdf = +1) must read as outside the radius-12 sphere",
    );

    // (4) (20, 0, 0): sdf = +8 ⇒ far exterior ⇒ true.
    assert!(
        params_with_sdf.is_outside_shape(Point3::new(20.0, 0.0, 0.0)),
        "(20, 0, 0) (sdf = +8) must read as outside the radius-12 sphere",
    );

    // (5) No-SDF default: `is_some_and` returns false when None ⇒
    // every point is "inside" by default.
    assert!(
        !params_without_sdf.is_outside_shape(Point3::new(0.0, 0.0, 0.0)),
        "no shape_sdf set ⇒ origin must read as inside (default false)",
    );
    assert!(
        !params_without_sdf.is_outside_shape(Point3::new(100.0, 100.0, 100.0)),
        "no shape_sdf set ⇒ far-exterior point must read as inside (default false)",
    );
}

// =============================================================================
// build_params* — gyroid lattice configurations (with / without / tiny)
// =============================================================================

/// With-SDF params: `LatticeParams::gyroid(10.0)` + density 0.3 +
/// resolution 15 + sphere-radius-12 shape SDF closure.
fn build_params_with_sdf() -> LatticeParams {
    LatticeParams::gyroid(CELL_SIZE)
        .with_density(DENSITY)
        .with_resolution(RESOLUTION)
        .with_shape_sdf(sphere_sdf(SPHERE_RADIUS))
}

/// Without-SDF params: same as `build_params_with_sdf` minus
/// `with_shape_sdf` ⇒ bbox-filling baseline for the with-vs-without
/// trimmed-vertex-count comparison and the no-SDF-default
/// `is_outside_shape` anchor.
fn build_params_without_sdf() -> LatticeParams {
    LatticeParams::gyroid(CELL_SIZE)
        .with_density(DENSITY)
        .with_resolution(RESOLUTION)
}

/// Tiny-sphere params: same density / resolution / `cell_size`, but
/// sphere SDF of radius 1 mm — produces a sharply smaller trimmed
/// lattice than `build_params_with_sdf` (or empty, depending on where
/// the gyroid wall happens to fall inside the unit-radius sphere).
fn build_params_tiny_sphere() -> LatticeParams {
    LatticeParams::gyroid(CELL_SIZE)
        .with_density(DENSITY)
        .with_resolution(RESOLUTION)
        .with_shape_sdf(sphere_sdf(TINY_RADIUS))
}

// =============================================================================
// verify_builder_chain — validate() + shape_sdf.is_some() + field locks
// =============================================================================

/// `LatticeParams::gyroid(10.0).with_density(0.3).with_resolution(15).with_shape_sdf(_)`
/// validates clean and the `shape_sdf` field is populated. Field locks
/// pin the builder-chain effects so a future regression in any of the
/// builders surfaces as an anchor failure here, not silently
/// downstream.
fn verify_builder_chain(params: &LatticeParams) {
    if params.validate().is_err() {
        unreachable!("build_params_with_sdf yields a valid LatticeParams");
    }
    assert!(
        params.shape_sdf.is_some(),
        "with_shape_sdf(_) must populate shape_sdf",
    );
    assert_relative_eq!(params.cell_size, CELL_SIZE, epsilon = TIGHT_TOL);
    assert_relative_eq!(params.density, DENSITY, epsilon = TIGHT_TOL);
    assert_eq!(params.resolution, RESOLUTION);
}

// =============================================================================
// verify_with_vs_without_trim — strict-inequality on vertex_count()
// =============================================================================

/// The SDF-clipped lattice has STRICTLY FEWER vertices than the
/// bbox-filling counterpart (same density, resolution, `cell_size`, and
/// bounds; only `with_shape_sdf` differs). Mirrors the in-tree
/// `test_gyroid_lattice_with_shape_sdf` at `generate.rs:782` (which
/// uses a smaller fixture but the same anchor structure).
fn verify_with_vs_without_trim(with_sdf: &LatticeResult, without_sdf: &LatticeResult) {
    assert!(
        with_sdf.vertex_count() < without_sdf.vertex_count(),
        "SDF-trimmed lattice ({}) must have strictly fewer vertices than \
         bbox-filling baseline ({})",
        with_sdf.vertex_count(),
        without_sdf.vertex_count(),
    );
}

// =============================================================================
// verify_per_vertex_distance_bound — every vertex within 13.667 mm
// =============================================================================

/// Every vertex `v` of the SDF-trimmed lattice satisfies
/// `(v - origin).norm() < sphere_radius + voxel_size + cushion ≈ 13.667`.
/// The `voxel_size` term accounts for marching-cubes producing an edge
/// vertex up to one voxel beyond the sphere boundary (the vertex sits
/// on an edge between an inside corner and an outside corner of a cell
/// straddling the surface); the `cushion` absorbs FP wobble at the
/// linear interpolation. Returns the empirical max distance for the
/// summary readout.
fn verify_per_vertex_distance_bound(result: &LatticeResult) -> f64 {
    let origin = Point3::origin();
    let mut max_dist = 0.0_f64;
    for (i, v) in result.mesh.vertices.iter().enumerate() {
        let dist = (v - origin).norm();
        assert!(
            dist < DISTANCE_BOUND,
            "vertex {i} at ({:.4}, {:.4}, {:.4}) is {dist:.4} mm from origin, \
             must be < {DISTANCE_BOUND:.4} mm (sphere_radius {SPHERE_RADIUS} + \
             voxel_size {VOXEL_SIZE:.4} + cushion {DISTANCE_CUSHION})",
            v.x,
            v.y,
            v.z,
        );
        if dist > max_dist {
            max_dist = dist;
        }
    }
    max_dist
}

// =============================================================================
// verify_cell_count_reports_total_bbox — F9-related fidelity claim
// =============================================================================

/// `result.cell_count` reports the TOTAL bbox cell count (NOT the
/// trimmed cell count): `(30 / 10)³ = 27`. The TPMS path's
/// `generate_tpms_lattice` computes
/// `cell_count = cells_x × cells_y × cells_z` pre-trim at
/// `generate.rs:417`, so SDF clipping cannot reduce this number. Spec
/// §5.8 calls this "F9-related" and mandates README documentation;
/// this anchor pins the behavior bit-exactly so any future change
/// surfaces here.
fn verify_cell_count_reports_total_bbox(result: &LatticeResult) {
    assert_eq!(
        result.cell_count, EXPECTED_CELL_COUNT,
        "cell_count must report TOTAL bbox cells (3³ = 27), NOT trimmed cells",
    );
}

// =============================================================================
// verify_tiny_sphere_edge_case — proves SDF clip is real
// =============================================================================

/// A sphere SDF of radius 1.0 mm clips drastically more lattice
/// geometry than the radius-12 fixture — the trimmed result has at
/// most one tenth the vertex count of the radius-12 trim.
/// Demonstrates the SDF clip is real, not a no-op.
fn verify_tiny_sphere_edge_case(tiny: &LatticeResult, with_sdf: &LatticeResult) {
    assert!(
        tiny.vertex_count() * 10 < with_sdf.vertex_count(),
        "tiny-sphere (radius 1) trim ({}) must be ≤ 1/10 of the radius-12 trim \
         ({}); demonstrates that SDF clipping is real",
        tiny.vertex_count(),
        with_sdf.vertex_count(),
    );
}

// =============================================================================
// print_summary — museum-plaque stdout
// =============================================================================

/// Bundled inputs for [`print_summary`]; avoids
/// `clippy::too_many_arguments` while keeping fields trivially
/// constructed at the call site (precedent: §5.4 / §5.6 / §5.7
/// `Summary` extraction).
struct Summary<'a> {
    with_sdf: &'a LatticeResult,
    without_sdf: &'a LatticeResult,
    tiny: &'a LatticeResult,
    max_dist: f64,
}

/// Print the human-readable summary. Extracted from `main` to keep
/// the entrypoint under clippy's `too_many_lines` cap.
#[allow(clippy::cast_precision_loss)] // vertex counts ≤ ~3 × 10⁵, well within f64 mantissa
fn print_summary(s: &Summary) {
    println!("==== mesh-lattice-shape-bounded ====");
    println!();
    println!(
        "fixture: 30 mm³ bbox centered at origin (-15..15), cell_size = {CELL_SIZE} mm, \
         resolution = {RESOLUTION}",
    );
    println!("         density = {DENSITY}, sphere SDF radius = {SPHERE_RADIUS} mm at origin");
    println!(
        "         MC voxel size = cell_size / resolution = {CELL_SIZE} / {RESOLUTION} = \
         {VOXEL_SIZE:.6} mm",
    );
    println!();
    println!("is_outside_shape(p) == sdf(p) > 0.0 (params.rs:415-417):");
    println!("  (0,  0, 0)   sdf = -12  ⇒ inside  (false)");
    println!("  (11, 0, 0)   sdf =  -1  ⇒ inside  (false)");
    println!("  (13, 0, 0)   sdf =  +1  ⇒ outside (true)");
    println!("  (20, 0, 0)   sdf =  +8  ⇒ outside (true)");
    println!("  no SDF set ⇒ is_outside_shape(any) == false (default)");
    println!();
    println!("LatticeParams::gyroid(10) builder chain:");
    println!("  validate()             = Ok(())");
    println!("  shape_sdf.is_some()    = true");
    println!();
    println!(
        "With-vs-without trim (generate_lattice over identical bbox + density + \
         resolution):",
    );
    println!(
        "  result_with_sdf.vertex_count()    = {} (trimmed)",
        s.with_sdf.vertex_count(),
    );
    println!(
        "  result_without_sdf.vertex_count() = {} (bbox-filling baseline)",
        s.without_sdf.vertex_count(),
    );
    let dropped = s.without_sdf.vertex_count() - s.with_sdf.vertex_count();
    let pct = 100.0 * dropped as f64 / s.without_sdf.vertex_count() as f64;
    println!("  strict <: trim drops {dropped} vertices ({pct:.1}%)");
    println!();
    println!("Per-vertex distance-to-origin bound:");
    println!(
        "  max norm(v) over with-SDF result = {:.4} mm  <  {DISTANCE_BOUND:.4} mm",
        s.max_dist,
    );
    println!(
        "                                    (= sphere_radius {SPHERE_RADIUS} + voxel_size \
         {VOXEL_SIZE:.4} + cushion {DISTANCE_CUSHION})",
    );
    println!();
    println!("cell_count fidelity (F9-related):");
    println!(
        "  result.cell_count = {} (BIT-EXACT; reports TOTAL bbox cells, not trimmed)",
        s.with_sdf.cell_count,
    );
    println!();
    println!("Edge case — tiny sphere (radius 1 mm):");
    println!(
        "  tiny.vertex_count() = {} (≤ {}/10 = {})",
        s.tiny.vertex_count(),
        s.with_sdf.vertex_count(),
        s.with_sdf.vertex_count() / 10,
    );
    println!();
}

// =============================================================================
// main
// =============================================================================

fn main() -> Result<()> {
    let params_with = build_params_with_sdf();
    let params_without = build_params_without_sdf();
    let params_tiny = build_params_tiny_sphere();

    verify_is_outside_shape_predicate(&params_with, &params_without);
    verify_builder_chain(&params_with);

    let bounds = (
        Point3::new(BBOX_MIN, BBOX_MIN, BBOX_MIN),
        Point3::new(BBOX_MAX, BBOX_MAX, BBOX_MAX),
    );

    let Ok(result_with) = generate_lattice(&params_with, bounds) else {
        unreachable!("validated with-SDF params + valid bounds: generate_lattice cannot fail",);
    };
    let Ok(result_without) = generate_lattice(&params_without, bounds) else {
        unreachable!("validated without-SDF params + valid bounds: generate_lattice cannot fail",);
    };
    let Ok(result_tiny) = generate_lattice(&params_tiny, bounds) else {
        unreachable!("validated tiny-sphere params + valid bounds: generate_lattice cannot fail",);
    };

    verify_with_vs_without_trim(&result_with, &result_without);
    let max_dist = verify_per_vertex_distance_bound(&result_with);
    verify_cell_count_reports_total_bbox(&result_with);
    verify_tiny_sphere_edge_case(&result_tiny, &result_with);

    print_summary(&Summary {
        with_sdf: &result_with,
        without_sdf: &result_without,
        tiny: &result_tiny,
        max_dist,
    });

    let out_dir = Path::new(env!("CARGO_MANIFEST_DIR")).join("out");
    std::fs::create_dir_all(&out_dir)?;

    let trimmed_path = out_dir.join("sphere_gyroid.ply");
    save_ply(&result_with.mesh, &trimmed_path, true)?;
    println!(
        "artifact: out/sphere_gyroid.ply ({}v, {}f, binary little-endian)",
        result_with.mesh.vertices.len(),
        result_with.mesh.faces.len(),
    );

    if WRITE_COMPARISON {
        let full_path = out_dir.join("sphere_gyroid_full.ply");
        save_ply(&result_without.mesh, &full_path, true)?;
        println!(
            "artifact: out/sphere_gyroid_full.ply ({}v, {}f, binary little-endian — \
             un-trimmed comparison)",
            result_without.mesh.vertices.len(),
            result_without.mesh.faces.len(),
        );
    }

    println!();
    println!(
        "OK — 4 is_outside_shape spatial probes + 2 no-SDF default probes + builder \
         validate + shape_sdf.is_some + with-vs-without strict-< vertex_count + \
         per-vertex distance bound (max < {DISTANCE_BOUND:.4}) + cell_count == 27 + \
         tiny-sphere ≤ 1/10 anchor all green",
    );

    Ok(())
}
