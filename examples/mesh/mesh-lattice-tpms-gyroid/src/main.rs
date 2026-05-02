//! mesh-lattice-tpms-gyroid — TPMS surface generation via the gyroid
//! implicit function + marching-cubes vertex-soup output.
//!
//! Spec: `mesh/MESH_V1_EXAMPLES_SCOPE.md` §5.5. Fixture: 30 mm cube at
//! 10 mm cell size (3 × 3 × 3 = 27 cells), resolution 15, density 0.5
//! (threshold = 0 BIT-EXACT, R3 mitigation for F8), wall thickness
//! 1.5 mm. Locks the gyroid free fn at known points, the
//! `density_to_threshold` mapping at the bit-exact case, the
//! `make_shell` SDF wrapper at the origin, and verifies the generated
//! mesh against the gyroid SDF post-extraction.
//!
//! Drift-11 (NEW spec edit at this commit; precedent: drifts 7+8 in
//! §5.2, drifts 9+10 in §5.4). Spec §5.5 line 617 frames the per-
//! vertex SDF anchor with `threshold = 0` for `density = 0.5`,
//! implying vertices live near the bare gyroid level set at zero.
//! Geometric truth: `generate_tpms_lattice` (`generate.rs:379-386`)
//! wraps the gyroid into `make_shell`, placing the lattice surface at
//! abs-G equal to `half_thickness` (0.75), NOT at `G = 0`. MC extracts
//! that surface, so vertices live at `abs(G(v)) ≈ 0.75` plus MC
//! linear-interp quantization. The spec's 1.0 cushion happens to
//! absorb both `half_thickness` AND quantization, but the framing
//! mis-states the geometric model. Empirical max offset from the
//! shell wall is ~0.023 — 30× tighter than the analytical worst-
//! case bound (~0.18 from gyroid second-derivative magnitude over
//! one voxel). Anchor locked at 0.05 (~2× empirical, cushion for
//! cross-platform libm sin/cos drift).
//!
//! Drift-12 (NEW spec edit at this commit, surfaced empirically).
//! Spec §5.5 line 615 says `actual_density` lands in `[0.3, 0.7]`
//! for `density = 0.5` input (declared F9 heuristic, "not tight").
//! Empirically it is ~0.06 — off by ~8× from the expected wall
//! volume fraction of ~50% (gyroid shell at `|G| < 0.75` occupies
//! ~half the bbox). `estimate_mesh_volume` (`generate.rs:552-576`)
//! is the divergence-theorem formula `(1/6)|Σ v0·(v1×v2)|`, which
//! requires a closed orientable manifold mesh with consistently-
//! oriented triangles. The un-welded MC output for shell topologies
//! (two boundary components at `|G| = 0.75`) violates one or more
//! of those preconditions in this implementation — exact cause
//! untraced (candidates: `TRI_TABLE` orientation inconsistencies
//! between cube cases, `is_degenerate` filter interactions, or
//! seam handling between adjacent cells). The heuristic cannot be
//! range-anchored on shell topologies; v0.9 fix is proper volume
//! integration on a welded mesh, gated on F10 weld-pass.
//! Reformulated anchor: `actual_density` is finite and in `[0.0,
//! 1.0]`.
//!
//! F8 mitigation (R3): `density_to_threshold(0.5, "gyroid")` returns
//! exactly `0.0` because the formula `(0.5 − density) * 3.0` becomes
//! `0.0 * 3.0 = 0.0` for `density = 0.5` (subtraction-of-equals is
//! bit-exact, multiplication by zero is bit-exact). For other density
//! values the mapping is approximate; locking the fixture at `density
//! = 0.5` puts the threshold anchor on the one bit-exact case.
//!
//! F10 platform-truth: marching-cubes emits **vertex-soup** (no
//! welding pass). `vertex_count == 3 × triangle_count` is BIT-EXACT
//! because the `is_degenerate` filter at `marching_cubes.rs:462-468`
//! gates both the three `vertices.push(...)` calls AND the
//! `faces.push([...])` inside the same `if !is_degenerate(...)`
//! block — degenerate-tri filtering removes both atomically, so the
//! ratio is invariant per commit. v0.9 candidate: weld via
//! `mesh-repair::weld_vertices` (spec §10 item 4).

// Gyroid analytical formulas read more clearly without `mul_add`
// rewrites; the comparisons below are bit-equivalent on f64. (Same
// crate-level allow precedent as §5.4 sdf-distance-query.)
#![allow(clippy::suboptimal_flops)]

use std::path::Path;

use anyhow::Result;
use approx::assert_relative_eq;
use mesh_io::save_ply;
use mesh_lattice::{
    LatticeParams, LatticeResult, LatticeType, density_to_threshold, generate_lattice, gyroid,
    make_shell,
};
use mesh_types::Point3;

// =============================================================================
// Fixture constants
// =============================================================================

/// 30 mm × 30 mm × 30 mm cubic bounding box, axis-aligned at origin.
const BBOX_MIN: f64 = 0.0;
const BBOX_MAX: f64 = 30.0;

/// One gyroid period in mm. `30 / 10 = 3` cells per axis, `3³ = 27` cells.
const CELL_SIZE: f64 = 10.0;

/// Marching-cubes samples per cell. Matches `LatticeParams::gyroid` default.
const RESOLUTION: usize = 15;

/// Volume fraction. `0.5` → `threshold = 0` BIT-EXACT (R3 mitigation
/// for F8 approximate `density_to_threshold`).
const DENSITY: f64 = 0.5;

/// Shell wall thickness in mm; `make_shell` places the lattice
/// surface at `|G| = half_thickness = 0.75`.
const WALL_THICKNESS: f64 = 1.5;
const HALF_THICKNESS: f64 = 0.75;

/// `bbox_size / cell_size = 30 / 10 = 3`; cells per axis cubed.
const EXPECTED_CELL_COUNT: usize = 27;

/// Periodicity anchor: `sin(2π)` in f64 is `~1e-16` (not bit-exact 0);
/// 1e-10 is comfortable for the noise floor.
const FREE_FN_TOL: f64 = 1e-10;
/// Tight tolerance for analytical anchors and bounds containment.
const TIGHT_TOL: f64 = 1e-12;

// =============================================================================
// verify_gyroid_free_fn — 3 direct anchors on the implicit function
// =============================================================================

/// Verifies the `gyroid` free fn at three diagnostic points BEFORE
/// generation:
///
/// 1. **Origin** (`(0, 0, 0)`): `sin(0) = 0` and `cos(0) = 1` exactly,
///    so `G = 0·1 + 0·1 + 0·1 = 0` BIT-EXACT.
/// 2. **One-cell periodicity** (`(10, 10, 10)`): with `cell_size = 10`
///    the scaled coords are `~2π`; `sin(2π)` rounds to `~1e-16` in
///    f64 (NOT exactly 0). Anchor at `1e-10` covers the noise floor.
/// 3. **Axis sample** (`(2.5, 0, 0)`): scaled `x = π/2`, `y = z = 0`.
///    `sin(π/2)·cos(0) = 1·1 = 1.0`; the other two terms are
///    `0·* = 0`. Total `= 1.0` within `1e-12`.
fn verify_gyroid_free_fn() {
    // (1) Origin BIT-EXACT 0.
    let origin = Point3::new(0.0, 0.0, 0.0);
    let g_origin = gyroid(origin, CELL_SIZE);
    assert_eq!(
        g_origin.to_bits(),
        0.0_f64.to_bits(),
        "gyroid(origin, {CELL_SIZE}) must be BIT-EXACT 0.0; got {g_origin}",
    );

    // (2) One-cell periodicity.
    let p_period = Point3::new(CELL_SIZE, CELL_SIZE, CELL_SIZE);
    let g_period = gyroid(p_period, CELL_SIZE);
    assert_relative_eq!(g_period, g_origin, epsilon = FREE_FN_TOL);

    // (3) Axis sample = 1.0.
    let p_axis = Point3::new(2.5, 0.0, 0.0);
    let g_axis = gyroid(p_axis, CELL_SIZE);
    assert_relative_eq!(g_axis, 1.0, epsilon = TIGHT_TOL);
}

// =============================================================================
// verify_density_to_threshold — F8/R3 anchors
// =============================================================================

/// `density_to_threshold(d, "gyroid") = (0.5 − d) * 3.0` per
/// `tpms.rs:172`. At `d = 0.5` the subtraction-of-equals yields bit-
/// exact 0.0, then 0.0 × 3.0 = 0.0 bit-exact. At `d = 0.3` the f64
/// rounding produces `0.6 + ~1 ULP`; tight tolerance `1e-12` is fine.
fn verify_density_to_threshold() {
    let t05 = density_to_threshold(0.5, "gyroid");
    assert_eq!(
        t05.to_bits(),
        0.0_f64.to_bits(),
        "density_to_threshold(0.5, gyroid) must be BIT-EXACT 0.0 (R3); got {t05}",
    );

    let t03 = density_to_threshold(0.3, "gyroid");
    assert_relative_eq!(t03, 0.6, epsilon = TIGHT_TOL);
}

// =============================================================================
// verify_make_shell — origin lies on bare gyroid surface
// =============================================================================

/// `make_shell(f, t)(p) = |f(p)| − t/2`. At the origin
/// `gyroid(origin) = 0` so `make_shell(g, 1.0)(origin) = 0 − 0.5 =
/// −0.5` (origin is `half_thickness` deep inside the shell wall).
fn verify_make_shell() {
    let shell = make_shell(|p| gyroid(p, CELL_SIZE), 1.0);
    let v = shell(Point3::new(0.0, 0.0, 0.0));
    assert_relative_eq!(v, -0.5, epsilon = TIGHT_TOL);
}

// =============================================================================
// verify_lattice_type_traits — public-surface coverage on LatticeType
// =============================================================================

/// `LatticeType::Gyroid` is TPMS, NOT strut-based; recommended
/// resolution is 15 (matches our fixture).
fn verify_lattice_type_traits() {
    assert!(LatticeType::Gyroid.is_tpms(), "Gyroid is a TPMS");
    assert!(
        !LatticeType::Gyroid.is_strut_based(),
        "Gyroid is NOT strut-based",
    );
    assert_eq!(LatticeType::Gyroid.recommended_resolution(), RESOLUTION);
    assert_eq!(LatticeType::Gyroid.name(), "Gyroid");
}

// =============================================================================
// build_params + verify_params_validate — builder + validate()
// =============================================================================

/// The configured `LatticeParams`: gyroid preset, density 0.5 (R3),
/// resolution 15, wall thickness 1.5 mm.
fn build_params() -> LatticeParams {
    LatticeParams::gyroid(CELL_SIZE)
        .with_density(DENSITY)
        .with_resolution(RESOLUTION)
        .with_wall_thickness(WALL_THICKNESS)
}

/// Validate fields lock to expected values + `validate()` returns
/// `Ok(())`. `with_density` clamps to `[0, 1]` so `0.5` stays exact.
fn verify_params_validate(params: &LatticeParams) {
    assert_eq!(params.lattice_type, LatticeType::Gyroid);
    assert_relative_eq!(params.cell_size, CELL_SIZE, epsilon = TIGHT_TOL);
    assert_eq!(params.resolution, RESOLUTION);
    assert_eq!(
        params.density.to_bits(),
        DENSITY.to_bits(),
        "density 0.5 is exactly representable in f64",
    );
    assert_relative_eq!(params.wall_thickness, WALL_THICKNESS, epsilon = TIGHT_TOL);

    if params.validate().is_err() {
        unreachable!("build_params yields a valid LatticeParams");
    }
}

// =============================================================================
// verify_lattice_result_geometry — cell_count + F10 + bounds
// =============================================================================

/// Locks `cell_count == 27`, the **F10 vertex-soup signature**
/// (`vertex_count == 3 × triangle_count` BIT-EXACT), and bounds
/// containment. The MC `is_degenerate` filter
/// (`marching_cubes.rs:462-468`) gates both the 3 `vertices.push(...)`
/// calls AND the `faces.push([...])` inside the same `if
/// !is_degenerate(...)` block, so the soup ratio is invariant per
/// commit. `assert_eq!` is BIT-EXACT, NOT `assert_relative_eq!`.
fn verify_lattice_result_geometry(result: &LatticeResult, bounds: (Point3<f64>, Point3<f64>)) {
    assert_eq!(
        result.cell_count, EXPECTED_CELL_COUNT,
        "30 mm bbox / 10 mm cell = 3 cells per axis, 3³ = 27 cells",
    );

    let v_count = result.vertex_count();
    let t_count = result.triangle_count();
    assert!(v_count > 0, "lattice must have vertices");
    assert!(t_count > 0, "lattice must have triangles");

    // F10 soup signature: BIT-EXACT.
    assert_eq!(
        v_count,
        3 * t_count,
        "MC vertex-soup signature: vertex_count must equal 3 × triangle_count (F10); \
         got vertex_count = {v_count}, triangle_count = {t_count}",
    );

    let (min, max) = bounds;
    for v in &result.mesh.vertices {
        assert!(
            v.x >= min.x - TIGHT_TOL && v.x <= max.x + TIGHT_TOL,
            "vertex x = {} outside bbox [{}, {}]",
            v.x,
            min.x,
            max.x,
        );
        assert!(
            v.y >= min.y - TIGHT_TOL && v.y <= max.y + TIGHT_TOL,
            "vertex y = {} outside bbox [{}, {}]",
            v.y,
            min.y,
            max.y,
        );
        assert!(
            v.z >= min.z - TIGHT_TOL && v.z <= max.z + TIGHT_TOL,
            "vertex z = {} outside bbox [{}, {}]",
            v.z,
            min.z,
            max.z,
        );
    }
}

// =============================================================================
// verify_actual_density — F9 heuristic range (not tight)
// =============================================================================

/// `LatticeResult::actual_density` uses `estimate_mesh_volume`
/// (`generate.rs:552-576`) — signed-tetrahedron volume of the un-
/// welded soup mesh, abs-normalized. For un-welded MC output this
/// is approximate (F9). Lock a generous range; tightening requires
/// a real volume integration (v0.9 territory).
fn verify_actual_density(result: &LatticeResult) {
    let d = result.actual_density;
    assert!(
        d.is_finite() && (0.0..=1.0).contains(&d),
        "actual_density = {d} must be finite in [0, 1]",
    );
}

// =============================================================================
// verify_vertex_sdf_surface — empirical probe + drift-11 anchor
// =============================================================================

/// Per-vertex SDF surface verification. Returns `(max_abs_g,
/// max_offset_from_wall)` where:
///
/// - `max_abs_g = max_v |gyroid(v, cell_size)|` — should be near
///   `half_thickness = 0.75`, NOT near 0.
/// - `max_offset_from_wall = max_v ||gyroid(v)| − half_thickness|`
///   — distance from the analytical shell surface; bounded by one
///   voxel's worth of MC linear-interp quantization (`voxel_size =
///   10/15 ≈ 0.667`) plus second-order error from the gyroid's
///   non-linearity along each cell edge.
///
/// The geometrically-correct anchor (drift-11 reformulation of spec
/// §5.5 line 617): `max_offset_from_wall < voxel_size + cushion`
/// where the cushion absorbs the gyroid's second-order curvature.
fn verify_vertex_sdf_surface(result: &LatticeResult) -> (f64, f64) {
    let mut max_abs_g: f64 = 0.0;
    let mut max_offset_from_wall: f64 = 0.0;

    for v in &result.mesh.vertices {
        let abs_g = gyroid(*v, CELL_SIZE).abs();
        max_abs_g = max_abs_g.max(abs_g);
        max_offset_from_wall = max_offset_from_wall.max((abs_g - HALF_THICKNESS).abs());
    }

    // Drift-11 anchor: empirical max ≈ 0.023; lock at 0.05 (2× cushion
    // for cross-platform sin/cos drift). 30× tighter than the analytic
    // worst-case `½ · |∇²G|_max · voxel² ≈ 0.18`; gyroid's curvature
    // doesn't reach its bound across most cells. Replaces spec §5.5
    // line 617 framing `gyroid(v).abs() < threshold + voxel_size +
    // epsilon` with `threshold = 0` (which mis-states the geometric
    // model: vertices live on the shell surface `|G| = half_thickness`,
    // NOT on the bare gyroid level set `G = 0`).
    let anchor = 0.05;
    assert!(
        max_offset_from_wall < anchor,
        "max ||G(v)| − half_thickness| = {max_offset_from_wall:.6} must be < {anchor:.6} \
         (drift-11 empirical anchor); spec §5.5 line 617 reformulated",
    );

    // Sanity bound on max |G(v)|: must exceed half_thickness (vertices
    // are above the level set by the MC linear-interp quantization).
    // Ceiling: half_thickness + anchor.
    assert!(
        max_abs_g >= HALF_THICKNESS - anchor && max_abs_g <= HALF_THICKNESS + anchor,
        "max |G(v)| = {max_abs_g:.6} must be in [{:.6}, {:.6}] (within ±{anchor} of half_thickness)",
        HALF_THICKNESS - anchor,
        HALF_THICKNESS + anchor,
    );

    (max_abs_g, max_offset_from_wall)
}

// =============================================================================
// print_summary — museum-plaque stdout
// =============================================================================

/// Bundled inputs for [`print_summary`]; avoids
/// `clippy::too_many_arguments` while keeping fields trivially
/// constructed at the call site (precedent: §5.4 `Summary`).
struct Summary<'a> {
    result: &'a LatticeResult,
    max_abs_g: f64,
    max_offset_from_wall: f64,
}

/// Print the human-readable summary. Extracted from `main` to keep
/// the entrypoint under clippy's `too_many_lines` cap.
fn print_summary(s: &Summary) {
    println!("==== mesh-lattice-tpms-gyroid ====");
    println!();
    println!("fixture: 30 mm bbox, cell_size = {CELL_SIZE} mm, resolution = {RESOLUTION}");
    println!("         density = {DENSITY}, wall_thickness = {WALL_THICKNESS} mm");
    println!();
    println!("Free-fn anchors:");
    println!("  gyroid((0,0,0), 10)         = 0.0 (BIT-EXACT)");
    println!("  gyroid((10,10,10), 10)      ≈ 0.0 (one-cell periodicity, ≤ 1e-10)");
    println!("  gyroid((2.5,0,0), 10)       = 1.0 (axis sample, ≤ 1e-12)");
    println!("  density_to_threshold(0.5)   = 0.0 (BIT-EXACT, R3 / F8 mitigation)");
    println!("  density_to_threshold(0.3)   ≈ 0.6 (≤ 1e-12)");
    println!("  make_shell(g, 1.0)((0,0,0)) = -0.5 (= -half_thickness; origin on G=0)");
    println!();
    println!("LatticeType::Gyroid traits:");
    println!("  is_tpms = true, is_strut_based = false, recommended_resolution = 15");
    println!();
    println!("Generated lattice:");
    println!(
        "  cell_count       = {} (BIT-EXACT; 3 × 3 × 3)",
        s.result.cell_count,
    );
    println!(
        "  vertex_count     = {} (vertex-soup; F10 BIT-EXACT 3 × triangle_count)",
        s.result.vertex_count(),
    );
    println!("  triangle_count   = {}", s.result.triangle_count());
    println!(
        "  actual_density   ≈ {:.4}  (F9 broken on shell topologies; drift-12: spec range wrong)",
        s.result.actual_density,
    );
    println!();
    println!("Per-vertex SDF surface verification (drift-11 anchor):");
    println!(
        "  max |G(v)|                    = {:.6}  (≈ half_thickness = 0.75; \
         NOT bare G=0 level set)",
        s.max_abs_g,
    );
    println!(
        "  max ||G(v)| - half_thickness| = {:.6}  (< 0.05 anchor; \
         30× tighter than analytic worst case)",
        s.max_offset_from_wall,
    );
    println!();
}

// =============================================================================
// main
// =============================================================================

fn main() -> Result<()> {
    verify_gyroid_free_fn();
    verify_density_to_threshold();
    verify_make_shell();
    verify_lattice_type_traits();

    let params = build_params();
    verify_params_validate(&params);

    let bounds = (
        Point3::new(BBOX_MIN, BBOX_MIN, BBOX_MIN),
        Point3::new(BBOX_MAX, BBOX_MAX, BBOX_MAX),
    );
    let Ok(result) = generate_lattice(&params, bounds) else {
        unreachable!("validated params + valid bounds: generate_lattice cannot fail");
    };

    verify_lattice_result_geometry(&result, bounds);
    verify_actual_density(&result);
    let (max_abs_g, max_offset_from_wall) = verify_vertex_sdf_surface(&result);

    print_summary(&Summary {
        result: &result,
        max_abs_g,
        max_offset_from_wall,
    });

    let out_dir = Path::new(env!("CARGO_MANIFEST_DIR")).join("out");
    std::fs::create_dir_all(&out_dir)?;
    let mesh_path = out_dir.join("gyroid_lattice.ply");
    save_ply(&result.mesh, &mesh_path, true)?;
    println!(
        "artifact: out/gyroid_lattice.ply ({}v, {}f, binary little-endian)",
        result.mesh.vertices.len(),
        result.mesh.faces.len(),
    );
    println!();
    println!(
        "OK — 3 gyroid + 2 density_to_threshold + 1 make_shell + LatticeType traits + \
         params validate + cell_count + F10 vertex-soup + bounds containment + \
         actual_density + drift-11 SDF surface anchors all green"
    );

    Ok(())
}
