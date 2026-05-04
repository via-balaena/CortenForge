//! sphere-sdf-eval — `Sdf` trait contract on `SphereSdf`.
//!
//! Sphere of radius 1 centred at the origin. Signed distance is
//! `‖p‖ − 1` (negative inside, zero on the surface, positive outside);
//! gradient is `p / ‖p‖` (unit-length on the entire domain except the
//! singularity at the centre, where `SphereSdf::grad` returns a
//! documented `Vec3::z()` fallback per `sim_soft::sdf_bridge::sdf`).
//!
//! Every numerical anchor in this example is bit-exact wherever the
//! arithmetic admits it (axis-aligned surface points, origin interior,
//! Pythagorean-triple exterior points where `√(a²+b²+c²)` is integer),
//! and tolerated at `1e-15` only where coordinate rounding forces a
//! fall-back to approximate FP comparison (off-axis surface point at
//! `(1/√3, 1/√3, 1/√3)`, off-axis interior, far exterior). Bit-exact
//! comparisons use `assert_relative_eq!(epsilon = 0.0)` per the
//! mesh-v1.0 convention to satisfy clippy's `float_cmp` lint.
//!
//! Bulk pass: an 11³ = 1331-point grid in `[−2, 2]³` at spacing 0.4 is
//! emitted as `out/sdf_grid.ply` with `extras["signed_distance"]` for
//! external colormap rendering, and the bit-exact identity
//! `eval(p) == ‖p‖ − radius` is asserted at every grid point (both
//! sides do the same arithmetic, so equality holds exactly).

// PLY field-data is single-precision on disk; converting f64 SDF
// values to f32 for `extras["signed_distance"]` is intrinsic to the
// PLY format. Same rationale as `mesh-sdf-distance-query`.
#![allow(clippy::cast_possible_truncation)]
// Grid-axis coords read as the textbook formula
// `-half_extent + i * spacing`; the `mul_add` rewrite obscures intent
// and the result is bit-equivalent here (no FMA-vs-separate-rounding
// observability since both sides feed into a downstream norm). Same
// precedent as mesh-sdf-distance-query bulk-grid construction.
#![allow(clippy::suboptimal_flops)]
// Grid axis indices are `0..11` so `usize as f64` is exactly
// representable (f64 mantissa is 52 bits; max index is 10). The cast
// is correctness-preserving on every supported platform.
#![allow(clippy::cast_precision_loss)]
// Cartesian coords spelt as `x, y, z` are reading-grade; renaming to
// `coord_x` etc. obscures the grid-coordinate semantics this function
// is built around.
#![allow(clippy::many_single_char_names)]

use std::path::Path;

use anyhow::Result;
use approx::assert_relative_eq;
use mesh_io::save_ply_attributed;
use mesh_types::{AttributedMesh, IndexedMesh, Point3};
use sim_soft::{Sdf, SphereSdf, Vec3};

// =============================================================================
// Constants
// =============================================================================

/// Unit sphere — radius 1.0 chosen so that axis-aligned surface points
/// `(±1, 0, 0)`, `(0, ±1, 0)`, `(0, 0, ±1)` are FP-exact-representable
/// and `√(1) = 1` is bit-exact, putting the on-surface eval at exactly
/// 0.0.
const SPHERE_RADIUS: f64 = 1.0;

/// FP-exact comparisons (axis-aligned surface, origin interior,
/// Pythagorean-triple exterior, grid-identity).
const EXACT_TOL: f64 = 0.0;

/// Approximate comparisons where coordinate rounding forces a sub-ε
/// fall-back (off-axis points where coords involve `1/√3`, `1/√6`, or
/// other non-dyadic divisors).
const APPROX_TOL: f64 = 1e-15;

/// Bulk-grid resolution per axis — 11 points endpoint-inclusive in
/// `[−2, 2]` at spacing 0.4. 11³ = 1331 grid points total. The middle
/// index lands exactly on the origin (the documented gradient
/// singularity).
const GRID_RES: usize = 11;
const GRID_HALF_EXTENT: f64 = 2.0;
const GRID_SPACING: f64 = 0.4;
const GRID_TOTAL: usize = GRID_RES * GRID_RES * GRID_RES;

/// Origin index in the 1D grid axis (`-2.0 + 5 * 0.4 = 0.0` exactly —
/// `5.0 * 0.4` rounds bit-exactly to 2.0 by ties-to-even).
const ORIGIN_GRID_INDEX: usize = 5;

// =============================================================================
// verify_surface_eval — eval = 0 on the sphere surface
// =============================================================================

/// Six axis-aligned surface points. `‖(±1, 0, 0)‖ = √(1) = 1` is
/// bit-exact, so `eval = 1 − 1 = 0` is bit-exact.
fn verify_surface_eval_axis_aligned(s: &SphereSdf) {
    for axis in [Vec3::x(), Vec3::y(), Vec3::z()] {
        for sign in [1.0, -1.0] {
            let p = sign * axis;
            assert_relative_eq!(s.eval(p), 0.0, epsilon = EXACT_TOL);
        }
    }
}

/// Off-axis surface point at `(1/√3, 1/√3, 1/√3)`. Each coord is the
/// correctly-rounded f64 nearest `1/√3`; squared and summed, the
/// result differs from 1.0 by at most 3 ULP. `√(1 + δ) − 1 ≈ δ/2`
/// stays under 1e-15 for any double-precision input.
fn verify_surface_eval_off_axis(s: &SphereSdf) {
    let inv_sqrt3 = 1.0 / 3.0_f64.sqrt();
    let p = Vec3::new(inv_sqrt3, inv_sqrt3, inv_sqrt3);
    assert_relative_eq!(s.eval(p), 0.0, epsilon = APPROX_TOL);
}

// =============================================================================
// verify_interior_eval — eval < 0 inside the sphere
// =============================================================================

/// Interior anchors with bit-exact arithmetic: origin (`‖0‖ = 0`),
/// half-axis (`0.5² = 0.25`, `√0.25 = 0.5`), and `(0, 0, 0.5)` — all
/// dyadic fractions whose squares and square-roots are
/// FP-representable exactly.
fn verify_interior_eval(s: &SphereSdf) {
    assert_relative_eq!(s.eval(Vec3::zeros()), -1.0, epsilon = EXACT_TOL);
    assert_relative_eq!(s.eval(Vec3::new(0.5, 0.0, 0.0)), -0.5, epsilon = EXACT_TOL);
    assert_relative_eq!(s.eval(Vec3::new(0.0, 0.0, 0.5)), -0.5, epsilon = EXACT_TOL);

    // Off-axis interior — (0.5, 0.5, 0.5) has norm √0.75 ≈ 0.8660…;
    // closed-form distance is √0.75 − 1.
    let p = Vec3::new(0.5, 0.5, 0.5);
    let expected = 0.75_f64.sqrt() - 1.0;
    assert_relative_eq!(s.eval(p), expected, epsilon = APPROX_TOL);
}

// =============================================================================
// verify_exterior_eval — eval > 0 outside the sphere
// =============================================================================

/// Pythagorean-triple exterior points: `(3, 4, 0)` and `(0, 3, 4)`
/// have norm exactly 5 (`√(9 + 16) = √25 = 5`), so `eval = 5 − 1 = 4`
/// bit-exact. Plus an axis-aligned far point and one off-axis far
/// point with closed-form approximate distance.
fn verify_exterior_eval(s: &SphereSdf) {
    assert_relative_eq!(s.eval(Vec3::new(2.0, 0.0, 0.0)), 1.0, epsilon = EXACT_TOL);
    assert_relative_eq!(s.eval(Vec3::new(3.0, 4.0, 0.0)), 4.0, epsilon = EXACT_TOL);
    assert_relative_eq!(s.eval(Vec3::new(0.0, 3.0, 4.0)), 4.0, epsilon = EXACT_TOL);

    // Far off-axis: (10, 10, 10) has norm √300 = 10√3.
    let p = Vec3::new(10.0, 10.0, 10.0);
    let expected = 300.0_f64.sqrt() - 1.0;
    assert_relative_eq!(s.eval(p), expected, epsilon = APPROX_TOL);
}

// =============================================================================
// verify_gradient_unit_length — ‖grad(p)‖ = 1 for all p ≠ 0
// =============================================================================

/// Axis-aligned points: `grad(±e_i) = ±e_i` whose norm is exactly 1
/// (one nonzero component of magnitude 1, squared sum = 1, √1 = 1
/// bit-exact). Off-axis points: norm within `1e-15` of 1.
fn verify_gradient_unit_length(s: &SphereSdf) {
    // 6 axis-aligned anchors — grad is exactly the unit axis vector.
    for axis in [Vec3::x(), Vec3::y(), Vec3::z()] {
        for sign in [1.0, -1.0] {
            let p = sign * axis;
            assert_relative_eq!(s.grad(p).norm(), 1.0, epsilon = EXACT_TOL);
        }
    }

    // Off-axis: surface points, interior, exterior, far. All ≠ origin.
    let inv_sqrt3 = 1.0 / 3.0_f64.sqrt();
    let off_axis = [
        Vec3::new(inv_sqrt3, inv_sqrt3, inv_sqrt3),
        Vec3::new(0.5, 0.5, 0.5),
        Vec3::new(3.0, 4.0, 0.0),
        Vec3::new(10.0, 10.0, 10.0),
    ];
    for p in off_axis {
        assert_relative_eq!(s.grad(p).norm(), 1.0, epsilon = APPROX_TOL);
    }
}

// =============================================================================
// verify_gradient_direction — grad(p) = p / ‖p‖
// =============================================================================

/// Axis-aligned points: `grad(p)` is bit-exactly the unit axis
/// vector. Pythagorean exterior `(3, 4, 0)`: norm = 5, grad =
/// `(3/5, 4/5, 0) = (0.6, 0.8, 0.0)` bit-exact (3/5 and 4/5 are
/// correctly-rounded f64 dyadics; nalgebra's component-wise
/// `Vector3 / scalar` divides each lane independently).
fn verify_gradient_direction(s: &SphereSdf) {
    assert_relative_eq!(
        s.grad(Vec3::new(1.0, 0.0, 0.0)),
        Vec3::x(),
        epsilon = EXACT_TOL
    );
    assert_relative_eq!(
        s.grad(Vec3::new(0.0, 1.0, 0.0)),
        Vec3::y(),
        epsilon = EXACT_TOL
    );
    assert_relative_eq!(
        s.grad(Vec3::new(0.0, 0.0, 1.0)),
        Vec3::z(),
        epsilon = EXACT_TOL
    );
    assert_relative_eq!(
        s.grad(Vec3::new(-1.0, 0.0, 0.0)),
        -Vec3::x(),
        epsilon = EXACT_TOL
    );

    // Direction is invariant to magnitude — grad(2 e_x) = e_x.
    assert_relative_eq!(
        s.grad(Vec3::new(2.0, 0.0, 0.0)),
        Vec3::x(),
        epsilon = EXACT_TOL
    );

    // Pythagorean: grad((3, 4, 0)) = (0.6, 0.8, 0.0) bit-exact.
    assert_relative_eq!(
        s.grad(Vec3::new(3.0, 4.0, 0.0)),
        Vec3::new(0.6, 0.8, 0.0),
        epsilon = EXACT_TOL,
    );

    // Off-axis: grad((1, 1, 1)) = (1/√3) (1, 1, 1).
    let inv_sqrt3 = 1.0 / 3.0_f64.sqrt();
    assert_relative_eq!(
        s.grad(Vec3::new(1.0, 1.0, 1.0)),
        Vec3::new(inv_sqrt3, inv_sqrt3, inv_sqrt3),
        epsilon = APPROX_TOL,
    );
}

// =============================================================================
// verify_origin_singularity — documented Vec3::z() fallback
// =============================================================================

/// At `p = 0` the gradient `p / ‖p‖` is undefined; `SphereSdf::grad`
/// returns `Vec3::z()` arbitrarily, documented as unobservable
/// downstream because the centre is strictly inside the SDF and the
/// mesher only queries near the zero set. Asserted here so the
/// example's PLY-grid pipeline doesn't silently shift if the fallback
/// ever changes.
fn verify_origin_singularity(s: &SphereSdf) {
    assert_relative_eq!(s.grad(Vec3::zeros()), Vec3::z(), epsilon = EXACT_TOL);
}

// =============================================================================
// Bulk grid — 11³ = 1331 points in [−2, 2]³ at spacing 0.4
// =============================================================================

/// Per-grid-point sample: world-space position, signed distance, and
/// gradient. Carried by value into the consistency check + PLY writer.
struct GridSample {
    p: Vec3,
    eval: f64,
    grad_norm: f64,
}

/// Build the grid by axis indices; coordinate `i → −half_extent + i ·
/// spacing`. At `i = ORIGIN_GRID_INDEX = 5`, the coordinate is
/// `−2.0 + 5.0 · 0.4 = 0.0` (5 · 0.4 rounds to 2.0 bit-exactly via
/// IEEE ties-to-even).
fn build_grid(s: &SphereSdf) -> Vec<GridSample> {
    let mut grid = Vec::with_capacity(GRID_TOTAL);
    for ix in 0..GRID_RES {
        let x = -GRID_HALF_EXTENT + (ix as f64) * GRID_SPACING;
        for iy in 0..GRID_RES {
            let y = -GRID_HALF_EXTENT + (iy as f64) * GRID_SPACING;
            for iz in 0..GRID_RES {
                let z = -GRID_HALF_EXTENT + (iz as f64) * GRID_SPACING;
                let p = Vec3::new(x, y, z);
                grid.push(GridSample {
                    p,
                    eval: s.eval(p),
                    grad_norm: s.grad(p).norm(),
                });
            }
        }
    }
    grid
}

/// Bit-exact identity: `SphereSdf::eval` is implemented as
/// `p.norm() − self.radius`; computing the same RHS in the test is
/// the same FP arithmetic, so equality holds exactly. Plus a unit-
/// length grad-norm check at every non-origin grid point and an
/// origin-fallback assert at the centre.
fn verify_grid_consistency(grid: &[GridSample]) -> (usize, usize, usize) {
    assert_eq!(grid.len(), GRID_TOTAL);

    let mut interior = 0usize;
    let mut surface = 0usize;
    let mut exterior = 0usize;

    for (i, g) in grid.iter().enumerate() {
        // Bit-exact identity at every grid point.
        let expected = g.p.norm() - SPHERE_RADIUS;
        assert_relative_eq!(g.eval, expected, epsilon = EXACT_TOL);

        // Unit gradient at every non-origin point. The origin (only at
        // `i = (5, 5, 5)` in 1-D-flattened index 5*121 + 5*11 + 5 = 665)
        // is the documented Z-fallback.
        let is_origin = i
            == ORIGIN_GRID_INDEX * GRID_RES * GRID_RES
                + ORIGIN_GRID_INDEX * GRID_RES
                + ORIGIN_GRID_INDEX;
        if is_origin {
            assert_relative_eq!(g.p, Vec3::zeros(), epsilon = EXACT_TOL);
            assert_relative_eq!(g.grad_norm, 1.0, epsilon = EXACT_TOL);
        } else {
            assert_relative_eq!(g.grad_norm, 1.0, epsilon = APPROX_TOL);
        }

        // Bucket by sign for the museum-plaque summary.
        if g.eval < 0.0 {
            interior += 1;
        } else if g.eval > 0.0 {
            exterior += 1;
        } else {
            surface += 1;
        }
    }

    assert_eq!(interior + surface + exterior, GRID_TOTAL);
    (interior, surface, exterior)
}

/// Write the bulk grid as a vertices-only PLY with
/// `extras["signed_distance"]` per-vertex scalar. Mirrors the
/// `examples/mesh/ply-with-custom-attributes` and
/// `examples/mesh/mesh-sdf-distance-query` patterns.
fn save_grid_ply(grid: &[GridSample], path: &Path) -> Result<()> {
    let vertices: Vec<Point3<f64>> = grid
        .iter()
        .map(|g| Point3::new(g.p.x, g.p.y, g.p.z))
        .collect();
    let faces: Vec<[u32; 3]> = Vec::new();
    let geometry = IndexedMesh::from_parts(vertices, faces);
    let mut attributed = AttributedMesh::new(geometry);
    let scalars: Vec<f32> = grid.iter().map(|g| g.eval as f32).collect();
    attributed.insert_extra("signed_distance", scalars)?;
    save_ply_attributed(&attributed, path, true)?;
    Ok(())
}

// =============================================================================
// print_summary — museum-plaque stdout
// =============================================================================

fn print_summary(interior: usize, surface: usize, exterior: usize, path: &Path) {
    println!("==== sphere-sdf-eval ====");
    println!();
    println!("input  : SphereSdf {{ radius: {SPHERE_RADIUS} }} (unit sphere at origin)");
    println!("         eval(p) = ‖p‖ − {SPHERE_RADIUS}");
    println!("         grad(p) = p / ‖p‖   (Vec3::z() at the singularity p = 0)");
    println!();
    println!("Anchor groups (all assertions exit-0 on success):");
    println!("  surface_eval_axis_aligned  : 6 points, eval = 0 bit-exact");
    println!("  surface_eval_off_axis      : (1/√3)·(1, 1, 1), eval ≈ 0 within 1e-15");
    println!("  interior_eval              : origin + half-axis + (0.5, 0.5, 0.5)");
    println!("  exterior_eval              : (2, 0, 0) + (3, 4, 0) + (0, 3, 4) + far");
    println!("  gradient_unit_length       : 6 axis-aligned + 4 off-axis points");
    println!("  gradient_direction         : axis-aligned + Pythagorean (3, 4, 0)");
    println!("  origin_singularity         : grad(0) = Vec3::z() documented fallback");
    println!();
    println!(
        "Bulk grid {GRID_RES}³ = {GRID_TOTAL} points in [−{GRID_HALF_EXTENT}, +{GRID_HALF_EXTENT}]³ at spacing {GRID_SPACING}:"
    );
    println!("  interior (eval < 0)        : {interior:>5}");
    println!("  surface  (eval = 0)        : {surface:>5}");
    println!("  exterior (eval > 0)        : {exterior:>5}");
    println!();
    println!("PLY    : {}", path.display());
    println!("         vertices-only point cloud + extras[\"signed_distance\"]");
    println!("         open in MeshLab / ParaView; colormap by signed_distance");
    println!("         to see the radial SDF gradient (negative inside, positive");
    println!("         outside, zero on the sphere surface).");
}

// =============================================================================
// main
// =============================================================================

fn main() -> Result<()> {
    let sphere = SphereSdf {
        radius: SPHERE_RADIUS,
    };

    verify_surface_eval_axis_aligned(&sphere);
    verify_surface_eval_off_axis(&sphere);
    verify_interior_eval(&sphere);
    verify_exterior_eval(&sphere);
    verify_gradient_unit_length(&sphere);
    verify_gradient_direction(&sphere);
    verify_origin_singularity(&sphere);

    let grid = build_grid(&sphere);
    let (interior, surface, exterior) = verify_grid_consistency(&grid);

    let out_dir = Path::new(env!("CARGO_MANIFEST_DIR")).join("out");
    std::fs::create_dir_all(&out_dir)?;
    let out_path = out_dir.join("sdf_grid.ply");
    save_grid_ply(&grid, &out_path)?;

    print_summary(interior, surface, exterior, &out_path);
    Ok(())
}
