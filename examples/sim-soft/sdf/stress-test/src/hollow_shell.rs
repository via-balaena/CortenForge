//! hollow-shell — sharp-CSG difference combinator on `SphereSdf`s.
//!
//! `SphereSdf{R_OUTER=1.0} \ SphereSdf{R_CAVITY=0.5}` composed via
//! [`DifferenceSdf`] (book Part 7 §00 §01 sharp-CSG difference operator
//! `φ(p) = max(φ_a(p), -φ_b(p))` under the `φ < 0 inside` convention).
//! The result is a thick-walled hollow shell — body interior lies
//! between the two operand surfaces (`R_CAVITY < |p| < R_OUTER`); the
//! cavity (`|p| < R_CAVITY`) and the exterior (`|p| > R_OUTER`) are
//! both *outside* the body. A 2-D xy-plane slice at `z = 0` is emitted
//! as `out/sdf_slice.ply` with two per-vertex scalars to visually
//! validate the donut cross-section.
//!
//! Radii `R_OUTER = 1.0` and `R_CAVITY = 0.5` are chosen for
//! FP-bit-exactness on every axis-aligned anchor (both are dyadic; the
//! mid-shell equidistant locus `(R_OUTER + R_CAVITY)/2 = 0.75` is
//! dyadic too). The slice grid spacing 0.0625 (= 2⁻⁴) is dyadic and
//! lattice-aligned with both radii: `-1.5 + 8·0.0625 = -1.0` (outer
//! hit), `-1.5 + 16·0.0625 = -0.5` (cavity hit), `-1.5 + 24·0.0625 =
//! 0.0` (origin), symmetrically positive. Bit-exact comparisons use
//! `assert_relative_eq!(epsilon = 0.0)` per the mesh-v1.0 convention
//! to satisfy clippy's `float_cmp` lint.
//!
//! Three operand regions × two active-operand branches = the
//! load-bearing teaching the example encodes:
//!
//! - **Outer-active branch (a-branch, `φ_a ≥ -φ_b`)** — wherever the
//!   point is closer to the outer surface than the cavity surface
//!   (radially: `|p| > 0.75`). Gradient is `+p/|p|` (radially outward).
//! - **Cavity-active branch (b-branch, `φ_a < -φ_b`)** — wherever the
//!   point is closer to the cavity surface (radially: `|p| < 0.75`).
//!   Gradient is `-p/|p|` (radially **inward**) — this matches the
//!   `φ < 0 inside` convention's outward-normal semantics: at the
//!   cavity wall, the body's outward normal points INTO the cavity.
//! - **Branch-flip locus** at `|p| = 0.75` — a sphere of radius 0.75
//!   sitting between the two operand surfaces. The `≥` tie-break in
//!   `DifferenceSdf::grad` deterministically picks the a-branch on
//!   this locus.

// PLY field-data is single-precision on disk; converting f64 SDF
// values to f32 for `extras["signed_distance"]` is intrinsic to the
// PLY format. Same rationale as the `sphere_eval` module.
#![allow(clippy::cast_possible_truncation)]
// Slice-axis coords read as the textbook formula
// `-half_extent + i * spacing`; the `mul_add` rewrite obscures intent
// and the result is bit-equivalent here. Same precedent as the
// `sphere_eval` module's bulk-grid construction.
#![allow(clippy::suboptimal_flops)]
// Slice axis indices are `0..49` so `usize as f64` is exactly
// representable (f64 mantissa is 52 bits; max index is 48). Cast is
// correctness-preserving on every supported platform.
#![allow(clippy::cast_precision_loss)]

use std::path::Path;

use anyhow::Result;
use approx::assert_relative_eq;
use mesh_io::save_ply_attributed;
use mesh_types::{AttributedMesh, IndexedMesh, Point3};
use sim_soft::{DifferenceSdf, Sdf, SphereSdf, Vec3};

// =============================================================================
// Constants
// =============================================================================

/// Outer-shell radius. Dyadic so axis-aligned points `(±1, 0, 0)` land
/// exactly on the outer-surface zero set (`√1 = 1` bit-exact).
const R_OUTER: f64 = 1.0;

/// Cavity radius. Dyadic so axis-aligned points `(±0.5, 0, 0)` land
/// exactly on the cavity-surface zero set.
const R_CAVITY: f64 = 0.5;

/// Branch-flip locus radius `(R_OUTER + R_CAVITY)/2 = 0.75` — the
/// sphere where `φ_a = -φ_b` and the `DifferenceSdf::grad` `≥`
/// tie-break selects the a-branch deterministically.
const BRANCH_FLIP_RADIUS: f64 = 0.75;

/// FP-exact comparisons (axis-aligned surface points, dyadic interior
/// anchors, Pythagorean-triple exterior, slice-grid identity). Every
/// anchor in this example is bit-exact — radii and probe points are
/// chosen dyadic so the FP arithmetic admits exact equality.
const EXACT_TOL: f64 = 0.0;

/// Slice resolution per axis — 49 points endpoint-inclusive in
/// `[−1.5, 1.5]` at spacing 0.0625. 49² = 2401 grid points total. The
/// middle index lands exactly on the origin (axis index 24); `R_CAVITY`
/// lands at axis index 32 (`-1.5 + 32·0.0625 = 0.5`) and `R_OUTER` at
/// axis index 40 (`-1.5 + 40·0.0625 = 1.0`).
const SLICE_RES: usize = 49;
const SLICE_HALF_EXTENT: f64 = 1.5;
const SLICE_SPACING: f64 = 0.0625;
const SLICE_TOTAL: usize = SLICE_RES * SLICE_RES;

// =============================================================================
// Constructors
// =============================================================================

/// `DifferenceSdf::new` consumes its operands by `Box<dyn Sdf>`, so the
/// ctor lives in a helper to keep `main()` readable.
fn build_hollow_shell() -> DifferenceSdf {
    DifferenceSdf::new(
        Box::new(SphereSdf { radius: R_OUTER }),
        Box::new(SphereSdf { radius: R_CAVITY }),
    )
}

// =============================================================================
// verify_outer_surface_eval — eval = 0 on the outer sphere
// =============================================================================

/// Six axis-aligned outer-surface points + one off-axis Pythagorean
/// triple `(0.6, 0.8, 0)` (norm² = 0.36 + 0.64 = 1.0 exactly in f64).
/// At `|p| = R_OUTER`: `φ_a = 0`, `φ_b = R_OUTER − R_CAVITY = 0.5`,
/// `-φ_b = -0.5`. `max(0, -0.5) = 0` bit-exact.
fn verify_outer_surface_eval(diff: &DifferenceSdf) {
    for axis in [Vec3::x(), Vec3::y(), Vec3::z()] {
        for sign in [1.0, -1.0] {
            let p = sign * R_OUTER * axis;
            assert_relative_eq!(diff.eval(Point3::from(p)), 0.0, epsilon = EXACT_TOL);
        }
    }
    // Pythagorean off-axis: (0.6)² + (0.8)² = 1.0 exactly in f64.
    assert_relative_eq!(
        diff.eval(Point3::new(0.6, 0.8, 0.0)),
        0.0,
        epsilon = EXACT_TOL,
    );
}

// =============================================================================
// verify_cavity_surface_eval — eval = 0 on the cavity sphere
// =============================================================================

/// Six axis-aligned cavity-surface points. At `|p| = R_CAVITY`:
/// `φ_a = R_CAVITY − R_OUTER = -0.5`, `φ_b = 0`, `-φ_b = 0`.
/// `max(-0.5, 0) = 0` bit-exact (b-branch selected by the tie-break).
fn verify_cavity_surface_eval(diff: &DifferenceSdf) {
    for axis in [Vec3::x(), Vec3::y(), Vec3::z()] {
        for sign in [1.0, -1.0] {
            let p = sign * R_CAVITY * axis;
            assert_relative_eq!(diff.eval(Point3::from(p)), 0.0, epsilon = EXACT_TOL);
        }
    }
}

// =============================================================================
// verify_shell_interior_eval — eval < 0 in the body (between the two surfaces)
// =============================================================================

/// Three radii inside the shell band, all dyadic for bit-exact
/// arithmetic.
///
/// - **Mid-shell `(0.75, 0, 0)`** — the equidistant locus. `φ_a = -0.25`,
///   `-φ_b = -0.25`. Both branches return -0.25; tie-break picks
///   a-branch. `max(-0.25, -0.25) = -0.25` bit-exact.
/// - **Near-outer `(0.875, 0, 0)`** — `φ_a = -0.125`, `-φ_b = -0.375`.
///   `max = -0.125` (a-branch active).
/// - **Near-cavity `(0.625, 0, 0)`** — `φ_a = -0.375`, `-φ_b = -0.125`.
///   `max = -0.125` (b-branch active).
fn verify_shell_interior_eval(diff: &DifferenceSdf) {
    assert_relative_eq!(
        diff.eval(Point3::new(BRANCH_FLIP_RADIUS, 0.0, 0.0)),
        -0.25,
        epsilon = EXACT_TOL,
    );
    assert_relative_eq!(
        diff.eval(Point3::new(0.875, 0.0, 0.0)),
        -0.125,
        epsilon = EXACT_TOL,
    );
    assert_relative_eq!(
        diff.eval(Point3::new(0.625, 0.0, 0.0)),
        -0.125,
        epsilon = EXACT_TOL,
    );
}

// =============================================================================
// verify_cavity_interior_eval — eval > 0 inside the cavity (outside the body)
// =============================================================================

/// Inside the cavity (`|p| < R_CAVITY`) the body SDF is positive: the
/// point is *outside* the hollow shell.
///
/// - **Origin** — `φ_a = -1.0`, `-φ_b = +0.5`. `max = +0.5` bit-exact
///   (cavity radius — the body wall is `R_CAVITY` away).
/// - **`(0.25, 0, 0)`** — `φ_a = -0.75`, `-φ_b = +0.25`. `max = +0.25`
///   bit-exact (distance to nearest body surface = the cavity wall at
///   `R_CAVITY − 0.25 = 0.25`).
fn verify_cavity_interior_eval(diff: &DifferenceSdf) {
    assert_relative_eq!(diff.eval(Point3::origin()), R_CAVITY, epsilon = EXACT_TOL);
    assert_relative_eq!(
        diff.eval(Point3::new(0.25, 0.0, 0.0)),
        0.25,
        epsilon = EXACT_TOL,
    );
}

// =============================================================================
// verify_exterior_eval — eval > 0 outside the outer sphere
// =============================================================================

/// `(1.5, 0, 0)`: dyadic, norm = 1.5. `φ_a = +0.5`, `-φ_b = -1.0`.
/// `max = +0.5` bit-exact. Pythagorean `(3, 4, 0)`: norm = 5 exactly.
/// `φ_a = 4`, `-φ_b = -4.5`. `max = 4` bit-exact.
fn verify_exterior_eval(diff: &DifferenceSdf) {
    assert_relative_eq!(
        diff.eval(Point3::new(1.5, 0.0, 0.0)),
        0.5,
        epsilon = EXACT_TOL,
    );
    assert_relative_eq!(
        diff.eval(Point3::new(3.0, 4.0, 0.0)),
        4.0,
        epsilon = EXACT_TOL,
    );
}

// =============================================================================
// verify_outer_active_grad — outer-branch active ⇒ grad = +p/|p|
// =============================================================================

/// In the outer-active region (`|p| > BRANCH_FLIP_RADIUS = 0.75`), the
/// gradient is the outer sphere's `p/|p|` — the body's outward normal
/// points radially outward, away from the origin.
fn verify_outer_active_grad(diff: &DifferenceSdf) {
    // Inside the shell, near the outer surface.
    assert_relative_eq!(
        diff.grad(Point3::new(0.875, 0.0, 0.0)),
        Vec3::x(),
        epsilon = EXACT_TOL,
    );
    // Outside the outer surface.
    assert_relative_eq!(
        diff.grad(Point3::new(1.5, 0.0, 0.0)),
        Vec3::x(),
        epsilon = EXACT_TOL,
    );
    // Negative axis (sign symmetry).
    assert_relative_eq!(
        diff.grad(Point3::new(-1.5, 0.0, 0.0)),
        -Vec3::x(),
        epsilon = EXACT_TOL,
    );
    // Pythagorean off-axis on the outer surface — `(0.6, 0.8, 0)/1 =
    // (0.6, 0.8, 0)` bit-exact.
    assert_relative_eq!(
        diff.grad(Point3::new(0.6, 0.8, 0.0)),
        Vec3::new(0.6, 0.8, 0.0),
        epsilon = EXACT_TOL,
    );
}

// =============================================================================
// verify_cavity_active_grad — cavity-branch active ⇒ grad = -p/|p| (INWARD)
// =============================================================================

/// In the cavity-active region (`|p| < BRANCH_FLIP_RADIUS = 0.75`), the
/// gradient is `-(cavity sphere's p/|p|)` — the body's outward normal
/// points radially **inward**, toward the origin. At the cavity wall
/// this matches physical intuition: the body's outward-facing side at
/// the cavity surface looks INTO the cavity.
fn verify_cavity_active_grad(diff: &DifferenceSdf) {
    // Inside the shell, near the cavity surface.
    assert_relative_eq!(
        diff.grad(Point3::new(0.625, 0.0, 0.0)),
        -Vec3::x(),
        epsilon = EXACT_TOL,
    );
    // Inside the cavity proper.
    assert_relative_eq!(
        diff.grad(Point3::new(0.25, 0.0, 0.0)),
        -Vec3::x(),
        epsilon = EXACT_TOL,
    );
    // Negative axis (sign symmetry — gradient still points inward
    // toward origin, so at -x̂ the inward direction is +x̂).
    assert_relative_eq!(
        diff.grad(Point3::new(-0.25, 0.0, 0.0)),
        Vec3::x(),
        epsilon = EXACT_TOL,
    );
}

// =============================================================================
// verify_branch_flip_locus — tie-break picks a-branch at |p| = 0.75
// =============================================================================

/// On the equidistant sphere `|p| = BRANCH_FLIP_RADIUS = 0.75` the two
/// operand branches return identical `eval` values (`-0.25`). The
/// `DifferenceSdf::grad` `≥` tie-break selects the a-branch
/// deterministically, so the gradient is the outer sphere's `p/|p|`
/// (radially outward) at every point on this locus.
fn verify_branch_flip_locus(diff: &DifferenceSdf) {
    for axis in [Vec3::x(), Vec3::y(), Vec3::z()] {
        for sign in [1.0, -1.0] {
            let p = sign * BRANCH_FLIP_RADIUS * axis;
            let p_pt = Point3::from(p);
            // Eval is -0.25 = -(R_OUTER - R_CAVITY)/2 bit-exact.
            assert_relative_eq!(diff.eval(p_pt), -0.25, epsilon = EXACT_TOL);
            // Grad is the unit axis vector (outer sphere's p/|p|).
            assert_relative_eq!(diff.grad(p_pt), sign * axis, epsilon = EXACT_TOL);
        }
    }
}

// =============================================================================
// verify_outer_surface_grad — at |p| = R_OUTER, a-branch active
// =============================================================================

/// On the outer surface `|p| = R_OUTER`: `φ_a = 0`, `-φ_b = -0.5`.
/// `0 ≥ -0.5` ⇒ a-branch ⇒ grad = outer sphere's `p/|p|`. Body's
/// outward normal points radially outward — physical intuition for
/// the outer wall.
fn verify_outer_surface_grad(diff: &DifferenceSdf) {
    for axis in [Vec3::x(), Vec3::y(), Vec3::z()] {
        for sign in [1.0, -1.0] {
            let p = sign * R_OUTER * axis;
            assert_relative_eq!(diff.grad(Point3::from(p)), sign * axis, epsilon = EXACT_TOL);
        }
    }
}

// =============================================================================
// verify_cavity_surface_grad — at |p| = R_CAVITY, b-branch active (INWARD)
// =============================================================================

/// On the cavity surface `|p| = R_CAVITY`: `φ_a = -0.5`, `-φ_b = 0`.
/// `-0.5 < 0` ⇒ b-branch ⇒ grad = `-(cavity sphere's p/|p|)`. Body's
/// outward normal points radially **inward** — into the cavity. This
/// is the load-bearing visual for the difference operator's geometric
/// meaning: a `Difference(A, B)` body has B's surface as its inner
/// wall, with normals flipped relative to B's own SDF.
fn verify_cavity_surface_grad(diff: &DifferenceSdf) {
    for axis in [Vec3::x(), Vec3::y(), Vec3::z()] {
        for sign in [1.0, -1.0] {
            let p = sign * R_CAVITY * axis;
            // Inward = -(p/|p|) = -(sign · axis).
            assert_relative_eq!(
                diff.grad(Point3::from(p)),
                -sign * axis,
                epsilon = EXACT_TOL
            );
        }
    }
}

// =============================================================================
// Slice grid — 49² = 2401 points in [−1.5, 1.5]² × {0}
// =============================================================================

/// Per-grid-point sample: 2-D position (z = 0), signed distance,
/// active-branch indicator (1.0 if a-branch, 0.0 if b-branch).
struct SliceSample {
    p: Vec3,
    eval: f64,
    a_active: bool,
}

/// Build the slice by axis indices; coordinate
/// `i → −SLICE_HALF_EXTENT + i · SLICE_SPACING`. At
/// `i = ORIGIN_AXIS_INDEX = 24`, the coordinate is
/// `−1.5 + 24 · 0.0625 = 0.0` (24 · 0.0625 rounds to 1.5 bit-exact
/// via IEEE ties-to-even).
fn build_slice(diff: &DifferenceSdf) -> Vec<SliceSample> {
    let mut slice = Vec::with_capacity(SLICE_TOTAL);
    let outer = SphereSdf { radius: R_OUTER };
    let cavity = SphereSdf { radius: R_CAVITY };
    for ix in 0..SLICE_RES {
        let x = -SLICE_HALF_EXTENT + (ix as f64) * SLICE_SPACING;
        for iy in 0..SLICE_RES {
            let y = -SLICE_HALF_EXTENT + (iy as f64) * SLICE_SPACING;
            let p = Vec3::new(x, y, 0.0);
            let p_pt = Point3::from(p);
            let phi_a = outer.eval(p_pt);
            let phi_b = cavity.eval(p_pt);
            slice.push(SliceSample {
                p,
                eval: diff.eval(p_pt),
                a_active: phi_a >= -phi_b,
            });
        }
    }
    slice
}

/// Bit-exact identity at every slice grid point: `DifferenceSdf::eval`
/// is implemented as `max(phi_a, -phi_b)` over the boxed operands;
/// computing the same RHS in the test is the same FP arithmetic, so
/// equality holds exactly. Active-branch field re-derives the
/// `phi_a >= -phi_b` predicate and asserts agreement with the field
/// captured during grid construction.
///
/// Returns `(interior_shell, surface, cavity_interior, exterior)`
/// bucket counts for the museum-plaque summary.
fn verify_slice_consistency(slice: &[SliceSample]) -> (usize, usize, usize, usize) {
    assert_eq!(slice.len(), SLICE_TOTAL);

    let mut interior = 0usize;
    let mut surface = 0usize;
    let mut cavity_interior = 0usize;
    let mut exterior = 0usize;

    for s in slice {
        // Bit-exact identity against the closed-form difference.
        let n = s.p.norm();
        let phi_a = n - R_OUTER;
        let neg_phi_b = -(n - R_CAVITY);
        let expected = phi_a.max(neg_phi_b);
        assert_relative_eq!(s.eval, expected, epsilon = EXACT_TOL);

        // Active-branch field consistency.
        let expected_a = phi_a >= neg_phi_b;
        assert_eq!(s.a_active, expected_a);

        // Bucket by region: interior shell (eval < 0), exterior (eval >
        // 0 AND outside outer), cavity_interior (eval > 0 AND inside
        // cavity), surface (eval == 0). Use the closed-form radius for
        // bucket separation; `eval > 0` is split by `n < R_CAVITY` vs
        // `n > R_OUTER`.
        if s.eval < 0.0 {
            interior += 1;
        } else if s.eval > 0.0 {
            if n < R_CAVITY {
                cavity_interior += 1;
            } else {
                exterior += 1;
            }
        } else {
            surface += 1;
        }
    }

    assert_eq!(interior + surface + cavity_interior + exterior, SLICE_TOTAL);
    (interior, surface, cavity_interior, exterior)
}

/// Write the slice as a vertices-only PLY with two per-vertex scalars:
/// `signed_distance` (analytic SDF — divergent: negative shell band,
/// zero on both circles, positive cavity + exterior) and
/// `active_branch` (categorical: 1.0 = outer-active, 0.0 =
/// cavity-active; visually shows the branch-flip circle at `|p| =
/// 0.75`). Mirrors the sibling `sphere_eval` module's two-scalar
/// pattern.
fn save_slice_ply(slice: &[SliceSample], path: &Path) -> Result<()> {
    let vertices: Vec<Point3<f64>> = slice
        .iter()
        .map(|s| Point3::new(s.p.x, s.p.y, s.p.z))
        .collect();
    let faces: Vec<[u32; 3]> = Vec::new();
    let geometry = IndexedMesh::from_parts(vertices, faces);
    let mut attributed = AttributedMesh::new(geometry);
    let signed_distance: Vec<f32> = slice.iter().map(|s| s.eval as f32).collect();
    let active_branch: Vec<f32> = slice
        .iter()
        .map(|s| if s.a_active { 1.0_f32 } else { 0.0_f32 })
        .collect();
    attributed.insert_extra("signed_distance", signed_distance)?;
    attributed.insert_extra("active_branch", active_branch)?;
    save_ply_attributed(&attributed, path, true)?;
    Ok(())
}

// =============================================================================
// print_summary — museum-plaque stdout
// =============================================================================

fn print_summary(
    interior: usize,
    surface: usize,
    cavity_interior: usize,
    exterior: usize,
    path: &Path,
) {
    println!("==== hollow-shell ====");
    println!();
    println!("input  : DifferenceSdf {{");
    println!("           a: SphereSdf {{ radius: {R_OUTER} }},  // outer wall");
    println!("           b: SphereSdf {{ radius: {R_CAVITY} }},  // cavity wall");
    println!("         }}");
    println!("         eval(p) = max(|p| − {R_OUTER}, −(|p| − {R_CAVITY}))");
    println!("         grad(p) = +p/|p|  if outer-active   (|p| ≥ {BRANCH_FLIP_RADIUS})");
    println!("                 = −p/|p|  if cavity-active  (|p| < {BRANCH_FLIP_RADIUS})");
    println!();
    println!("Anchor groups (all assertions exit-0 on success):");
    println!("  outer_surface_eval         : 6 axis-aligned + 1 Pythagorean, eval = 0");
    println!("  cavity_surface_eval        : 6 axis-aligned, eval = 0");
    println!("  shell_interior_eval        : mid-shell + near-outer + near-cavity");
    println!("  cavity_interior_eval       : origin + intermediate, eval > 0");
    println!("  exterior_eval              : axis + Pythagorean (3, 4, 0)");
    println!("  outer_active_grad          : grad = +p/|p| where outer-active");
    println!("  cavity_active_grad         : grad = -p/|p| where cavity-active (INWARD)");
    println!("  branch_flip_locus          : at |p| = 0.75, tie-break picks a-branch");
    println!("  outer_surface_grad         : at |p| = R_OUTER, a-active, outward normal");
    println!("  cavity_surface_grad        : at |p| = R_CAVITY, b-active, INWARD normal");
    println!("  slice_consistency          : bit-exact eval + a_active match @ all 2401 pts");
    println!();
    println!(
        "Slice {SLICE_RES}² = {SLICE_TOTAL} points in [−{SLICE_HALF_EXTENT}, +{SLICE_HALF_EXTENT}]² × {{0}} at spacing {SLICE_SPACING}:"
    );
    println!("  shell interior (eval < 0)  : {interior:>5}");
    println!("  cavity interior (eval > 0) : {cavity_interior:>5}");
    println!("  exterior (eval > 0)        : {exterior:>5}");
    println!("  surface (eval = 0)         : {surface:>5}");
    println!();
    println!("PLY    : {}", path.display());
    println!("         vertices-only point cloud + 2 per-vertex scalars:");
    println!("           extras[\"signed_distance\"] — analytic SDF, divergent");
    println!("           extras[\"active_branch\"]    — 0/1 categorical (cavity/outer)");
    println!("         open in cf-view, the workspace's unified visual-review viewer:");
    println!("           cargo run -p cf-viewer --release -- <path>");
    println!("         default-picks active_branch (alphabetical first; shows the");
    println!("         branch-flip circle at |p| = 0.75 as a sharp ring); scalar");
    println!("         dropdown switches to signed_distance (donut cross-section —");
    println!("         negative shell band, zero on both circles, positive cavity +");
    println!("         exterior; divergent bwr).");
}

// =============================================================================
// main
// =============================================================================

pub fn run() -> Result<()> {
    let diff = build_hollow_shell();

    verify_outer_surface_eval(&diff);
    verify_cavity_surface_eval(&diff);
    verify_shell_interior_eval(&diff);
    verify_cavity_interior_eval(&diff);
    verify_exterior_eval(&diff);
    verify_outer_active_grad(&diff);
    verify_cavity_active_grad(&diff);
    verify_branch_flip_locus(&diff);
    verify_outer_surface_grad(&diff);
    verify_cavity_surface_grad(&diff);

    let slice = build_slice(&diff);
    let (interior, surface, cavity_interior, exterior) = verify_slice_consistency(&slice);

    let out_dir = Path::new(env!("CARGO_MANIFEST_DIR")).join("out");
    std::fs::create_dir_all(&out_dir)?;
    let out_path = out_dir.join("sdf_slice.ply");
    save_slice_ply(&slice, &out_path)?;

    print_summary(interior, surface, cavity_interior, exterior, &out_path);

    Ok(())
}
