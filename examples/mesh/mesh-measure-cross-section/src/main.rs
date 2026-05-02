//! mesh-measure-cross-section — planar slicing of a 32-segment closed cylinder.
//!
//! Spec: `mesh/MESH_V1_EXAMPLES_SCOPE.md` §5.2. Demonstrates the full
//! `mesh-measure::cross_section` + `cross_sections` +
//! `circumference_at_height` + `area_at_height` public surface against
//! a hand-authored UV-cylinder fixture. Anchors landed across §6.2 #5
//! (skeleton at commit `2a3e6701`) and §6.2 #6 (66 vertex coords, 128
//! face-winding cosine similarities, single mid-slice, 10-slice stack,
//! convenience helpers, out-of-mesh slice, plane-normal normalization).
//!
//! Fixture: 66 verts + 128 tris closed cylinder of radius 5 mm,
//! height 10 mm, axis along +Z —
//! - **Bottom ring** (verts 0..31): 32-segment regular polygon at
//!   z=0; vertex i at `(5·cos(2π·i/32), 5·sin(2π·i/32), 0)`.
//! - **Top ring** (verts 32..63): same `(x, y)` at z=10.
//! - **Cap centers**: bottom (vert 64) at `(0, 0, 0)`; top (vert 65)
//!   at `(0, 0, 10)`.
//!
//! Faces: 32 bottom-cap fan tris (outward normal -z) + 32 top-cap fan
//! tris (outward normal +z) + 64 wall tris (32 quads × 2; outward
//! normal radial at chord midpoint) = 128 total.
//!
//! Per spec §4.7, sin/cos derived coords use `1e-12` tolerance, not
//! bit-exact (`f64::sin` / `f64::cos` are NOT correctly-rounded;
//! cross-platform libm versions can drift at the last bit). The
//! per-face winding anchor uses cosine similarity per spec §7 R6;
//! the analytical match is near 1.

// Cross-product unit-normal computation reads as the textbook formula
// `e1.y*e2.z - e1.z*e2.y`; the `mul_add` rewrite obscures intent and
// produces bit-equivalent results on integer + sin/cos vertex
// coordinates. (Same precedent as printability-showcase
// verify_fixture_geometry.)
#![allow(clippy::suboptimal_flops)]
// `i as f64` for ring-segment-index → angle conversion; `i` is bounded
// by RING_SEGMENTS = 32, far below f64's 2^53 mantissa.
#![allow(clippy::cast_precision_loss)]

use std::path::Path;

use anyhow::Result;
use approx::assert_relative_eq;
use mesh_io::save_ply;
use mesh_measure::{
    CrossSection, area_at_height, circumference_at_height, cross_section, cross_sections,
};
use mesh_types::{IndexedMesh, Point3, Vector3};

// =============================================================================
// Constants
// =============================================================================

/// Cylinder radius (mm). FP-exact integer.
const CYLINDER_RADIUS: f64 = 5.0;

/// Cylinder height (mm). FP-exact integer.
const CYLINDER_HEIGHT: f64 = 10.0;

/// UV ring segment count. 32 yields chord-shrinkage area error of
/// ~0.64% vs analytical π·r²; per spec §5.2, anchors target the
/// polygon shoelace, NOT the analytical circle.
const RING_SEGMENTS: usize = 32;

/// Same as `RING_SEGMENTS` typed `u32` for face-index arithmetic.
const RING_SEGMENTS_U32: u32 = 32;

/// Total vertex count: 32 bottom-ring + 32 top-ring + 2 cap centers.
const TOTAL_VERT_COUNT: usize = 66;

/// Total face count: 32 bottom-cap fan + 32 top-cap fan + 64 wall tris.
const TOTAL_FACE_COUNT: usize = 128;

/// Vertex index of the bottom-cap center.
const BOT_CENTER_IDX: u32 = 64;

/// Vertex index of the top-cap center.
const TOP_CENTER_IDX: u32 = 65;

// =============================================================================
// Tolerances
// =============================================================================

/// Per-vertex coordinate tolerance — sin/cos derived ring coords (NOT
/// FP-exact since `f64::sin` / `f64::cos` aren't correctly-rounded);
/// `1e-12` covers cross-platform libm drift per spec §4.7.
const VERTEX_TOL: f64 = 1e-12;

/// Per-face winding cosine-similarity floor — spec §7 R6 sets
/// `> 0.99` as the worst-case anchor; the analytical direction match
/// is ~1, so a tighter floor passes comfortably and surfaces any
/// winding flip.
const COSINE_SIM_MIN: f64 = 0.999_999_9;

/// Single-slice perimeter / area / slice-stack tolerance.
/// `cross_section`'s shoelace + chord-length accumulation has
/// FP roundoff bounded near `1e-10` for 64 sin/cos-derived vertices.
const SHOELACE_TOL: f64 = 1e-10;

/// `circumference_at_height` / `area_at_height` invoke `cross_section`
/// with the same inputs as the explicit single-slice call. Outputs
/// match bit-exactly; tolerance is comparison slack.
const HELPER_TOL: f64 = 1e-12;

/// `plane_normal` magnitude tolerance — `cross_section` normalizes the
/// input internally; output magnitude is 1 modulo one `sqrt` roundoff.
const NORMAL_MAG_TOL: f64 = 1e-12;

/// Centroid tolerance for the LIBRARY's biased centroid (naive
/// average over chain-closure-duplicated points). The TRUE polygon
/// centroid is `(0, 0, 5)`; library returns `V_first / 65` where
/// `V_first` is the chain's anchor vertex (deterministic per
/// face-emission order). Anchored to literal precision below; see
/// spec §10 item 11 for the v0.9 candidate (polygon centroid vs
/// naive-average).
const CENTROID_TOL: f64 = 1e-12;

/// Empirical biased centroid x of the mid-slice. Library returns
/// `sum / N` where N = 65 (64 unique perimeter points + 1 chain-closure
/// duplicate); for our symmetric 32-gon perimeter `sum_unique = 0`,
/// so `centroid = V_dup / 65` where `V_dup` is the chain-closure-
/// duplicated point. Probe shows `V_dup = mid_(16, 17)` at z=5 (a
/// wall-quad diagonal midpoint on the cylinder's −x side, where the
/// chain happens to close after spiraling out from face 64 = wall
/// tri `A_0 = [bot_0, bot_1, top_1]`). Analytical:
/// `mid_(16,17).x = -2.5·(1 + cos(π/16))`, giving
/// `centroid_x = -(1 + cos(π/16)) / 26 ≈ -0.0762`. See spec §10
/// item 11 for the v0.9 candidate (proper polygon centroid via
/// shoelace, not naive-average over chain-closed points).
const MID_SLICE_CENTROID_X: f64 = -0.076_184_049_246_278_27;

/// Empirical biased centroid y. `mid_(16,17).y = -2.5·sin(π/16)` →
/// `centroid_y = -sin(π/16) / 26 ≈ -0.00750`.
const MID_SLICE_CENTROID_Y: f64 = -0.007_503_473_923_697_397;

/// Empirical biased centroid z. All 65 stored points lie at z=5, so
/// `sum_z = 325` and `centroid_z = 325/65 = 5.0` (FP-exact).
const MID_SLICE_CENTROID_Z: f64 = 5.0;

/// Expected mid-slice contour count (single closed loop).
const MID_SLICE_CONTOUR_COUNT: usize = 1;

/// Slice-stack count + spacing — 10 horizontal slices at z = 0.5..9.5,
/// 1 mm apart. All inside the cylinder; all produce the same polygon
/// area (z-uniform cylinder).
const STACK_COUNT: usize = 10;
const STACK_SPACING: f64 = 1.0;
const STACK_START_Z: f64 = 0.5;

/// Out-of-mesh slice z-coordinate.
const OOB_Z: f64 = 100.0;

// =============================================================================
// Fixture builder
// =============================================================================

/// Build the 32-segment closed-cylinder fixture.
///
/// Vertex layout: `[bot_0..bot_31, top_0..top_31, BOT_CENTER, TOP_CENTER]`.
/// Face layout: `[bottom_cap_fan_0..31, top_cap_fan_0..31, wall_A_0,
/// wall_B_0, wall_A_1, wall_B_1, ..., wall_A_31, wall_B_31]`.
fn build_fixture() -> IndexedMesh {
    let mut vertices: Vec<Point3<f64>> = Vec::with_capacity(TOTAL_VERT_COUNT);

    // Bottom ring (verts 0..31) — z=0.
    for i in 0..RING_SEGMENTS {
        let theta = (i as f64) * std::f64::consts::TAU / (RING_SEGMENTS as f64);
        vertices.push(Point3::new(
            CYLINDER_RADIUS * theta.cos(),
            CYLINDER_RADIUS * theta.sin(),
            0.0,
        ));
    }

    // Top ring (verts 32..63) — z=CYLINDER_HEIGHT.
    for i in 0..RING_SEGMENTS {
        let theta = (i as f64) * std::f64::consts::TAU / (RING_SEGMENTS as f64);
        vertices.push(Point3::new(
            CYLINDER_RADIUS * theta.cos(),
            CYLINDER_RADIUS * theta.sin(),
            CYLINDER_HEIGHT,
        ));
    }

    // Cap centers (verts 64, 65).
    vertices.push(Point3::new(0.0, 0.0, 0.0));
    vertices.push(Point3::new(0.0, 0.0, CYLINDER_HEIGHT));

    let mut faces: Vec<[u32; 3]> = Vec::with_capacity(TOTAL_FACE_COUNT);

    // Bottom-cap fan (32 tris). [BOT_CENTER, bot_(i+1), bot_i] gives
    // outward normal in -z (CCW from below the cap).
    for i in 0..RING_SEGMENTS_U32 {
        let j = (i + 1) % RING_SEGMENTS_U32;
        faces.push([BOT_CENTER_IDX, j, i]);
    }

    // Top-cap fan (32 tris). [TOP_CENTER, top_i, top_(i+1)] gives
    // outward normal in +z (CCW from above the cap).
    for i in 0..RING_SEGMENTS_U32 {
        let j = (i + 1) % RING_SEGMENTS_U32;
        faces.push([TOP_CENTER_IDX, RING_SEGMENTS_U32 + i, RING_SEGMENTS_U32 + j]);
    }

    // Wall (64 tris). Side k connects bot_k → bot_(k+1) → top_(k+1)
    // → top_k. Quad split into [bot_k, bot_(k+1), top_(k+1)] (tri A)
    // + [bot_k, top_(k+1), top_k] (tri B). Outward normal is radial
    // at the chord midpoint angle.
    for k in 0..RING_SEGMENTS_U32 {
        let kp1 = (k + 1) % RING_SEGMENTS_U32;
        let bot_k = k;
        let bot_kp1 = kp1;
        let top_k = RING_SEGMENTS_U32 + k;
        let top_kp1 = RING_SEGMENTS_U32 + kp1;
        faces.push([bot_k, bot_kp1, top_kp1]); // tri A
        faces.push([bot_k, top_kp1, top_k]); // tri B
    }

    IndexedMesh::from_parts(vertices, faces)
}

// =============================================================================
// verify_fixture_geometry — math-pass-first invariant
// =============================================================================

/// Lock every visible property of the hand-authored cylinder as a
/// numerical invariant. Per `feedback_math_pass_first_handauthored`,
/// a successful `cargo run --release` exit-0 with this verifier
/// active is equivalent to a clean visual inspection.
fn verify_fixture_geometry(mesh: &IndexedMesh) {
    assert_eq!(
        mesh.vertices.len(),
        TOTAL_VERT_COUNT,
        "fixture must have {TOTAL_VERT_COUNT} vertices; got {}",
        mesh.vertices.len(),
    );
    assert_eq!(
        mesh.faces.len(),
        TOTAL_FACE_COUNT,
        "fixture must have {TOTAL_FACE_COUNT} faces; got {}",
        mesh.faces.len(),
    );

    // (1) Per-vertex coordinates within VERTEX_TOL of the analytical
    //     expected coords.
    for i in 0..RING_SEGMENTS {
        let theta = (i as f64) * std::f64::consts::TAU / (RING_SEGMENTS as f64);
        let expected_x = CYLINDER_RADIUS * theta.cos();
        let expected_y = CYLINDER_RADIUS * theta.sin();

        let bot = mesh.vertices[i];
        assert_relative_eq!(bot.x, expected_x, epsilon = VERTEX_TOL);
        assert_relative_eq!(bot.y, expected_y, epsilon = VERTEX_TOL);
        assert_relative_eq!(bot.z, 0.0, epsilon = VERTEX_TOL);

        let top = mesh.vertices[RING_SEGMENTS + i];
        assert_relative_eq!(top.x, expected_x, epsilon = VERTEX_TOL);
        assert_relative_eq!(top.y, expected_y, epsilon = VERTEX_TOL);
        assert_relative_eq!(top.z, CYLINDER_HEIGHT, epsilon = VERTEX_TOL);
    }
    let bot_c = mesh.vertices[BOT_CENTER_IDX as usize];
    assert_relative_eq!(bot_c.x, 0.0, epsilon = VERTEX_TOL);
    assert_relative_eq!(bot_c.y, 0.0, epsilon = VERTEX_TOL);
    assert_relative_eq!(bot_c.z, 0.0, epsilon = VERTEX_TOL);
    let top_c = mesh.vertices[TOP_CENTER_IDX as usize];
    assert_relative_eq!(top_c.x, 0.0, epsilon = VERTEX_TOL);
    assert_relative_eq!(top_c.y, 0.0, epsilon = VERTEX_TOL);
    assert_relative_eq!(top_c.z, CYLINDER_HEIGHT, epsilon = VERTEX_TOL);

    // (2) Per-face winding cross-product unit normals — cosine
    //     similarity > COSINE_SIM_MIN with the analytical outward
    //     direction.
    for (face_idx, face) in mesh.faces.iter().enumerate() {
        let p0 = mesh.vertices[face[0] as usize];
        let p1 = mesh.vertices[face[1] as usize];
        let p2 = mesh.vertices[face[2] as usize];
        let e1 = [p1.x - p0.x, p1.y - p0.y, p1.z - p0.z];
        let e2 = [p2.x - p0.x, p2.y - p0.y, p2.z - p0.z];
        let n = [
            e1[1] * e2[2] - e1[2] * e2[1],
            e1[2] * e2[0] - e1[0] * e2[2],
            e1[0] * e2[1] - e1[1] * e2[0],
        ];
        let len = (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt();
        assert!(
            len > 0.0,
            "face {face_idx} has degenerate cross product (zero-area triangle)",
        );
        let unit = [n[0] / len, n[1] / len, n[2] / len];
        let expected = expected_face_normal(face_idx);
        let cos_sim = unit[0] * expected[0] + unit[1] * expected[1] + unit[2] * expected[2];
        assert!(
            cos_sim > COSINE_SIM_MIN,
            "face {face_idx} winding cos-sim {cos_sim:.10} < {COSINE_SIM_MIN}; \
             actual unit normal = {unit:?}, expected outward = {expected:?}",
        );
    }
}

/// Expected outward unit normal direction for face `face_idx`, given
/// the face emission order in [`build_fixture`]:
///
/// - Faces `0..32`: bottom-cap fan, outward = `(0, 0, -1)`.
/// - Faces `32..64`: top-cap fan, outward = `(0, 0, +1)`.
/// - Faces `64..128`: wall tris (2 per side, 32 sides). For tri at
///   index `64 + 2k + r` (r ∈ {0, 1}, k ∈ 0..32), outward is the
///   chord-midpoint radial direction `(cos(α_k), sin(α_k), 0)` with
///   `α_k = (2k + 1)·π / RING_SEGMENTS`.
fn expected_face_normal(face_idx: usize) -> [f64; 3] {
    if face_idx < RING_SEGMENTS {
        [0.0, 0.0, -1.0]
    } else if face_idx < 2 * RING_SEGMENTS {
        [0.0, 0.0, 1.0]
    } else {
        let k = (face_idx - 2 * RING_SEGMENTS) / 2;
        let mid_angle = (2.0 * (k as f64) + 1.0) * std::f64::consts::PI / (RING_SEGMENTS as f64);
        [mid_angle.cos(), mid_angle.sin(), 0.0]
    }
}

// =============================================================================
// verify_mid_slice — single cross_section anchors
// =============================================================================

/// Lock every `CrossSection` field on the mid-height slice (z=5)
/// against analytical + empirical anchors per spec §5.2.
///
/// Geometric note: each wall side's quad is split into 2 tris via
/// the diagonal `bot_k → top_(k+1)`. At any z ∈ (0, 10), the diagonal
/// is sliced at parameter `t = z/h`, producing one interior point
/// PER side that lies on the chord between the two outer ring
/// intersection points. Chain output: 64 unique perimeter points
/// (32 outer ring + 32 chord-interior) traversing the same convex
/// 32-gon shape, with one chain-closure duplicate ⇒ 65 stored points.
/// Polygon area = 32-gon area = `400·sin(π/16)`; perimeter =
/// 32-gon perimeter = `320·sin(π/32)`.
fn verify_mid_slice(mid: &CrossSection) {
    let expected_area = 400.0 * (std::f64::consts::PI / 16.0).sin();
    let expected_perimeter = 320.0 * (std::f64::consts::PI / 32.0).sin();

    assert_relative_eq!(mid.area, expected_area, epsilon = SHOELACE_TOL);
    assert_relative_eq!(mid.perimeter, expected_perimeter, epsilon = SHOELACE_TOL);

    assert_eq!(
        mid.contour_count, MID_SLICE_CONTOUR_COUNT,
        "mid-slice should have exactly 1 contour"
    );
    assert!(
        mid.is_closed(),
        "mid-slice is_closed() should be true (semantically: contour_count > 0)"
    );
    assert!(!mid.is_empty(), "mid-slice should be non-empty");

    // Centroid: library returns naive-average sum/N over chain-closure-
    // duplicated points (NOT polygon centroid). For our symmetric 32-gon
    // perimeter, sum_unique = 0; sum = V_dup; centroid = V_dup / 65.
    // See spec §10 item 11 for the v0.9 candidate fix.
    assert_relative_eq!(mid.centroid.x, MID_SLICE_CENTROID_X, epsilon = CENTROID_TOL);
    assert_relative_eq!(mid.centroid.y, MID_SLICE_CENTROID_Y, epsilon = CENTROID_TOL);
    assert_relative_eq!(mid.centroid.z, MID_SLICE_CENTROID_Z, epsilon = CENTROID_TOL);

    // Plane normal is normalized internally regardless of input magnitude.
    assert_relative_eq!(mid.plane_normal.norm(), 1.0, epsilon = NORMAL_MAG_TOL);
    assert_relative_eq!(mid.plane_normal.x, 0.0, epsilon = NORMAL_MAG_TOL);
    assert_relative_eq!(mid.plane_normal.y, 0.0, epsilon = NORMAL_MAG_TOL);
    assert_relative_eq!(mid.plane_normal.z, 1.0, epsilon = NORMAL_MAG_TOL);

    // Plane origin echoes the input plane_point.
    assert_relative_eq!(mid.plane_origin.x, 0.0, epsilon = VERTEX_TOL);
    assert_relative_eq!(mid.plane_origin.y, 0.0, epsilon = VERTEX_TOL);
    assert_relative_eq!(mid.plane_origin.z, 5.0, epsilon = VERTEX_TOL);

    // Bounds tighter than the cylinder's bounding cylinder.
    let (bmin, bmax) = mid.bounds;
    assert!(
        bmin.x >= -CYLINDER_RADIUS - VERTEX_TOL && bmax.x <= CYLINDER_RADIUS + VERTEX_TOL,
        "mid-slice x bounds ({}, {}) should be within ±{CYLINDER_RADIUS}",
        bmin.x,
        bmax.x,
    );
    assert!(
        bmin.y >= -CYLINDER_RADIUS - VERTEX_TOL && bmax.y <= CYLINDER_RADIUS + VERTEX_TOL,
        "mid-slice y bounds ({}, {}) should be within ±{CYLINDER_RADIUS}",
        bmin.y,
        bmax.y,
    );
    assert_relative_eq!(bmin.z, 5.0, epsilon = VERTEX_TOL);
    assert_relative_eq!(bmax.z, 5.0, epsilon = VERTEX_TOL);
}

// =============================================================================
// verify_slice_stack — cross_sections anchors
// =============================================================================

/// Lock the 10-slice stack at z=0.5..9.5 against the polygon-shoelace
/// area `400·sin(π/16)` per spec drift-8 resolution. Cylinder is
/// z-uniform and the polygon shape is invariant in z (the per-side
/// chord-interior point stays on the chord at parameter z/h),
/// so every slice has the same area + perimeter to FP precision.
fn verify_slice_stack(mesh: &IndexedMesh) {
    let stack = cross_sections(
        mesh,
        Point3::new(0.0, 0.0, STACK_START_Z),
        Vector3::z(),
        STACK_COUNT,
        STACK_SPACING,
    );
    assert_eq!(
        stack.len(),
        STACK_COUNT,
        "cross_sections should return exactly {STACK_COUNT} slices",
    );

    let expected_area = 400.0 * (std::f64::consts::PI / 16.0).sin();
    let expected_perimeter = 320.0 * (std::f64::consts::PI / 32.0).sin();
    for (i, slice) in stack.iter().enumerate() {
        assert_relative_eq!(slice.area, expected_area, epsilon = SHOELACE_TOL);
        assert_relative_eq!(slice.perimeter, expected_perimeter, epsilon = SHOELACE_TOL);
        assert_eq!(slice.contour_count, 1, "stack[{i}] should have 1 contour");
        assert!(slice.is_closed(), "stack[{i}] should be closed");
        let expected_z = STACK_START_Z + (i as f64) * STACK_SPACING;
        assert_relative_eq!(slice.plane_origin.z, expected_z, epsilon = VERTEX_TOL);
    }
}

// =============================================================================
// verify_helpers — circumference_at_height / area_at_height
// =============================================================================

/// `circumference_at_height` and `area_at_height` are wrappers that
/// invoke `cross_section` with `Point3::new(0, 0, z)` + `Vector3::z()`.
/// Outputs match the explicit single-slice call bit-equivalently
/// (FP determinism); `HELPER_TOL` is just comparison slack.
fn verify_helpers(mesh: &IndexedMesh, mid: &CrossSection) {
    let circ = circumference_at_height(mesh, 5.0);
    let area = area_at_height(mesh, 5.0);
    assert_relative_eq!(circ, mid.perimeter, epsilon = HELPER_TOL);
    assert_relative_eq!(area, mid.area, epsilon = HELPER_TOL);
}

// =============================================================================
// verify_out_of_mesh — empty cross-section
// =============================================================================

/// At z=100 (far above the cylinder), no triangle straddles the plane;
/// `cross_section` returns the default empty `CrossSection`.
fn verify_out_of_mesh(mesh: &IndexedMesh) {
    let oob = cross_section(mesh, Point3::new(0.0, 0.0, OOB_Z), Vector3::z());
    assert!(oob.is_empty(), "z=100 slice should be empty");
    assert_eq!(oob.contour_count, 0, "z=100 contour_count should be 0");
    // `cross_section` returns `CrossSection::default()` when there's no
    // intersection, which sets area + perimeter to literal `0.0`. Use
    // `assert_relative_eq!` for the f64 comparison (clippy's float_cmp).
    assert_relative_eq!(oob.area, 0.0, epsilon = 0.0);
    assert_relative_eq!(oob.perimeter, 0.0, epsilon = 0.0);
    assert!(!oob.is_closed(), "z=100 is_closed() should be false");
    // Plane origin still echoes the input.
    assert_relative_eq!(oob.plane_origin.z, OOB_Z, epsilon = VERTEX_TOL);
    // plane_normal is still normalized (input was unit Vector3::z()).
    assert_relative_eq!(oob.plane_normal.norm(), 1.0, epsilon = NORMAL_MAG_TOL);
}

// =============================================================================
// verify_plane_normal_normalization
// =============================================================================

/// Caller passes `Vector3::new(0, 0, 2)` (magnitude 2); `cross_section`
/// normalizes internally so the output `plane_normal` has unit
/// magnitude AND the slice geometry matches the explicit-unit call
/// (slice plane is the same).
fn verify_plane_normal_normalization(mesh: &IndexedMesh, mid: &CrossSection) {
    let unnorm = cross_section(
        mesh,
        Point3::new(0.0, 0.0, 5.0),
        Vector3::new(0.0, 0.0, 2.0),
    );
    assert_relative_eq!(unnorm.plane_normal.norm(), 1.0, epsilon = NORMAL_MAG_TOL);
    assert_relative_eq!(unnorm.plane_normal.z, 1.0, epsilon = NORMAL_MAG_TOL);
    // Same slice → same area + perimeter as the unit-input call.
    assert_relative_eq!(unnorm.area, mid.area, epsilon = SHOELACE_TOL);
    assert_relative_eq!(unnorm.perimeter, mid.perimeter, epsilon = SHOELACE_TOL);
}

// =============================================================================
// print_summary — museum-plaque stdout
// =============================================================================

/// Print the human-readable summary: input fixture, mid-slice metrics,
/// slice-stack values, helper-call equivalence, out-of-mesh + un-normalized
/// plane-normal demos. Extracted from `main` to keep the entrypoint
/// under clippy's `too_many_lines` cap.
fn print_summary(mesh: &IndexedMesh, mid: &CrossSection) {
    println!("==== mesh-measure-cross-section ====");
    println!();
    println!(
        "input  : {}-vertex, {}-triangle 32-segment closed cylinder",
        mesh.vertices.len(),
        mesh.faces.len(),
    );
    println!(
        "         radius {CYLINDER_RADIUS} mm, height {CYLINDER_HEIGHT} mm, axis +Z, \
         caps closed (fan tris from cap centers)"
    );
    println!();

    let analytical_area = std::f64::consts::PI * CYLINDER_RADIUS * CYLINDER_RADIUS;
    let analytical_perimeter = std::f64::consts::TAU * CYLINDER_RADIUS;
    println!("Mid-slice (z=5) cross_section:");
    println!(
        "  area      = {:.6} mm²  (polygon shoelace; analytical π·r² = {:.6}, \
         {:.2}% chord-shrinkage)",
        mid.area,
        analytical_area,
        (1.0 - mid.area / analytical_area) * 100.0,
    );
    println!(
        "  perimeter = {:.6} mm    (32 chords; analytical 2πr = {:.6}, \
         {:.2}% chord-shrinkage)",
        mid.perimeter,
        analytical_perimeter,
        (1.0 - mid.perimeter / analytical_perimeter) * 100.0,
    );
    println!(
        "  centroid  = ({:.6}, {:.6}, {:.6})   ← biased by V_dup/65 vs true \
         (0, 0, 5); see spec §10 item 11",
        mid.centroid.x, mid.centroid.y, mid.centroid.z,
    );
    println!(
        "  contour_count = {}, is_closed() = {}, |plane_normal| = {:.15}",
        mid.contour_count,
        mid.is_closed(),
        mid.plane_normal.norm(),
    );
    println!();

    let stack = cross_sections(
        mesh,
        Point3::new(0.0, 0.0, STACK_START_Z),
        Vector3::z(),
        STACK_COUNT,
        STACK_SPACING,
    );
    println!(
        "Slice stack (cross_sections, start={STACK_START_Z}, normal=+Z, \
         count={STACK_COUNT}, spacing={STACK_SPACING}):"
    );
    println!(
        "  z values  : {:?}",
        stack.iter().map(|s| s.plane_origin.z).collect::<Vec<_>>(),
    );
    println!(
        "  all areas = {:.6} mm² (z-uniform cylinder)",
        stack[0].area
    );
    println!();

    println!("Convenience helpers:");
    println!(
        "  circumference_at_height(_, 5.0) = {:.6} (matches mid-slice perimeter)",
        circumference_at_height(mesh, 5.0),
    );
    println!(
        "  area_at_height(_, 5.0)          = {:.6} (matches mid-slice area)",
        area_at_height(mesh, 5.0),
    );
    println!();

    let oob = cross_section(mesh, Point3::new(0.0, 0.0, OOB_Z), Vector3::z());
    println!(
        "Out-of-mesh slice at z={OOB_Z}: is_empty={}, contour_count={}, area={}",
        oob.is_empty(),
        oob.contour_count,
        oob.area,
    );

    let unnorm = cross_section(
        mesh,
        Point3::new(0.0, 0.0, 5.0),
        Vector3::new(0.0, 0.0, 2.0),
    );
    println!(
        "Un-normalized plane_normal Vector3::new(0, 0, 2) → output \
         |plane_normal| = {:.15}",
        unnorm.plane_normal.norm(),
    );
    println!();
}

// =============================================================================
// main
// =============================================================================

fn main() -> Result<()> {
    let mesh = build_fixture();
    verify_fixture_geometry(&mesh);

    let mid = cross_section(&mesh, Point3::new(0.0, 0.0, 5.0), Vector3::z());
    verify_mid_slice(&mid);
    verify_slice_stack(&mesh);
    verify_helpers(&mesh, &mid);
    verify_out_of_mesh(&mesh);
    verify_plane_normal_normalization(&mesh, &mid);

    print_summary(&mesh, &mid);

    let out_dir = Path::new(env!("CARGO_MANIFEST_DIR")).join("out");
    std::fs::create_dir_all(&out_dir)?;
    let mesh_path = out_dir.join("cylinder.ply");
    save_ply(&mesh, &mesh_path, false)?;
    println!(
        "artifact: out/cylinder.ply ({}v, {}f, ASCII)",
        mesh.vertices.len(),
        mesh.faces.len(),
    );
    println!();
    println!(
        "OK — 66 vertex + 128 winding + mid-slice + slice-stack + helpers + \
         out-of-mesh + plane-normal-normalization anchors all green"
    );

    Ok(())
}
