//! SDF-based shell generation (`.high_quality()` preset) around a closed
//! cube.
//!
//! Builds a closed 10mm cube (8 verts, 6 walls × 2 = 12 triangles
//! with the same diagonal-triangulation pattern as the open-topped
//! box used in [`crate::fast`]), runs it through
//! `mesh_shell::ShellBuilder` with the `.high_quality()` preset, and
//! saves two PLY artifacts:
//!
//!   * `out/hq_before.ply` — input closed cube
//!   * `out/hq_shell.ply`  — generated shell with 2mm walls; outer
//!     surface is the SDF level set at distance 2.0mm from the inner
//!     cube, marched into a triangle mesh + welded + per-face flipped
//!     by the engine fix that landed in commits 11.5.1 + 11.5.2.
//!
//! Three pedagogical points the module anchors:
//!
//!   1. **The SDF level set produces UNIFORM perpendicular wall
//!      thickness regardless of input triangulation, modulo
//!      half-voxel MC discretization noise.** This is the load-bearing
//!      contrast vs [`crate::fast`], where the same
//!      diagonal-triangulation pattern produced 0.667mm thinnest
//!      perpendicular wall at vert 2 (33% of the parameterized 2.0mm).
//!      Wall thickness is measured via brute-force closest-point on an
//!      SDF of the outer-only mesh, sampled at 24 wall-interior points
//!      (4 per wall × 6 walls); every sample lands within
//!      `WALL_TOLERANCE_MM` of `WALL_THICKNESS_MM`.
//!   2. **The cost paid for uniform thickness is a marching-cubes outer
//!      surface (welded, no 1:1 vertex correspondence) with cell-scale
//!      chamfering at sharp creases.** The outer surface comes from
//!      `mesh-offset`'s marching cubes. The cube's 8 VERTICES round into
//!      Steiner-Minkowski sphere octants of radius `wall_thickness`; the
//!      cube's 12 EDGES round into cylindrical fillets of the same
//!      radius. This is GENUINE level-set curvature, not MC
//!      discretization artifact (compare the inward case in
//!      `offset/stress-test`, where the level set has sharp corners and
//!      MC chamfers them as a tessellation artifact).
//!   3. **`has_consistent_winding == true` on the SDF shell.** A closed
//!      input has no boundary, so no rim is generated and there is no
//!      inner/outer/rim seam to reconcile; the outer MC surface is
//!      per-face flipped by the 11.5.2 fix and the inner surface is
//!      winding-reversed, giving one consistently-oriented manifold.
//!      (The normal-method [`crate::fast`] companion DOES generate a rim
//!      on its open input and is likewise consistently wound, via the
//!      separate rim-winding fix.)
//!
//! Wall thickness uniformity is the load-bearing assertion: 24
//! sample points span the 6 walls (4 per wall, displaced 3mm from
//! every cube edge so they're well clear of the 2mm-radius fillet
//! zone), and the distance from each sample to the closest outer-mesh
//! point lands in
//! `[WALL_THICKNESS_MM - WALL_TOLERANCE_MM, WALL_THICKNESS_MM +
//! WALL_TOLERANCE_MM]`. Tolerance is set to half the voxel size +
//! cushion (0.2mm at 0.3mm voxel) to admit the cell-scale MC
//! tessellation noise without admitting genuine level-set curvature.
//!
//! See sibling [`crate::fast`] for the normal-method companion
//! (1-to-1 vertex correspondence; triangulation-skewed perpendicular
//! thickness).

// Mesh processing uses u32 indices throughout; truncating a usize
// vertex count to u32 would only matter for meshes with >4B vertices
// (impossible at 64 GB+ memory). Mirrors the same allow in
// `mesh/mesh-io/src/ply.rs`.
#![allow(clippy::cast_possible_truncation)]
// Vert/face ratio printout casts usize counts to f64 — meshes with
// >2^52 verts exceed practical limits, so the precision loss is
// acceptable. Mirrors `mesh/mesh-measure/src/cross_section.rs`.
#![allow(clippy::cast_precision_loss)]

use std::path::Path;

use anyhow::{Result, anyhow};
use mesh_io::save_ply;
use mesh_measure::cross_section;
use mesh_repair::validate_mesh;
use mesh_sdf::{TriMeshDistance, UnsignedDistance};
use mesh_shell::{ShellBuildResult, ShellBuilder, WallGenerationMethod};
use mesh_types::{Bounded, IndexedMesh, Point3, Vector3};

use crate::common::{print_diagnostics, print_shell_stats, verify_round_trip};

/// Side length of the input closed cube, in mesh units. The
/// `mesh-shell` API uses mm naming throughout; meshes carry no
/// inherent unit, so we treat these as mm. Same value as
/// [`crate::fast`] so the input topology is directly comparable
/// (verts 0–7 + diagonal triangulation match exactly, plus 2 top
/// faces to close the cube).
const SIDE_MM: f64 = 10.0;

/// Wall thickness in mm. Same value as [`crate::fast`] so the
/// per-vertex-offset table from the contrast README applies directly:
/// where the normal method measured 0.667mm thinnest perpendicular at
/// vert 2 (33% thinning), the SDF method measures `~2.0mm` uniformly at
/// every wall-interior sample.
const WALL_THICKNESS_MM: f64 = 2.0;

/// Voxel size for the SDF marching-cubes pass, in mm. Matches the
/// `.high_quality()` preset's default. Set explicitly via
/// `.voxel_size(VOXEL_MM)` after `.high_quality()` so the module
/// reads as self-documenting.
const VOXEL_MM: f64 = 0.3;

/// Tolerance for the wall-thickness uniformity assertion. Half the
/// voxel size = 0.15mm (the worst-case MC vertex displacement);
/// 0.2mm gives cushion for face-interior interpolation drift.
/// Tighter than admitting genuine level-set curvature (which would
/// need >> `wall_thickness` near corners).
const WALL_TOLERANCE_MM: f64 = 0.2;

pub fn run() -> Result<()> {
    let before = closed_cube(SIDE_MM);

    // .high_quality() sets `wall_method = Sdf`, `sdf_voxel_size_mm =
    // 0.3`, AND `validate = true`. The chained `.voxel_size(VOXEL_MM)`
    // re-asserts the voxel size after the preset (idempotent here at
    // 0.3, but documents intent so future changes to the preset
    // default surface as a one-line diff).
    let result = ShellBuilder::new(&before)
        .wall_thickness(WALL_THICKNESS_MM)
        .high_quality()
        .voxel_size(VOXEL_MM)
        .build()
        .map_err(|e| anyhow!("shell build failed: {e}"))?;
    let shell = &result.mesh;

    let out_dir = Path::new(env!("CARGO_MANIFEST_DIR")).join("out");
    std::fs::create_dir_all(&out_dir)?;
    let before_path = out_dir.join("hq_before.ply");
    let shell_path = out_dir.join("hq_shell.ply");
    save_ply(&before, &before_path, true)?;
    save_ply(shell, &shell_path, true)?;

    println!("==== shell-generation-high-quality (SDF method) ====");
    println!();
    println!("input  : closed {SIDE_MM:.0}mm cube (6 walls × 2 = 12 faces, no boundary)");
    println!(
        "config : ShellBuilder::new(&before).wall_thickness({WALL_THICKNESS_MM}).high_quality().voxel_size({VOXEL_MM})"
    );
    println!();

    print_diagnostics("before  (closed cube)              ", &before);
    println!();
    print_diagnostics("shell   (SDF-based, 2mm walls)     ", shell);
    println!();
    print_shell_stats(&result);
    println!();
    print_cross_section(shell, SIDE_MM / 2.0);
    println!();

    verify_before(&before);
    verify_shell(&before, &result)?;
    verify_round_trip("hq before", &before, &before_path)?;
    verify_round_trip("hq shell", shell, &shell_path)?;

    println!("OK — SDF-based shell built, validated, and uniform-thickness within tolerance");
    println!();

    Ok(())
}

/// Construct an axis-aligned closed cube of given side length,
/// anchored at the origin. Vertex layout matches
/// [`crate::fast`]'s `open_topped_box` exactly for verts 0–7; faces
/// are the same 10 wall-triangle faces (5 walls × 2 tris) plus 2
/// top-wall triangles `[4,5,6]` and `[4,6,7]` to close the cube. 12
/// triangle faces total (6 walls × 2 each), all outward-wound.
fn closed_cube(side: f64) -> IndexedMesh {
    let mut mesh = IndexedMesh::new();
    // 8 vertices at the cube corners (same indexing as the open box).
    mesh.vertices.push(Point3::new(0.0, 0.0, 0.0));
    mesh.vertices.push(Point3::new(side, 0.0, 0.0));
    mesh.vertices.push(Point3::new(side, side, 0.0));
    mesh.vertices.push(Point3::new(0.0, side, 0.0));
    mesh.vertices.push(Point3::new(0.0, 0.0, side));
    mesh.vertices.push(Point3::new(side, 0.0, side));
    mesh.vertices.push(Point3::new(side, side, side));
    mesh.vertices.push(Point3::new(0.0, side, side));

    // 12 faces — 6 walls × 2 triangles each, outward winding.
    // Bottom (-z)
    mesh.faces.push([0, 2, 1]);
    mesh.faces.push([0, 3, 2]);
    // Top (+z)  — closes the open-box's missing top face
    mesh.faces.push([4, 5, 6]);
    mesh.faces.push([4, 6, 7]);
    // Front (-y)
    mesh.faces.push([0, 1, 5]);
    mesh.faces.push([0, 5, 4]);
    // Back (+y)
    mesh.faces.push([2, 3, 7]);
    mesh.faces.push([2, 7, 6]);
    // Left (-x)
    mesh.faces.push([0, 4, 7]);
    mesh.faces.push([0, 7, 3]);
    // Right (+x)
    mesh.faces.push([1, 2, 6]);
    mesh.faces.push([1, 6, 5]);

    mesh
}

/// Print a cross-section of the shell at z = `z_mm` (mid-height of
/// the cube by default). For the closed-cube SDF shell, this slice
/// captures both rings — the inner 10×10 cube ring and the outer
/// 14×14 level-set ring with 2mm-radius rounded corners (no
/// top/bottom rounding zones at mid-height; the rounding is purely
/// from 2D Steiner-Minkowski applied to the cross-section square).
/// `contour_count` should be 2 (one ring per surface). Perimeter sum
/// is `8 × side + 2π × wall_thickness ≈ 92.57mm` (analytical;
/// observed within MC discretization). The perimeter difference
/// between outer and inner = `2π × wall_thickness ≈ 12.57mm` —
/// 8 straight-segment displacements contribute 0 net (they stay
/// the same length on the offset), and the 4 quarter-circle arcs
/// of radius `wall_thickness` total `2π × wall_thickness`.
fn print_cross_section(shell: &IndexedMesh, z_mm: f64) {
    let section = cross_section(shell, Point3::new(0.0, 0.0, z_mm), Vector3::z());
    println!("Cross-section at z = {z_mm:.1}mm:");
    println!("  contour_count  : {}", section.contour_count);
    println!("  perimeter (sum): {:.4} mm", section.perimeter);
    println!("  area           : {:.4} mm²", section.area);
    println!(
        "  bounds         : ({:+.3}, {:+.3}, {:+.3}) → ({:+.3}, {:+.3}, {:+.3})",
        section.bounds.0.x,
        section.bounds.0.y,
        section.bounds.0.z,
        section.bounds.1.x,
        section.bounds.1.y,
        section.bounds.1.z,
    );
}

/// Pre-build: closed cube has `signed_volume` exactly 1000mm³ (= side³),
/// is watertight + manifold + outward-wound, and has no boundary
/// edges. The exact volume comes from divergence-theorem integration
/// over outward-wound faces; the closed cube has no half-integral
/// open-mesh quirk. Drift catcher for the input shape: any change to
/// `closed_cube` topology or winding flips this.
fn verify_before(mesh: &IndexedMesh) {
    let report = validate_mesh(mesh);
    assert_eq!(report.vertex_count, 8);
    assert_eq!(report.face_count, 12);
    assert!(report.is_manifold);
    assert!(report.is_watertight, "closed cube has no boundary edges");
    assert_eq!(report.boundary_edge_count, 0);
    assert_eq!(report.non_manifold_edge_count, 0);
    assert!(!report.is_inside_out, "outward winding ⇒ signed_volume > 0");

    let want_vol = SIDE_MM.powi(3);
    let got_vol = mesh.signed_volume();
    assert!(
        (got_vol - want_vol).abs() < 1e-9,
        "closed cube signed_volume should be exactly {want_vol}; got {got_vol}",
    );

    let aabb = mesh.aabb();
    assert!(aabb.min.x.abs() < 1e-12 && aabb.min.y.abs() < 1e-12 && aabb.min.z.abs() < 1e-12);
    assert!(
        (aabb.max.x - SIDE_MM).abs() < 1e-12
            && (aabb.max.y - SIDE_MM).abs() < 1e-12
            && (aabb.max.z - SIDE_MM).abs() < 1e-12,
    );
}

/// Post-build: the shell anchors FIVE concerns, delegated to
/// dedicated helpers:
///
/// 1. **Topological structure** (`verify_topology`) —
///    `wall_method == Sdf`, `boundary_size == 0`, `rim_face_count ==
///    0` (closed input → no rim), `outer_vertex_count >>
///    inner_vertex_count` (MC re-triangulation), vert/face ratio
///    well below the soup-signature 3.0 (welded by 11.5.2 fix).
/// 2. **Validation surface** (`verify_validation`) —
///    `is_watertight && is_manifold && is_printable() &&
///    has_consistent_winding`, issues list empty.
/// 3. **Properly wound in face-normal sense** (`verify_winding`) —
///    `signed_volume > 0`, `is_inside_out == false`. The 11.5.2 fix
///    flips MC's inside-out winding via per-face `face.swap(1, 2)`
///    before assembling the shell.
/// 4. **Uniform perpendicular wall thickness** (`verify_wall_uniformity`) —
///    24 wall-interior sample points, distance to outer mesh in
///    `[WALL_THICKNESS_MM - WALL_TOLERANCE_MM, WALL_THICKNESS_MM +
///    WALL_TOLERANCE_MM]` for every sample. THE primary anchor.
/// 5. **PLY round-trip** (`verify_round_trip`, called from run) —
///    vertex/face counts survive the I/O boundary.
fn verify_shell(before: &IndexedMesh, result: &ShellBuildResult) -> Result<()> {
    let shell = &result.mesh;
    let stats = &result.shell_stats;

    verify_topology(shell, stats, result.offset_applied, before);
    verify_validation(stats)?;
    verify_winding(shell);
    verify_wall_uniformity(shell, stats, before)?;

    Ok(())
}

/// Topological structure of the SDF shell. Closed input has NO
/// boundary edges, so the rim-stitching short-circuits (per
/// `generate_rim_for_sdf_shell`) and `rim_face_count == 0`. Outer is
/// MC-derived + welded (per the 11.5.2 fix); the vert/face ratio
/// signature of welded vs soup is `~0.5` vs `3.0` — the assertion
/// uses a generous `< 1.0` threshold to admit MC mesh-quality
/// variation while still catching any future regression that
/// removes the weld pass.
fn verify_topology(
    shell: &IndexedMesh,
    stats: &mesh_shell::ShellGenerationResult,
    offset_applied: bool,
    before: &IndexedMesh,
) {
    let n_inner_verts = before.vertices.len();
    let n_inner_faces = before.faces.len();

    assert_eq!(stats.inner_vertex_count, n_inner_verts);
    assert_eq!(stats.wall_method, WallGenerationMethod::Sdf);
    assert_eq!(
        stats.boundary_size, 0,
        "closed input has no boundary loops → SDF rim short-circuits",
    );
    assert_eq!(
        stats.rim_face_count, 0,
        "closed input has no rim ⇒ rim_face_count == 0",
    );

    // Outer is MC-derived; vertex/face counts depend on voxel size.
    // Anchor the topological signature: outer is MUCH bigger than
    // input, vert/face ratio is welded-not-soup.
    assert!(
        stats.outer_vertex_count > 10 * n_inner_verts,
        "outer should be MC-re-triangulated (>> 10 × inner verts); got {} vs {n_inner_verts}",
        stats.outer_vertex_count,
    );
    let outer_face_count = stats.total_face_count - n_inner_faces - stats.rim_face_count;
    let vert_face_ratio = stats.outer_vertex_count as f64 / outer_face_count.max(1) as f64;
    assert!(
        vert_face_ratio < 1.0,
        "outer vert/face ratio < 1.0 (welded; soup would be 3.0); got {vert_face_ratio:.3}",
    );

    // Shell mesh: inner verts + outer verts; inner faces + outer faces.
    assert_eq!(
        shell.vertices.len(),
        n_inner_verts + stats.outer_vertex_count,
    );
    assert_eq!(shell.faces.len(), stats.total_face_count);
    assert_eq!(
        stats.total_face_count,
        n_inner_faces + outer_face_count,
        "no rim ⇒ total = inner + outer",
    );

    assert!(
        !offset_applied,
        "no .offset(...) call ⇒ offset_applied is false",
    );
}

/// Validation surface for the SDF shell on closed input.
/// `is_watertight && is_manifold && is_printable()`, with
/// `has_consistent_winding == true` (closed input → no rim → no
/// seam to reconcile). Issues list is empty.
fn verify_validation(stats: &mesh_shell::ShellGenerationResult) -> Result<()> {
    let validation = stats
        .validation
        .as_ref()
        .ok_or_else(|| anyhow!(".high_quality() should populate stats.validation"))?;
    assert!(
        validation.is_watertight,
        "SDF shell on closed input is watertight"
    );
    assert!(validation.is_manifold);
    assert!(validation.is_printable());
    assert_eq!(validation.boundary_edge_count, 0);
    assert_eq!(validation.non_manifold_edge_count, 0);
    assert!(
        validation.has_consistent_winding,
        "closed-input SDF shell has no rim ⇒ no edge-direction conflict",
    );
    assert!(
        validation.issues.is_empty(),
        "expected no shell issues; got: {:?}",
        validation.issues,
    );
    Ok(())
}

/// Face-normal winding of the assembled shell. Outer faces are
/// per-face flipped by the 11.5.2 fix (`face.swap(1, 2)` on every
/// MC-emitted outer face) so the face-normal divergence integral
/// yields a positive `signed_volume`. Inner faces are
/// reversed-from-input by `generate_shell_sdf` so cavity normals
/// point inward (= outward from wall material).
fn verify_winding(shell: &IndexedMesh) {
    let report = validate_mesh(shell);
    assert!(
        !report.is_inside_out,
        "shell should be properly wound (no inside-out winding); got is_inside_out=true",
    );
    let vol = shell.signed_volume();
    assert!(
        vol > 0.0,
        "shell wall signed_volume should be positive; got {vol}",
    );
}

/// Uniform perpendicular wall thickness. THE primary anchor of the
/// SDF method. Sample 24 wall-interior points (4 per wall × 6 walls,
/// at `(3, 3)`, `(7, 3)`, `(3, 7)`, `(7, 7)` in each wall's local
/// 2D coords; displaced 3mm from every cube edge so they're well
/// clear of the 2mm-radius corner/edge fillet zone). For each
/// sample, query the SDF of the OUTER-only mesh and confirm
/// distance ≈ `WALL_THICKNESS_MM` within `WALL_TOLERANCE_MM`.
///
/// The level-set math says: at a wall-interior point ≥ `wall_thickness`
/// away from any cube edge, the closest level-set point is
/// perpendicular to the wall plane at distance exactly
/// `wall_thickness`. The MC tessellation discretizes the level set
/// at voxel resolution, so the measured distance lands in
/// `[wall_thickness - half_voxel, wall_thickness + half_voxel]`.
/// Tolerance 0.2mm at 0.3mm voxel admits the discretization noise
/// without admitting genuine level-set curvature near corners.
///
/// Builds an outer-only mesh by extracting the second half of the
/// shell's vertex array + the slice of `shell.faces` after the
/// `n_inner_faces` reversed inner faces. Brute-force closest-point
/// over ~26k MC outer faces is sub-second in release mode.
fn verify_wall_uniformity(
    shell: &IndexedMesh,
    stats: &mesh_shell::ShellGenerationResult,
    before: &IndexedMesh,
) -> Result<()> {
    let outer_only = extract_outer_only(shell, stats.inner_vertex_count, before.faces.len());
    // Post-D arc: TriMeshDistance is the bare unsigned-distance oracle;
    // wall-uniformity only needs `.distance()` (sign-irrelevant), so
    // skip composing a Signed and avoid the flood-fill grid build.
    let sdf = TriMeshDistance::new(outer_only)
        .map_err(|e| anyhow!("TriMeshDistance construction failed on outer-only mesh: {e}"))?;

    let samples = wall_interior_samples(SIDE_MM);
    assert_eq!(samples.len(), 24, "24 wall-interior samples expected");

    let mut min_dist = f64::INFINITY;
    let mut max_dist = f64::NEG_INFINITY;
    for sample in &samples {
        let dist = sdf.distance(*sample);
        min_dist = min_dist.min(dist);
        max_dist = max_dist.max(dist);
        let drift = (dist - WALL_THICKNESS_MM).abs();
        assert!(
            drift < WALL_TOLERANCE_MM,
            "sample {sample:?}: distance-to-outer = {dist:.4}, drift from wall_thickness = \
             {drift:.4}mm exceeds tolerance {WALL_TOLERANCE_MM}mm",
        );
    }

    let drift_below = (WALL_THICKNESS_MM - min_dist).abs();
    let drift_above = (max_dist - WALL_THICKNESS_MM).abs();
    let max_drift = drift_below.max(drift_above);
    println!("wall thickness uniformity (24 wall-interior samples):");
    println!("  expected       : {WALL_THICKNESS_MM:.4} mm (uniform)");
    println!("  min measured   : {min_dist:.4} mm");
    println!("  max measured   : {max_dist:.4} mm");
    println!("  max |drift|    : {max_drift:.4} mm (tolerance {WALL_TOLERANCE_MM}mm)");
    Ok(())
}

/// Build an outer-only `IndexedMesh` by slicing the assembled shell.
/// The shell's vertex array is `[inner_verts..., outer_verts...]`
/// and the face array is `[reversed_inner_faces..., outer_faces...,
/// rim_faces...]` (with rim empty for closed input). Outer faces use
/// `inner_vertex_count`-offset indices; subtract that offset to get a
/// freestanding mesh suitable for SDF construction.
fn extract_outer_only(
    shell: &IndexedMesh,
    inner_vertex_count: usize,
    n_inner_faces: usize,
) -> IndexedMesh {
    let mut outer = IndexedMesh::new();
    let inner_count_u32 = inner_vertex_count as u32;
    outer
        .vertices
        .extend_from_slice(&shell.vertices[inner_vertex_count..]);
    // rim_face_count is 0 for closed input, so all faces past the
    // first n_inner_faces are outer faces.
    for face in &shell.faces[n_inner_faces..] {
        outer.faces.push([
            face[0] - inner_count_u32,
            face[1] - inner_count_u32,
            face[2] - inner_count_u32,
        ]);
    }
    outer
}

/// 24 wall-interior sample points (4 per wall × 6 walls). Each
/// sample is at `(3, 3)`, `(7, 3)`, `(3, 7)`, `(7, 7)` in the wall's
/// local 2D coords (the two coordinates spanning the wall plane).
/// All samples are 3mm from every cube edge — well clear of the
/// 2mm-radius fillet zone — so each sample's closest level-set
/// point is perpendicular to the wall plane at distance
/// exactly `wall_thickness` (modulo MC tessellation noise).
fn wall_interior_samples(side: f64) -> Vec<Point3<f64>> {
    let mut samples = Vec::with_capacity(24);
    let coords = [3.0, 7.0]; // 3mm from each wall edge; 4mm apart
    // Bottom wall (z = 0): vary x, y
    for &x in &coords {
        for &y in &coords {
            samples.push(Point3::new(x, y, 0.0));
        }
    }
    // Top wall (z = side): vary x, y
    for &x in &coords {
        for &y in &coords {
            samples.push(Point3::new(x, y, side));
        }
    }
    // Front wall (y = 0): vary x, z
    for &x in &coords {
        for &z in &coords {
            samples.push(Point3::new(x, 0.0, z));
        }
    }
    // Back wall (y = side): vary x, z
    for &x in &coords {
        for &z in &coords {
            samples.push(Point3::new(x, side, z));
        }
    }
    // Left wall (x = 0): vary y, z
    for &y in &coords {
        for &z in &coords {
            samples.push(Point3::new(0.0, y, z));
        }
    }
    // Right wall (x = side): vary y, z
    for &y in &coords {
        for &z in &coords {
            samples.push(Point3::new(side, y, z));
        }
    }
    samples
}
