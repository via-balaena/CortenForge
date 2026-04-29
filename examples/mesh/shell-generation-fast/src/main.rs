//! Normal-based shell generation around an open-topped box.
//!
//! Builds a 10mm cube with the top face missing (5 walls, 10 faces,
//! 4-edge top boundary loop), runs it through `mesh_shell::ShellBuilder`
//! with the `.fast()` preset (validation re-enabled via the
//! `.validate(true)` override), and saves two PLY artifacts:
//!
//!   * `out/before.ply` — input open-topped box
//!   * `out/shell.ply` — generated shell with 2mm walls + rim closing
//!     the top opening
//!
//! Three pedagogical points the example anchors:
//!
//!   1. **Normal-based shell preserves vertex correspondence.**
//!      `generate_shell_normal` duplicates each input vertex (one
//!      inner copy at original position, one outer copy at
//!      `vertex + averaged_unit_normal × wall_thickness`). Vertex
//!      `i` in the inner surface and vertex `i + n` in the outer
//!      surface are 1-to-1; the offset vector
//!      `outer[i] - inner[i]` has magnitude exactly `wall_thickness`
//!      for every input vertex. This is the topological signature
//!      of the `.fast()` preset and the load-bearing distinction
//!      from the SDF-based `.high_quality()` companion (commit 12),
//!      which routes the outer surface through marching cubes and
//!      breaks the correspondence.
//!   2. **Wall thickness perpendicular to faces depends on
//!      triangulation.** The averaged unit normal at each vertex
//!      is `normalize(Σ unit_face_normal_at_v)` — incident faces
//!      contribute equally per face, NOT equally per wall, so a
//!      vertex on a wall's diagonal (incident to BOTH triangles of
//!      that wall) gets twice that wall's normal weight as an
//!      off-diagonal vertex. The simple formula `wall_thickness/√k`
//!      (where `k` is the count of incident perpendicular walls)
//!      applies ONLY when triangle counts per incident wall are
//!      equal — true at verts 0, 3, 4, 7 in this triangulation
//!      (giving `2.0/√3 ≈ 1.155mm` and `2.0/√2 ≈ 1.414mm`
//!      respectively) but NOT at verts 1, 2, 5, 6 where the
//!      counts differ.  The example anchors all 8 vertices' offset
//!      vectors as a closed-form table derived from incident
//!      triangle counts (see `verify_offsets`).
//!   3. **The rim closes the open boundary into a watertight,
//!      manifold, printable shell — but `mesh-shell` 0.7.x ships
//!      with a rim-winding inconsistency.** Input has 4 boundary
//!      edges (top perimeter); `generate_rim` emits 2 triangles
//!      per boundary edge bridging inner-top to outer-top ⇒ 8 rim
//!      faces. Result is watertight + manifold + printable. BUT
//!      `has_consistent_winding == false` with one
//!      `InconsistentWinding` issue: the rim quads use the
//!      original-winding boundary direction, conflicting with the
//!      reversed-winding inner-shell faces at the seam. The shell
//!      still bounds a volume correctly (`signed_volume > 0`,
//!      `is_inside_out == false`); the inconsistency is in
//!      edge-traversal direction, not in face-normal direction.
//!      Tracked as a fixable platform issue per
//!      `feedback_improve_mesh_crate`.
//!
//! See `examples/mesh/README.md` for cadence; see
//! `docs/studies/mesh_architecture/src/50-shell-and-print.md` for the
//! depth pass on the printable-shell pipeline. See sibling
//! `examples/mesh/shell-generation-high-quality/` for the SDF-based
//! companion (uniform wall thickness; chamfered MC artifacts at
//! sharp creases).

use std::path::Path;

use anyhow::{Result, anyhow};
use mesh_io::{load_ply, save_ply};
use mesh_repair::validate_mesh;
use mesh_shell::{ShellBuildResult, ShellBuilder, WallGenerationMethod};
use mesh_types::{Bounded, IndexedMesh, Point3};

/// Side length of the input open-topped box, in mesh units. The
/// `mesh-shell` API uses mm naming throughout (`wall_thickness_mm`,
/// `voxel_size_mm`); meshes carry no inherent unit, so we treat
/// these mesh units as mm for narrative clarity. Matches
/// `mesh_shell::shell::generate::tests::create_open_box` exactly so
/// the example anchors what the crate's own test fixtures cover.
const SIDE_MM: f64 = 10.0;

/// Wall thickness in mm. Matches the crate's `test_build_simple`
/// fixture exactly. 20% wall ratio at 10mm side is heavy by
/// 3D-print standards (typical FDM walls are 1-3mm at 50-100mm
/// part scale ⇒ 1-5%) but is visually unmissable in any viewer
/// and gives clean closed-form numerical anchors at the corners
/// (see `verify_shell`).
const WALL_THICKNESS_MM: f64 = 2.0;

fn main() -> Result<()> {
    let before = open_topped_box(SIDE_MM);

    // .fast() sets `wall_method = Normal` AND `validate = false`.
    // The chained `.validate(true)` AFTER `.fast()` re-enables
    // post-build validation (see API surface section in README).
    // Order matters: `.validate(true).fast()` would be wiped back
    // to false because `.fast()` resets `validate` unconditionally.
    let result = ShellBuilder::new(&before)
        .wall_thickness(WALL_THICKNESS_MM)
        .fast()
        .validate(true)
        .build()
        .map_err(|e| anyhow!("shell build failed: {e}"))?;
    let shell = result.mesh.clone();

    let out_dir = Path::new(env!("CARGO_MANIFEST_DIR")).join("out");
    std::fs::create_dir_all(&out_dir)?;
    let before_path = out_dir.join("before.ply");
    let shell_path = out_dir.join("shell.ply");
    save_ply(&before, &before_path, true)?;
    save_ply(&shell, &shell_path, true)?;

    println!("==== shell-generation-fast ====");
    println!();
    println!("input  : open-topped {SIDE_MM:.0}mm box (5 walls, 10 faces, 4-edge top boundary)");
    println!(
        "config : ShellBuilder::new(&before).wall_thickness({WALL_THICKNESS_MM}).fast().validate(true)"
    );
    println!();

    print_diagnostics("before  (open-topped box)             ", &before);
    println!();
    print_diagnostics("shell   (normal-based, 2mm walls)     ", &shell);
    println!();
    print_shell_stats(&result);
    println!();

    verify_before(&before);
    verify_shell(&before, &result)?;

    let before_loaded = load_ply(&before_path)?;
    let shell_loaded = load_ply(&shell_path)?;
    verify_round_trip(&[
        ("before", &before, &before_loaded),
        ("shell", &shell, &shell_loaded),
    ]);

    println!("artifacts:");
    println!(
        "  out/before.ply : {}v, {}f (round-trip verified)",
        before_loaded.vertices.len(),
        before_loaded.faces.len(),
    );
    println!(
        "  out/shell.ply  : {}v, {}f (round-trip verified)",
        shell_loaded.vertices.len(),
        shell_loaded.faces.len(),
    );
    println!();
    println!("OK — normal-based shell built, validated, and printable");

    Ok(())
}

/// Construct an axis-aligned open-topped box of given side length,
/// anchored at the origin. Verbatim shape from
/// `mesh_shell::shell::generate::tests::create_open_box` scaled by
/// `side / 10.0`: 8 verts at the cube corners + 5 walls × 2
/// triangles each = 10 faces with outward winding. Top face
/// missing, so the 4 top edges form a single boundary loop.
fn open_topped_box(side: f64) -> IndexedMesh {
    let mut mesh = IndexedMesh::new();
    // 8 vertices at the cube corners.
    mesh.vertices.push(Point3::new(0.0, 0.0, 0.0));
    mesh.vertices.push(Point3::new(side, 0.0, 0.0));
    mesh.vertices.push(Point3::new(side, side, 0.0));
    mesh.vertices.push(Point3::new(0.0, side, 0.0));
    mesh.vertices.push(Point3::new(0.0, 0.0, side));
    mesh.vertices.push(Point3::new(side, 0.0, side));
    mesh.vertices.push(Point3::new(side, side, side));
    mesh.vertices.push(Point3::new(0.0, side, side));

    // 10 faces — 5 walls × 2 triangles each, outward winding.
    // Bottom (-z)
    mesh.faces.push([0, 2, 1]);
    mesh.faces.push([0, 3, 2]);
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
    // Top is OPEN — the 4 top edges (between verts {4,5,6,7})
    // form the boundary loop the rim will close.

    mesh
}

/// Print a labeled diagnostic block: counts, AABB, signed volume,
/// surface area, and the topology flags from `validate_mesh`
/// (`is_manifold`, `is_watertight`, `is_inside_out`,
/// `boundary_edge_count`).
fn print_diagnostics(label: &str, mesh: &IndexedMesh) {
    let report = validate_mesh(mesh);
    let aabb = mesh.aabb();
    println!("{label}:");
    println!("  vertices       : {}", report.vertex_count);
    println!("  faces          : {}", report.face_count);
    println!(
        "  AABB           : ({:+.3}, {:+.3}, {:+.3}) → ({:+.3}, {:+.3}, {:+.3})",
        aabb.min.x, aabb.min.y, aabb.min.z, aabb.max.x, aabb.max.y, aabb.max.z,
    );
    println!("  signed_volume  : {:.4}", mesh.signed_volume());
    println!("  surface_area   : {:.4}", mesh.surface_area());
    println!("  is_manifold    : {}", report.is_manifold);
    println!("  is_watertight  : {}", report.is_watertight);
    println!("  is_inside_out  : {}", report.is_inside_out);
    println!("  boundary_edges : {}", report.boundary_edge_count);
}

/// Print the structured `ShellGenerationResult` field-by-field,
/// followed by the optional `ShellValidationResult` (populated
/// because `.validate(true)` overrode the `.fast()` default). The
/// `ShellValidationResult` `Display` impl is multi-line.
fn print_shell_stats(result: &ShellBuildResult) {
    let stats = &result.shell_stats;
    println!("ShellGenerationResult:");
    println!("  inner_vertex_count : {}", stats.inner_vertex_count);
    println!("  outer_vertex_count : {}", stats.outer_vertex_count);
    println!("  rim_face_count     : {}", stats.rim_face_count);
    println!("  total_face_count   : {}", stats.total_face_count);
    println!("  boundary_size      : {}", stats.boundary_size);
    println!("  wall_method        : {}", stats.wall_method);
    println!("  offset_applied     : {}", result.offset_applied);
    println!();
    match &stats.validation {
        Some(validation) => print!("{validation}"),
        None => println!(
            "(validation skipped — `.fast()` left validate=false; should not happen with .validate(true) override)"
        ),
    }
}

/// Pre-build: open-topped box has a clean structural fingerprint.
/// `mesh_repair::validate_mesh` reports it as outward-wound (the
/// partial signed-volume integral over the 5 walls is positive
/// because vertex 0 = origin is a corner of the box, so the four
/// faces incident to it contribute exactly 0; the remaining six
/// faces contribute +1000/6 mm³ each ⇒ ~666.67 mm³ total at
/// `SIDE_MM = 10`), manifold, NOT watertight (4 boundary edges =
/// the missing top face's perimeter).
fn verify_before(mesh: &IndexedMesh) {
    let report = validate_mesh(mesh);
    assert_eq!(report.vertex_count, 8);
    assert_eq!(report.face_count, 10);
    assert!(report.is_manifold);
    assert!(
        !report.is_watertight,
        "open-topped box has the missing top face's perimeter as boundary",
    );
    assert_eq!(
        report.boundary_edge_count, 4,
        "open top contributes exactly 4 boundary edges",
    );
    assert_eq!(report.non_manifold_edge_count, 0);
    assert!(
        !report.is_inside_out,
        "outward winding ⇒ partial signed_volume integral is positive",
    );

    let aabb = mesh.aabb();
    let want_max = SIDE_MM;
    assert!(aabb.min.x.abs() < 1e-12 && aabb.min.y.abs() < 1e-12 && aabb.min.z.abs() < 1e-12);
    assert!(
        (aabb.max.x - want_max).abs() < 1e-12
            && (aabb.max.y - want_max).abs() < 1e-12
            && (aabb.max.z - want_max).abs() < 1e-12,
    );
}

/// Post-build: the shell has a precise topological fingerprint plus
/// closed-form expectations on the offset vectors. Anchors span FOUR
/// concerns, delegated to three helpers + an inline check:
///
/// 1. **Topological structure** (`verify_topology`) —
///    `vertex_count == 2 × inner_vertex_count`,
///    `total_face_count == 2 × inner_face_count + 2 × boundary_size`,
///    `rim_face_count == 2 × boundary_size`, all populated from
///    `ShellGenerationResult`.
/// 2. **Per-vertex offset vectors** (`verify_offsets`) — for every
///    `i in 0..n`, `(outer[i] - inner[i]).norm() == wall_thickness`
///    AND the offset DIRECTION matches a closed-form table derived
///    from incident-triangle counts in the input triangulation.
/// 3. **Validation surface as platform truth** (`verify_validation`) —
///    `shell_stats.validation` is `Some` because `.validate(true)`
///    overrode the `.fast()` default. The shell is watertight,
///    manifold, and printable, BUT `has_consistent_winding == false`
///    with one `InconsistentWinding` issue — anchored as the
///    mesh-shell 0.7.x rim winding quirk.
/// 4. **Properly wound in face-normal sense** (inline) —
///    `signed_volume > 0` (wall material volume, positive), and
///    `is_inside_out == false`. No MC-style inside-out quirk; no
///    three-artifact pattern needed. This is the pedagogical
///    contrast vs the MC-based `mesh-offset-*` examples, where the
///    output IS inside-out (face normals flipped). The
///    edge-direction inconsistency from the rim winding quirk is
///    a SEPARATE concern from face-normal direction; both are
///    asserted here.
fn verify_shell(before: &IndexedMesh, result: &ShellBuildResult) -> Result<()> {
    let shell = &result.mesh;
    let stats = &result.shell_stats;
    let n = before.vertices.len();
    let n_inner_faces = before.faces.len();

    verify_topology(shell, stats, result.offset_applied, n, n_inner_faces);
    verify_offsets(shell, n);
    verify_validation(stats)?;

    // Outer faces keep input's outward winding (input is outward-wound
    // on its 5 walls); inner faces are deliberately winding-flipped
    // by `generate_shell_normal` so cavity normals point inward (i.e.
    // outward from the shell wall material). Both surfaces are
    // outward-everywhere from the SHELL's perspective ⇒ signed_volume
    // > 0 (wall material volume), `is_inside_out = false`. No
    // MC-style inside-out quirk; no per-face flip remediation needed.
    let report = validate_mesh(shell);
    assert!(
        !report.is_inside_out,
        "shell should be properly wound (no inside-out winding)",
    );
    let vol = shell.signed_volume();
    assert!(vol > 0.0, "shell wall volume should be positive; got {vol}");

    Ok(())
}

/// Topological structure: vertex doubling, face accounting,
/// boundary-loop carry-through. All counts populated by
/// `ShellGenerationResult` are anchored against closed-form
/// expectations from the input.
fn verify_topology(
    shell: &IndexedMesh,
    stats: &mesh_shell::ShellGenerationResult,
    offset_applied: bool,
    n: usize,
    n_inner_faces: usize,
) {
    assert_eq!(stats.inner_vertex_count, n);
    assert_eq!(stats.outer_vertex_count, n);
    assert_eq!(
        stats.boundary_size, 4,
        "open-top box has a single 4-edge boundary loop",
    );
    assert_eq!(
        stats.rim_face_count,
        2 * stats.boundary_size,
        "rim emits 2 triangles per boundary edge (one quad split into 2 tris)",
    );
    assert_eq!(
        stats.total_face_count,
        2 * n_inner_faces + stats.rim_face_count,
        "shell faces = inner ({n_inner_faces}) + outer ({n_inner_faces}) + rim ({})",
        stats.rim_face_count,
    );
    assert_eq!(shell.vertices.len(), 2 * n);
    assert_eq!(shell.faces.len(), stats.total_face_count);
    assert_eq!(stats.wall_method, WallGenerationMethod::Normal);
    assert!(
        !offset_applied,
        "no .offset(...) call ⇒ offset_applied is false",
    );
}

/// Per-vertex offset vectors. Anchors TWO properties:
///
/// 1. **Magnitude** — `(outer[i] - inner[i]).norm() == wall_thickness`
///    for every vertex (universal, triangulation-independent).
/// 2. **Direction** — closed-form per vertex from the unnormalized
///    sum of incident unit face normals. The averaged unit normal
///    is `s / |s|` where `s` is the per-vertex sum below; the
///    offset is `s × wall_thickness / |s|`.
///
/// The unnormalized sums are derived from incident-face counts in
/// the input triangulation. Each incident triangle contributes its
/// face's unit normal (`±x`, `±y`, or `±z` for our axis-aligned
/// box). A vertex on a wall's diagonal is incident to BOTH
/// triangles of that wall ⇒ that wall's normal contributes ×2;
/// off-diagonal vertices get ×1. See README's "Per-vertex offset
/// derivation" section for the per-vertex incidence breakdown.
fn verify_offsets(shell: &IndexedMesh, n: usize) {
    let thickness_tol = 1e-12;
    for i in 0..n {
        let inner = &shell.vertices[i];
        let outer = &shell.vertices[i + n];
        let magnitude = (outer - inner).norm();
        assert!(
            (magnitude - WALL_THICKNESS_MM).abs() < thickness_tol,
            "offset[{i}] magnitude should equal wall_thickness ({WALL_THICKNESS_MM}); got {magnitude}",
        );
    }

    // (vert_idx, [sx, sy, sz]) — unnormalized sum of incident unit
    // face normals at that vertex. Comments enumerate the incident
    // triangles per wall.
    let sum_normals: [(usize, [f64; 3]); 8] = [
        // 2 bot(-z) + 2 front(-y) + 2 left(-x); on diagonal of all
        // 3 incident walls ⇒ simple `1/√3` formula applies.
        (0, [-2.0, -2.0, -2.0]),
        // 1 bot + 1 front + 2 right(+x); off-diagonal on bot + front,
        // on diagonal of right ⇒ asymmetric.
        (1, [2.0, -1.0, -1.0]),
        // 2 bot + 2 back(+y) + 1 right; off-diagonal on right ⇒
        // asymmetric.
        (2, [1.0, 2.0, -2.0]),
        // 1 bot + 1 back + 1 left; off-diagonal on every incident
        // wall, but contributions are equal (1 each) ⇒ simple
        // `1/√3` formula applies.
        (3, [-1.0, 1.0, -1.0]),
        // 1 front + 1 left; top open ⇒ z=0; equal weights ⇒
        // simple `1/√2` formula applies.
        (4, [-1.0, -1.0, 0.0]),
        // 2 front + 1 right; top open ⇒ z=0; asymmetric.
        (5, [1.0, -2.0, 0.0]),
        // 1 back + 2 right; top open ⇒ z=0; asymmetric.
        (6, [2.0, 1.0, 0.0]),
        // 2 back + 2 left; top open ⇒ z=0; equal weights ⇒
        // simple `1/√2` formula applies.
        (7, [-2.0, 2.0, 0.0]),
    ];

    let component_tol = 1e-9;
    for (i, [sx, sy, sz]) in sum_normals {
        let inner = &shell.vertices[i];
        let outer = &shell.vertices[i + n];
        let got = outer - inner;
        let s_norm = sx.mul_add(sx, sy.mul_add(sy, sz * sz)).sqrt();
        let want_x = sx * WALL_THICKNESS_MM / s_norm;
        let want_y = sy * WALL_THICKNESS_MM / s_norm;
        let want_z = sz * WALL_THICKNESS_MM / s_norm;
        let dx = got.x - want_x;
        let dy = got.y - want_y;
        let dz = got.z - want_z;
        let dist = dx.mul_add(dx, dy.mul_add(dy, dz * dz)).sqrt();
        assert!(
            dist < component_tol,
            "vert {i}: offset ({:.6}, {:.6}, {:.6}) should equal ({want_x:.6}, {want_y:.6}, {want_z:.6}); dist {dist:.2e}",
            got.x,
            got.y,
            got.z,
        );
    }
}

/// Validation surface (`shell_stats.validation`, populated because
/// `.validate(true)` overrode the `.fast()` default).
/// `ShellValidationResult`'s adjacency-derived flags
/// (`is_watertight`, `is_manifold`, `boundary_edge_count`,
/// `non_manifold_edge_count`) report the shell as printable. BUT
/// `has_consistent_winding == false` with one `InconsistentWinding`
/// issue — see README's "mesh-shell 0.7.x rim winding quirk"
/// section. The rim quads use the original-winding boundary direction,
/// conflicting with the reversed-inner-face edge direction at the
/// seam; the `check_winding_consistency` BFS catches this. The shell
/// remains TOPOLOGICALLY printable (`is_printable() == true`) because
/// edge-direction inconsistency doesn't affect the surface's ability
/// to bound a volume — every edge still has exactly 2 incident faces,
/// every face's outward normal still points outward from the wall
/// material, and `signed_volume` is positive.
fn verify_validation(stats: &mesh_shell::ShellGenerationResult) -> Result<()> {
    let validation = stats
        .validation
        .as_ref()
        .ok_or_else(|| anyhow!(".validate(true) override should populate stats.validation"))?;
    assert!(validation.is_watertight, "rim should close the open top");
    assert!(validation.is_manifold);
    assert!(
        validation.is_printable(),
        "shell is printable (watertight + manifold) despite the rim winding quirk",
    );
    assert_eq!(validation.boundary_edge_count, 0);
    assert_eq!(validation.non_manifold_edge_count, 0);
    // The mesh-shell 0.7.x rim winding quirk:
    assert!(
        !validation.has_consistent_winding,
        "mesh-shell 0.7.x rim winding quirk: rim quads use original-winding boundary direction, conflicting with reversed-inner-face edge direction at the seam",
    );
    assert_eq!(
        validation.issues.len(),
        1,
        "expected exactly one issue (InconsistentWinding); got: {:?}",
        validation.issues,
    );
    assert!(
        validation
            .issues
            .iter()
            .any(|i| matches!(i, mesh_shell::ShellIssue::InconsistentWinding)),
        "the issue should be InconsistentWinding; got: {:?}",
        validation.issues,
    );
    Ok(())
}

/// Verify each `(label, in_memory, loaded)` triple: PLY round-trip
/// preserves vertex and face counts exactly (no welding, no
/// de-duplication on the I/O path).
fn verify_round_trip(pairs: &[(&str, &IndexedMesh, &IndexedMesh)]) {
    for (label, in_memory, loaded) in pairs {
        assert_eq!(
            loaded.vertices.len(),
            in_memory.vertices.len(),
            "{label}: vertex count drift across PLY round-trip",
        );
        assert_eq!(
            loaded.faces.len(),
            in_memory.faces.len(),
            "{label}: face count drift across PLY round-trip",
        );
    }
}
