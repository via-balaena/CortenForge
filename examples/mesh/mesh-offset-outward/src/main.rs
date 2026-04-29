//! Outward offset of a unit cube via SDF + marching cubes.
//!
//! Builds `mesh_types::unit_cube()` and offsets it outward by 0.1
//! mesh units using `mesh_offset::offset_mesh`. Saves THREE PLY
//! artifacts and round-trips all three via `load_ply`:
//!
//!   * `out/before.ply` — input unit cube
//!   * `out/after.ply` — raw `offset_mesh` output (inside-out, the
//!     platform-truth artifact)
//!   * `out/after_flipped.ply` — same geometry with winding reversed
//!     per face (viewer-friendly; outward-shaded in any viewer)
//!
//! Two pedagogical points the example anchors:
//!
//!   1. **Vertex-soup output.** Marching cubes does not share
//!      vertices across triangles, so `vertex_count == 3 * face_count`
//!      and every edge is a boundary edge (`!is_watertight`). The
//!      mesh is geometrically closed but topologically disconnected.
//!      `validate_mesh::is_manifold` still reports `true` because it
//!      checks "no edge has 3+ faces" — and the soup has every edge
//!      with exactly 1 face. Watertightness is the more discriminating
//!      check. To get a true manifold mesh, follow up with
//!      `mesh_repair::weld_vertices`.
//!   2. **Inside-out winding.** The mesh-offset MC tables happen to
//!      emit inward-pointing triangles for this iso/inside convention,
//!      so `signed_volume < 0` and `is_inside_out == true`. The
//!      magnitude still matches Steiner-Minkowski to <0.1%, so the
//!      level set is geometrically correct — only the orientation is
//!      flipped. The example demonstrates the per-face winding flip
//!      remediation (`[a,b,c] → [a,c,b]`) which works on soup meshes
//!      regardless of adjacency, unlike `mesh_repair::fix_winding_order`
//!      which BFS-traverses adjacency and is a no-op on disconnected
//!      triangles. Inside-out output is a known platform quirk in
//!      mesh-offset 0.7.x.
//!
//! See `examples/mesh/README.md` for cadence; see
//! `docs/studies/mesh_architecture/src/30-sdf-and-offset.md` for the
//! depth pass on the mesh→SDF bridge.

use std::path::Path;

use anyhow::Result;
use mesh_io::{load_ply, save_ply};
use mesh_offset::{OffsetConfig, offset_mesh};
use mesh_repair::validate_mesh;
use mesh_types::{Bounded, IndexedMesh, unit_cube};

/// Offset distance in mesh units. Positive = expansion (dilation).
const OFFSET_DISTANCE: f64 = 0.1;

/// Marching-cubes grid cell size. 0.025 = 4 cells per `OFFSET_DISTANCE`,
/// the "knee" of the visual quality curve where corners look crisp
/// without faceting and the run still finishes sub-second on M-series.
const GRID_RESOLUTION: f64 = 0.025;

fn main() -> Result<()> {
    let before = unit_cube();
    let config = OffsetConfig::default().with_resolution(GRID_RESOLUTION);
    let after = offset_mesh(&before, OFFSET_DISTANCE, &config)?;
    let mut after_flipped = after.clone();
    flip_winding(&mut after_flipped);

    let out_dir = Path::new(env!("CARGO_MANIFEST_DIR")).join("out");
    std::fs::create_dir_all(&out_dir)?;
    let before_path = out_dir.join("before.ply");
    let after_path = out_dir.join("after.ply");
    let after_flipped_path = out_dir.join("after_flipped.ply");
    save_ply(&before, &before_path, true)?;
    save_ply(&after, &after_path, true)?;
    save_ply(&after_flipped, &after_flipped_path, true)?;

    println!("==== mesh-offset-outward ====");
    println!();
    println!("input  : unit_cube() — 8v 12f, AABB [0,1]^3");
    println!("config : OffsetConfig::default().with_resolution({GRID_RESOLUTION})");
    println!("offset : +{OFFSET_DISTANCE} (outward, dilation)");
    println!();

    print_offset_diagnostics("before        (unit_cube)            ", &before);
    println!();
    print_offset_diagnostics("after         (raw MC, inside-out)   ", &after);
    println!();
    print_offset_diagnostics("after_flipped (winding reversed)     ", &after_flipped);
    println!();

    verify_before(&before);
    verify_after(&after);
    verify_after_flipped(&after, &after_flipped);

    let before_loaded = load_ply(&before_path)?;
    let after_loaded = load_ply(&after_path)?;
    let after_flipped_loaded = load_ply(&after_flipped_path)?;
    verify_round_trip(&[
        ("before", &before, &before_loaded),
        ("after", &after, &after_loaded),
        ("after_flipped", &after_flipped, &after_flipped_loaded),
    ]);

    println!("artifacts:");
    println!(
        "  out/before.ply         : {}v, {}f (round-trip verified)",
        before_loaded.vertices.len(),
        before_loaded.faces.len(),
    );
    println!(
        "  out/after.ply          : {}v, {}f (round-trip verified) [inside-out — platform truth]",
        after_loaded.vertices.len(),
        after_loaded.faces.len(),
    );
    println!(
        "  out/after_flipped.ply  : {}v, {}f (round-trip verified) [outward — viewer-friendly]",
        after_flipped_loaded.vertices.len(),
        after_flipped_loaded.faces.len(),
    );
    println!();
    println!("OK — outward offset verified");

    Ok(())
}

/// Reverse the winding of every triangle in place: `[a, b, c] → [a, c, b]`.
///
/// Works on any mesh regardless of adjacency, including marching-cubes
/// soup output. Per-face operation, no vertex changes — positions and
/// counts are preserved exactly; only triangle orientation flips.
///
/// Use this instead of `mesh_repair::fix_winding_order` when the input
/// is a vertex-soup mesh (every triangle disconnected). The
/// adjacency-based BFS in `fix_winding_order` is a no-op on soup
/// meshes because no two faces share any edge by index.
fn flip_winding(mesh: &mut IndexedMesh) {
    for face in &mut mesh.faces {
        face.swap(1, 2);
    }
}

/// Print a labeled diagnostic block: counts, AABB, signed volume,
/// surface area, and the three adjacency-derived flags from
/// `validate_mesh`.
fn print_offset_diagnostics(label: &str, mesh: &IndexedMesh) {
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
}

/// Pre-offset: `unit_cube` is the canonical clean input — manifold,
/// watertight, outward-wound, volume = 1, AABB = `[0,1]^3`.
fn verify_before(mesh: &IndexedMesh) {
    let report = validate_mesh(mesh);
    assert_eq!(report.vertex_count, 8);
    assert_eq!(report.face_count, 12);
    assert!(report.is_manifold);
    assert!(report.is_watertight);
    assert!(!report.is_inside_out);

    let vol = mesh.signed_volume();
    assert!(
        (vol - 1.0).abs() < 1e-10,
        "unit_cube signed_volume should be 1.0, got {vol}",
    );

    let aabb = mesh.aabb();
    assert!(aabb.min.x.abs() < 1e-12 && aabb.min.y.abs() < 1e-12 && aabb.min.z.abs() < 1e-12);
    assert!(
        (aabb.max.x - 1.0).abs() < 1e-12
            && (aabb.max.y - 1.0).abs() < 1e-12
            && (aabb.max.z - 1.0).abs() < 1e-12,
    );
}

/// Post-offset: assertions span both the GEOMETRIC level set (AABB,
/// volume, winding) and the TOPOLOGICAL marching-cubes output
/// (vertex non-sharing, every-edge-is-boundary). See README's
/// "The vertex-soup output" section for the algorithmic root cause.
fn verify_after(mesh: &IndexedMesh) {
    let report = validate_mesh(mesh);
    let aabb = mesh.aabb();

    // ── Geometric: level set is at the right place ─────────────────
    // The level set is bilinearly interpolated between SDF grid
    // samples, so each axis can land anywhere within half a grid cell
    // of the analytical position.
    let aabb_tol = GRID_RESOLUTION.mul_add(0.5, 1e-9);
    let want_min = -OFFSET_DISTANCE;
    let want_max = 1.0 + OFFSET_DISTANCE;

    for (label, got_min, got_max) in [
        ("x", aabb.min.x, aabb.max.x),
        ("y", aabb.min.y, aabb.max.y),
        ("z", aabb.min.z, aabb.max.z),
    ] {
        assert!(
            (got_min - want_min).abs() <= aabb_tol,
            "AABB.min.{label}: want {want_min:+.3} ± {aabb_tol:.4}, got {got_min:.4}",
        );
        assert!(
            (got_max - want_max).abs() <= aabb_tol,
            "AABB.max.{label}: want {want_max:+.3} ± {aabb_tol:.4}, got {got_max:.4}",
        );
    }

    // Steiner-Minkowski formula for a unit cube + offset d:
    //   V_d = 1 + 6d + 3πd² + (4π/3)d³
    //   1        = original cube
    //   6d       = six face slabs of thickness d
    //   3πd²     = twelve quarter-cylinders along the cube edges
    //   (4π/3)d³ = eight corner sphere octants summing to a full sphere
    // Marching-cubes discretization introduces few-percent error;
    // 5% is comfortable headroom at GRID_RESOLUTION = 0.025.
    let d = OFFSET_DISTANCE;
    let pi = std::f64::consts::PI;
    let cube_term = 1.0;
    let face_term = 6.0 * d;
    let edge_term = 3.0 * pi * d.powi(2);
    let corner_term = (4.0 * pi / 3.0) * d.powi(3);
    let expected_vol = cube_term + face_term + edge_term + corner_term;
    // Compare magnitudes — the MC output is inside-out (see below), so
    // signed_volume is the negative of the Steiner value.
    let vol_mag = mesh.signed_volume().abs();
    assert!(
        ((vol_mag - expected_vol) / expected_vol).abs() < 0.05,
        "|signed_volume| should match Steiner formula ({expected_vol:.4}) within 5%; got {vol_mag:.4}",
    );

    // Inside-out winding is a known mesh-offset 0.7.x quirk — the MC
    // tables produce inward-pointing triangles for this iso/inside
    // convention. Anchored as the current behavior; flipping this to
    // `!is_inside_out` becomes the right test once mesh-offset's
    // winding convention is corrected (or the example calls
    // `fix_winding_order` itself).
    assert!(
        report.is_inside_out,
        "mesh-offset 0.7.x quirk: MC output is inside-out (signed_volume < 0)",
    );

    // ── Topological: marching cubes does not share vertices ────────
    // Every triangle gets 3 fresh vertex entries (see
    // `mesh-offset/src/marching_cubes.rs::process_cell`). Therefore:
    //   vertex_count == 3 * face_count            (exact)
    //   every edge appears in exactly one face    (boundary saturated)
    //   is_watertight == false                    (every edge is boundary)
    //   is_manifold == true                       (no edge has 3+ faces)
    // The manifold flag is misleading here — `validate_mesh` checks
    // "no edge has 3+ incident faces," which the soup trivially
    // satisfies. Watertightness is the discriminating signal.
    assert_eq!(
        report.vertex_count,
        3 * report.face_count,
        "marching cubes output: vertex_count must equal 3 × face_count",
    );
    assert!(
        report.face_count > 12,
        "offset surface should have more triangles than the input cube; got {}",
        report.face_count,
    );
    assert!(
        report.is_manifold,
        "soup mesh trivially passes is_manifold (no edge has 3+ faces); flag is not the right discriminator here",
    );
    assert!(
        !report.is_watertight,
        "non-shared MC vertices ⇒ every edge is boundary ⇒ !is_watertight",
    );
    assert_eq!(
        report.boundary_edge_count,
        3 * report.face_count,
        "soup mesh: every edge is unique boundary; boundary_edge_count == 3 × face_count",
    );
}

/// Post-flip: positions identical to `after`, `signed_volume` exactly
/// negated, `is_inside_out` flips false. All topological flags
/// unchanged (still soup, still `!is_watertight`, still trivially
/// `is_manifold`).
fn verify_after_flipped(after: &IndexedMesh, flipped: &IndexedMesh) {
    let after_report = validate_mesh(after);
    let flipped_report = validate_mesh(flipped);

    // Positions and counts are identical (clone + face-index swap only).
    assert_eq!(flipped_report.vertex_count, after_report.vertex_count);
    assert_eq!(flipped_report.face_count, after_report.face_count);
    assert_eq!(
        flipped.vertices, after.vertices,
        "vertex array must be byte-identical"
    );
    let after_aabb = after.aabb();
    let flipped_aabb = flipped.aabb();
    assert_eq!(flipped_aabb.min, after_aabb.min);
    assert_eq!(flipped_aabb.max, after_aabb.max);

    // Per-face winding flip negates each tetrahedral signed-volume
    // contribution exactly — the sum is the exact arithmetic negation
    // (modulo IEEE-754 add ordering, well within 1e-9 absolute).
    let after_vol = after.signed_volume();
    let flipped_vol = flipped.signed_volume();
    assert!(
        (flipped_vol + after_vol).abs() < 1e-9,
        "signed_volume should be exact negation: {after_vol:.6} + {flipped_vol:.6} = {:.2e}",
        flipped_vol + after_vol,
    );

    // The win: outward winding now, viewer-friendly artifact.
    assert!(
        flipped_vol > 0.0,
        "after flip, signed_volume should be positive (outward winding)"
    );
    assert!(
        !flipped_report.is_inside_out,
        "after flip, validate_mesh should report !is_inside_out",
    );

    // Topology unchanged — vertices weren't touched, only face index
    // order. The mesh is still a soup; the flip is a presentation
    // change, not a topological repair.
    assert_eq!(flipped_report.vertex_count, 3 * flipped_report.face_count);
    assert!(!flipped_report.is_watertight);
    assert!(flipped_report.is_manifold);
    assert_eq!(
        flipped_report.boundary_edge_count,
        3 * flipped_report.face_count,
    );
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
