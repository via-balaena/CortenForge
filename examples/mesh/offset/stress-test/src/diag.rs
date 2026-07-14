//! Shared diagnostics and checks for the offset modules: the genus
//! derivation, a labeled report block, the input-fixture guard, the
//! clean-manifold gate, the analytical-AABB check, the out/ save, and the
//! PLY round-trip check — all identical across the inward/outward twins;
//! only the direction-specific volume oracle stays in each module.

use std::path::{Path, PathBuf};

use anyhow::Result;
use mesh_io::{load_ply, save_ply};
use mesh_repair::{MeshReport, find_connected_components, validate_mesh};
use mesh_types::{Bounded, IndexedMesh};

/// Genus of a closed orientable 2-manifold from its `(V, E, F)` counts:
/// rearranging the Euler characteristic `χ = V − E + F = 2 − 2g` gives
/// `g = (E + 2 − V − F) / 2`. A clean single-shell offset is genus 0
/// (`χ = 2`); spurious marching-cubes handles show up as `g > 0`
/// (`χ < 2`).
///
/// Computed entirely in `usize` (counts are `u32`-indexed, so no
/// overflow) to stay clippy-clean without signed casts. Returns `None`
/// when genus is not a well-defined non-negative integer:
/// - `E + 2 < V + F` (`χ > 2`) — more than one connected component;
/// - `E + 2 − V − F` odd — the defect is not `2g`, which signals a
///   non-manifold or non-orientable mesh (genus is only defined for a
///   closed orientable 2-manifold).
///
/// Callers gate closedness / manifoldness separately; this only reports
/// genus honestly and refuses to print a plausible-but-wrong number.
#[must_use]
pub const fn genus(report: &MeshReport) -> Option<usize> {
    match (report.edge_count + 2).checked_sub(report.vertex_count + report.face_count) {
        Some(twice_g) => {
            if twice_g % 2 == 0 {
                Some(twice_g / 2)
            } else {
                None
            }
        }
        None => None,
    }
}

/// Format genus for display: the number, or `n/a` when it is not
/// well-defined (multiple components, or a non-manifold defect).
#[must_use]
pub fn genus_str(report: &MeshReport) -> String {
    genus(report).map_or_else(|| "n/a".to_owned(), |g| g.to_string())
}

/// Print a labeled diagnostic block: counts, genus, AABB, signed volume,
/// surface area, and the adjacency-derived flags from `validate_mesh`.
pub fn print_diagnostics(label: &str, mesh: &IndexedMesh) {
    let report = validate_mesh(mesh);
    let aabb = mesh.aabb();
    println!("{label}:");
    println!("  vertices       : {}", report.vertex_count);
    println!("  edges          : {}", report.edge_count);
    println!("  faces          : {}", report.face_count);
    println!("  genus          : {}", genus_str(&report));
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

/// Guard the input fixture before offsetting: every downstream oracle
/// (the analytical AABB, the `(1 + 2d)³` / Steiner volume formulas) is
/// hard-coded to `unit_cube()` being a clean unit cube at `[0, 1]³`, so a
/// regression there is named here directly instead of surfacing as a
/// confusing offset mismatch. Exact anchors are appropriate — the cube is
/// deterministic hand-authored geometry, not a mesher output.
pub fn verify_unit_cube_input(mesh: &IndexedMesh) {
    let report = validate_mesh(mesh);
    assert_eq!(report.vertex_count, 8, "unit_cube must have 8 vertices");
    assert_eq!(report.face_count, 12, "unit_cube must have 12 faces");
    assert!(
        report.is_watertight && report.is_manifold && !report.is_inside_out,
        "unit_cube must be a clean watertight outward-wound manifold",
    );
    let vol = mesh.signed_volume();
    assert!(
        (vol - 1.0).abs() < 1e-10,
        "unit_cube signed_volume must be 1.0; got {vol}",
    );
    let aabb = mesh.aabb();
    for (label, lo, hi) in [
        ("x", aabb.min.x, aabb.max.x),
        ("y", aabb.min.y, aabb.max.y),
        ("z", aabb.min.z, aabb.max.z),
    ] {
        assert!(
            lo.abs() < 1e-12 && (hi - 1.0).abs() < 1e-12,
            "unit_cube AABB.{label} must be [0, 1]; got [{lo}, {hi}]",
        );
    }
}

/// Gate the topology shared by both offset directions: a single connected,
/// watertight, outward-wound, genus-0 2-manifold that marching cubes
/// actually tessellated. Identical for inward and outward — only the
/// volume oracle differs, so that stays in each module.
///
/// The single-component check (`find_connected_components`) is what makes
/// `genus == Some(0)` unambiguous: `χ = 2` alone also admits a
/// disconnected pair whose Euler characteristics sum to 2 (e.g. a torus
/// plus a sphere), so connectedness + `χ = 2` together pin genus 0.
pub fn assert_clean_offset_manifold(mesh: &IndexedMesh, label: &str) {
    let report = validate_mesh(mesh);
    assert!(
        report.is_watertight,
        "{label} offset must be watertight (every edge shared by 2 faces)",
    );
    assert!(
        report.is_manifold && report.non_manifold_edge_count == 0,
        "{label} offset must be manifold (no edge with 3+ faces)",
    );
    assert!(
        !report.is_inside_out && mesh.signed_volume() > 0.0,
        "{label} offset must be outward-wound (signed_volume > 0)",
    );
    assert!(
        find_connected_components(mesh).is_connected(),
        "{label} offset must be a single connected shell",
    );
    assert_eq!(
        genus(&report),
        Some(0),
        "{label} offset must be a genus-0 manifold (χ = 2); got genus {}",
        genus_str(&report),
    );
    // Marching cubes actually tessellated a surface (not a passed-through
    // or trivially-coarse cube that would satisfy the invariants above).
    assert!(
        report.face_count > 12,
        "{label} offset surface should have more triangles than the 12-face input cube; got {}",
        report.face_count,
    );
}

/// Create the crate-local `out/` directory and save `mesh` to
/// `out/<out_name>`, returning the path (for the round-trip check).
/// Centralizes the `out/` convention shared by both modules.
pub fn save_to_out(out_name: &str, mesh: &IndexedMesh) -> Result<PathBuf> {
    let out_dir = Path::new(env!("CARGO_MANIFEST_DIR")).join("out");
    std::fs::create_dir_all(&out_dir)?;
    let path = out_dir.join(out_name);
    save_ply(mesh, &path, true)?;
    Ok(path)
}

/// Assert the offset of a unit cube lands at its analytical AABB.
/// Offsetting `unit_cube()` (AABB `[0, 1]³`) by `offset_distance` moves
/// each face to `-offset_distance` / `1 + offset_distance`. The
/// bilinearly-interpolated marching-cubes level set lands within half a
/// grid cell of that analytical position on each axis. Identical
/// expectation for inward (`d < 0`) and outward (`d > 0`).
pub fn assert_offset_aabb(mesh: &IndexedMesh, offset_distance: f64, resolution: f64) {
    let aabb = mesh.aabb();
    let tol = resolution.mul_add(0.5, 1e-9);
    let want_min = -offset_distance;
    let want_max = 1.0 + offset_distance;
    for (label, got_min, got_max) in [
        ("x", aabb.min.x, aabb.max.x),
        ("y", aabb.min.y, aabb.max.y),
        ("z", aabb.min.z, aabb.max.z),
    ] {
        assert!(
            (got_min - want_min).abs() <= tol,
            "AABB.min.{label}: want {want_min:+.3} ± {tol:.4}, got {got_min:.4}",
        );
        assert!(
            (got_max - want_max).abs() <= tol,
            "AABB.max.{label}: want {want_max:+.3} ± {tol:.4}, got {got_max:.4}",
        );
    }
}

/// Save `mesh` to `path`, reload it, and assert the PLY round-trip
/// preserves vertex and face counts exactly (no welding or
/// de-duplication on the I/O path).
pub fn verify_round_trip(label: &str, mesh: &IndexedMesh, path: &Path) -> Result<()> {
    let loaded = load_ply(path)?;
    assert_eq!(
        loaded.vertices.len(),
        mesh.vertices.len(),
        "{label}: vertex count drift across PLY round-trip",
    );
    assert_eq!(
        loaded.faces.len(),
        mesh.faces.len(),
        "{label}: face count drift across PLY round-trip",
    );
    Ok(())
}
