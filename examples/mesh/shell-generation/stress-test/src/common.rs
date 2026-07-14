//! Diagnostics shared by both shell-generation modules — identical in the
//! former `fast` / `high-quality` examples, hoisted here.

use mesh_io::load_ply;
use mesh_repair::validate_mesh;
use mesh_shell::ShellBuildResult;
use mesh_types::{Bounded, IndexedMesh};

/// Print a labeled diagnostic block: counts, AABB, signed volume,
/// surface area, and the topology flags from `validate_mesh`.
pub fn print_diagnostics(label: &str, mesh: &IndexedMesh) {
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

/// Print the structured `ShellGenerationResult` field-by-field, followed
/// by the `ShellValidationResult` `Display` block. Validation is always
/// `Some` for the presets these modules use (`.fast().validate(true)` and
/// `.high_quality()` both set `validate = true`); the `None` branch is
/// defensive.
pub fn print_shell_stats(result: &ShellBuildResult) {
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
        None => {
            println!("(validation missing — unexpected; the preset/override sets validate=true)");
        }
    }
}

/// Save `mesh` to `path`, reload it, and assert the PLY round-trip
/// preserves vertex and face counts exactly (no welding, no
/// de-duplication on the I/O path).
pub fn verify_round_trip(
    label: &str,
    mesh: &IndexedMesh,
    path: &std::path::Path,
) -> anyhow::Result<()> {
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
