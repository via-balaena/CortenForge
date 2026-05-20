//! Shared fixtures for mesh-sdf's test modules.
//!
//! Test-only (gated under `#[cfg(test)]` at the lib.rs mod declaration);
//! `pub(crate)` so `sdf::tests` and `sdf_adapter::tests` can both
//! consume the same definitions without duplicating fixture code.

use mesh_types::{IndexedMesh, Point3};

/// Regular tetrahedron with the bottom face on z=0 and apex above.
///
/// Bottom face winding `[0, 2, 1]` is CCW from below — the outward
/// face normal of the bottom is `-z`. The other three faces wind CCW
/// from outside.
pub(crate) fn unit_tetrahedron() -> IndexedMesh {
    let mut mesh = IndexedMesh::new();
    mesh.vertices.push(Point3::new(0.0, 0.0, 0.0));
    mesh.vertices.push(Point3::new(1.0, 0.0, 0.0));
    mesh.vertices.push(Point3::new(0.5, 0.866, 0.0));
    mesh.vertices.push(Point3::new(0.5, 0.289, 0.816));
    mesh.faces.push([0, 2, 1]); // bottom (outward = -z)
    mesh.faces.push([0, 1, 3]); // front
    mesh.faces.push([1, 2, 3]); // right
    mesh.faces.push([2, 0, 3]); // left
    mesh
}
