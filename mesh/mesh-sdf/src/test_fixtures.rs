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

/// UV-tessellated sphere of `radius` about the origin, outward-wound.
///
/// The one fixture whose exact signed distance is known everywhere in space —
/// `φ(p) = ‖p‖ − radius` — so a test can score the oracle against closed-form
/// truth at any sample point rather than at a handful of hand-picked ones.
///
/// `n_lat` stacks × `n_lon` sectors; poles are triangle fans, interior stacks
/// are quads split in two. The **pole fans carry the smallest triangles**,
/// which is what makes this fixture useful for probing the area floor below
/// which parry stops computing a pseudo-normal.
///
/// Counts are `u32` — the same type `IndexedMesh` indexes faces with — so the
/// whole construction is cast-free: `f64::from(u32)` is lossless, and no vertex
/// id is ever narrowed.
pub(crate) fn uv_sphere(radius: f64, n_lat: u32, n_lon: u32) -> IndexedMesh {
    assert!(n_lat >= 2 && n_lon >= 3, "degenerate sphere tessellation");
    let mut mesh = IndexedMesh::new();
    mesh.vertices.push(Point3::new(0.0, 0.0, radius)); // north pole
    for i in 1..n_lat {
        let theta = std::f64::consts::PI * f64::from(i) / f64::from(n_lat);
        let (st, ct) = theta.sin_cos();
        for j in 0..n_lon {
            let phi = 2.0 * std::f64::consts::PI * f64::from(j) / f64::from(n_lon);
            let (sp, cp) = phi.sin_cos();
            mesh.vertices
                .push(Point3::new(radius * st * cp, radius * st * sp, radius * ct));
        }
    }
    mesh.vertices.push(Point3::new(0.0, 0.0, -radius)); // south pole

    // Ring `i` (1-based interior stack), sector `j` wrapping. Vertex 0 is the
    // north pole, so interior stack `i` starts at `1 + (i - 1) * n_lon`.
    let ring = |i: u32, j: u32| 1 + (i - 1) * n_lon + j % n_lon;
    let south = 1 + (n_lat - 1) * n_lon;
    for j in 0..n_lon {
        mesh.faces.push([0, ring(1, j), ring(1, j + 1)]);
    }
    for i in 1..(n_lat - 1) {
        for j in 0..n_lon {
            let (a, b) = (ring(i, j), ring(i, j + 1));
            let (c, d) = (ring(i + 1, j), ring(i + 1, j + 1));
            mesh.faces.push([a, c, d]);
            mesh.faces.push([a, d, b]);
        }
    }
    for j in 0..n_lon {
        mesh.faces
            .push([south, ring(n_lat - 1, j + 1), ring(n_lat - 1, j)]);
    }
    mesh
}
