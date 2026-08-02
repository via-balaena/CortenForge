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
pub(crate) fn uv_sphere(radius: f64, n_lat: usize, n_lon: usize) -> IndexedMesh {
    assert!(n_lat >= 2 && n_lon >= 3, "degenerate sphere tessellation");
    let mut mesh = IndexedMesh::new();
    mesh.vertices.push(Point3::new(0.0, 0.0, radius)); // north pole
    for i in 1..n_lat {
        #[allow(clippy::cast_precision_loss)] // small loop counters
        let theta = std::f64::consts::PI * (i as f64) / (n_lat as f64);
        let (st, ct) = theta.sin_cos();
        for j in 0..n_lon {
            #[allow(clippy::cast_precision_loss)]
            let phi = 2.0 * std::f64::consts::PI * (j as f64) / (n_lon as f64);
            let (sp, cp) = phi.sin_cos();
            mesh.vertices
                .push(Point3::new(radius * st * cp, radius * st * sp, radius * ct));
        }
    }
    mesh.vertices.push(Point3::new(0.0, 0.0, -radius)); // south pole

    // Vertex ids are `(n_lat - 1) * n_lon + 2` at most — far below `u32::MAX`
    // for any tessellation a test would ask for.
    #[allow(clippy::cast_possible_truncation)]
    let south = (mesh.vertices.len() - 1) as u32;
    #[allow(clippy::cast_possible_truncation)]
    let lon = n_lon as u32;
    let ring = |i: u32, j: u32| 1 + (i - 1) * lon + j % lon; // 1-based interior stack
    for j in 0..lon {
        mesh.faces.push([0, ring(1, j), ring(1, j + 1)]);
    }
    #[allow(clippy::cast_possible_truncation)]
    for i in 1..(n_lat as u32 - 1) {
        for j in 0..lon {
            let (a, b) = (ring(i, j), ring(i, j + 1));
            let (c, d) = (ring(i + 1, j), ring(i + 1, j + 1));
            mesh.faces.push([a, c, d]);
            mesh.faces.push([a, d, b]);
        }
    }
    #[allow(clippy::cast_possible_truncation)]
    let last = n_lat as u32 - 1;
    for j in 0..lon {
        mesh.faces.push([south, ring(last, j + 1), ring(last, j)]);
    }
    mesh
}
