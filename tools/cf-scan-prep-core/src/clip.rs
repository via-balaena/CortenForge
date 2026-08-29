//! Half-space clipping of a mesh against a plane.
//!
//! Moved verbatim from the flat `lib.rs` (pure module split).

use mesh_repair::remove_unreferenced_vertices;
use mesh_types::{IndexedMesh, Point3};
use nalgebra::Vector3;

/// Linear interpolation between two `Point3<f64>` positions.
/// `t = 0` returns `a`; `t = 1` returns `b`.
pub fn lerp_point(a: &Point3<f64>, b: &Point3<f64>, t: f64) -> Point3<f64> {
    Point3::new(
        a.x + t * (b.x - a.x),
        a.y + t * (b.y - a.y),
        a.z + t * (b.z - a.z),
    )
}

/// True-plane-intersection mesh clip against a plane in the mesh's
/// own coordinate frame, expressed as the equation
/// `plane_normal · v >= plane_d` (kept side).
///
/// Shared core algorithm — [`clip_mesh_against_plane`] (the
/// centerline-trim cuts at each end) derives its inputs and then
/// forwards here. Pre-CSP.4d there was also a
/// `clip_mesh_against_world_z` for the now-retired Clip-floor
/// panel; that wrapper is gone, but `clip_mesh_against_plane_eq`
/// remained as the shared core.
///
/// For each triangle:
/// - **All 3 vertices on/above** the plane → keep as-is.
/// - **All 3 below** → drop.
/// - **Mixed** → compute intersection points on the crossing edges +
///   triangulate the surviving polygon (3 or 4 vertices) via a fan
///   from the first vertex. The result is a clean planar cut along
///   the plane, suitable for capping (boundary stays planar so
///   `mesh-repair`'s plane fit gets `R² ≈ 1.0`).
///
/// Vertices exactly on the plane (`dist == 0`) are classified as
/// "above" so the surrounding triangle is kept; this avoids
/// degenerate sub-triangles in the cut.
///
/// Output mesh's vertex buffer = original vertices + new intersection
/// vertices appended; `remove_unreferenced_vertices` strips the
/// dropped (all-below) original vertices afterwards so the result is
/// tight.
pub fn clip_mesh_against_plane_eq(
    mesh: &IndexedMesh,
    plane_normal: Vector3<f64>,
    plane_d: f64,
) -> IndexedMesh {
    // Output buffers — start with the original vertices (later
    // stripped of unreferenced); append intersection points as we go.
    let mut new_vertices: Vec<Point3<f64>> = mesh.vertices.clone();
    let mut new_faces: Vec<[u32; 3]> = Vec::with_capacity(mesh.faces.len());

    for face in &mesh.faces {
        let verts = [
            mesh.vertices[face[0] as usize],
            mesh.vertices[face[1] as usize],
            mesh.vertices[face[2] as usize],
        ];
        // Signed distance from the plane. dist >= 0 → above (keep).
        let dists = [
            plane_normal.dot(&verts[0].coords) - plane_d,
            plane_normal.dot(&verts[1].coords) - plane_d,
            plane_normal.dot(&verts[2].coords) - plane_d,
        ];
        let above_count = dists.iter().filter(|d| **d >= 0.0).count();

        if above_count == 3 {
            new_faces.push(*face);
            continue;
        }
        if above_count == 0 {
            continue;
        }

        // Mixed case: walk the triangle CCW, collecting above-plane
        // vertices + intersection points where edges cross. The
        // resulting polygon has 3 (1-above) or 4 (2-above) vertices —
        // both convex, so fan-triangulate from vertex[0].
        let mut poly_indices: Vec<u32> = Vec::with_capacity(4);
        for i in 0..3 {
            let next = (i + 1) % 3;
            let d_curr = dists[i];
            let d_next = dists[next];
            if d_curr >= 0.0 {
                poly_indices.push(face[i]);
            }
            // Edge crosses the plane iff the signs of d_curr and
            // d_next differ. The `>= 0.0` convention puts zeros on
            // the "above" side, so an edge with one zero and one
            // negative still crosses.
            if (d_curr >= 0.0) != (d_next >= 0.0) {
                let t = d_curr / (d_curr - d_next);
                let intersection = lerp_point(&verts[i], &verts[next], t);
                #[allow(clippy::cast_possible_truncation)]
                let new_idx = new_vertices.len() as u32;
                new_vertices.push(intersection);
                poly_indices.push(new_idx);
            }
        }

        // Fan triangulation from poly_indices[0].
        for i in 1..(poly_indices.len() - 1) {
            new_faces.push([poly_indices[0], poly_indices[i], poly_indices[i + 1]]);
        }
    }

    let mut clipped = IndexedMesh::with_capacity(new_vertices.len(), new_faces.len());
    clipped.vertices = new_vertices;
    clipped.faces = new_faces;
    remove_unreferenced_vertices(&mut clipped);
    clipped
}

// CSP.4c — `clip_mesh_against_world_z` removed alongside
// `ClipState`. The shared `clip_mesh_against_plane_eq` primitive
// below is still load-bearing for `trim_mesh_along_centerline`.

/// Clip `mesh` against an arbitrary plane defined by a point on the
/// plane and a normal. Kept side: where
/// `(p - plane_point) · plane_normal >= 0`. CSP.4b — used by
/// [`trim_mesh_along_centerline`](crate::trim_mesh_along_centerline) to clip each end of the centerline
/// at a user-chosen distance.
///
/// `plane_normal` MUST be a unit vector — the algorithm relies on the
/// dot product producing signed distances. Callers passing a
/// non-unit normal will get a clip that succeeds but with wrong
/// intersection-point math; cheaper than asserting here.
pub fn clip_mesh_against_plane(
    mesh: &IndexedMesh,
    plane_point: Point3<f64>,
    plane_normal: Vector3<f64>,
) -> IndexedMesh {
    // `plane_normal · (p - plane_point) >= 0`
    //   ⇔ `plane_normal · p >= plane_normal · plane_point`
    let plane_d = plane_normal.dot(&plane_point.coords);
    clip_mesh_against_plane_eq(mesh, plane_normal, plane_d)
}
