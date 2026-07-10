//! Brush-based face selection over an [`IndexedMesh`].
//!
//! A **surface brush** selects faces: given a seed face (typically the one under
//! a cursor ray-hit), it covers every face whose centroid lies within a radius
//! of the seed's centroid, optionally restricted to faces whose normal points a
//! similar direction (so a brush on a flat region doesn't spill onto the steep
//! walls beside it).
//!
//! [`FaceField`] precomputes the per-face centroids and unit normals once, so
//! repeated [`FaceField::brush`] queries against a fixed surface are cheap. It
//! is deliberately anatomy-free and interaction-framework-free — it returns a
//! plain set of face ids; what a caller does with them (paint, erase, trim,
//! bore, attach) is the caller's concern. A paint GUI is only its first
//! consumer; a tendon-site picker, a subtractive-bore selector, and a scan
//! trimmer all reduce to the same query.

// L0 library posture (matches `mesh-loft` / `mesh-repair` / `cf-geometry`):
// production code never panics — deny `unwrap`/`expect` outside tests so a
// future edit can't smuggle one into the primitive.
#![cfg_attr(not(test), deny(clippy::unwrap_used, clippy::expect_used))]

use cf_geometry::IndexedMesh;
use nalgebra::{Point3, Vector3};

/// Fallback normal for a degenerate (zero-area) face, so a brush query never
/// divides by zero: an arbitrary but unit vector.
const DEGENERATE_NORMAL: Vector3<f64> = Vector3::new(0.0, 0.0, 1.0);

/// The smallest cross-product magnitude treated as a real face normal; below
/// this the face is degenerate and gets [`DEGENERATE_NORMAL`].
const MIN_NORMAL_LEN: f64 = 1e-12;

/// Per-face centroids and unit normals precomputed from a mesh, for repeated
/// brush queries against a fixed surface.
///
/// Both vectors are indexed by face id (parallel to the source mesh's `faces`),
/// so [`FaceField::brush`] can answer purely from this precomputed geometry
/// without re-reading the mesh.
#[derive(Debug, Clone)]
pub struct FaceField {
    /// Centroid of each face, in the mesh's native coordinates.
    centroids: Vec<Point3<f64>>,
    /// Unit normal of each face; degenerate faces carry `DEGENERATE_NORMAL`.
    normals: Vec<Vector3<f64>>,
}

impl FaceField {
    /// Precompute the centroid and unit normal of every face in `mesh`.
    ///
    /// The normal is the normalized `(b - a) × (c - a)` of the face's triangle;
    /// a zero-area face (cross-product shorter than `MIN_NORMAL_LEN`) gets
    /// `DEGENERATE_NORMAL` instead of a division by zero.
    #[must_use]
    pub fn new(mesh: &IndexedMesh) -> Self {
        let mut centroids = Vec::with_capacity(mesh.faces.len());
        let mut normals = Vec::with_capacity(mesh.faces.len());
        for &[a, b, c] in &mesh.faces {
            let (a, b, c) = (
                mesh.vertices[a as usize],
                mesh.vertices[b as usize],
                mesh.vertices[c as usize],
            );
            centroids.push(Point3::from((a.coords + b.coords + c.coords) / 3.0));
            let n = (b - a).cross(&(c - a));
            let len = n.norm();
            normals.push(if len > MIN_NORMAL_LEN {
                n / len
            } else {
                DEGENERATE_NORMAL
            });
        }
        Self { centroids, normals }
    }

    /// The number of faces.
    #[must_use]
    pub const fn len(&self) -> usize {
        self.centroids.len()
    }

    /// Whether the field has no faces.
    #[must_use]
    pub const fn is_empty(&self) -> bool {
        self.centroids.is_empty()
    }

    /// The face ids the brush covers: every face whose centroid is within
    /// `radius` of face `seed`'s centroid.
    ///
    /// When `max_angle_deg` is `Some(a)`, the result is additionally restricted
    /// to faces whose normal is within `a` degrees of `seed`'s normal — so a
    /// brush on a flat region does not spill onto the steep walls beside it.
    /// `None` disables the filter (distance only).
    ///
    /// The `seed` face itself is always covered (its centroid is at distance 0).
    /// An out-of-range `seed` yields an empty selection.
    #[must_use]
    pub fn brush(&self, seed: usize, radius: f64, max_angle_deg: Option<f64>) -> Vec<usize> {
        let (Some(&centre), Some(&reference)) = (self.centroids.get(seed), self.normals.get(seed))
        else {
            return Vec::new();
        };
        let r2 = radius * radius;
        let cos_min = max_angle_deg.map(|a| a.to_radians().cos());
        let mut hits = Vec::new();
        for (f, (c, n)) in self.centroids.iter().zip(self.normals.iter()).enumerate() {
            if (c - centre).norm_squared() > r2 {
                continue;
            }
            if let Some(cos_min) = cos_min {
                if n.dot(&reference) < cos_min {
                    continue; // normal too different from the seed face
                }
            }
            hits.push(f);
        }
        hits
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    /// A tiny triangle in the z=0 plane centred at `(cx, 0, 0)`, normal +Z.
    fn flat_triangle_at(cx: f64) -> [Point3<f64>; 3] {
        [
            Point3::new(cx - 0.1, -0.1, 0.0),
            Point3::new(cx + 0.1, -0.1, 0.0),
            Point3::new(cx, 0.2, 0.0),
        ]
    }

    /// Build a mesh from a list of independent triangles (3 verts each).
    // Test fixtures hold a handful of vertices; the usize→u32 index cast can't truncate.
    #[allow(clippy::cast_possible_truncation)]
    fn mesh_of(triangles: &[[Point3<f64>; 3]]) -> IndexedMesh {
        let mut vertices = Vec::new();
        let mut faces = Vec::new();
        for tri in triangles {
            let base = vertices.len() as u32;
            vertices.extend_from_slice(tri);
            faces.push([base, base + 1, base + 2]);
        }
        IndexedMesh::from_parts(vertices, faces)
    }

    #[test]
    fn new_computes_centroid_and_unit_normal() {
        let field = FaceField::new(&mesh_of(&[flat_triangle_at(2.0)]));
        assert_eq!(field.len(), 1);
        assert!(!field.is_empty());
        assert!((field.centroids[0] - Point3::new(2.0, 0.0, 0.0)).norm() < 1e-12);
        assert!((field.normals[0] - Vector3::new(0.0, 0.0, 1.0)).norm() < 1e-12);
    }

    #[test]
    fn degenerate_face_gets_fallback_normal() {
        // Three collinear points → zero-area face.
        let tri = [
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(2.0, 0.0, 0.0),
        ];
        let field = FaceField::new(&mesh_of(&[tri]));
        assert_eq!(field.normals[0], DEGENERATE_NORMAL);
    }

    #[test]
    fn brush_covers_faces_within_radius() {
        // Five faces with centroids at x = 0,1,2,3,4.
        let field = FaceField::new(&mesh_of(&[
            flat_triangle_at(0.0),
            flat_triangle_at(1.0),
            flat_triangle_at(2.0),
            flat_triangle_at(3.0),
            flat_triangle_at(4.0),
        ]));
        // Brush at face 2 (x=2) with radius 1.5 covers x = 1,2,3.
        let mut hits = field.brush(2, 1.5, None);
        hits.sort_unstable();
        assert_eq!(hits, vec![1, 2, 3]);
    }

    #[test]
    fn brush_always_covers_the_seed_face() {
        let field = FaceField::new(&mesh_of(&[flat_triangle_at(0.0), flat_triangle_at(100.0)]));
        // Tiny radius: only the seed is within range.
        assert_eq!(field.brush(1, 0.001, None), vec![1]);
    }

    #[test]
    fn brush_normal_filter_excludes_steep_faces() {
        // Face 0: flat (normal +Z). Face 1: a triangle in the x=const plane
        // (normal ±X), centroid adjacent to face 0 so it is within radius.
        let flat = flat_triangle_at(0.0);
        let steep = [
            Point3::new(0.05, -0.1, -0.1),
            Point3::new(0.05, 0.1, -0.1),
            Point3::new(0.05, 0.0, 0.2),
        ];
        let field = FaceField::new(&mesh_of(&[flat, steep]));
        // Without the filter both are within radius.
        let mut all = field.brush(0, 1.0, None);
        all.sort_unstable();
        assert_eq!(all, vec![0, 1]);
        // With a 35° filter the ~90°-different steep face is excluded.
        assert_eq!(field.brush(0, 1.0, Some(35.0)), vec![0]);
    }

    #[test]
    fn brush_keeps_a_face_exactly_on_the_radius() {
        // Centroids at x=0 (seed) and x=2. With radius exactly 2 the far face's
        // squared centroid-distance equals r², and the strict `>` keeps it; a
        // hair under the radius drops it.
        let field = FaceField::new(&mesh_of(&[flat_triangle_at(0.0), flat_triangle_at(2.0)]));
        let mut on_edge = field.brush(0, 2.0, None);
        on_edge.sort_unstable();
        assert_eq!(on_edge, vec![0, 1], "a face exactly at the radius is kept");
        assert_eq!(field.brush(0, 1.999, None), vec![0], "just inside drops it");
    }

    #[test]
    fn brush_out_of_range_seed_is_empty() {
        let field = FaceField::new(&mesh_of(&[flat_triangle_at(0.0)]));
        assert!(field.brush(9, 10.0, None).is_empty());
    }
}
