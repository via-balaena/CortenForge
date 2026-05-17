//! Signed distance field computation.
//!
//! Computes the signed distance from any point to the nearest surface of a mesh.
//! Backed by parry3d's BVH-accelerated TriMesh: per-query cost is O(log faces).
//!
//! # Sign contract
//!
//! For **watertight** meshes, sign is reliable (pseudo-normal-based via parry).
//! For **non-manifold** meshes (e.g., cap-stripped open meshes), sign is
//! undefined; consumers should use [`SignedDistanceField::unsigned_distance`]
//! instead. The construction never panics on non-manifold input.

use mesh_types::IndexedMesh;
use nalgebra::Point3;
use parry3d::math::{Point as ParryPoint, Real as ParryReal};
use parry3d::query::PointQuery;
use parry3d::shape::{TriMesh, TriMeshFlags};

use crate::error::{SdfError, SdfResult};

/// A signed distance field for a mesh.
///
/// Provides efficient distance queries via parry3d's BVH-accelerated TriMesh.
#[derive(Debug, Clone)]
pub struct SignedDistanceField {
    /// The mesh for which this SDF was computed (kept for the `mesh()` accessor).
    mesh: IndexedMesh,
    /// parry's BVH-backed TriMesh; backs all queries.
    tri_mesh: TriMesh,
}

impl SignedDistanceField {
    /// Create a new signed distance field from a mesh.
    ///
    /// # Arguments
    ///
    /// * `mesh` - The mesh to compute the SDF for
    ///
    /// # Errors
    ///
    /// Returns an error if the mesh is empty.
    ///
    /// # Example
    ///
    /// ```
    /// use mesh_types::{IndexedMesh, Point3};
    /// use mesh_sdf::SignedDistanceField;
    ///
    /// let mut mesh = IndexedMesh::new();
    /// mesh.vertices.push(Point3::new(0.0, 0.0, 0.0));
    /// mesh.vertices.push(Point3::new(1.0, 0.0, 0.0));
    /// mesh.vertices.push(Point3::new(0.0, 1.0, 0.0));
    /// mesh.faces.push([0, 1, 2]);
    ///
    /// let sdf = SignedDistanceField::new(mesh);
    /// assert!(sdf.is_ok());
    /// ```
    pub fn new(mesh: IndexedMesh) -> SdfResult<Self> {
        if mesh.faces.is_empty() {
            return Err(SdfError::EmptyMesh);
        }

        let tri_mesh = build_tri_mesh(&mesh);

        Ok(Self { mesh, tri_mesh })
    }

    /// Query the signed distance at a point.
    ///
    /// Returns the distance to the nearest surface. Positive values indicate
    /// the point is outside the mesh, negative values indicate inside.
    ///
    /// Sign is reliable for watertight meshes; undefined for non-manifold
    /// meshes (see crate-level docs).
    ///
    /// # Arguments
    ///
    /// * `point` - The point to query
    ///
    /// # Example
    ///
    /// ```
    /// use mesh_types::IndexedMesh;
    /// use mesh_sdf::SignedDistanceField;
    /// use nalgebra::Point3;
    ///
    /// // Create a simple triangle
    /// let mut mesh = IndexedMesh::new();
    /// mesh.vertices.push(Point3::new(0.0, 0.0, 0.0));
    /// mesh.vertices.push(Point3::new(10.0, 0.0, 0.0));
    /// mesh.vertices.push(Point3::new(5.0, 10.0, 0.0));
    /// mesh.faces.push([0, 1, 2]);
    ///
    /// let sdf = SignedDistanceField::new(mesh).unwrap();
    ///
    /// // Point above the triangle
    /// let dist = sdf.distance(Point3::new(5.0, 5.0, 5.0));
    /// assert!(dist > 0.0); // Positive = outside
    /// ```
    #[must_use]
    pub fn distance(&self, point: Point3<f64>) -> f64 {
        let projection = self
            .tri_mesh
            .project_local_point(&f64_to_parry(point), false);
        let unsigned = (point - parry_to_f64(projection.point)).norm();
        if projection.is_inside {
            -unsigned
        } else {
            unsigned
        }
    }

    /// Query the unsigned distance at a point.
    ///
    /// Returns the absolute distance to the nearest surface.
    #[must_use]
    pub fn unsigned_distance(&self, point: Point3<f64>) -> f64 {
        let projection = self
            .tri_mesh
            .project_local_point(&f64_to_parry(point), false);
        (point - parry_to_f64(projection.point)).norm()
    }

    /// Query the closest point on the mesh surface.
    ///
    /// # Arguments
    ///
    /// * `point` - The query point
    ///
    /// # Returns
    ///
    /// The closest point on the mesh surface.
    #[must_use]
    pub fn closest_point(&self, point: Point3<f64>) -> Point3<f64> {
        let projection = self
            .tri_mesh
            .project_local_point(&f64_to_parry(point), false);
        parry_to_f64(projection.point)
    }

    /// Check if a point is inside the mesh.
    ///
    /// Reliable for watertight meshes (pseudo-normal-based); undefined for
    /// non-manifold meshes.
    #[must_use]
    pub fn is_inside(&self, point: Point3<f64>) -> bool {
        self.tri_mesh
            .project_local_point(&f64_to_parry(point), false)
            .is_inside
    }

    /// Get a reference to the underlying mesh.
    #[must_use]
    pub fn mesh(&self) -> &IndexedMesh {
        &self.mesh
    }
}

/// Build a parry `TriMesh` from an `IndexedMesh`.
///
/// Sets `TriMeshFlags::ORIENTED` so pseudo-normals are computed; this enables
/// the reliable `is_inside` path on watertight meshes. On non-manifold input,
/// pseudo-normal computation may be partial — `is_inside` then becomes
/// undefined (consumers should use `unsigned_distance`).
fn build_tri_mesh(mesh: &IndexedMesh) -> TriMesh {
    let vertices: Vec<ParryPoint<ParryReal>> = mesh
        .vertices
        .iter()
        .map(|v| ParryPoint::new(v.x as ParryReal, v.y as ParryReal, v.z as ParryReal))
        .collect();
    let indices: Vec<[u32; 3]> = mesh.faces.to_vec();
    TriMesh::with_flags(vertices, indices, TriMeshFlags::ORIENTED)
}

#[inline]
fn f64_to_parry(p: Point3<f64>) -> ParryPoint<ParryReal> {
    ParryPoint::new(p.x as ParryReal, p.y as ParryReal, p.z as ParryReal)
}

#[inline]
fn parry_to_f64(p: ParryPoint<ParryReal>) -> Point3<f64> {
    Point3::new(p.x as f64, p.y as f64, p.z as f64)
}

/// Compute the signed distance from a point to a mesh without precomputation.
///
/// For one-off queries, this is simpler than creating a `SignedDistanceField`.
/// For multiple queries, prefer creating a `SignedDistanceField` once and
/// reusing it — construction includes a BVH build that's wasted on a single query.
///
/// # Arguments
///
/// * `point` - The point to query
/// * `mesh` - The mesh
///
/// # Returns
///
/// The signed distance (positive = outside, negative = inside). Returns
/// `f64::MAX` on an empty mesh. Sign contract matches
/// [`SignedDistanceField::distance`].
///
/// # Example
///
/// ```
/// use mesh_types::IndexedMesh;
/// use mesh_sdf::signed_distance;
/// use nalgebra::Point3;
///
/// let mut mesh = IndexedMesh::new();
/// mesh.vertices.push(Point3::new(0.0, 0.0, 0.0));
/// mesh.vertices.push(Point3::new(10.0, 0.0, 0.0));
/// mesh.vertices.push(Point3::new(5.0, 10.0, 0.0));
/// mesh.faces.push([0, 1, 2]);
///
/// let dist = signed_distance(Point3::new(5.0, 5.0, 5.0), &mesh);
/// ```
#[must_use]
pub fn signed_distance(point: Point3<f64>, mesh: &IndexedMesh) -> f64 {
    if mesh.faces.is_empty() {
        return f64::MAX;
    }
    let tri_mesh = build_tri_mesh(mesh);
    let projection = tri_mesh.project_local_point(&f64_to_parry(point), false);
    let unsigned = (point - parry_to_f64(projection.point)).norm();
    if projection.is_inside {
        -unsigned
    } else {
        unsigned
    }
}

/// Compute the unsigned distance from a point to a mesh.
///
/// # Arguments
///
/// * `point` - The point to query
/// * `mesh` - The mesh
///
/// # Returns
///
/// The unsigned (absolute) distance to the nearest surface. Returns
/// `f64::MAX` on an empty mesh.
#[must_use]
pub fn unsigned_distance(point: Point3<f64>, mesh: &IndexedMesh) -> f64 {
    if mesh.faces.is_empty() {
        return f64::MAX;
    }
    let tri_mesh = build_tri_mesh(mesh);
    let projection = tri_mesh.project_local_point(&f64_to_parry(point), false);
    (point - parry_to_f64(projection.point)).norm()
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use mesh_types::Point3;

    fn simple_triangle() -> IndexedMesh {
        let mut mesh = IndexedMesh::new();
        mesh.vertices.push(Point3::new(0.0, 0.0, 0.0));
        mesh.vertices.push(Point3::new(10.0, 0.0, 0.0));
        mesh.vertices.push(Point3::new(5.0, 10.0, 0.0));
        mesh.faces.push([0, 1, 2]);
        mesh
    }

    fn unit_tetrahedron() -> IndexedMesh {
        let mut mesh = IndexedMesh::new();
        mesh.vertices.push(Point3::new(0.0, 0.0, 0.0));
        mesh.vertices.push(Point3::new(1.0, 0.0, 0.0));
        mesh.vertices.push(Point3::new(0.5, 0.866, 0.0));
        mesh.vertices.push(Point3::new(0.5, 0.289, 0.816));

        // CCW winding when viewed from outside
        mesh.faces.push([0, 2, 1]); // bottom
        mesh.faces.push([0, 1, 3]); // front
        mesh.faces.push([1, 2, 3]); // right
        mesh.faces.push([2, 0, 3]); // left
        mesh
    }

    /// Open-top unit cube (no +z face): 10 triangles instead of 12.
    /// Non-manifold by construction — sign is undefined by contract.
    fn open_top_unit_cube() -> IndexedMesh {
        let mut mesh = IndexedMesh::new();
        // 8 corners of the unit cube
        mesh.vertices.push(Point3::new(0.0, 0.0, 0.0)); // 0: ---
        mesh.vertices.push(Point3::new(1.0, 0.0, 0.0)); // 1: +--
        mesh.vertices.push(Point3::new(1.0, 1.0, 0.0)); // 2: ++-
        mesh.vertices.push(Point3::new(0.0, 1.0, 0.0)); // 3: -+-
        mesh.vertices.push(Point3::new(0.0, 0.0, 1.0)); // 4: --+
        mesh.vertices.push(Point3::new(1.0, 0.0, 1.0)); // 5: +-+
        mesh.vertices.push(Point3::new(1.0, 1.0, 1.0)); // 6: +++
        mesh.vertices.push(Point3::new(0.0, 1.0, 1.0)); // 7: -++

        // Bottom (-z), outward normal -z
        mesh.faces.push([0, 2, 1]);
        mesh.faces.push([0, 3, 2]);
        // -y face
        mesh.faces.push([0, 1, 5]);
        mesh.faces.push([0, 5, 4]);
        // +x face
        mesh.faces.push([1, 2, 6]);
        mesh.faces.push([1, 6, 5]);
        // +y face
        mesh.faces.push([2, 3, 7]);
        mesh.faces.push([2, 7, 6]);
        // -x face
        mesh.faces.push([3, 0, 4]);
        mesh.faces.push([3, 4, 7]);
        // +z face deliberately omitted -> open top, non-manifold.
        mesh
    }

    /// Closed pyramid (square base + apex), watertight via a cap-fan on the
    /// base. This is the kind of geometry where the old face-normal sign
    /// heuristic could flip far from the surface — at probes near the apex
    /// axis, "the closest face's plane happens to lie between the probe and
    /// the rest of the body" (mesh-sdf's documented failure mode). parry's
    /// pseudo-normal at the apex aggregates the four sloped-side normals,
    /// giving a consistent outward direction regardless of which slope face
    /// happens to be returned as the BVH leaf.
    fn closed_pyramid() -> IndexedMesh {
        let mut mesh = IndexedMesh::new();
        // 4 base corners at z=0, apex at z=1.
        mesh.vertices.push(Point3::new(-1.0, -1.0, 0.0)); // 0
        mesh.vertices.push(Point3::new(1.0, -1.0, 0.0)); // 1
        mesh.vertices.push(Point3::new(1.0, 1.0, 0.0)); // 2
        mesh.vertices.push(Point3::new(-1.0, 1.0, 0.0)); // 3
        mesh.vertices.push(Point3::new(0.0, 0.0, 1.0)); // 4: apex

        // Base, outward normal -z (CCW from below).
        mesh.faces.push([0, 2, 1]);
        mesh.faces.push([0, 3, 2]);
        // 4 sloped sides, CCW from outside.
        mesh.faces.push([0, 1, 4]); // -y side
        mesh.faces.push([1, 2, 4]); // +x side
        mesh.faces.push([2, 3, 4]); // +y side
        mesh.faces.push([3, 0, 4]); // -x side
        mesh
    }

    #[test]
    fn sdf_new_empty_mesh() {
        let mesh = IndexedMesh::new();
        let result = SignedDistanceField::new(mesh);
        assert!(result.is_err());
    }

    #[test]
    fn sdf_new_valid_mesh() {
        let mesh = simple_triangle();
        let result = SignedDistanceField::new(mesh);
        assert!(result.is_ok());
    }

    #[test]
    fn sdf_distance_above_triangle() {
        let mesh = simple_triangle();
        let sdf = SignedDistanceField::new(mesh).expect("should create SDF");

        // Point directly above center of triangle
        let dist = sdf.distance(Point3::new(5.0, 3.33, 5.0));
        assert_relative_eq!(dist.abs(), 5.0, epsilon = 0.1);
    }

    #[test]
    fn sdf_distance_on_surface() {
        let mesh = simple_triangle();
        let sdf = SignedDistanceField::new(mesh).expect("should create SDF");

        // Point on the triangle
        let dist = sdf.distance(Point3::new(5.0, 3.0, 0.0));
        assert_relative_eq!(dist.abs(), 0.0, epsilon = 0.01);
    }

    #[test]
    fn sdf_closest_point() {
        let mesh = simple_triangle();
        let sdf = SignedDistanceField::new(mesh).expect("should create SDF");

        // Point above center
        let query = Point3::new(5.0, 3.0, 5.0);
        let closest = sdf.closest_point(query);

        // Closest point should be on the plane z=0
        assert_relative_eq!(closest.z, 0.0, epsilon = 0.01);
    }

    #[test]
    fn signed_distance_standalone() {
        let mesh = simple_triangle();
        let point = Point3::new(5.0, 3.0, 5.0);

        let dist = signed_distance(point, &mesh);
        assert!(dist.abs() > 0.0);
    }

    #[test]
    fn unsigned_distance_standalone() {
        let mesh = simple_triangle();
        let point = Point3::new(5.0, 3.0, 5.0);

        let dist = unsigned_distance(point, &mesh);
        assert!(dist >= 0.0);
        assert_relative_eq!(dist, 5.0, epsilon = 0.1);
    }

    #[test]
    fn sdf_inside_tetrahedron() {
        let mesh = unit_tetrahedron();
        let sdf = SignedDistanceField::new(mesh).expect("should create SDF");

        // Centroid of tetrahedron is inside
        let centroid = Point3::new(0.5, 0.385, 0.204);
        assert!(sdf.is_inside(centroid) || sdf.distance(centroid) < 0.1);
    }

    #[test]
    fn sdf_outside_tetrahedron() {
        let mesh = unit_tetrahedron();
        let sdf = SignedDistanceField::new(mesh).expect("should create SDF");

        // Point far outside
        let outside = Point3::new(10.0, 10.0, 10.0);
        assert!(!sdf.is_inside(outside));
    }

    #[test]
    fn sdf_mesh_accessor() {
        let mesh = simple_triangle();
        let sdf = SignedDistanceField::new(mesh).expect("should create SDF");

        assert_eq!(sdf.mesh().faces.len(), 1);
    }

    #[test]
    fn empty_mesh_distance() {
        let mesh = IndexedMesh::new();
        let dist = unsigned_distance(Point3::new(0.0, 0.0, 0.0), &mesh);
        assert_eq!(dist, f64::MAX);
    }

    /// Far-field sign is reliable on a watertight pyramid (cap-fan style)
    /// — the failure mode the old face-normal heuristic produced on real
    /// cleaned scans (see project memory `pinned-floor-visual-gate-postmortem`,
    /// `min_sdf = -89.5 mm` on a 71 mm body). With parry's pseudo-normals,
    /// probes far above / below / lateral of the pyramid all get the
    /// expected outside sign, independent of which BVH leaf face is hit.
    #[test]
    fn far_field_sign_reliable_on_pathological_cap_fan() {
        let mesh = closed_pyramid();
        let sdf = SignedDistanceField::new(mesh).expect("should create SDF");

        // Probes far outside in every cardinal direction.
        let probes_outside = [
            Point3::new(0.0, 0.0, 5.0),  // above apex
            Point3::new(0.0, 0.0, -5.0), // below base
            Point3::new(5.0, 0.0, 0.5),  // far +x at mid-height
            Point3::new(-5.0, 0.0, 0.5), // far -x at mid-height
            Point3::new(0.0, 5.0, 0.5),  // far +y at mid-height
            Point3::new(0.0, -5.0, 0.5), // far -y at mid-height
        ];
        for p in probes_outside {
            let d = sdf.distance(p);
            assert!(
                d > 0.0,
                "outside probe at {p:?} must have positive distance, got {d}"
            );
        }

        // Probe deep inside, near apex on axis (where the old heuristic
        // could be tripped by an arbitrary sloped-side leaf face).
        let inside = Point3::new(0.0, 0.0, 0.5);
        let d_in = sdf.distance(inside);
        assert!(
            d_in < 0.0,
            "inside probe must have negative distance, got {d_in}"
        );
    }

    /// Non-manifold contract test per spec §8.
    ///
    /// On an open-top cube, `unsigned_distance` must remain correct,
    /// deterministic, and finite. `distance` must remain finite. We do NOT
    /// pin the sign — it is undefined by contract on non-manifold input.
    #[test]
    fn non_manifold_unsigned_distance_correct_and_deterministic() {
        let open_cube = open_top_unit_cube();
        let sdf = SignedDistanceField::new(open_cube).expect("should create SDF");
        let probes = [
            Point3::new(0.5, 0.5, 2.0),  // above the open top
            Point3::new(0.5, 0.5, -1.0), // below the closed bottom
            Point3::new(2.0, 0.5, 0.5),  // beside the closed +x wall
        ];
        for p in probes {
            let d1 = sdf.unsigned_distance(p);
            let d2 = sdf.unsigned_distance(p);
            assert_eq!(d1, d2, "unsigned_distance must be deterministic");
            assert!(d1.is_finite() && d1 >= 0.0);

            let signed = sdf.distance(p);
            assert!(signed.is_finite(), "distance finite even on non-manifold");
            // NO assertion on sign — undefined by contract.
        }
    }
}
