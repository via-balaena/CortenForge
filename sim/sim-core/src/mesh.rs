//! Non-convex triangle mesh collision detection.
//!
//! This module provides collision detection for arbitrary triangle meshes without
//! requiring convexification. It uses a BVH (Bounding Volume Hierarchy) for efficient
//! broad-phase culling of triangles, and dedicated triangle-primitive tests for
//! accurate narrow-phase collision detection.
//!
//! # Overview
//!
//! Unlike `ConvexMesh` which requires convex geometry, `TriangleMesh` supports:
//! - Arbitrary non-convex geometry (concave objects, objects with holes)
//! - Large meshes with many triangles (BVH acceleration)
//! - Accurate collision response from individual triangles
//!
//! # Algorithm
//!
//! 1. **BVH Construction**: Build a bounding volume hierarchy over triangles
//! 2. **Broad Phase**: Query BVH to find potentially colliding triangles
//! 3. **Narrow Phase**: Test each candidate triangle against the other shape
//! 4. **Contact Generation**: Return the deepest penetrating contact
//!
//! # Usage
//!
//! ```ignore
//! use sim_core::{CollisionShape, TriangleMeshData};
//! use nalgebra::Point3;
//! use std::sync::Arc;
//!
//! // Create vertices and triangle indices
//! let vertices = vec![
//!     Point3::new(0.0, 0.0, 0.0),
//!     Point3::new(1.0, 0.0, 0.0),
//!     Point3::new(0.5, 1.0, 0.0),
//!     Point3::new(0.5, 0.5, 1.0),
//! ];
//! let indices = vec![0, 1, 2, 0, 1, 3, 1, 2, 3, 0, 2, 3]; // 4 triangles (tetrahedron)
//!
//! // Build the mesh with BVH
//! let mesh_data = TriangleMeshData::new(vertices, indices);
//!
//! // Create collision shape
//! let shape = CollisionShape::triangle_mesh(Arc::new(mesh_data));
//! ```
//!
//! # Performance
//!
//! - BVH construction: O(n log n) where n is the number of triangles
//! - Collision query: O(log n) average case, O(n) worst case
//! - Memory: O(n) for vertices, indices, and BVH nodes

use nalgebra::{Point3, Vector3};
use sim_types::Pose;

use crate::mid_phase::{Bvh, BvhPrimitive};

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

/// Small epsilon for numerical comparisons.
const EPSILON: f64 = 1e-10;

/// A single triangle defined by three vertex indices.
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct Triangle {
    /// Index of first vertex.
    pub v0: usize,
    /// Index of second vertex.
    pub v1: usize,
    /// Index of third vertex.
    pub v2: usize,
}

impl Triangle {
    /// Create a new triangle from vertex indices.
    #[must_use]
    pub const fn new(v0: usize, v1: usize, v2: usize) -> Self {
        Self { v0, v1, v2 }
    }
}

/// Triangle mesh collision data.
///
/// Stores vertices, triangle indices, and a BVH for efficient collision queries.
/// The mesh is defined in local coordinates and will be transformed using the
/// body's pose during collision detection.
#[derive(Debug, Clone)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct TriangleMeshData {
    /// Vertices of the mesh in local coordinates.
    vertices: Vec<Point3<f64>>,
    /// Triangle definitions (groups of 3 vertex indices).
    triangles: Vec<Triangle>,
    /// Cached AABB for the entire mesh (local coordinates).
    aabb_min: Point3<f64>,
    aabb_max: Point3<f64>,
    /// BVH for efficient collision queries.
    /// Skipped during serialization, rebuilt on demand.
    #[cfg_attr(feature = "serde", serde(skip))]
    bvh: Option<Bvh>,
}

impl PartialEq for TriangleMeshData {
    fn eq(&self, other: &Self) -> bool {
        self.vertices == other.vertices && self.triangles == other.triangles
    }
}

impl TriangleMeshData {
    /// Create a new triangle mesh from vertices and indices.
    ///
    /// # Arguments
    ///
    /// * `vertices` - The mesh vertices in local coordinates
    /// * `indices` - Triangle indices (must be a multiple of 3)
    ///
    /// # Panics
    ///
    /// Panics if `indices.len()` is not a multiple of 3 or if any index is out of bounds.
    #[must_use]
    #[allow(clippy::needless_pass_by_value)] // Taking Vec for API consistency with other constructors
    pub fn new(vertices: Vec<Point3<f64>>, indices: Vec<usize>) -> Self {
        assert!(
            indices.len() % 3 == 0,
            "Triangle indices must be a multiple of 3"
        );

        // Validate indices
        let max_vertex = vertices.len();
        for &idx in &indices {
            assert!(
                idx < max_vertex,
                "Triangle index {} out of bounds (max: {})",
                idx,
                max_vertex - 1
            );
        }

        // Build triangles
        let triangles: Vec<Triangle> = indices
            .chunks(3)
            .map(|chunk| Triangle::new(chunk[0], chunk[1], chunk[2]))
            .collect();

        // Compute AABB
        let (aabb_min, aabb_max) = Self::compute_aabb(&vertices);

        // Build BVH
        let bvh = Self::build_bvh(&vertices, &triangles);

        Self {
            vertices,
            triangles,
            aabb_min,
            aabb_max,
            bvh: Some(bvh),
        }
    }

    /// Create a triangle mesh from separate vertex and triangle arrays.
    ///
    /// # Panics
    ///
    /// Panics if any triangle index is out of bounds.
    #[must_use]
    pub fn from_triangles(vertices: Vec<Point3<f64>>, triangles: Vec<Triangle>) -> Self {
        // Validate indices
        let max_vertex = vertices.len();
        for tri in &triangles {
            assert!(
                tri.v0 < max_vertex && tri.v1 < max_vertex && tri.v2 < max_vertex,
                "Triangle index out of bounds"
            );
        }

        // Compute AABB
        let (aabb_min, aabb_max) = Self::compute_aabb(&vertices);

        // Build BVH
        let bvh = Self::build_bvh(&vertices, &triangles);

        Self {
            vertices,
            triangles,
            aabb_min,
            aabb_max,
            bvh: Some(bvh),
        }
    }

    /// Compute the AABB for a set of vertices.
    fn compute_aabb(vertices: &[Point3<f64>]) -> (Point3<f64>, Point3<f64>) {
        if vertices.is_empty() {
            return (Point3::origin(), Point3::origin());
        }

        let mut min = Point3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
        let mut max = Point3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);

        for v in vertices {
            min.x = min.x.min(v.x);
            min.y = min.y.min(v.y);
            min.z = min.z.min(v.z);
            max.x = max.x.max(v.x);
            max.y = max.y.max(v.y);
            max.z = max.z.max(v.z);
        }

        (min, max)
    }

    /// Build a BVH from the triangles.
    fn build_bvh(vertices: &[Point3<f64>], triangles: &[Triangle]) -> Bvh {
        let primitives: Vec<BvhPrimitive> = triangles
            .iter()
            .enumerate()
            .map(|(idx, tri)| {
                let v0 = vertices[tri.v0];
                let v1 = vertices[tri.v1];
                let v2 = vertices[tri.v2];
                BvhPrimitive::from_triangle(v0, v1, v2, idx)
            })
            .collect();

        Bvh::build(primitives)
    }

    /// Ensure the BVH is built (for deserialized meshes).
    fn ensure_bvh(&mut self) {
        if self.bvh.is_none() {
            self.bvh = Some(Self::build_bvh(&self.vertices, &self.triangles));
        }
    }

    /// Get the vertices.
    #[must_use]
    pub fn vertices(&self) -> &[Point3<f64>] {
        &self.vertices
    }

    /// Get the triangles.
    #[must_use]
    pub fn triangles(&self) -> &[Triangle] {
        &self.triangles
    }

    /// Get the number of triangles.
    #[must_use]
    pub fn triangle_count(&self) -> usize {
        self.triangles.len()
    }

    /// Get the number of vertices.
    #[must_use]
    pub fn vertex_count(&self) -> usize {
        self.vertices.len()
    }

    /// Get the local-space AABB.
    #[must_use]
    pub fn aabb(&self) -> (Point3<f64>, Point3<f64>) {
        (self.aabb_min, self.aabb_max)
    }

    /// Get the center of the AABB.
    #[must_use]
    pub fn center(&self) -> Point3<f64> {
        Point3::from((self.aabb_min.coords + self.aabb_max.coords) * 0.5)
    }

    /// Get a triangle's vertices.
    #[must_use]
    pub fn triangle_vertices(&self, tri: &Triangle) -> (Point3<f64>, Point3<f64>, Point3<f64>) {
        (
            self.vertices[tri.v0],
            self.vertices[tri.v1],
            self.vertices[tri.v2],
        )
    }

    /// Get a triangle by index.
    #[must_use]
    pub fn get_triangle(&self, index: usize) -> Option<&Triangle> {
        self.triangles.get(index)
    }

    /// Compute the normal of a triangle.
    #[must_use]
    pub fn triangle_normal(&self, tri: &Triangle) -> Vector3<f64> {
        let (v0, v1, v2) = self.triangle_vertices(tri);
        let e1 = v1 - v0;
        let e2 = v2 - v0;
        let normal = e1.cross(&e2);
        let norm = normal.norm();
        if norm > EPSILON {
            normal / norm
        } else {
            Vector3::z() // Degenerate triangle
        }
    }

    /// Get the BVH for collision queries.
    #[must_use]
    pub fn bvh(&self) -> Option<&Bvh> {
        self.bvh.as_ref()
    }
}

/// Contact result from triangle mesh collision.
#[derive(Debug, Clone)]
pub struct MeshContact {
    /// Contact point on the mesh surface (world space).
    pub point: Point3<f64>,
    /// Surface normal at contact point (pointing outward from mesh).
    pub normal: Vector3<f64>,
    /// Penetration depth (positive when object is inside the mesh surface).
    pub penetration: f64,
    /// Index of the triangle that generated this contact.
    pub triangle_index: usize,
}

// =============================================================================
// Triangle-Primitive Collision Tests
// =============================================================================

/// Test collision between a triangle and a sphere.
///
/// Returns the contact point, normal, and penetration depth if colliding.
#[must_use]
pub fn triangle_sphere_contact(
    v0: Point3<f64>,
    v1: Point3<f64>,
    v2: Point3<f64>,
    sphere_center: Point3<f64>,
    sphere_radius: f64,
) -> Option<(Point3<f64>, Vector3<f64>, f64)> {
    // Find the closest point on the triangle to the sphere center
    let closest = closest_point_on_triangle(v0, v1, v2, sphere_center);

    // Check if the closest point is within sphere radius
    let diff = sphere_center - closest;
    let dist_sq = diff.norm_squared();
    let radius_sq = sphere_radius * sphere_radius;

    if dist_sq > radius_sq {
        return None;
    }

    let dist = dist_sq.sqrt();
    let penetration = sphere_radius - dist;

    // Normal points from triangle toward sphere center
    let normal = if dist > EPSILON {
        diff / dist
    } else {
        // Sphere center is on the triangle - use triangle normal
        let e1 = v1 - v0;
        let e2 = v2 - v0;
        let tri_normal = e1.cross(&e2);
        let norm = tri_normal.norm();
        if norm > EPSILON {
            tri_normal / norm
        } else {
            Vector3::z()
        }
    };

    Some((closest, normal, penetration))
}

/// Find the closest point on a triangle to a given point.
///
/// Uses barycentric coordinates to determine the closest feature
/// (vertex, edge, or face).
#[must_use]
pub fn closest_point_on_triangle(
    v0: Point3<f64>,
    v1: Point3<f64>,
    v2: Point3<f64>,
    p: Point3<f64>,
) -> Point3<f64> {
    // Check if P is in vertex region outside V0
    let ab = v1 - v0;
    let ac = v2 - v0;
    let ap = p - v0;

    let d1 = ab.dot(&ap);
    let d2 = ac.dot(&ap);
    if d1 <= 0.0 && d2 <= 0.0 {
        return v0; // Closest to vertex V0
    }

    // Check if P is in vertex region outside V1
    let bp = p - v1;
    let d3 = ab.dot(&bp);
    let d4 = ac.dot(&bp);
    if d3 >= 0.0 && d4 <= d3 {
        return v1; // Closest to vertex V1
    }

    // Check if P is in edge region of AB
    let vc = d1 * d4 - d3 * d2;
    if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
        let v = d1 / (d1 - d3);
        return Point3::from(v0.coords + ab * v); // On edge AB
    }

    // Check if P is in vertex region outside V2
    let cp = p - v2;
    let d5 = ab.dot(&cp);
    let d6 = ac.dot(&cp);
    if d6 >= 0.0 && d5 <= d6 {
        return v2; // Closest to vertex V2
    }

    // Check if P is in edge region of AC
    let vb = d5 * d2 - d1 * d6;
    if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
        let w = d2 / (d2 - d6);
        return Point3::from(v0.coords + ac * w); // On edge AC
    }

    // Check if P is in edge region of BC
    let va = d3 * d6 - d5 * d4;
    if va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0 {
        let w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return Point3::from(v1.coords + (v2 - v1) * w); // On edge BC
    }

    // P is inside the triangle
    let denom = 1.0 / (va + vb + vc);
    let v = vb * denom;
    let w = vc * denom;
    Point3::from(v0.coords + ab * v + ac * w)
}

/// Test collision between a triangle and a capsule.
///
/// The capsule is defined by two endpoints and a radius.
#[must_use]
pub fn triangle_capsule_contact(
    v0: Point3<f64>,
    v1: Point3<f64>,
    v2: Point3<f64>,
    capsule_start: Point3<f64>,
    capsule_end: Point3<f64>,
    capsule_radius: f64,
) -> Option<(Point3<f64>, Vector3<f64>, f64)> {
    // Find the closest point on the capsule axis to the triangle
    let closest_on_axis =
        closest_point_on_segment_to_triangle(capsule_start, capsule_end, v0, v1, v2);

    // Now test as a sphere at that point
    triangle_sphere_contact(v0, v1, v2, closest_on_axis, capsule_radius)
}

/// Find the point on a line segment closest to a triangle.
fn closest_point_on_segment_to_triangle(
    seg_start: Point3<f64>,
    seg_end: Point3<f64>,
    v0: Point3<f64>,
    v1: Point3<f64>,
    v2: Point3<f64>,
) -> Point3<f64> {
    // Sample points along the segment and find the one closest to the triangle
    // A more accurate method would solve the closest pair problem analytically,
    // but this is simpler and sufficient for most cases.
    let segment = seg_end - seg_start;
    let segment_len = segment.norm();

    if segment_len < EPSILON {
        return seg_start;
    }

    // Sample at endpoints and middle
    let samples = [0.0, 0.25, 0.5, 0.75, 1.0];
    let mut best_point = seg_start;
    let mut best_dist_sq = f64::INFINITY;

    for t in samples {
        let point_on_seg = Point3::from(seg_start.coords + segment * t);
        let closest_on_tri = closest_point_on_triangle(v0, v1, v2, point_on_seg);
        let dist_sq = (point_on_seg - closest_on_tri).norm_squared();

        if dist_sq < best_dist_sq {
            best_dist_sq = dist_sq;
            best_point = point_on_seg;
        }
    }

    best_point
}

/// Test collision between a triangle and a box (AABB test followed by SAT).
///
/// This is more complex as it requires proper separation axis testing.
#[must_use]
pub fn triangle_box_contact(
    v0: Point3<f64>,
    v1: Point3<f64>,
    v2: Point3<f64>,
    box_center: Point3<f64>,
    box_rotation: &nalgebra::UnitQuaternion<f64>,
    half_extents: &Vector3<f64>,
) -> Option<(Point3<f64>, Vector3<f64>, f64)> {
    // Transform triangle to box local space
    let inv_rot = box_rotation.inverse();
    let v0_local = Point3::from(inv_rot * (v0 - box_center));
    let v1_local = Point3::from(inv_rot * (v1 - box_center));
    let v2_local = Point3::from(inv_rot * (v2 - box_center));

    // Use AABB-triangle test in box local space (box is axis-aligned)
    if !aabb_triangle_intersect(*half_extents, v0_local, v1_local, v2_local) {
        return None;
    }

    // Find the closest point on the triangle to the box
    // Test box corners against the triangle and triangle vertices against box
    let mut best_penetration = f64::NEG_INFINITY;
    let mut best_point = Point3::origin();
    let mut best_normal = Vector3::z();

    // Test triangle vertices against box
    for v in [v0_local, v1_local, v2_local] {
        let clamped = Point3::new(
            v.x.clamp(-half_extents.x, half_extents.x),
            v.y.clamp(-half_extents.y, half_extents.y),
            v.z.clamp(-half_extents.z, half_extents.z),
        );

        let diff = v - clamped;
        let dist = diff.norm();

        // If vertex is inside the box
        if v.x.abs() <= half_extents.x && v.y.abs() <= half_extents.y && v.z.abs() <= half_extents.z
        {
            // Find which face is closest
            let dx = half_extents.x - v.x.abs();
            let dy = half_extents.y - v.y.abs();
            let dz = half_extents.z - v.z.abs();

            let penetration = dx.min(dy).min(dz);
            if penetration > best_penetration {
                best_penetration = penetration;
                best_point = clamped;
                if dx <= dy && dx <= dz {
                    best_normal = Vector3::new(v.x.signum(), 0.0, 0.0);
                } else if dy <= dz {
                    best_normal = Vector3::new(0.0, v.y.signum(), 0.0);
                } else {
                    best_normal = Vector3::new(0.0, 0.0, v.z.signum());
                }
            }
        } else if dist < half_extents.norm() && dist > EPSILON {
            // Vertex outside but close
            let penetration = -dist;
            if penetration > best_penetration {
                best_penetration = penetration;
                best_point = clamped;
                best_normal = -diff / dist;
            }
        }
    }

    if best_penetration <= 0.0 {
        return None;
    }

    // Transform result back to world space
    let world_point = box_center + box_rotation * best_point.coords;
    let world_normal = box_rotation * best_normal;

    Some((Point3::from(world_point), world_normal, best_penetration))
}

/// AABB-triangle intersection test (in box-local space where box is axis-aligned).
fn aabb_triangle_intersect(
    half_extents: Vector3<f64>,
    v0: Point3<f64>,
    v1: Point3<f64>,
    v2: Point3<f64>,
) -> bool {
    // Use separating axis theorem

    // Test triangle AABB against box AABB first
    let tri_min = Point3::new(
        v0.x.min(v1.x).min(v2.x),
        v0.y.min(v1.y).min(v2.y),
        v0.z.min(v1.z).min(v2.z),
    );
    let tri_max = Point3::new(
        v0.x.max(v1.x).max(v2.x),
        v0.y.max(v1.y).max(v2.y),
        v0.z.max(v1.z).max(v2.z),
    );

    if tri_max.x < -half_extents.x || tri_min.x > half_extents.x {
        return false;
    }
    if tri_max.y < -half_extents.y || tri_min.y > half_extents.y {
        return false;
    }
    if tri_max.z < -half_extents.z || tri_min.z > half_extents.z {
        return false;
    }

    // Test triangle normal as separating axis
    let e0 = v1 - v0;
    let e1 = v2 - v1;
    let e2 = v0 - v2;
    let normal = e0.cross(&e1);

    if !test_axis_triangle_box(&normal, &half_extents, v0, v1, v2) {
        return false;
    }

    // Test cross products of box axes with triangle edges
    let box_axes = [Vector3::x(), Vector3::y(), Vector3::z()];
    let edges = [e0, e1, e2];

    for axis in &box_axes {
        for edge in &edges {
            let cross = axis.cross(edge);
            if cross.norm_squared() > EPSILON * EPSILON
                && !test_axis_triangle_box(&cross, &half_extents, v0, v1, v2)
            {
                return false;
            }
        }
    }

    true
}

/// Test a separating axis for triangle-box intersection.
fn test_axis_triangle_box(
    axis: &Vector3<f64>,
    half_extents: &Vector3<f64>,
    v0: Point3<f64>,
    v1: Point3<f64>,
    v2: Point3<f64>,
) -> bool {
    // Project triangle vertices onto axis
    let p0 = axis.dot(&v0.coords);
    let p1 = axis.dot(&v1.coords);
    let p2 = axis.dot(&v2.coords);

    let tri_min = p0.min(p1).min(p2);
    let tri_max = p0.max(p1).max(p2);

    // Project box onto axis (box is centered at origin)
    let box_radius = half_extents.x * axis.x.abs()
        + half_extents.y * axis.y.abs()
        + half_extents.z * axis.z.abs();

    // Check for separation
    tri_min <= box_radius && tri_max >= -box_radius
}

// =============================================================================
// Mesh Collision Query Functions
// =============================================================================

/// Query a triangle mesh for contact with a sphere.
///
/// Uses BVH acceleration to find candidate triangles, then tests each one.
#[must_use]
pub fn mesh_sphere_contact(
    mesh: &mut TriangleMeshData,
    mesh_pose: &Pose,
    sphere_center: Point3<f64>,
    sphere_radius: f64,
) -> Option<MeshContact> {
    // Ensure BVH is built
    mesh.ensure_bvh();

    // Transform sphere center to mesh local space
    let local_center = mesh_pose.inverse_transform_point(&sphere_center);

    // Query BVH for candidate triangles
    let query_aabb = crate::broad_phase::Aabb::from_center(
        local_center,
        Vector3::new(sphere_radius, sphere_radius, sphere_radius),
    );

    let bvh = mesh.bvh.as_ref()?;
    let candidates = bvh.query(&query_aabb);

    // Test each candidate triangle
    let mut best_contact: Option<MeshContact> = None;
    let mut max_penetration = 0.0;

    for tri_idx in candidates {
        let tri = mesh.triangles.get(tri_idx)?;
        let (v0, v1, v2) = mesh.triangle_vertices(tri);

        if let Some((local_point, local_normal, penetration)) =
            triangle_sphere_contact(v0, v1, v2, local_center, sphere_radius)
        {
            if penetration > max_penetration {
                max_penetration = penetration;
                best_contact = Some(MeshContact {
                    point: mesh_pose.transform_point(&local_point),
                    normal: mesh_pose.rotation * local_normal,
                    penetration,
                    triangle_index: tri_idx,
                });
            }
        }
    }

    best_contact
}

/// Query a triangle mesh for contact with a capsule.
#[must_use]
pub fn mesh_capsule_contact(
    mesh: &mut TriangleMeshData,
    mesh_pose: &Pose,
    capsule_start: Point3<f64>,
    capsule_end: Point3<f64>,
    capsule_radius: f64,
) -> Option<MeshContact> {
    // Ensure BVH is built
    mesh.ensure_bvh();

    // Transform capsule to mesh local space
    let local_start = mesh_pose.inverse_transform_point(&capsule_start);
    let local_end = mesh_pose.inverse_transform_point(&capsule_end);

    // Compute AABB for the capsule
    let min_x = local_start.x.min(local_end.x) - capsule_radius;
    let min_y = local_start.y.min(local_end.y) - capsule_radius;
    let min_z = local_start.z.min(local_end.z) - capsule_radius;
    let max_x = local_start.x.max(local_end.x) + capsule_radius;
    let max_y = local_start.y.max(local_end.y) + capsule_radius;
    let max_z = local_start.z.max(local_end.z) + capsule_radius;

    let query_aabb = crate::broad_phase::Aabb::new(
        Point3::new(min_x, min_y, min_z),
        Point3::new(max_x, max_y, max_z),
    );

    let bvh = mesh.bvh.as_ref()?;
    let candidates = bvh.query(&query_aabb);

    // Test each candidate triangle
    let mut best_contact: Option<MeshContact> = None;
    let mut max_penetration = 0.0;

    for tri_idx in candidates {
        let tri = mesh.triangles.get(tri_idx)?;
        let (v0, v1, v2) = mesh.triangle_vertices(tri);

        if let Some((local_point, local_normal, penetration)) =
            triangle_capsule_contact(v0, v1, v2, local_start, local_end, capsule_radius)
        {
            if penetration > max_penetration {
                max_penetration = penetration;
                best_contact = Some(MeshContact {
                    point: mesh_pose.transform_point(&local_point),
                    normal: mesh_pose.rotation * local_normal,
                    penetration,
                    triangle_index: tri_idx,
                });
            }
        }
    }

    best_contact
}

/// Query a triangle mesh for contact with a box.
#[must_use]
pub fn mesh_box_contact(
    mesh: &mut TriangleMeshData,
    mesh_pose: &Pose,
    box_pose: &Pose,
    half_extents: &Vector3<f64>,
) -> Option<MeshContact> {
    // Ensure BVH is built
    mesh.ensure_bvh();

    // Transform box to mesh local space
    let local_box_center = mesh_pose.inverse_transform_point(&box_pose.position);
    let local_box_rotation = mesh_pose.rotation.inverse() * box_pose.rotation;

    // Compute AABB for the box in mesh local space
    let rot_mat = local_box_rotation.to_rotation_matrix();
    let extent_x = (rot_mat[(0, 0)] * half_extents.x).abs()
        + (rot_mat[(0, 1)] * half_extents.y).abs()
        + (rot_mat[(0, 2)] * half_extents.z).abs();
    let extent_y = (rot_mat[(1, 0)] * half_extents.x).abs()
        + (rot_mat[(1, 1)] * half_extents.y).abs()
        + (rot_mat[(1, 2)] * half_extents.z).abs();
    let extent_z = (rot_mat[(2, 0)] * half_extents.x).abs()
        + (rot_mat[(2, 1)] * half_extents.y).abs()
        + (rot_mat[(2, 2)] * half_extents.z).abs();

    let query_aabb = crate::broad_phase::Aabb::from_center(
        local_box_center,
        Vector3::new(extent_x, extent_y, extent_z),
    );

    let bvh = mesh.bvh.as_ref()?;
    let candidates = bvh.query(&query_aabb);

    // Test each candidate triangle
    let mut best_contact: Option<MeshContact> = None;
    let mut max_penetration = 0.0;

    for tri_idx in candidates {
        let tri = mesh.triangles.get(tri_idx)?;
        let (v0, v1, v2) = mesh.triangle_vertices(tri);

        if let Some((local_point, local_normal, penetration)) = triangle_box_contact(
            v0,
            v1,
            v2,
            local_box_center,
            &local_box_rotation,
            half_extents,
        ) {
            if penetration > max_penetration {
                max_penetration = penetration;
                best_contact = Some(MeshContact {
                    point: mesh_pose.transform_point(&local_point),
                    normal: mesh_pose.rotation * local_normal,
                    penetration,
                    triangle_index: tri_idx,
                });
            }
        }
    }

    best_contact
}

// =============================================================================
// Tests
// =============================================================================

#[cfg(test)]
#[allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::float_cmp,
    clippy::similar_names,
    clippy::cast_precision_loss
)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    /// Create a simple tetrahedron mesh for testing.
    fn create_tetrahedron() -> TriangleMeshData {
        let vertices = vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.5, 1.0, 0.0),
            Point3::new(0.5, 0.33, 0.8),
        ];
        let indices = vec![
            0, 1, 2, // bottom
            0, 1, 3, // front
            1, 2, 3, // right
            0, 2, 3, // left
        ];
        TriangleMeshData::new(vertices, indices)
    }

    /// Create a simple cube mesh for testing.
    fn create_cube() -> TriangleMeshData {
        let vertices = vec![
            // Bottom face
            Point3::new(-0.5, -0.5, -0.5),
            Point3::new(0.5, -0.5, -0.5),
            Point3::new(0.5, 0.5, -0.5),
            Point3::new(-0.5, 0.5, -0.5),
            // Top face
            Point3::new(-0.5, -0.5, 0.5),
            Point3::new(0.5, -0.5, 0.5),
            Point3::new(0.5, 0.5, 0.5),
            Point3::new(-0.5, 0.5, 0.5),
        ];
        // Two triangles per face
        let indices = vec![
            // Bottom (-Z)
            0, 1, 2, 0, 2, 3, // Top (+Z)
            4, 6, 5, 4, 7, 6, // Front (-Y)
            0, 5, 1, 0, 4, 5, // Back (+Y)
            2, 7, 3, 2, 6, 7, // Left (-X)
            0, 7, 4, 0, 3, 7, // Right (+X)
            1, 6, 2, 1, 5, 6,
        ];
        TriangleMeshData::new(vertices, indices)
    }

    #[test]
    fn test_mesh_creation() {
        let mesh = create_tetrahedron();
        assert_eq!(mesh.vertex_count(), 4);
        assert_eq!(mesh.triangle_count(), 4);
        assert!(mesh.bvh().is_some());
    }

    #[test]
    fn test_mesh_aabb() {
        let mesh = create_cube();
        let (min, max) = mesh.aabb();
        assert_relative_eq!(min.x, -0.5, epsilon = 1e-10);
        assert_relative_eq!(max.x, 0.5, epsilon = 1e-10);
    }

    #[test]
    fn test_closest_point_on_triangle_vertex() {
        let v0 = Point3::new(0.0, 0.0, 0.0);
        let v1 = Point3::new(1.0, 0.0, 0.0);
        let v2 = Point3::new(0.0, 1.0, 0.0);

        // Point closest to v0
        let p = Point3::new(-1.0, -1.0, 0.0);
        let closest = closest_point_on_triangle(v0, v1, v2, p);
        assert_relative_eq!(closest.x, 0.0, epsilon = 1e-10);
        assert_relative_eq!(closest.y, 0.0, epsilon = 1e-10);
    }

    #[test]
    fn test_closest_point_on_triangle_face() {
        let v0 = Point3::new(0.0, 0.0, 0.0);
        let v1 = Point3::new(1.0, 0.0, 0.0);
        let v2 = Point3::new(0.0, 1.0, 0.0);

        // Point above the triangle center
        let p = Point3::new(0.25, 0.25, 1.0);
        let closest = closest_point_on_triangle(v0, v1, v2, p);
        assert_relative_eq!(closest.x, 0.25, epsilon = 1e-10);
        assert_relative_eq!(closest.y, 0.25, epsilon = 1e-10);
        assert_relative_eq!(closest.z, 0.0, epsilon = 1e-10);
    }

    #[test]
    fn test_triangle_sphere_contact_hit() {
        let v0 = Point3::new(0.0, 0.0, 0.0);
        let v1 = Point3::new(1.0, 0.0, 0.0);
        let v2 = Point3::new(0.0, 1.0, 0.0);

        // Sphere just above the triangle
        let center = Point3::new(0.25, 0.25, 0.3);
        let radius = 0.5;

        let contact = triangle_sphere_contact(v0, v1, v2, center, radius);
        assert!(contact.is_some());

        let (point, normal, penetration) = contact.unwrap();
        assert!(penetration > 0.0);
        assert!(normal.z > 0.9); // Normal should point upward
        assert_relative_eq!(point.z, 0.0, epsilon = 1e-10);
    }

    #[test]
    fn test_triangle_sphere_contact_miss() {
        let v0 = Point3::new(0.0, 0.0, 0.0);
        let v1 = Point3::new(1.0, 0.0, 0.0);
        let v2 = Point3::new(0.0, 1.0, 0.0);

        // Sphere far above the triangle
        let center = Point3::new(0.25, 0.25, 2.0);
        let radius = 0.5;

        let contact = triangle_sphere_contact(v0, v1, v2, center, radius);
        assert!(contact.is_none());
    }

    #[test]
    fn test_mesh_sphere_contact() {
        let mut mesh = create_cube();
        let mesh_pose = Pose::identity();

        // Sphere penetrating the top of the cube
        let center = Point3::new(0.0, 0.0, 0.7);
        let radius = 0.3;

        let contact = mesh_sphere_contact(&mut mesh, &mesh_pose, center, radius);
        assert!(contact.is_some());

        let c = contact.unwrap();
        assert!(c.penetration > 0.0);
        assert!(c.normal.z > 0.5); // Normal should point upward
    }

    #[test]
    fn test_mesh_sphere_contact_miss() {
        let mut mesh = create_cube();
        let mesh_pose = Pose::identity();

        // Sphere far from the cube
        let center = Point3::new(0.0, 0.0, 2.0);
        let radius = 0.3;

        let contact = mesh_sphere_contact(&mut mesh, &mesh_pose, center, radius);
        assert!(contact.is_none());
    }

    #[test]
    fn test_triangle_capsule_contact() {
        let v0 = Point3::new(-1.0, -1.0, 0.0);
        let v1 = Point3::new(1.0, -1.0, 0.0);
        let v2 = Point3::new(0.0, 1.0, 0.0);

        // Vertical capsule near the triangle
        let start = Point3::new(0.0, 0.0, 0.2);
        let end = Point3::new(0.0, 0.0, 1.0);
        let radius = 0.3;

        let contact = triangle_capsule_contact(v0, v1, v2, start, end, radius);
        assert!(contact.is_some());

        let (_, normal, penetration) = contact.unwrap();
        assert!(penetration > 0.0);
        assert!(normal.z > 0.9);
    }

    #[test]
    fn test_bvh_query() {
        let mesh = create_cube();
        let bvh = mesh.bvh().unwrap();

        // Query that overlaps the top face of the cube (at z=0.5)
        let query = crate::broad_phase::Aabb::from_center(
            Point3::new(0.0, 0.0, 0.5),
            Vector3::new(0.2, 0.2, 0.1),
        );
        let results = bvh.query(&query);
        assert!(
            !results.is_empty(),
            "Query near top face should find triangles"
        );
    }

    #[test]
    fn test_triangle_normal() {
        let mesh = create_tetrahedron();
        let tri = mesh.get_triangle(0).unwrap();
        let normal = mesh.triangle_normal(tri);

        // Triangle 0 is the bottom face (XY plane), normal should be -Z
        assert!(normal.z.abs() > 0.9);
    }
}
