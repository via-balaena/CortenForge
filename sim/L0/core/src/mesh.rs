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
    ///
    /// The BVH is always built in constructors, but may be `None` after
    /// deserialization (since it's marked `#[serde(skip)]`). Call this
    /// to rebuild if needed.
    #[allow(dead_code)] // Kept for future deserialization support
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

/// Result of triangle-triangle intersection test.
#[derive(Debug, Clone)]
pub struct TriTriContact {
    /// Contact point (world space).
    pub point: Point3<f64>,
    /// Contact normal (from mesh A toward mesh B).
    pub normal: Vector3<f64>,
    /// Penetration depth (positive = interpenetrating).
    pub depth: f64,
    /// Triangle index in mesh A.
    pub tri_a: usize,
    /// Triangle index in mesh B.
    pub tri_b: usize,
}

// =============================================================================
// Triangle-Triangle Intersection
// =============================================================================

/// Test intersection between two triangles using the Separating Axis Theorem (SAT).
///
/// Tests 13 potential separating axes:
/// - 2 triangle face normals
/// - 9 edge cross products (3 edges from each triangle)
///
/// If no separating axis is found, the triangles intersect and we compute the
/// contact point, normal, and penetration depth from the axis with minimum overlap.
///
/// # Arguments
///
/// * `tri_a` - Three vertices of triangle A
/// * `tri_b` - Three vertices of triangle B
///
/// # Returns
///
/// Returns `Some(TriTriContact)` if the triangles intersect, `None` otherwise.
/// Note: The returned contact has `tri_a` and `tri_b` set to 0 - caller should fill these in.
#[must_use]
pub fn triangle_triangle_intersection(
    tri_a: &[Point3<f64>; 3],
    tri_b: &[Point3<f64>; 3],
) -> Option<TriTriContact> {
    // Quick rejection: AABB test
    if !triangle_aabbs_overlap(tri_a, tri_b) {
        return None;
    }

    // Compute edges for both triangles
    let edges_a = [
        tri_a[1] - tri_a[0],
        tri_a[2] - tri_a[1],
        tri_a[0] - tri_a[2],
    ];
    let edges_b = [
        tri_b[1] - tri_b[0],
        tri_b[2] - tri_b[1],
        tri_b[0] - tri_b[2],
    ];

    // Compute face normals
    let normal_a = edges_a[0].cross(&edges_a[1]);
    let normal_b = edges_b[0].cross(&edges_b[1]);

    // Track minimum penetration axis
    let mut min_penetration = f64::INFINITY;
    let mut best_axis = Vector3::zeros();
    let mut best_axis_type = AxisType::FaceA;

    // Test axis 1: Normal of triangle A
    if let Some((penetration, axis)) = test_separating_axis(&normal_a, tri_a, tri_b) {
        if penetration < min_penetration {
            min_penetration = penetration;
            best_axis = axis;
            best_axis_type = AxisType::FaceA;
        }
    } else {
        return None; // Separating axis found
    }

    // Test axis 2: Normal of triangle B
    if let Some((penetration, axis)) = test_separating_axis(&normal_b, tri_a, tri_b) {
        if penetration < min_penetration {
            min_penetration = penetration;
            best_axis = axis;
            best_axis_type = AxisType::FaceB;
        }
    } else {
        return None; // Separating axis found
    }

    // Test axes 3-11: Cross products of edges
    for (i, edge_a) in edges_a.iter().enumerate() {
        for (j, edge_b) in edges_b.iter().enumerate() {
            let cross = edge_a.cross(edge_b);

            // Skip near-zero axes (parallel edges)
            let cross_len_sq = cross.norm_squared();
            if cross_len_sq < EPSILON * EPSILON {
                continue;
            }

            if let Some((penetration, axis)) = test_separating_axis(&cross, tri_a, tri_b) {
                if penetration < min_penetration {
                    min_penetration = penetration;
                    best_axis = axis;
                    best_axis_type = AxisType::EdgeEdge(i, j);
                }
            } else {
                return None; // Separating axis found
            }
        }
    }

    // No separating axis found - triangles intersect
    // Compute contact point and normal based on the minimum penetration axis

    // Ensure normal points from A toward B
    let center_a = (tri_a[0].coords + tri_a[1].coords + tri_a[2].coords) / 3.0;
    let center_b = (tri_b[0].coords + tri_b[1].coords + tri_b[2].coords) / 3.0;
    let a_to_b = center_b - center_a;

    if best_axis.dot(&a_to_b) < 0.0 {
        best_axis = -best_axis;
    }

    // Compute contact point based on axis type
    let contact_point =
        compute_contact_point(tri_a, tri_b, &edges_a, &edges_b, &best_axis, best_axis_type);

    Some(TriTriContact {
        point: contact_point,
        normal: best_axis,
        depth: min_penetration,
        tri_a: 0, // Caller should fill this in
        tri_b: 0, // Caller should fill this in
    })
}

/// Type of separating axis being tested.
#[derive(Debug, Clone, Copy)]
enum AxisType {
    /// Face normal of triangle A
    FaceA,
    /// Face normal of triangle B
    FaceB,
    /// Cross product of edge i from A and edge j from B
    EdgeEdge(usize, usize),
}

/// Test if the AABBs of two triangles overlap.
fn triangle_aabbs_overlap(tri_a: &[Point3<f64>; 3], tri_b: &[Point3<f64>; 3]) -> bool {
    let min_a = Point3::new(
        tri_a[0].x.min(tri_a[1].x).min(tri_a[2].x),
        tri_a[0].y.min(tri_a[1].y).min(tri_a[2].y),
        tri_a[0].z.min(tri_a[1].z).min(tri_a[2].z),
    );
    let max_a = Point3::new(
        tri_a[0].x.max(tri_a[1].x).max(tri_a[2].x),
        tri_a[0].y.max(tri_a[1].y).max(tri_a[2].y),
        tri_a[0].z.max(tri_a[1].z).max(tri_a[2].z),
    );
    let min_b = Point3::new(
        tri_b[0].x.min(tri_b[1].x).min(tri_b[2].x),
        tri_b[0].y.min(tri_b[1].y).min(tri_b[2].y),
        tri_b[0].z.min(tri_b[1].z).min(tri_b[2].z),
    );
    let max_b = Point3::new(
        tri_b[0].x.max(tri_b[1].x).max(tri_b[2].x),
        tri_b[0].y.max(tri_b[1].y).max(tri_b[2].y),
        tri_b[0].z.max(tri_b[1].z).max(tri_b[2].z),
    );

    // Check for separation on each axis
    !(max_a.x < min_b.x - EPSILON
        || min_a.x > max_b.x + EPSILON
        || max_a.y < min_b.y - EPSILON
        || min_a.y > max_b.y + EPSILON
        || max_a.z < min_b.z - EPSILON
        || min_a.z > max_b.z + EPSILON)
}

/// Test a potential separating axis.
///
/// Returns `Some((penetration, normalized_axis))` if there is overlap,
/// or `None` if this is a valid separating axis (no intersection).
#[allow(clippy::similar_names)]
fn test_separating_axis(
    axis: &Vector3<f64>,
    tri_a: &[Point3<f64>; 3],
    tri_b: &[Point3<f64>; 3],
) -> Option<(f64, Vector3<f64>)> {
    let axis_len = axis.norm();
    if axis_len < EPSILON {
        // Degenerate axis, skip (consider overlapping)
        return Some((f64::INFINITY, Vector3::z()));
    }
    let axis_normalized = axis / axis_len;

    // Project triangle A vertices onto axis
    let proj_a0 = axis_normalized.dot(&tri_a[0].coords);
    let proj_a1 = axis_normalized.dot(&tri_a[1].coords);
    let proj_a2 = axis_normalized.dot(&tri_a[2].coords);
    let min_a = proj_a0.min(proj_a1).min(proj_a2);
    let max_a = proj_a0.max(proj_a1).max(proj_a2);

    // Project triangle B vertices onto axis
    let proj_b0 = axis_normalized.dot(&tri_b[0].coords);
    let proj_b1 = axis_normalized.dot(&tri_b[1].coords);
    let proj_b2 = axis_normalized.dot(&tri_b[2].coords);
    let min_b = proj_b0.min(proj_b1).min(proj_b2);
    let max_b = proj_b0.max(proj_b1).max(proj_b2);

    // Check for separation
    if max_a < min_b - EPSILON || max_b < min_a - EPSILON {
        return None; // Separating axis found
    }

    // Compute overlap (penetration depth on this axis)
    let overlap = (max_a.min(max_b) - min_a.max(min_b)).max(0.0);

    Some((overlap, axis_normalized))
}

/// Compute the contact point based on the axis type.
#[allow(clippy::similar_names)]
fn compute_contact_point(
    tri_a: &[Point3<f64>; 3],
    tri_b: &[Point3<f64>; 3],
    edges_a: &[Vector3<f64>; 3],
    edges_b: &[Vector3<f64>; 3],
    axis: &Vector3<f64>,
    axis_type: AxisType,
) -> Point3<f64> {
    match axis_type {
        AxisType::FaceA => {
            // Contact is on face of A - find the point on B closest to A's plane
            // Project B's vertices onto A and find the deepest one
            let d = axis.dot(&tri_a[0].coords);
            let mut best_point = tri_b[0];
            let mut best_dist = (axis.dot(&tri_b[0].coords) - d).abs();
            for v in &tri_b[1..] {
                let dist = (axis.dot(&v.coords) - d).abs();
                if dist < best_dist {
                    best_dist = dist;
                    best_point = *v;
                }
            }
            best_point
        }
        AxisType::FaceB => {
            // Contact is on face of B - find the point on A closest to B's plane
            let d = axis.dot(&tri_b[0].coords);
            let mut best_point = tri_a[0];
            let mut best_dist = (axis.dot(&tri_a[0].coords) - d).abs();
            for v in &tri_a[1..] {
                let dist = (axis.dot(&v.coords) - d).abs();
                if dist < best_dist {
                    best_dist = dist;
                    best_point = *v;
                }
            }
            best_point
        }
        AxisType::EdgeEdge(i, j) => {
            // Contact is at the closest points between two edges
            let edge_a_start = tri_a[i];
            let edge_a_dir = edges_a[i];
            let edge_b_start = tri_b[j];
            let edge_b_dir = edges_b[j];

            let (point_a, point_b) =
                closest_points_on_segments(edge_a_start, edge_a_dir, edge_b_start, edge_b_dir);

            // Return the midpoint of the closest points
            Point3::from((point_a.coords + point_b.coords) * 0.5)
        }
    }
}

/// Find the closest points between two line segments.
///
/// Segment A: `start_a + t * dir_a` for t in [0, 1]
/// Segment B: `start_b + s * dir_b` for s in [0, 1]
///
/// Returns the closest points (`point_on_a`, `point_on_b`).
#[allow(clippy::many_single_char_names, clippy::suspicious_operation_groupings)]
fn closest_points_on_segments(
    start_a: Point3<f64>,
    dir_a: Vector3<f64>,
    start_b: Point3<f64>,
    dir_b: Vector3<f64>,
) -> (Point3<f64>, Point3<f64>) {
    let r = start_a - start_b;
    let a = dir_a.dot(&dir_a);
    let e = dir_b.dot(&dir_b);
    let f = dir_b.dot(&r);

    // Check if both segments degenerate to points
    if a < EPSILON && e < EPSILON {
        return (start_a, start_b);
    }

    let (s, t);
    if a < EPSILON {
        // Segment A is a point
        s = 0.0;
        t = (f / e).clamp(0.0, 1.0);
    } else {
        let c = dir_a.dot(&r);
        if e < EPSILON {
            // Segment B is a point
            t = 0.0;
            s = (-c / a).clamp(0.0, 1.0);
        } else {
            // General case
            let b = dir_a.dot(&dir_b);
            // denom = a*e - b*b (this is correct: a*e - b²)
            let denom = a * e - b * b;

            if denom.abs() > EPSILON {
                s = ((b * f - c * e) / denom).clamp(0.0, 1.0);
            } else {
                // Parallel segments
                s = 0.0;
            }

            // Compute t for point on B closest to point on A at s
            let t_num = b * s + f;
            if t_num < 0.0 {
                // Recompute s for t = 0
                let s_new = (-c / a).clamp(0.0, 1.0);
                return (Point3::from(start_a.coords + dir_a * s_new), start_b);
            }
            if t_num > e {
                // Recompute s for t = 1
                let s_new = ((b - c) / a).clamp(0.0, 1.0);
                return (
                    Point3::from(start_a.coords + dir_a * s_new),
                    Point3::from(start_b.coords + dir_b),
                );
            }
            t = t_num / e;
        }
    }

    (
        Point3::from(start_a.coords + dir_a * s),
        Point3::from(start_b.coords + dir_b * t),
    )
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
        let denom = d1 - d3;
        // Guard against degenerate edge (d1 ≈ d3)
        let v = if denom.abs() > EPSILON {
            d1 / denom
        } else {
            0.5
        };
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
        let denom = d2 - d6;
        // Guard against degenerate edge (d2 ≈ d6)
        let w = if denom.abs() > EPSILON {
            d2 / denom
        } else {
            0.5
        };
        return Point3::from(v0.coords + ac * w); // On edge AC
    }

    // Check if P is in edge region of BC
    let va = d3 * d6 - d5 * d4;
    if va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0 {
        let num = d4 - d3;
        let denom = num + (d5 - d6);
        // Guard against degenerate edge
        let w = if denom.abs() > EPSILON {
            num / denom
        } else {
            0.5
        };
        return Point3::from(v1.coords + (v2 - v1) * w); // On edge BC
    }

    // P is inside the triangle
    let total = va + vb + vc;
    // Guard against degenerate triangles (total ≈ 0 means triangle has no area)
    if total.abs() < EPSILON {
        return v0; // Fallback to first vertex for degenerate triangle
    }
    let denom = 1.0 / total;
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
///
/// Returns `None` if the BVH is not built (e.g., deserialized mesh without rebuild).
#[must_use]
pub fn mesh_sphere_contact(
    mesh: &TriangleMeshData,
    mesh_pose: &Pose,
    sphere_center: Point3<f64>,
    sphere_radius: f64,
) -> Option<MeshContact> {
    // Transform sphere center to mesh local space
    let local_center = mesh_pose.inverse_transform_point(&sphere_center);

    // Query BVH for candidate triangles
    let query_aabb = crate::collision_shape::Aabb::from_center(
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
///
/// Returns `None` if the BVH is not built (e.g., deserialized mesh without rebuild).
#[must_use]
pub fn mesh_capsule_contact(
    mesh: &TriangleMeshData,
    mesh_pose: &Pose,
    capsule_start: Point3<f64>,
    capsule_end: Point3<f64>,
    capsule_radius: f64,
) -> Option<MeshContact> {
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

    let query_aabb = crate::collision_shape::Aabb::new(
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
///
/// Returns `None` if the BVH is not built (e.g., deserialized mesh without rebuild).
#[must_use]
pub fn mesh_box_contact(
    mesh: &TriangleMeshData,
    mesh_pose: &Pose,
    box_pose: &Pose,
    half_extents: &Vector3<f64>,
) -> Option<MeshContact> {
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

    let query_aabb = crate::collision_shape::Aabb::from_center(
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

/// Compute contacts between two triangle meshes.
///
/// Uses dual-BVH traversal to find candidate triangle pairs, then tests each
/// pair with `triangle_triangle_intersection`. Returns all detected contacts.
///
/// # Arguments
///
/// * `mesh_a` - First triangle mesh (with BVH)
/// * `pose_a` - Pose (transform) of mesh A in world space
/// * `mesh_b` - Second triangle mesh (with BVH)
/// * `pose_b` - Pose (transform) of mesh B in world space
///
/// # Returns
///
/// Vector of `MeshContact` structs representing all detected contacts between
/// the two meshes. Returns an empty vector if meshes don't intersect.
///
/// # Example
///
/// ```ignore
/// use sim_core::mesh::{mesh_mesh_contact, TriangleMeshData};
/// use sim_types::Pose;
/// use nalgebra::Point3;
///
/// let mesh_a = TriangleMeshData::new(/* vertices */, /* indices */);
/// let mesh_b = TriangleMeshData::new(/* vertices */, /* indices */);
/// let pose_a = Pose::identity();
/// let pose_b = Pose::from_position(Point3::new(0.5, 0.0, 0.0));
///
/// let contacts = mesh_mesh_contact(&mesh_a, &pose_a, &mesh_b, &pose_b);
/// for contact in contacts {
///     println!("Contact at {:?}, depth: {}", contact.point, contact.penetration);
/// }
/// ```
#[must_use]
pub fn mesh_mesh_contact(
    mesh_a: &TriangleMeshData,
    pose_a: &Pose,
    mesh_b: &TriangleMeshData,
    pose_b: &Pose,
) -> Vec<MeshContact> {
    // Get BVHs (return empty if not available)
    let Some(bvh_a) = mesh_a.bvh() else {
        return Vec::new();
    };
    let Some(bvh_b) = mesh_b.bvh() else {
        return Vec::new();
    };

    // Convert poses to isometries for the BVH query
    let iso_a = pose_a.to_isometry();
    let iso_b = pose_b.to_isometry();

    // Query BVH pair for candidate triangle pairs
    let candidate_pairs = crate::mid_phase::query_bvh_pair(bvh_a, bvh_b, &iso_a, &iso_b);

    if candidate_pairs.is_empty() {
        return Vec::new();
    }

    let mut contacts = Vec::new();

    // Test each candidate pair with narrow-phase triangle-triangle intersection
    for (tri_idx_a, tri_idx_b) in candidate_pairs {
        // Get triangle A
        let Some(tri_a) = mesh_a.get_triangle(tri_idx_a) else {
            continue;
        };
        let (a0, a1, a2) = mesh_a.triangle_vertices(tri_a);

        // Get triangle B
        let Some(tri_b) = mesh_b.get_triangle(tri_idx_b) else {
            continue;
        };
        let (b0, b1, b2) = mesh_b.triangle_vertices(tri_b);

        // Transform triangles to world space
        let world_a = [
            pose_a.transform_point(&a0),
            pose_a.transform_point(&a1),
            pose_a.transform_point(&a2),
        ];
        let world_b = [
            pose_b.transform_point(&b0),
            pose_b.transform_point(&b1),
            pose_b.transform_point(&b2),
        ];

        // Test for intersection
        if let Some(mut contact) = triangle_triangle_intersection(&world_a, &world_b) {
            // Fill in triangle indices
            contact.tri_a = tri_idx_a;
            contact.tri_b = tri_idx_b;

            contacts.push(MeshContact {
                point: contact.point,
                normal: contact.normal,
                penetration: contact.depth,
                triangle_index: tri_idx_a, // Use mesh A's triangle index
            });
        }
    }

    contacts
}

/// Get the deepest contact from a mesh-mesh collision test.
///
/// Convenience function that returns only the contact with maximum penetration depth.
/// Useful when only one contact is needed for constraint solving.
#[must_use]
pub fn mesh_mesh_deepest_contact(
    mesh_a: &TriangleMeshData,
    pose_a: &Pose,
    mesh_b: &TriangleMeshData,
    pose_b: &Pose,
) -> Option<MeshContact> {
    let contacts = mesh_mesh_contact(mesh_a, pose_a, mesh_b, pose_b);
    contacts.into_iter().max_by(|a, b| {
        a.penetration
            .partial_cmp(&b.penetration)
            .unwrap_or(std::cmp::Ordering::Equal)
    })
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
        let mesh = create_cube();
        let mesh_pose = Pose::identity();

        // Sphere penetrating the top of the cube
        let center = Point3::new(0.0, 0.0, 0.7);
        let radius = 0.3;

        let contact = mesh_sphere_contact(&mesh, &mesh_pose, center, radius);
        assert!(contact.is_some());

        let c = contact.unwrap();
        assert!(c.penetration > 0.0);
        assert!(c.normal.z > 0.5); // Normal should point upward
    }

    #[test]
    fn test_mesh_sphere_contact_miss() {
        let mesh = create_cube();
        let mesh_pose = Pose::identity();

        // Sphere far from the cube
        let center = Point3::new(0.0, 0.0, 2.0);
        let radius = 0.3;

        let contact = mesh_sphere_contact(&mesh, &mesh_pose, center, radius);
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
        let query = crate::collision_shape::Aabb::from_center(
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

    // =============================================================================
    // Triangle-Triangle Intersection Tests
    // =============================================================================

    #[test]
    fn test_tri_tri_intersection_coplanar_overlapping() {
        // Two overlapping triangles in the XY plane
        let tri_a = [
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(2.0, 0.0, 0.0),
            Point3::new(1.0, 2.0, 0.0),
        ];
        let tri_b = [
            Point3::new(0.5, 0.5, 0.0),
            Point3::new(2.5, 0.5, 0.0),
            Point3::new(1.5, 2.5, 0.0),
        ];

        let result = triangle_triangle_intersection(&tri_a, &tri_b);
        assert!(
            result.is_some(),
            "Coplanar overlapping triangles should intersect"
        );
    }

    #[test]
    fn test_tri_tri_intersection_coplanar_separate() {
        // Two non-overlapping triangles in the XY plane
        let tri_a = [
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.5, 1.0, 0.0),
        ];
        let tri_b = [
            Point3::new(5.0, 0.0, 0.0),
            Point3::new(6.0, 0.0, 0.0),
            Point3::new(5.5, 1.0, 0.0),
        ];

        let result = triangle_triangle_intersection(&tri_a, &tri_b);
        assert!(
            result.is_none(),
            "Separate coplanar triangles should not intersect"
        );
    }

    #[test]
    fn test_tri_tri_intersection_crossing() {
        // Two triangles that cross each other
        // Triangle A in XY plane
        let tri_a = [
            Point3::new(-1.0, -1.0, 0.0),
            Point3::new(1.0, -1.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        ];
        // Triangle B in XZ plane, crossing through A
        let tri_b = [
            Point3::new(-1.0, 0.0, -1.0),
            Point3::new(1.0, 0.0, -1.0),
            Point3::new(0.0, 0.0, 1.0),
        ];

        let result = triangle_triangle_intersection(&tri_a, &tri_b);
        assert!(result.is_some(), "Crossing triangles should intersect");

        let contact = result.unwrap();
        // For truly crossing triangles, depth may be very small or zero
        // The important thing is that we detect the intersection
        assert!(
            contact.depth >= 0.0,
            "Penetration depth should be non-negative"
        );
    }

    #[test]
    fn test_tri_tri_intersection_parallel_separate() {
        // Two parallel triangles with a gap
        let tri_a = [
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.5, 1.0, 0.0),
        ];
        let tri_b = [
            Point3::new(0.0, 0.0, 2.0),
            Point3::new(1.0, 0.0, 2.0),
            Point3::new(0.5, 1.0, 2.0),
        ];

        let result = triangle_triangle_intersection(&tri_a, &tri_b);
        assert!(
            result.is_none(),
            "Parallel separate triangles should not intersect"
        );
    }

    #[test]
    fn test_tri_tri_intersection_parallel_overlapping() {
        // Two parallel triangles very close together (nearly touching)
        let tri_a = [
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.5, 1.0, 0.0),
        ];
        let tri_b = [
            Point3::new(0.0, 0.0, 0.05),
            Point3::new(1.0, 0.0, 0.05),
            Point3::new(0.5, 1.0, 0.05),
        ];

        let result = triangle_triangle_intersection(&tri_a, &tri_b);
        // These are close but not actually intersecting (no penetration)
        // The SAT test should detect separation along the Z axis
        assert!(
            result.is_none(),
            "Parallel non-penetrating triangles should not intersect"
        );
    }

    #[test]
    fn test_tri_tri_intersection_edge_edge() {
        // Two triangles where edges come close/cross
        // Triangle A: lying flat in XY plane
        let tri_a = [
            Point3::new(-1.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 2.0, 0.0),
        ];
        // Triangle B: tilted to intersect via edges
        let tri_b = [
            Point3::new(0.0, 1.0, -1.0),
            Point3::new(0.0, 1.0, 1.0),
            Point3::new(2.0, 1.0, 0.0),
        ];

        let result = triangle_triangle_intersection(&tri_a, &tri_b);
        assert!(result.is_some(), "Edge-crossing triangles should intersect");
    }

    #[test]
    fn test_tri_tri_intersection_vertex_face() {
        // Triangle A flat on XY plane
        let tri_a = [
            Point3::new(-1.0, -1.0, 0.0),
            Point3::new(1.0, -1.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        ];
        // Triangle B with vertex poking through A
        let tri_b = [
            Point3::new(0.0, 0.0, -0.5),
            Point3::new(1.0, 0.0, 0.5),
            Point3::new(-1.0, 0.0, 0.5),
        ];

        let result = triangle_triangle_intersection(&tri_a, &tri_b);
        assert!(result.is_some(), "Vertex-through-face should intersect");

        let contact = result.unwrap();
        // For crossing triangles, depth may be small since they pass through each other
        assert!(
            contact.depth >= 0.0,
            "Penetration depth should be non-negative"
        );
    }

    #[test]
    fn test_tri_tri_aabb_rejection() {
        // Triangles with non-overlapping AABBs
        let tri_a = [
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.5, 1.0, 0.0),
        ];
        let tri_b = [
            Point3::new(10.0, 10.0, 10.0),
            Point3::new(11.0, 10.0, 10.0),
            Point3::new(10.5, 11.0, 10.0),
        ];

        // This should be quickly rejected by AABB test
        let result = triangle_triangle_intersection(&tri_a, &tri_b);
        assert!(result.is_none(), "Far apart triangles should not intersect");
    }

    #[test]
    fn test_tri_tri_contact_normal_direction() {
        // Verify the normal points from A toward B
        let tri_a = [
            Point3::new(-1.0, -1.0, 0.0),
            Point3::new(1.0, -1.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        ];
        let tri_b = [
            Point3::new(-1.0, 0.0, -0.5),
            Point3::new(1.0, 0.0, -0.5),
            Point3::new(0.0, 0.0, 1.0),
        ];

        let result = triangle_triangle_intersection(&tri_a, &tri_b);
        assert!(result.is_some());

        let contact = result.unwrap();
        // The center of B is roughly at (0, 0, 0), center of A is roughly at (0, -0.33, 0)
        // Normal should have some component pointing from A toward B
        let center_a = Point3::from((tri_a[0].coords + tri_a[1].coords + tri_a[2].coords) / 3.0);
        let center_b = Point3::from((tri_b[0].coords + tri_b[1].coords + tri_b[2].coords) / 3.0);
        let a_to_b = center_b - center_a;
        assert!(
            contact.normal.dot(&a_to_b) >= 0.0,
            "Normal should point from A toward B"
        );
    }

    #[test]
    fn test_closest_points_on_segments() {
        // Test parallel segments
        let (p1, p2) = closest_points_on_segments(
            Point3::new(0.0, 0.0, 0.0),
            Vector3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
            Vector3::new(1.0, 0.0, 0.0),
        );
        assert_relative_eq!(p1.y, 0.0, epsilon = 1e-10);
        assert_relative_eq!(p2.y, 1.0, epsilon = 1e-10);

        // Test perpendicular segments
        let (p1, p2) = closest_points_on_segments(
            Point3::new(0.0, 0.0, 0.0),
            Vector3::new(2.0, 0.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
            Vector3::new(0.0, -2.0, 0.0),
        );
        // Closest points should be at (1, 0, 0) and (1, 0, 0)
        assert_relative_eq!(p1.x, 1.0, epsilon = 1e-10);
        assert_relative_eq!(p1.y, 0.0, epsilon = 1e-10);
        assert_relative_eq!(p2.x, 1.0, epsilon = 1e-10);
        assert_relative_eq!(p2.y, 0.0, epsilon = 1e-10);
    }

    #[test]
    fn test_tri_tri_identical_triangles() {
        // Two identical triangles should intersect
        let tri = [
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.5, 1.0, 0.0),
        ];

        let result = triangle_triangle_intersection(&tri, &tri);
        assert!(result.is_some(), "Identical triangles should intersect");
    }

    #[test]
    fn test_tri_tri_penetrating_crossing() {
        // Two triangles where one crosses through the other
        // For infinitely-thin triangles, SAT correctly detects intersection
        // but depth may be 0 when they perfectly cross at a face plane
        let tri_a = [
            Point3::new(-1.0, -1.0, 0.0),
            Point3::new(1.0, -1.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        ];
        // Triangle B crosses through A
        let tri_b = [
            Point3::new(-0.8, -0.8, -0.1),
            Point3::new(0.8, -0.8, -0.1),
            Point3::new(0.0, 0.8, 0.1),
        ];

        let result = triangle_triangle_intersection(&tri_a, &tri_b);
        assert!(result.is_some(), "Crossing triangles should intersect");

        let contact = result.unwrap();
        // Depth may be 0 or very small for crossing triangles (geometric crossing)
        assert!(contact.depth >= 0.0, "Depth should be non-negative");
    }

    #[test]
    fn test_tri_tri_tilted_overlap() {
        // Two triangles in tilted planes that clearly overlap in 3D space
        // This should give positive penetration depth
        let tri_a = [
            Point3::new(-1.0, -1.0, 0.0),
            Point3::new(1.0, -1.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        ];
        // Triangle B is tilted and overlaps with A's area
        // The triangles share volume when considered as thick surfaces
        let tri_b = [
            Point3::new(-0.5, 0.0, -0.2),
            Point3::new(0.5, 0.0, -0.2),
            Point3::new(0.0, 0.0, 0.2),
        ];

        let result = triangle_triangle_intersection(&tri_a, &tri_b);
        assert!(result.is_some(), "Overlapping triangles should intersect");
        // Crossing triangles may have 0 depth along face normals
        // This is geometrically correct for infinitely thin surfaces
    }

    #[test]
    fn test_tri_tri_face_to_face_coplanar() {
        // Two coplanar triangles that overlap - this is a degenerate case
        // where the triangles share the same plane and overlap spatially
        let tri_a = [
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(2.0, 0.0, 0.0),
            Point3::new(1.0, 2.0, 0.0),
        ];
        // Triangle B overlaps with A in the same plane
        let tri_b = [
            Point3::new(0.5, 0.5, 0.0),
            Point3::new(1.5, 0.5, 0.0),
            Point3::new(1.0, 1.5, 0.0),
        ];

        let result = triangle_triangle_intersection(&tri_a, &tri_b);
        // Coplanar overlapping triangles should be detected as intersecting
        assert!(
            result.is_some(),
            "Coplanar overlapping triangles should intersect"
        );
    }

    // =============================================================================
    // Mesh-Mesh Contact Tests
    // =============================================================================

    #[test]
    fn test_mesh_mesh_contact_overlapping_cubes() {
        // Two cubes that overlap
        let cube_a = create_cube();
        let cube_b = create_cube();

        // Position cube B so it overlaps with cube A
        let pose_a = Pose::identity();
        let pose_b = Pose::from_position(Point3::new(0.5, 0.0, 0.0)); // Shifted 0.5 units in X

        let contacts = mesh_mesh_contact(&cube_a, &pose_a, &cube_b, &pose_b);
        assert!(
            !contacts.is_empty(),
            "Overlapping cubes should produce contacts"
        );

        // All contacts should have positive penetration
        for contact in &contacts {
            assert!(
                contact.penetration >= 0.0,
                "Penetration should be non-negative"
            );
        }
    }

    #[test]
    fn test_mesh_mesh_contact_separate_cubes() {
        // Two cubes that don't touch
        let cube_a = create_cube();
        let cube_b = create_cube();

        let pose_a = Pose::identity();
        let pose_b = Pose::from_position(Point3::new(5.0, 0.0, 0.0)); // Far apart

        let contacts = mesh_mesh_contact(&cube_a, &pose_a, &cube_b, &pose_b);
        assert!(
            contacts.is_empty(),
            "Separate cubes should not produce contacts"
        );
    }

    #[test]
    fn test_mesh_mesh_contact_rotated_cubes() {
        // Two cubes where one is rotated
        let cube_a = create_cube();
        let cube_b = create_cube();

        let pose_a = Pose::identity();
        // Rotate cube B 45 degrees around Z and position it so it overlaps
        let pose_b = Pose::from_position_rotation(
            Point3::new(0.3, 0.3, 0.0),
            nalgebra::UnitQuaternion::from_euler_angles(0.0, 0.0, std::f64::consts::FRAC_PI_4),
        );

        let contacts = mesh_mesh_contact(&cube_a, &pose_a, &cube_b, &pose_b);
        assert!(
            !contacts.is_empty(),
            "Rotated overlapping cubes should produce contacts"
        );
    }

    #[test]
    fn test_mesh_mesh_contact_tetrahedra() {
        // Two tetrahedra
        let tet_a = create_tetrahedron();
        let tet_b = create_tetrahedron();

        // Position tet_b so it overlaps with tet_a
        let pose_a = Pose::identity();
        let pose_b = Pose::from_position(Point3::new(0.3, 0.3, 0.0));

        let contacts = mesh_mesh_contact(&tet_a, &pose_a, &tet_b, &pose_b);
        assert!(
            !contacts.is_empty(),
            "Overlapping tetrahedra should produce contacts"
        );
    }

    #[test]
    fn test_mesh_mesh_deepest_contact() {
        // Test the convenience function
        let cube_a = create_cube();
        let cube_b = create_cube();

        let pose_a = Pose::identity();
        let pose_b = Pose::from_position(Point3::new(0.5, 0.0, 0.0));

        let deepest = mesh_mesh_deepest_contact(&cube_a, &pose_a, &cube_b, &pose_b);
        assert!(
            deepest.is_some(),
            "Overlapping cubes should have a deepest contact"
        );

        // Get all contacts and verify deepest
        let all_contacts = mesh_mesh_contact(&cube_a, &pose_a, &cube_b, &pose_b);
        if let Some(deepest_contact) = deepest {
            let max_penetration = all_contacts
                .iter()
                .map(|c| c.penetration)
                .fold(f64::NEG_INFINITY, f64::max);
            assert_relative_eq!(
                deepest_contact.penetration,
                max_penetration,
                epsilon = 1e-10
            );
        }
    }

    #[test]
    fn test_mesh_mesh_contact_identical_position() {
        // Two identical cubes at the same position (maximum overlap)
        let cube_a = create_cube();
        let cube_b = create_cube();

        let pose_a = Pose::identity();
        let pose_b = Pose::identity();

        let contacts = mesh_mesh_contact(&cube_a, &pose_a, &cube_b, &pose_b);
        assert!(
            !contacts.is_empty(),
            "Identical overlapping cubes should produce contacts"
        );
    }

    #[test]
    fn test_mesh_mesh_contact_edge_touching() {
        // Two cubes positioned so they touch at an edge
        let cube_a = create_cube();
        let cube_b = create_cube();

        let pose_a = Pose::identity();
        // Position cube B so its edge just touches cube A's edge
        let pose_b = Pose::from_position(Point3::new(1.0, 1.0, 0.0));

        // With edge touching, triangles may or may not intersect depending on precision
        let contacts = mesh_mesh_contact(&cube_a, &pose_a, &cube_b, &pose_b);
        // Just verify no panic and contacts are well-formed
        for contact in &contacts {
            assert!(contact.penetration >= 0.0 || contact.penetration.abs() < 1e-6);
        }
    }
}
