//! GJK (Gilbert-Johnson-Keerthi) and EPA (Expanding Polytope Algorithm) implementation.
//!
//! This module provides narrow-phase collision detection for convex shapes using
//! the GJK algorithm to determine intersection, and EPA to compute penetration
//! depth and contact normal when shapes overlap.
//!
//! # Algorithm Overview
//!
//! ## GJK (Gilbert-Johnson-Keerthi)
//!
//! GJK works in Minkowski space (the "Minkowski difference" of two shapes).
//! If two convex shapes overlap, their Minkowski difference contains the origin.
//! GJK iteratively builds a simplex (point, line, triangle, tetrahedron) that
//! tries to enclose the origin.
//!
//! **Complexity**: O(n) where n is the number of vertices, typically converges
//! in 20-30 iterations for complex shapes.
//!
//! ## EPA (Expanding Polytope Algorithm)
//!
//! When GJK determines that shapes overlap (origin is inside the Minkowski difference),
//! EPA finds the closest point on the boundary of the Minkowski difference to the origin.
//! This gives us the penetration depth and contact normal.
//!
//! **Complexity**: O(n²) in the worst case, but typically much faster.
//!
//! # Usage
//!
//! ```ignore
//! use sim_core::gjk_epa::{gjk_intersection, gjk_epa_contact};
//! use sim_core::CollisionShape;
//! use sim_types::Pose;
//!
//! let shape_a = CollisionShape::sphere(1.0);
//! let shape_b = CollisionShape::box_shape(Vector3::new(0.5, 0.5, 0.5));
//! let pose_a = Pose::identity();
//! let pose_b = Pose::from_position(Point3::new(1.0, 0.0, 0.0));
//!
//! // Check for intersection
//! if gjk_intersection(&shape_a, &pose_a, &shape_b, &pose_b, 35) {
//!     // Get contact information
//!     if let Some(contact) = gjk_epa_contact(&shape_a, &pose_a, &shape_b, &pose_b, 35, 1e-6) {
//!         println!("Penetration: {}", contact.penetration);
//!         println!("Normal: {:?}", contact.normal);
//!     }
//! }
//! ```
//!
//! # References
//!
//! - Gilbert, Johnson, Keerthi: "A Fast Procedure for Computing the Distance
//!   Between Complex Objects in Three-Dimensional Space" (1988)
//! - van den Bergen: "Collision Detection in Interactive 3D Environments" (2003)
//! - Casey Muratori's GJK video series

use nalgebra::{Point3, Vector3};
use sim_types::Pose;
use std::cell::Cell;

use crate::CollisionShape;
use crate::convex_hull::HullGraph;

/// Tolerance for numerical comparisons in GJK/EPA.
const EPSILON: f64 = 1e-8;

/// Default GJK max iterations (used when no Model context available).
/// MuJoCo default: 35. Historical CortenForge value: 64.
pub const GJK_MAX_ITERATIONS: usize = 35;

/// Default EPA max iterations (used when no Model context available).
/// MuJoCo default: 35. Historical CortenForge value: 64.
pub const EPA_MAX_ITERATIONS: usize = 35;

/// Maximum faces in EPA polytope.
const EPA_MAX_FACES: usize = 128;

/// Default EPA convergence tolerance.
/// MuJoCo default: 1e-6.
pub const EPA_TOLERANCE: f64 = 1e-6;

/// Result of a GJK query.
#[derive(Debug, Clone)]
pub struct GjkResult {
    /// Whether the shapes intersect.
    pub intersecting: bool,
    /// The final simplex (for EPA if intersecting).
    pub simplex: Simplex,
    /// Number of iterations used.
    pub iterations: usize,
}

/// Result of EPA (penetration information).
#[derive(Debug, Clone)]
pub struct EpaResult {
    /// Penetration depth (positive when overlapping).
    pub depth: f64,
    /// Contact normal (points from B toward A).
    pub normal: Vector3<f64>,
    /// Contact point on the Minkowski difference boundary.
    pub point: Point3<f64>,
    /// Number of iterations used.
    pub iterations: usize,
}

/// Contact information from GJK+EPA.
#[derive(Debug, Clone)]
pub struct GjkContact {
    /// Contact point in world space (approximately on the surface of shape A).
    pub point: Point3<f64>,
    /// Contact normal (points from B toward A).
    pub normal: Vector3<f64>,
    /// Penetration depth (positive when overlapping).
    pub penetration: f64,
}

/// A simplex used in GJK iteration.
///
/// Can be a point (1), line segment (2), triangle (3), or tetrahedron (4).
#[derive(Debug, Clone, Default)]
pub struct Simplex {
    /// Points in the simplex.
    points: [MinkowskiPoint; 4],
    /// Number of points in the simplex (1-4).
    size: usize,
}

/// A point in Minkowski space, with support points from both shapes.
#[derive(Debug, Clone, Copy, Default)]
pub struct MinkowskiPoint {
    /// The point in Minkowski space (`support_a` - `support_b`).
    pub point: Point3<f64>,
    /// Support point from shape A in world space.
    pub support_a: Point3<f64>,
    /// Support point from shape B in world space.
    pub support_b: Point3<f64>,
}

impl MinkowskiPoint {
    /// Create a new Minkowski point.
    fn new(support_a: Point3<f64>, support_b: Point3<f64>) -> Self {
        Self {
            point: Point3::from(support_a - support_b),
            support_a,
            support_b,
        }
    }
}

impl Simplex {
    /// Create a new empty simplex.
    #[must_use]
    pub fn new() -> Self {
        Self::default()
    }

    /// Push a point onto the simplex.
    pub fn push(&mut self, point: MinkowskiPoint) {
        // Shift existing points
        for i in (1..=self.size.min(3)).rev() {
            self.points[i] = self.points[i - 1];
        }
        self.points[0] = point;
        self.size = (self.size + 1).min(4);
    }

    /// Get the number of points.
    #[must_use]
    pub fn len(&self) -> usize {
        self.size
    }

    /// Check if the simplex is empty.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.size == 0
    }

    /// Get points as a slice.
    #[must_use]
    pub fn points(&self) -> &[MinkowskiPoint] {
        &self.points[..self.size]
    }

    /// Get a point by index.
    #[must_use]
    pub fn get(&self, index: usize) -> Option<&MinkowskiPoint> {
        if index < self.size {
            Some(&self.points[index])
        } else {
            None
        }
    }

    /// Set the simplex to contain only specific points.
    fn set(&mut self, points: &[MinkowskiPoint]) {
        self.size = points.len().min(4);
        for (i, p) in points.iter().take(4).enumerate() {
            self.points[i] = *p;
        }
    }
}

// =============================================================================
// Support Functions
// =============================================================================

/// Compute the support point for a collision shape in a given direction.
///
/// The support function returns the point on the shape's surface that is
/// furthest in the given direction. This is the key operation for GJK.
///
/// # Arguments
///
/// * `shape` - The collision shape
/// * `pose` - The pose (position and rotation) of the shape
/// * `direction` - The direction to find the support point (world space)
///
/// # Returns
///
/// The support point in world space.
#[must_use]
pub fn support(shape: &CollisionShape, pose: &Pose, direction: &Vector3<f64>) -> Point3<f64> {
    match shape {
        CollisionShape::Sphere { radius } => support_sphere(pose, *radius, direction),
        CollisionShape::Box { half_extents } => support_box(pose, half_extents, direction),
        CollisionShape::Capsule {
            half_length,
            radius,
        } => support_capsule(pose, *half_length, *radius, direction),
        CollisionShape::ConvexMesh {
            vertices,
            graph,
            warm_start,
        } => support_convex_mesh(pose, vertices, graph.as_ref(), warm_start, direction),
        CollisionShape::Plane { normal, distance } => {
            support_plane(pose, normal, *distance, direction)
        }
        CollisionShape::Cylinder {
            half_length,
            radius,
        } => support_cylinder(pose, *half_length, *radius, direction),
        CollisionShape::Ellipsoid { radii } => support_ellipsoid(pose, radii, direction),
        CollisionShape::HeightField { data } => {
            // Height fields are not convex, so GJK/EPA is not ideal.
            // Return an extreme point from the AABB as a fallback.
            let (local_min, local_max) = data.aabb();
            let local_dir = pose.rotation.inverse() * direction;

            let local_support = Point3::new(
                if local_dir.x >= 0.0 {
                    local_max.x
                } else {
                    local_min.x
                },
                if local_dir.y >= 0.0 {
                    local_max.y
                } else {
                    local_min.y
                },
                if local_dir.z >= 0.0 {
                    local_max.z
                } else {
                    local_min.z
                },
            );

            pose.transform_point(&local_support)
        }
        CollisionShape::Sdf { data } => {
            // SDFs are not convex, so GJK/EPA is not ideal.
            // Return an extreme point from the AABB as a fallback.
            // Dedicated SDF collision is handled separately in collision/sdf_collide.rs.
            let (local_min, local_max) = data.aabb();
            let local_dir = pose.rotation.inverse() * direction;

            let local_support = Point3::new(
                if local_dir.x >= 0.0 {
                    local_max.x
                } else {
                    local_min.x
                },
                if local_dir.y >= 0.0 {
                    local_max.y
                } else {
                    local_min.y
                },
                if local_dir.z >= 0.0 {
                    local_max.z
                } else {
                    local_min.z
                },
            );

            pose.transform_point(&local_support)
        }
        CollisionShape::TriangleMesh { data } => {
            // Triangle meshes are not convex, so GJK/EPA is not ideal.
            // Return an extreme point from the AABB as a fallback.
            // Dedicated triangle mesh collision is handled separately in collision/mesh_collide.rs.
            let (local_min, local_max) = data.aabb();
            let local_dir = pose.rotation.inverse() * direction;

            let local_support = Point3::new(
                if local_dir.x >= 0.0 {
                    local_max.x
                } else {
                    local_min.x
                },
                if local_dir.y >= 0.0 {
                    local_max.y
                } else {
                    local_min.y
                },
                if local_dir.z >= 0.0 {
                    local_max.z
                } else {
                    local_min.z
                },
            );

            pose.transform_point(&local_support)
        }
    }
}

/// Return all support points on a convex shape's contact face in a given direction.
///
/// For shapes with flat faces (Box, ConvexMesh), returns all vertices that
/// share the maximum dot product with `direction` (within `EPSILON`). For
/// smooth shapes (Sphere, Capsule, Ellipsoid, Cylinder), returns a single
/// support point (there's only one extremal point on a curved surface).
///
/// Used by MULTICCD to enumerate flat-face contact vertices.
pub fn support_face_points(
    shape: &CollisionShape,
    pose: &Pose,
    direction: &Vector3<f64>,
) -> Vec<Point3<f64>> {
    match shape {
        CollisionShape::Box { half_extents } => {
            // A box face perpendicular to direction has 4 vertices
            let local_dir = pose.rotation.inverse() * direction;
            // For each axis, determine if the face is on the + or - side,
            // or if the direction is perpendicular (both sides contribute)
            let signs: Vec<Vec<f64>> = (0..3)
                .map(|i| {
                    if local_dir[i].abs() < EPSILON {
                        vec![-1.0, 1.0] // perpendicular — both sides
                    } else if local_dir[i] > 0.0 {
                        vec![1.0]
                    } else {
                        vec![-1.0]
                    }
                })
                .collect();
            let mut points = Vec::new();
            for &sx in &signs[0] {
                for &sy in &signs[1] {
                    for &sz in &signs[2] {
                        let local_pt = Point3::new(
                            sx * half_extents.x,
                            sy * half_extents.y,
                            sz * half_extents.z,
                        );
                        points.push(pose.transform_point(&local_pt));
                    }
                }
            }
            points
        }
        CollisionShape::ConvexMesh {
            vertices,
            graph: _,
            warm_start: _,
        } => {
            if vertices.is_empty() {
                return vec![pose.position];
            }
            let local_dir = pose.rotation.inverse() * direction;
            // Find max dot product
            let max_dot = vertices
                .iter()
                .map(|v| v.coords.dot(&local_dir))
                .fold(f64::NEG_INFINITY, f64::max);
            // Return all vertices within EPSILON of max_dot
            vertices
                .iter()
                .filter(|v| (v.coords.dot(&local_dir) - max_dot).abs() < EPSILON * 10.0)
                .map(|v| pose.transform_point(v))
                .collect()
        }
        // Smooth shapes: single support point
        _ => vec![support(shape, pose, direction)],
    }
}

/// Support function for a sphere.
fn support_sphere(pose: &Pose, radius: f64, direction: &Vector3<f64>) -> Point3<f64> {
    let dir_norm = direction.norm();
    if dir_norm < EPSILON {
        return pose.position;
    }
    let unit_dir = direction / dir_norm;
    Point3::from(pose.position.coords + unit_dir * radius)
}

/// Support function for a box.
fn support_box(pose: &Pose, half_extents: &Vector3<f64>, direction: &Vector3<f64>) -> Point3<f64> {
    // Transform direction to local space
    let local_dir = pose.rotation.inverse() * direction;

    // Support point in local space: sign(d_i) * half_extent_i for each axis
    let local_support = Point3::new(
        half_extents.x * local_dir.x.signum(),
        half_extents.y * local_dir.y.signum(),
        half_extents.z * local_dir.z.signum(),
    );

    // Transform back to world space
    pose.transform_point(&local_support)
}

/// Support function for a capsule.
fn support_capsule(
    pose: &Pose,
    half_length: f64,
    radius: f64,
    direction: &Vector3<f64>,
) -> Point3<f64> {
    // Capsule is aligned along local Z-axis
    let local_dir = pose.rotation.inverse() * direction;

    // Find which endpoint is further in the direction
    let local_center = if local_dir.z >= 0.0 {
        Point3::new(0.0, 0.0, half_length)
    } else {
        Point3::new(0.0, 0.0, -half_length)
    };

    // Add sphere support from that endpoint
    let dir_norm = direction.norm();
    let sphere_offset = if dir_norm > EPSILON {
        direction * (radius / dir_norm)
    } else {
        Vector3::zeros()
    };

    Point3::from(pose.transform_point(&local_center).coords + sphere_offset)
}

/// Support function for a convex mesh.
///
/// Selects between hill-climbing (O(√n), when graph is available) and
/// exhaustive scan (O(n)). Both strategies warm-start from the cached
/// vertex index.
///
/// MuJoCo ref: `mjc_hillclimbSupport()` and `mjc_meshSupport()` in
/// `engine_collision_convex.c`.
fn support_convex_mesh(
    pose: &Pose,
    vertices: &[Point3<f64>],
    graph: Option<&HullGraph>,
    warm_start: &Cell<usize>,
    direction: &Vector3<f64>,
) -> Point3<f64> {
    if vertices.is_empty() {
        return pose.position;
    }

    let local_dir = pose.rotation.inverse() * direction;

    let best_idx = if let Some(g) = graph {
        // Hill-climbing (steepest-ascent): warm-start from cached vertex
        let start = warm_start.get().min(vertices.len() - 1);
        let idx = hill_climb_support(vertices, g, &local_dir, start);
        warm_start.set(idx);
        idx
    } else {
        // Exhaustive: warm-start by initializing max from cached vertex
        let cached = warm_start.get().min(vertices.len() - 1);
        let mut best = cached;
        let mut max_dot = vertices[cached].coords.dot(&local_dir);
        for (i, v) in vertices.iter().enumerate() {
            let dot = v.coords.dot(&local_dir);
            if dot > max_dot {
                max_dot = dot;
                best = i;
            }
        }
        warm_start.set(best);
        best
    };

    pose.transform_point(&vertices[best_idx])
}

/// Steepest-ascent hill-climbing support on convex hull graph.
///
/// Matches MuJoCo's `mjc_hillclimbSupport()`: at each step, examine ALL
/// neighbors of the current vertex, move to the neighbor with the highest
/// dot product, repeat until no neighbor improves.
///
/// Guaranteed to find the global maximum because the dot product
/// is a linear (unimodal) function on a convex surface.
pub(crate) fn hill_climb_support(
    vertices: &[Point3<f64>],
    graph: &HullGraph,
    direction: &Vector3<f64>,
    start: usize,
) -> usize {
    let mut current = start;
    let mut max_dot = f64::NEG_INFINITY;

    loop {
        let prev = current;
        // Scan ALL neighbors of current vertex (steepest-ascent)
        for &neighbor in &graph.adjacency[current] {
            let dot = vertices[neighbor].coords.dot(direction);
            if dot > max_dot {
                max_dot = dot;
                current = neighbor;
            }
        }
        // Converged: no neighbor improved
        if current == prev {
            return current;
        }
    }
}

/// Support function for a plane.
///
/// Planes are infinite, so we return a point very far in the direction
/// projected onto the plane. This allows GJK to work with planes as one
/// of the shapes (typically the ground).
fn support_plane(
    _pose: &Pose, // Planes are defined in world space
    normal: &Vector3<f64>,
    distance: f64,
    direction: &Vector3<f64>,
) -> Point3<f64> {
    // Project direction onto the plane
    let tangent = direction - normal * direction.dot(normal);
    let tangent_norm = tangent.norm();

    // Point on plane closest to origin along normal
    let base = Point3::from(*normal * distance);

    if tangent_norm > EPSILON {
        // Move very far along the tangent direction
        const LARGE: f64 = 1e6;
        Point3::from(base.coords + tangent.normalize() * LARGE)
    } else {
        base
    }
}

/// Support function for a cylinder.
///
/// A cylinder is like a capsule but with flat caps instead of hemispherical caps.
/// The support point is either on the circular edge of a cap or on the cylindrical surface.
fn support_cylinder(
    pose: &Pose,
    half_length: f64,
    radius: f64,
    direction: &Vector3<f64>,
) -> Point3<f64> {
    // Transform direction to local space
    let local_dir = pose.rotation.inverse() * direction;

    // Decompose direction into axial (Z) and radial (XY) components
    let axial_component = local_dir.z;
    let radial_dir = Vector3::new(local_dir.x, local_dir.y, 0.0);
    let radial_norm = radial_dir.norm();

    // Choose the cap based on axial direction
    let local_z = if axial_component >= 0.0 {
        half_length
    } else {
        -half_length
    };

    // Choose point on the circular edge based on radial direction
    let (local_x, local_y) = if radial_norm > EPSILON {
        let unit_radial = radial_dir / radial_norm;
        (unit_radial.x * radius, unit_radial.y * radius)
    } else {
        // Direction is purely axial - any point on the cap edge works
        (radius, 0.0)
    };

    let local_support = Point3::new(local_x, local_y, local_z);
    pose.transform_point(&local_support)
}

/// Support function for an ellipsoid.
///
/// An ellipsoid is a scaled sphere. The support function scales the direction
/// by the inverse radii, normalizes, then scales back by the radii.
fn support_ellipsoid(pose: &Pose, radii: &Vector3<f64>, direction: &Vector3<f64>) -> Point3<f64> {
    // Validate radii to avoid issues with zero/near-zero radii
    if radii.x < EPSILON || radii.y < EPSILON || radii.z < EPSILON {
        return pose.position;
    }

    // Transform direction to local space
    let local_dir = pose.rotation.inverse() * direction;

    // Scale the direction by the radii (equivalent to finding support on unit sphere
    // then scaling the result)
    let scaled_dir = Vector3::new(
        local_dir.x * radii.x,
        local_dir.y * radii.y,
        local_dir.z * radii.z,
    );

    let scaled_norm = scaled_dir.norm();
    if scaled_norm < EPSILON {
        // Degenerate direction - return center
        return pose.position;
    }

    // The support point on the ellipsoid surface
    // p = radii * normalized(radii * dir)
    // = radii * (radii * dir) / |radii * dir|
    // = radii^2 * dir / |radii * dir|
    let local_support = Point3::new(
        radii.x * scaled_dir.x / scaled_norm,
        radii.y * scaled_dir.y / scaled_norm,
        radii.z * scaled_dir.z / scaled_norm,
    );

    pose.transform_point(&local_support)
}

// =============================================================================
// GJK Algorithm
// =============================================================================

/// Compute the support point on the Minkowski difference A - B.
fn support_minkowski(
    shape_a: &CollisionShape,
    pose_a: &Pose,
    shape_b: &CollisionShape,
    pose_b: &Pose,
    direction: &Vector3<f64>,
) -> MinkowskiPoint {
    let support_a = support(shape_a, pose_a, direction);
    let support_b = support(shape_b, pose_b, &-direction);
    MinkowskiPoint::new(support_a, support_b)
}

/// Check if two shapes intersect using GJK.
///
/// This is a fast intersection test that returns true if the shapes overlap.
/// For penetration information, use [`gjk_epa_contact`].
#[must_use]
pub fn gjk_intersection(
    shape_a: &CollisionShape,
    pose_a: &Pose,
    shape_b: &CollisionShape,
    pose_b: &Pose,
    max_iterations: usize,
) -> bool {
    gjk_query(shape_a, pose_a, shape_b, pose_b, max_iterations).intersecting
}

/// Run the full GJK algorithm and return detailed results.
#[must_use]
pub fn gjk_query(
    shape_a: &CollisionShape,
    pose_a: &Pose,
    shape_b: &CollisionShape,
    pose_b: &Pose,
    max_iterations: usize,
) -> GjkResult {
    // Initial direction: from center of A to center of B
    let center_a = pose_a.position;
    let center_b = pose_b.position;
    let dir_vec = center_b - center_a;
    let dir_norm = dir_vec.norm();

    // Handle degenerate case where centers coincide (or nearly coincide)
    // Must check BEFORE normalize() to avoid NaN
    let mut direction = if dir_norm > EPSILON {
        dir_vec / dir_norm
    } else {
        Vector3::x()
    };

    let mut simplex = Simplex::new();

    // Get first support point
    let first = support_minkowski(shape_a, pose_a, shape_b, pose_b, &direction);
    simplex.push(first);

    // New search direction: toward origin from first point
    direction = -first.point.coords;

    for iteration in 0..max_iterations {
        // Check for degenerate direction
        if direction.norm_squared() < EPSILON * EPSILON {
            // Origin is on the simplex - shapes are touching/intersecting
            return GjkResult {
                intersecting: true,
                simplex,
                iterations: iteration,
            };
        }

        direction = direction.normalize();

        // Get new support point
        let new_point = support_minkowski(shape_a, pose_a, shape_b, pose_b, &direction);

        // Check if we passed the origin
        // If new_point · direction < 0, origin is not in that direction
        if new_point.point.coords.dot(&direction) < -EPSILON {
            // We couldn't pass the origin, shapes don't intersect
            return GjkResult {
                intersecting: false,
                simplex,
                iterations: iteration,
            };
        }

        // Add to simplex and update
        simplex.push(new_point);

        // Handle simplex: find the closest feature to the origin
        // and potentially detect if the origin is enclosed
        if do_simplex(&mut simplex, &mut direction) {
            // Origin is enclosed - shapes intersect
            return GjkResult {
                intersecting: true,
                simplex,
                iterations: iteration,
            };
        }
    }

    // Max iterations reached - assume no intersection
    GjkResult {
        intersecting: false,
        simplex,
        iterations: max_iterations,
    }
}

/// Process the simplex and update the search direction.
///
/// Returns true if the origin is enclosed by the simplex (intersection).
fn do_simplex(simplex: &mut Simplex, direction: &mut Vector3<f64>) -> bool {
    match simplex.len() {
        2 => do_simplex_line(simplex, direction),
        3 => do_simplex_triangle(simplex, direction),
        4 => do_simplex_tetrahedron(simplex, direction),
        _ => false,
    }
}

/// Handle line simplex (2 points).
fn do_simplex_line(simplex: &mut Simplex, direction: &mut Vector3<f64>) -> bool {
    let a = simplex.points[0].point;
    let b = simplex.points[1].point;

    let ab = b - a;
    let ao = -a.coords; // Vector from A to origin

    if ab.dot(&ao) > 0.0 {
        // Origin is in the region of the line segment
        // Direction perpendicular to AB, toward origin
        *direction = triple_product(&ab, &ao, &ab);
    } else {
        // Origin is beyond A, keep only A
        simplex.set(&[simplex.points[0]]);
        *direction = ao;
    }

    false
}

/// Handle triangle simplex (3 points).
fn do_simplex_triangle(simplex: &mut Simplex, direction: &mut Vector3<f64>) -> bool {
    let a = simplex.points[0].point;
    let b = simplex.points[1].point;
    let c = simplex.points[2].point;

    let ab = b - a;
    let ac = c - a;
    let ao = -a.coords;

    let abc = ab.cross(&ac); // Normal of the triangle

    // Check if origin is above or below the triangle
    if abc.cross(&ac).dot(&ao) > 0.0 {
        // Origin is outside edge AC
        if ac.dot(&ao) > 0.0 {
            simplex.set(&[simplex.points[0], simplex.points[2]]);
            *direction = triple_product(&ac, &ao, &ac);
        } else {
            return simplex_line_case(simplex, direction, &ab, &ao);
        }
    } else if ab.cross(&abc).dot(&ao) > 0.0 {
        // Origin is outside edge AB
        return simplex_line_case(simplex, direction, &ab, &ao);
    } else {
        // Origin is inside the triangle region (above or below)
        if abc.dot(&ao) > 0.0 {
            // Above the triangle
            *direction = abc;
        } else {
            // Below the triangle - reverse winding
            simplex.set(&[simplex.points[0], simplex.points[2], simplex.points[1]]);
            *direction = -abc;
        }
    }

    false
}

/// Helper for triangle simplex - handle the AB edge case.
fn simplex_line_case(
    simplex: &mut Simplex,
    direction: &mut Vector3<f64>,
    ab: &Vector3<f64>,
    ao: &Vector3<f64>,
) -> bool {
    if ab.dot(ao) > 0.0 {
        simplex.set(&[simplex.points[0], simplex.points[1]]);
        *direction = triple_product(ab, ao, ab);
    } else {
        simplex.set(&[simplex.points[0]]);
        *direction = *ao;
    }
    false
}

/// Handle tetrahedron simplex (4 points).
fn do_simplex_tetrahedron(simplex: &mut Simplex, direction: &mut Vector3<f64>) -> bool {
    let a = simplex.points[0].point;
    let b = simplex.points[1].point;
    let c = simplex.points[2].point;
    let d = simplex.points[3].point;

    let ab = b - a;
    let ac = c - a;
    let ad = d - a;
    let ao = -a.coords;

    let abc = ab.cross(&ac);
    let acd = ac.cross(&ad);
    let adb = ad.cross(&ab);

    // Check each face
    if abc.dot(&ao) > 0.0 {
        // Origin is in front of face ABC
        simplex.set(&[simplex.points[0], simplex.points[1], simplex.points[2]]);
        return do_simplex_triangle(simplex, direction);
    }

    if acd.dot(&ao) > 0.0 {
        // Origin is in front of face ACD
        simplex.set(&[simplex.points[0], simplex.points[2], simplex.points[3]]);
        return do_simplex_triangle(simplex, direction);
    }

    if adb.dot(&ao) > 0.0 {
        // Origin is in front of face ADB
        simplex.set(&[simplex.points[0], simplex.points[3], simplex.points[1]]);
        return do_simplex_triangle(simplex, direction);
    }

    // Origin is inside the tetrahedron
    true
}

/// Triple product: (A × B) × C = B(A·C) - A(B·C).
///
/// Useful for computing a vector perpendicular to A and pointing toward C.
#[inline]
fn triple_product(a: &Vector3<f64>, b: &Vector3<f64>, c: &Vector3<f64>) -> Vector3<f64> {
    b * a.dot(c) - a * b.dot(c)
}

// =============================================================================
// EPA Algorithm
// =============================================================================

/// A face in the EPA polytope.
#[derive(Debug, Clone)]
struct EpaFace {
    /// Indices of the three vertices forming this face.
    vertices: [usize; 3],
    /// Outward-facing normal of the face.
    normal: Vector3<f64>,
    /// Distance from origin to the face (along normal).
    distance: f64,
}

/// Run EPA to find penetration depth and contact normal.
///
/// Should only be called after GJK has determined that shapes intersect.
/// The simplex from GJK is used to initialize the polytope.
#[must_use]
pub fn epa_query(
    shape_a: &CollisionShape,
    pose_a: &Pose,
    shape_b: &CollisionShape,
    pose_b: &Pose,
    simplex: &Simplex,
    max_iterations: usize,
    tolerance: f64,
) -> Option<EpaResult> {
    // We need at least a tetrahedron to start EPA
    if simplex.len() < 4 {
        // Try to build a tetrahedron from the simplex
        return epa_with_expanded_simplex(
            shape_a,
            pose_a,
            shape_b,
            pose_b,
            simplex,
            max_iterations,
            tolerance,
        );
    }

    // Initialize polytope with the tetrahedron
    let mut vertices: Vec<MinkowskiPoint> = simplex.points().to_vec();
    let mut faces: Vec<EpaFace> = Vec::with_capacity(EPA_MAX_FACES);

    // Create initial tetrahedron faces (ensure correct winding)
    let initial_faces = [[0, 1, 2], [0, 2, 3], [0, 3, 1], [1, 3, 2]];

    for indices in initial_faces {
        if let Some(face) = create_face(&vertices, indices) {
            faces.push(face);
        }
    }

    // Fix face orientations (all normals should point outward)
    fix_face_orientations(&vertices, &mut faces);

    for iteration in 0..max_iterations {
        // Find the closest face to the origin
        let closest_idx = find_closest_face(&faces)?;
        let closest = &faces[closest_idx];

        // Get a new support point in the direction of the closest face's normal
        let new_point = support_minkowski(shape_a, pose_a, shape_b, pose_b, &closest.normal);

        // Check convergence: if new point is not significantly further than the face
        let new_distance = new_point.point.coords.dot(&closest.normal);
        if new_distance - closest.distance < tolerance {
            // Converged
            return Some(EpaResult {
                depth: closest.distance,
                normal: closest.normal,
                point: Point3::from(closest.normal * closest.distance),
                iterations: iteration,
            });
        }

        // Add new vertex
        let new_vertex_idx = vertices.len();
        vertices.push(new_point);

        // Remove faces that can "see" the new point and collect the edges
        let mut edges: Vec<(usize, usize)> = Vec::new();
        let mut i = 0;
        while i < faces.len() {
            let face = &faces[i];
            let face_point = vertices[face.vertices[0]].point;
            let to_new = new_point.point - face_point;

            if face.normal.dot(&to_new) > 0.0 {
                // This face can see the new point, remove it
                // Add its edges to the edge list (for later retriangulation)
                let v = face.vertices;
                add_edge(&mut edges, v[0], v[1]);
                add_edge(&mut edges, v[1], v[2]);
                add_edge(&mut edges, v[2], v[0]);
                faces.swap_remove(i);
            } else {
                i += 1;
            }
        }

        // Create new faces from the unique edges and the new vertex
        for (v1, v2) in edges {
            if let Some(face) = create_face(&vertices, [new_vertex_idx, v1, v2]) {
                faces.push(face);
            }
        }

        // Sanity check
        if faces.len() > EPA_MAX_FACES {
            break;
        }
    }

    // Return best result found
    let closest_idx = find_closest_face(&faces)?;
    let closest = &faces[closest_idx];
    Some(EpaResult {
        depth: closest.distance,
        normal: closest.normal,
        point: Point3::from(closest.normal * closest.distance),
        iterations: max_iterations,
    })
}

/// Expand a simplex with fewer than 4 points into a tetrahedron for EPA.
fn epa_with_expanded_simplex(
    shape_a: &CollisionShape,
    pose_a: &Pose,
    shape_b: &CollisionShape,
    pose_b: &Pose,
    simplex: &Simplex,
    max_iterations: usize,
    tolerance: f64,
) -> Option<EpaResult> {
    let mut vertices: Vec<MinkowskiPoint> = simplex.points().to_vec();

    // Expand to a tetrahedron by adding support points in different directions
    let search_dirs = [
        Vector3::x(),
        Vector3::y(),
        Vector3::z(),
        -Vector3::x(),
        -Vector3::y(),
        -Vector3::z(),
    ];

    for dir in &search_dirs {
        if vertices.len() >= 4 {
            break;
        }

        let new_point = support_minkowski(shape_a, pose_a, shape_b, pose_b, dir);

        // Check if this point is actually new (not too close to existing points)
        let is_new = vertices
            .iter()
            .all(|v| (v.point - new_point.point).norm() > EPSILON);

        if is_new {
            vertices.push(new_point);
        }
    }

    if vertices.len() < 4 {
        // Degenerate case - shapes might be very thin
        return None;
    }

    // Create a new simplex and recurse
    let mut new_simplex = Simplex::new();
    for v in vertices.iter().take(4) {
        new_simplex.push(*v);
    }

    epa_query(
        shape_a,
        pose_a,
        shape_b,
        pose_b,
        &new_simplex,
        max_iterations,
        tolerance,
    )
}

/// Create a face from three vertex indices.
fn create_face(vertices: &[MinkowskiPoint], indices: [usize; 3]) -> Option<EpaFace> {
    let a = vertices[indices[0]].point;
    let b = vertices[indices[1]].point;
    let c = vertices[indices[2]].point;

    let ab = b - a;
    let ac = c - a;
    let normal = ab.cross(&ac);

    let norm = normal.norm();
    if norm < EPSILON {
        // Degenerate face
        return None;
    }

    let normal = normal / norm;
    let distance = a.coords.dot(&normal);

    Some(EpaFace {
        vertices: indices,
        normal,
        distance,
    })
}

/// Fix face orientations so all normals point outward (away from origin).
fn fix_face_orientations(vertices: &[MinkowskiPoint], faces: &mut [EpaFace]) {
    // Validate vertices before computing centroid
    if vertices.is_empty() {
        return;
    }

    // Compute centroid
    #[allow(clippy::cast_precision_loss)]
    let centroid: Vector3<f64> = vertices
        .iter()
        .map(|v| v.point.coords)
        .sum::<Vector3<f64>>()
        / vertices.len() as f64;

    for face in faces.iter_mut() {
        let face_point = vertices[face.vertices[0]].point;
        let to_centroid = centroid - face_point.coords;

        // If normal points toward centroid, flip it
        if face.normal.dot(&to_centroid) > 0.0 {
            face.normal = -face.normal;
            face.distance = -face.distance;
            face.vertices.swap(1, 2);
        }
    }
}

/// Find the face closest to the origin.
fn find_closest_face(faces: &[EpaFace]) -> Option<usize> {
    if faces.is_empty() {
        return None;
    }

    // Filter out faces with NaN distances, then find minimum
    faces
        .iter()
        .enumerate()
        .filter(|(_, f)| f.distance.is_finite())
        .min_by(|(_, a), (_, b)| {
            // Safe: we filtered NaN above, so partial_cmp will never return None
            a.distance
                .abs()
                .partial_cmp(&b.distance.abs())
                .unwrap_or(std::cmp::Ordering::Equal)
        })
        .map(|(i, _)| i)
}

/// Add an edge to the edge list, removing it if it already exists (shared edge).
fn add_edge(edges: &mut Vec<(usize, usize)>, v1: usize, v2: usize) {
    // Check if this edge already exists (in either order)
    let existing = edges
        .iter()
        .position(|&(a, b)| (a == v2 && b == v1) || (a == v1 && b == v2));

    if let Some(idx) = existing {
        // Shared edge - remove it
        edges.swap_remove(idx);
    } else {
        // New edge
        edges.push((v1, v2));
    }
}

// =============================================================================
// High-Level API
// =============================================================================

/// Compute contact information between two shapes using GJK+EPA.
///
/// Returns `None` if the shapes don't intersect or if contact computation fails.
#[must_use]
pub fn gjk_epa_contact(
    shape_a: &CollisionShape,
    pose_a: &Pose,
    shape_b: &CollisionShape,
    pose_b: &Pose,
    max_iterations: usize,
    tolerance: f64,
) -> Option<GjkContact> {
    // Run GJK
    let gjk_result = gjk_query(shape_a, pose_a, shape_b, pose_b, max_iterations);

    if !gjk_result.intersecting {
        return None;
    }

    // Run EPA to get penetration info
    let epa_result = epa_query(
        shape_a,
        pose_a,
        shape_b,
        pose_b,
        &gjk_result.simplex,
        max_iterations,
        tolerance,
    )?;

    // Contact point: support vertex on shape A in the -normal direction.
    let contact_point = support(shape_a, pose_a, &(-epa_result.normal));

    Some(GjkContact {
        point: contact_point,
        normal: epa_result.normal,
        penetration: epa_result.depth.max(0.0),
    })
}

// =============================================================================
// GJK Distance Query
// =============================================================================

/// Result of GJK distance query for non-overlapping convex shapes.
#[derive(Debug, Clone)]
pub struct GjkDistanceResult {
    /// Minimum separating distance (> 0 if non-overlapping).
    pub distance: f64,
    /// Closest point on shape A (in world frame).
    pub witness_a: Point3<f64>,
    /// Closest point on shape B (in world frame).
    pub witness_b: Point3<f64>,
    /// Number of iterations used.
    pub iterations: usize,
}

/// Compute minimum separating distance between two non-overlapping convex shapes.
///
/// Returns `None` if shapes are overlapping (use `gjk_epa_contact` instead).
/// Uses GJK distance algorithm (van den Bergen, 2003): iteratively refine the
/// closest point on the Minkowski difference to the origin.
///
/// `max_iterations` and `tolerance` come from `model.ccd_iterations` and
/// `model.ccd_tolerance` respectively.
#[must_use]
pub fn gjk_distance(
    shape_a: &CollisionShape,
    pose_a: &Pose,
    shape_b: &CollisionShape,
    pose_b: &Pose,
    max_iterations: usize,
    tolerance: f64,
) -> Option<GjkDistanceResult> {
    let center_a = pose_a.position;
    let center_b = pose_b.position;
    let dir_vec = center_b - center_a;
    let dir_norm = dir_vec.norm();

    let mut direction = if dir_norm > EPSILON {
        dir_vec / dir_norm
    } else {
        Vector3::x()
    };

    let mut simplex = Simplex::new();

    // Get first support point
    let first = support_minkowski(shape_a, pose_a, shape_b, pose_b, &direction);
    simplex.push(first);

    // Track closest distance to detect convergence
    let mut closest_dist_sq = first.point.coords.norm_squared();
    direction = -first.point.coords;

    for iteration in 0..max_iterations {
        let dir_norm_sq = direction.norm_squared();
        if dir_norm_sq < EPSILON * EPSILON {
            // Origin is on the simplex — shapes are touching (distance = 0)
            // Transition to EPA for penetration
            return None;
        }

        let dir_normalized = direction / dir_norm_sq.sqrt();
        let new_point = support_minkowski(shape_a, pose_a, shape_b, pose_b, &dir_normalized);

        // Check if we made progress toward the origin.
        // If the new support point doesn't pass the current closest point
        // in the search direction, the shapes may overlap.
        let dot = new_point.point.coords.dot(&dir_normalized);
        if dot < EPSILON {
            // Cannot get closer to origin — shapes don't overlap.
            // Current simplex contains the closest feature.
            break;
        }

        simplex.push(new_point);

        // Find closest point on simplex to origin and reduce simplex
        let (closest_point, bary) = closest_point_on_simplex_to_origin(&simplex);
        let new_dist_sq = closest_point.norm_squared();

        // Convergence check: relative distance improvement < tolerance.
        let improvement = closest_dist_sq - new_dist_sq;
        if improvement < tolerance * tolerance
            || (closest_dist_sq > EPSILON && improvement / closest_dist_sq < tolerance)
        {
            // Converged — compute witness points from barycentric coords
            let (wa, wb) = recover_witness_points(&simplex, &bary);
            return Some(GjkDistanceResult {
                distance: new_dist_sq.sqrt(),
                witness_a: wa,
                witness_b: wb,
                iterations: iteration,
            });
        }

        closest_dist_sq = new_dist_sq;

        // Reduce simplex to closest sub-feature
        reduce_simplex_to_closest(&mut simplex, &bary);

        // New search direction: from closest point toward origin
        direction = -closest_point;
    }

    // Max iterations or early break — compute final result
    let (closest_point, bary) = closest_point_on_simplex_to_origin(&simplex);
    let dist = closest_point.norm();
    if dist < EPSILON {
        return None; // Touching — use EPA
    }
    let (wa, wb) = recover_witness_points(&simplex, &bary);
    Some(GjkDistanceResult {
        distance: dist,
        witness_a: wa,
        witness_b: wb,
        iterations: max_iterations,
    })
}

/// Compute the closest point on the simplex to the origin.
///
/// Returns (closest_point_vector, barycentric_coordinates).
/// Barycentric coordinates are used to recover witness points from
/// the `MinkowskiPoint`'s `support_a`/`support_b` fields.
fn closest_point_on_simplex_to_origin(simplex: &Simplex) -> (Vector3<f64>, Vec<(usize, f64)>) {
    match simplex.len() {
        1 => {
            // Point: closest point is the point itself
            let p = simplex.points[0].point.coords;
            (p, vec![(0, 1.0)])
        }
        2 => {
            // Line segment: project origin onto segment [A, B]
            let a = simplex.points[0].point.coords;
            let b = simplex.points[1].point.coords;
            let ab = b - a;
            let ao = -a;
            let ab_sq = ab.norm_squared();
            if ab_sq < EPSILON * EPSILON {
                return (a, vec![(0, 1.0)]);
            }
            let t = (ao.dot(&ab) / ab_sq).clamp(0.0, 1.0);
            let closest = a + t * ab;
            (closest, vec![(0, 1.0 - t), (1, t)])
        }
        3 => {
            // Triangle: project origin onto triangle plane, check Voronoi regions
            closest_point_on_triangle_to_origin(simplex)
        }
        _ => {
            // Should not happen in distance mode (simplex reduced before size 4)
            let p = simplex.points[0].point.coords;
            (p, vec![(0, 1.0)])
        }
    }
}

/// Full Voronoi region analysis for the triangle case.
/// Projects origin onto the triangle ABC and checks all 7 Voronoi regions.
#[allow(clippy::many_single_char_names)] // standard geometric notation (a, b, c, v, w)
fn closest_point_on_triangle_to_origin(simplex: &Simplex) -> (Vector3<f64>, Vec<(usize, f64)>) {
    let a = simplex.points[0].point.coords;
    let b = simplex.points[1].point.coords;
    let c = simplex.points[2].point.coords;

    let ab = b - a;
    let ac = c - a;
    let ao = -a;

    let d1 = ab.dot(&ao);
    let d2 = ac.dot(&ao);

    // Vertex A region
    if d1 <= 0.0 && d2 <= 0.0 {
        return (a, vec![(0, 1.0)]);
    }

    let bo = -b;
    let d3 = ab.dot(&bo);
    let d4 = ac.dot(&bo);

    // Vertex B region
    if d3 >= 0.0 && d4 <= d3 {
        return (b, vec![(1, 1.0)]);
    }

    // Edge AB region
    let vc = d1 * d4 - d3 * d2;
    if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
        let v = d1 / (d1 - d3);
        let closest = a + v * ab;
        return (closest, vec![(0, 1.0 - v), (1, v)]);
    }

    let co = -c;
    let d5 = ab.dot(&co);
    let d6 = ac.dot(&co);

    // Vertex C region
    if d6 >= 0.0 && d5 <= d6 {
        return (c, vec![(2, 1.0)]);
    }

    // Edge AC region
    let vb = d5 * d2 - d1 * d6;
    if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
        let w = d2 / (d2 - d6);
        let closest = a + w * ac;
        return (closest, vec![(0, 1.0 - w), (2, w)]);
    }

    // Edge BC region
    let va = d3 * d6 - d5 * d4;
    if va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0 {
        let w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        let closest = b + w * (c - b);
        return (closest, vec![(1, 1.0 - w), (2, w)]);
    }

    // Interior of triangle
    let denom = 1.0 / (va + vb + vc);
    let v = vb * denom;
    let w = vc * denom;
    let closest = a + v * ab + w * ac;
    (closest, vec![(0, 1.0 - v - w), (1, v), (2, w)])
}

/// Recover world-space witness points from barycentric coordinates.
fn recover_witness_points(simplex: &Simplex, bary: &[(usize, f64)]) -> (Point3<f64>, Point3<f64>) {
    let mut wa = Vector3::zeros();
    let mut wb = Vector3::zeros();
    for &(idx, weight) in bary {
        wa += simplex.points[idx].support_a.coords * weight;
        wb += simplex.points[idx].support_b.coords * weight;
    }
    (Point3::from(wa), Point3::from(wb))
}

/// Reduce simplex to only the vertices that contribute to the closest point.
fn reduce_simplex_to_closest(simplex: &mut Simplex, bary: &[(usize, f64)]) {
    let active: Vec<MinkowskiPoint> = bary
        .iter()
        .filter(|(_, w)| *w > EPSILON)
        .map(|&(idx, _)| simplex.points[idx])
        .collect();
    simplex.set(&active);
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

    fn pose_at(x: f64, y: f64, z: f64) -> Pose {
        Pose::from_position(Point3::new(x, y, z))
    }

    #[test]
    fn test_support_sphere() {
        let pose = pose_at(1.0, 2.0, 3.0);
        let radius = 0.5;

        let dir = Vector3::x();
        let support = support_sphere(&pose, radius, &dir);
        assert_relative_eq!(support.x, 1.5, epsilon = 1e-10);
        assert_relative_eq!(support.y, 2.0, epsilon = 1e-10);
        assert_relative_eq!(support.z, 3.0, epsilon = 1e-10);
    }

    #[test]
    fn test_support_box() {
        let pose = Pose::identity();
        let half_extents = Vector3::new(1.0, 2.0, 3.0);

        // Support in +X direction
        let support = support_box(&pose, &half_extents, &Vector3::x());
        assert_relative_eq!(support.x, 1.0, epsilon = 1e-10);

        // Support in diagonal direction
        let dir = Vector3::new(1.0, 1.0, 1.0).normalize();
        let support = support_box(&pose, &half_extents, &dir);
        assert_relative_eq!(support.x, 1.0, epsilon = 1e-10);
        assert_relative_eq!(support.y, 2.0, epsilon = 1e-10);
        assert_relative_eq!(support.z, 3.0, epsilon = 1e-10);
    }

    #[test]
    fn test_gjk_spheres_intersecting() {
        let shape_a = CollisionShape::sphere(1.0);
        let shape_b = CollisionShape::sphere(1.0);

        let pose_a = pose_at(0.0, 0.0, 0.0);
        let pose_b = pose_at(1.5, 0.0, 0.0); // Centers 1.5 apart, radii sum to 2

        assert!(gjk_intersection(
            &shape_a,
            &pose_a,
            &shape_b,
            &pose_b,
            GJK_MAX_ITERATIONS
        ));
    }

    #[test]
    fn test_gjk_spheres_separated() {
        let shape_a = CollisionShape::sphere(1.0);
        let shape_b = CollisionShape::sphere(1.0);

        let pose_a = pose_at(0.0, 0.0, 0.0);
        let pose_b = pose_at(3.0, 0.0, 0.0); // Centers 3 apart, radii sum to 2

        assert!(!gjk_intersection(
            &shape_a,
            &pose_a,
            &shape_b,
            &pose_b,
            GJK_MAX_ITERATIONS
        ));
    }

    #[test]
    fn test_gjk_sphere_box_intersecting() {
        let shape_a = CollisionShape::sphere(1.0);
        let shape_b = CollisionShape::box_shape(Vector3::new(0.5, 0.5, 0.5));

        let pose_a = pose_at(0.0, 0.0, 0.0);
        let pose_b = pose_at(1.0, 0.0, 0.0); // Should overlap

        assert!(gjk_intersection(
            &shape_a,
            &pose_a,
            &shape_b,
            &pose_b,
            GJK_MAX_ITERATIONS
        ));
    }

    #[test]
    fn test_gjk_convex_mesh() {
        // Create a cube as a convex mesh
        let vertices = vec![
            Point3::new(-0.5, -0.5, -0.5),
            Point3::new(0.5, -0.5, -0.5),
            Point3::new(0.5, 0.5, -0.5),
            Point3::new(-0.5, 0.5, -0.5),
            Point3::new(-0.5, -0.5, 0.5),
            Point3::new(0.5, -0.5, 0.5),
            Point3::new(0.5, 0.5, 0.5),
            Point3::new(-0.5, 0.5, 0.5),
        ];
        let shape_a = CollisionShape::convex_mesh(vertices);
        let shape_b = CollisionShape::sphere(0.5);

        let pose_a = pose_at(0.0, 0.0, 0.0);
        let pose_b = pose_at(0.8, 0.0, 0.0); // Should overlap

        assert!(gjk_intersection(
            &shape_a,
            &pose_a,
            &shape_b,
            &pose_b,
            GJK_MAX_ITERATIONS
        ));
    }

    #[test]
    fn test_gjk_epa_contact_spheres() {
        let shape_a = CollisionShape::sphere(1.0);
        let shape_b = CollisionShape::sphere(1.0);

        let pose_a = pose_at(0.0, 0.0, 0.0);
        let pose_b = pose_at(1.5, 0.0, 0.0);

        let contact = gjk_epa_contact(
            &shape_a,
            &pose_a,
            &shape_b,
            &pose_b,
            GJK_MAX_ITERATIONS,
            EPA_TOLERANCE,
        );
        assert!(contact.is_some());

        let contact = contact.unwrap();
        // Penetration should be 0.5 (radii sum 2, centers 1.5 apart)
        assert_relative_eq!(contact.penetration, 0.5, epsilon = 0.1);
        // Normal should be along X axis (either direction, as long as it's normalized)
        assert!(contact.normal.x.abs() > 0.9);
        assert_relative_eq!(contact.normal.norm(), 1.0, epsilon = 1e-6);
    }

    #[test]
    fn test_gjk_tetrahedra() {
        let shape_a = CollisionShape::tetrahedron(0.5);
        let shape_b = CollisionShape::tetrahedron(0.5);

        let pose_a = pose_at(0.0, 0.0, 0.0);
        let pose_b = pose_at(0.3, 0.0, 0.0); // Should overlap

        assert!(gjk_intersection(
            &shape_a,
            &pose_a,
            &shape_b,
            &pose_b,
            GJK_MAX_ITERATIONS
        ));

        // Separated
        let pose_b_far = pose_at(2.0, 0.0, 0.0);
        assert!(!gjk_intersection(
            &shape_a,
            &pose_a,
            &shape_b,
            &pose_b_far,
            GJK_MAX_ITERATIONS
        ));
    }

    #[test]
    fn test_simplex_operations() {
        let mut simplex = Simplex::new();
        assert!(simplex.is_empty());

        let p1 = MinkowskiPoint::new(Point3::new(1.0, 0.0, 0.0), Point3::origin());
        simplex.push(p1);
        assert_eq!(simplex.len(), 1);

        let p2 = MinkowskiPoint::new(Point3::new(0.0, 1.0, 0.0), Point3::origin());
        simplex.push(p2);
        assert_eq!(simplex.len(), 2);
    }

    // =========================================================================
    // Cylinder tests
    // =========================================================================

    #[test]
    fn test_support_cylinder() {
        let pose = Pose::identity();
        let half_length = 1.0;
        let radius = 0.5;

        // Support in +Z direction (top cap center edge)
        let support = support_cylinder(&pose, half_length, radius, &Vector3::z());
        assert_relative_eq!(support.z, half_length, epsilon = 1e-10);

        // Support in +X direction (side of cylinder)
        let support = support_cylinder(&pose, half_length, radius, &Vector3::x());
        assert_relative_eq!(support.x, radius, epsilon = 1e-10);

        // Support in diagonal direction
        let dir = Vector3::new(1.0, 0.0, 1.0).normalize();
        let support = support_cylinder(&pose, half_length, radius, &dir);
        assert_relative_eq!(support.z, half_length, epsilon = 1e-10);
        assert_relative_eq!(support.x, radius, epsilon = 1e-10);
    }

    #[test]
    fn test_gjk_cylinder_sphere_intersecting() {
        let shape_a = CollisionShape::cylinder(1.0, 0.5);
        let shape_b = CollisionShape::sphere(0.5);

        let pose_a = pose_at(0.0, 0.0, 0.0);
        let pose_b = pose_at(0.8, 0.0, 0.0); // Should overlap

        assert!(gjk_intersection(
            &shape_a,
            &pose_a,
            &shape_b,
            &pose_b,
            GJK_MAX_ITERATIONS
        ));
    }

    #[test]
    fn test_gjk_cylinder_sphere_separated() {
        let shape_a = CollisionShape::cylinder(1.0, 0.5);
        let shape_b = CollisionShape::sphere(0.5);

        let pose_a = pose_at(0.0, 0.0, 0.0);
        let pose_b = pose_at(2.0, 0.0, 0.0); // Far away

        assert!(!gjk_intersection(
            &shape_a,
            &pose_a,
            &shape_b,
            &pose_b,
            GJK_MAX_ITERATIONS
        ));
    }

    #[test]
    fn test_gjk_cylinder_plane_contact() {
        let shape_a = CollisionShape::cylinder(1.0, 0.5);
        let shape_b = CollisionShape::plane(Vector3::z(), 0.0);

        // Cylinder centered at z=0.5, so bottom cap touches ground at z=-0.5
        let pose_a = pose_at(0.0, 0.0, 0.5);
        let pose_b = Pose::identity();

        // Should be penetrating by 0.5
        let contact = gjk_epa_contact(
            &shape_a,
            &pose_a,
            &shape_b,
            &pose_b,
            GJK_MAX_ITERATIONS,
            EPA_TOLERANCE,
        );
        assert!(contact.is_some());

        let contact = contact.unwrap();
        assert!(contact.penetration > 0.0);
    }

    #[test]
    fn test_gjk_cylinder_cylinder_intersecting() {
        let shape_a = CollisionShape::cylinder(1.0, 0.5);
        let shape_b = CollisionShape::cylinder(1.0, 0.5);

        let pose_a = pose_at(0.0, 0.0, 0.0);
        let pose_b = pose_at(0.8, 0.0, 0.0); // Should overlap

        assert!(gjk_intersection(
            &shape_a,
            &pose_a,
            &shape_b,
            &pose_b,
            GJK_MAX_ITERATIONS
        ));
    }

    // =========================================================================
    // Ellipsoid tests
    // =========================================================================

    #[test]
    fn test_support_ellipsoid() {
        let pose = Pose::identity();
        let radii = Vector3::new(2.0, 1.0, 0.5);

        // Support in +X direction
        let support = support_ellipsoid(&pose, &radii, &Vector3::x());
        assert_relative_eq!(support.x, 2.0, epsilon = 1e-10);
        assert_relative_eq!(support.y, 0.0, epsilon = 1e-10);
        assert_relative_eq!(support.z, 0.0, epsilon = 1e-10);

        // Support in +Y direction
        let support = support_ellipsoid(&pose, &radii, &Vector3::y());
        assert_relative_eq!(support.x, 0.0, epsilon = 1e-10);
        assert_relative_eq!(support.y, 1.0, epsilon = 1e-10);
        assert_relative_eq!(support.z, 0.0, epsilon = 1e-10);

        // Support in +Z direction
        let support = support_ellipsoid(&pose, &radii, &Vector3::z());
        assert_relative_eq!(support.x, 0.0, epsilon = 1e-10);
        assert_relative_eq!(support.y, 0.0, epsilon = 1e-10);
        assert_relative_eq!(support.z, 0.5, epsilon = 1e-10);
    }

    #[test]
    fn test_support_ellipsoid_equals_sphere() {
        // An ellipsoid with equal radii should behave like a sphere
        let pose = pose_at(1.0, 2.0, 3.0);
        let radii = Vector3::new(0.5, 0.5, 0.5);

        let dir = Vector3::new(1.0, 1.0, 1.0).normalize();
        let support_ellipsoid_pt = support_ellipsoid(&pose, &radii, &dir);
        let support_sphere_pt = support_sphere(&pose, 0.5, &dir);

        assert_relative_eq!(support_ellipsoid_pt.x, support_sphere_pt.x, epsilon = 1e-10);
        assert_relative_eq!(support_ellipsoid_pt.y, support_sphere_pt.y, epsilon = 1e-10);
        assert_relative_eq!(support_ellipsoid_pt.z, support_sphere_pt.z, epsilon = 1e-10);
    }

    #[test]
    fn test_gjk_ellipsoid_sphere_intersecting() {
        let shape_a = CollisionShape::ellipsoid_xyz(2.0, 1.0, 0.5);
        let shape_b = CollisionShape::sphere(0.5);

        let pose_a = pose_at(0.0, 0.0, 0.0);
        let pose_b = pose_at(2.0, 0.0, 0.0); // Just touching at ellipsoid's long axis

        assert!(gjk_intersection(
            &shape_a,
            &pose_a,
            &shape_b,
            &pose_b,
            GJK_MAX_ITERATIONS
        ));
    }

    #[test]
    fn test_gjk_ellipsoid_sphere_separated() {
        let shape_a = CollisionShape::ellipsoid_xyz(2.0, 1.0, 0.5);
        let shape_b = CollisionShape::sphere(0.5);

        let pose_a = pose_at(0.0, 0.0, 0.0);
        let pose_b = pose_at(3.0, 0.0, 0.0); // Far away

        assert!(!gjk_intersection(
            &shape_a,
            &pose_a,
            &shape_b,
            &pose_b,
            GJK_MAX_ITERATIONS
        ));
    }

    #[test]
    fn test_gjk_ellipsoid_plane_contact() {
        let shape_a = CollisionShape::ellipsoid_xyz(2.0, 1.0, 0.5);
        let shape_b = CollisionShape::plane(Vector3::z(), 0.0);

        // Ellipsoid centered at z=0.3, so bottom (z-radius = 0.5) penetrates
        let pose_a = pose_at(0.0, 0.0, 0.3);
        let pose_b = Pose::identity();

        let contact = gjk_epa_contact(
            &shape_a,
            &pose_a,
            &shape_b,
            &pose_b,
            GJK_MAX_ITERATIONS,
            EPA_TOLERANCE,
        );
        assert!(contact.is_some());

        let contact = contact.unwrap();
        // Penetration = 0.5 - 0.3 = 0.2
        assert_relative_eq!(contact.penetration, 0.2, epsilon = 0.1);
    }

    #[test]
    fn test_gjk_ellipsoid_ellipsoid_intersecting() {
        let shape_a = CollisionShape::ellipsoid_xyz(1.0, 0.5, 0.5);
        let shape_b = CollisionShape::ellipsoid_xyz(1.0, 0.5, 0.5);

        let pose_a = pose_at(0.0, 0.0, 0.0);
        let pose_b = pose_at(1.5, 0.0, 0.0); // Should overlap along X

        assert!(gjk_intersection(
            &shape_a,
            &pose_a,
            &shape_b,
            &pose_b,
            GJK_MAX_ITERATIONS
        ));
    }

    // =========================================================================
    // GJK Distance tests (Spec D S1)
    // =========================================================================

    /// T1: AC1 — separated spheres, analytically derived
    #[test]
    fn test_gjk_distance_separated_spheres() {
        let shape_a = CollisionShape::sphere(1.0);
        let shape_b = CollisionShape::sphere(1.0);
        let pose_a = pose_at(0.0, 0.0, 0.0);
        let pose_b = pose_at(4.0, 0.0, 0.0);

        let result = gjk_distance(
            &shape_a,
            &pose_a,
            &shape_b,
            &pose_b,
            GJK_MAX_ITERATIONS,
            EPA_TOLERANCE,
        );
        assert!(result.is_some(), "separated spheres should return Some");

        let r = result.unwrap();
        // Center distance 4.0 minus sum of radii 2.0 = separation 2.0
        assert_relative_eq!(r.distance, 2.0, epsilon = 1e-6);
        // Witness A should be at x ≈ 1.0 (surface of sphere A toward B)
        assert_relative_eq!(r.witness_a.x, 1.0, epsilon = 1e-4);
        // Witness B should be at x ≈ 3.0 (surface of sphere B toward A)
        assert_relative_eq!(r.witness_b.x, 3.0, epsilon = 1e-4);
    }

    /// T2: AC2 — overlapping spheres return None
    #[test]
    fn test_gjk_distance_overlapping_spheres_return_none() {
        let shape_a = CollisionShape::sphere(1.0);
        let shape_b = CollisionShape::sphere(1.0);
        let pose_a = pose_at(0.0, 0.0, 0.0);
        let pose_b = pose_at(1.0, 0.0, 0.0); // Overlapping: center dist < sum of radii

        let result = gjk_distance(
            &shape_a,
            &pose_a,
            &shape_b,
            &pose_b,
            GJK_MAX_ITERATIONS,
            EPA_TOLERANCE,
        );
        assert!(result.is_none(), "overlapping spheres should return None");
    }

    /// T12: Edge case — spheres just touching (distance ≈ 0)
    #[test]
    fn test_gjk_distance_just_touching_spheres() {
        let shape_a = CollisionShape::sphere(1.0);
        let shape_b = CollisionShape::sphere(1.0);
        let pose_a = pose_at(0.0, 0.0, 0.0);
        let pose_b = pose_at(2.0, 0.0, 0.0); // Exactly touching

        let result = gjk_distance(
            &shape_a,
            &pose_a,
            &shape_b,
            &pose_b,
            GJK_MAX_ITERATIONS,
            EPA_TOLERANCE,
        );
        // At the boundary: may return Some with distance ≈ 0 or None
        if let Some(r) = result {
            assert!(
                r.distance < 1e-3,
                "distance should be near zero, got {}",
                r.distance
            );
        }
        // Both outcomes (None or Some(~0)) are acceptable at the boundary
    }

    /// T16: AC1 variant — separated boxes, exercises triangle simplex case
    #[test]
    fn test_gjk_distance_separated_boxes() {
        let shape_a = CollisionShape::box_shape(Vector3::new(0.5, 0.5, 0.5));
        let shape_b = CollisionShape::box_shape(Vector3::new(0.5, 0.5, 0.5));
        let pose_a = pose_at(0.0, 0.0, 0.0);
        let pose_b = pose_at(3.0, 0.0, 0.0); // Separation = 3.0 - 0.5 - 0.5 = 2.0

        let result = gjk_distance(
            &shape_a,
            &pose_a,
            &shape_b,
            &pose_b,
            GJK_MAX_ITERATIONS,
            EPA_TOLERANCE,
        );
        assert!(result.is_some(), "separated boxes should return Some");

        let r = result.unwrap();
        assert_relative_eq!(r.distance, 2.0, epsilon = 1e-4);
    }

    /// T14: Edge case — ccd_iterations=0 → graceful handling
    #[test]
    fn test_gjk_epa_contact_zero_iterations() {
        let shape_a = CollisionShape::sphere(1.0);
        let shape_b = CollisionShape::sphere(1.0);
        let pose_a = pose_at(0.0, 0.0, 0.0);
        let pose_b = pose_at(1.5, 0.0, 0.0);

        // With 0 iterations, should not crash
        let contact = gjk_epa_contact(&shape_a, &pose_a, &shape_b, &pose_b, 0, EPA_TOLERANCE);
        // May return None (no iterations to converge) — just verify no panic
        let _ = contact;
    }

    /// T18: Edge case — ccd_tolerance=0 → solver runs to max iterations
    #[test]
    fn test_gjk_epa_contact_zero_tolerance() {
        let shape_a = CollisionShape::sphere(1.0);
        let shape_b = CollisionShape::sphere(1.0);
        let pose_a = pose_at(0.0, 0.0, 0.0);
        let pose_b = pose_at(1.5, 0.0, 0.0);

        // With tolerance=0.0, convergence check (< 0) is never true, so runs to max iterations
        let contact = gjk_epa_contact(
            &shape_a,
            &pose_a,
            &shape_b,
            &pose_b,
            GJK_MAX_ITERATIONS,
            0.0,
        );
        // Should still produce a valid contact (just exhausts iterations)
        assert!(
            contact.is_some(),
            "should produce contact even with zero tolerance"
        );
    }
}
