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
//! if gjk_intersection(&shape_a, &pose_a, &shape_b, &pose_b) {
//!     // Get contact information
//!     if let Some(contact) = gjk_epa_contact(&shape_a, &pose_a, &shape_b, &pose_b) {
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

use crate::CollisionShape;

/// Tolerance for numerical comparisons in GJK/EPA.
const EPSILON: f64 = 1e-8;

/// Maximum iterations for GJK before giving up.
const GJK_MAX_ITERATIONS: usize = 64;

/// Maximum iterations for EPA before giving up.
const EPA_MAX_ITERATIONS: usize = 64;

/// Maximum faces in EPA polytope.
const EPA_MAX_FACES: usize = 128;

/// EPA convergence tolerance.
const EPA_TOLERANCE: f64 = 1e-6;

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
        CollisionShape::ConvexMesh { vertices } => support_convex_mesh(pose, vertices, direction),
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
            // Dedicated SDF collision is handled separately in world.rs.
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
            // Dedicated triangle mesh collision is handled separately in world.rs.
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
fn support_convex_mesh(
    pose: &Pose,
    vertices: &[Point3<f64>],
    direction: &Vector3<f64>,
) -> Point3<f64> {
    // Transform direction to local space for efficiency
    let local_dir = pose.rotation.inverse() * direction;

    // Find the vertex with maximum dot product
    let mut max_dot = f64::NEG_INFINITY;
    let mut best_vertex = Point3::origin();

    for vertex in vertices {
        let dot = vertex.coords.dot(&local_dir);
        if dot > max_dot {
            max_dot = dot;
            best_vertex = *vertex;
        }
    }

    // Transform to world space
    pose.transform_point(&best_vertex)
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
) -> bool {
    gjk_query(shape_a, pose_a, shape_b, pose_b).intersecting
}

/// Run the full GJK algorithm and return detailed results.
#[must_use]
pub fn gjk_query(
    shape_a: &CollisionShape,
    pose_a: &Pose,
    shape_b: &CollisionShape,
    pose_b: &Pose,
) -> GjkResult {
    // Initial direction: from center of A to center of B
    let center_a = pose_a.position;
    let center_b = pose_b.position;
    let mut direction = (center_b - center_a).normalize();

    // Handle degenerate case where centers coincide
    if direction.norm() < EPSILON {
        direction = Vector3::x();
    }

    let mut simplex = Simplex::new();

    // Get first support point
    let first = support_minkowski(shape_a, pose_a, shape_b, pose_b, &direction);
    simplex.push(first);

    // New search direction: toward origin from first point
    direction = -first.point.coords;

    for iteration in 0..GJK_MAX_ITERATIONS {
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
        iterations: GJK_MAX_ITERATIONS,
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
) -> Option<EpaResult> {
    // We need at least a tetrahedron to start EPA
    if simplex.len() < 4 {
        // Try to build a tetrahedron from the simplex
        return epa_with_expanded_simplex(shape_a, pose_a, shape_b, pose_b, simplex);
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

    for iteration in 0..EPA_MAX_ITERATIONS {
        // Find the closest face to the origin
        let closest_idx = find_closest_face(&faces)?;
        let closest = &faces[closest_idx];

        // Get a new support point in the direction of the closest face's normal
        let new_point = support_minkowski(shape_a, pose_a, shape_b, pose_b, &closest.normal);

        // Check convergence: if new point is not significantly further than the face
        let new_distance = new_point.point.coords.dot(&closest.normal);
        if new_distance - closest.distance < EPA_TOLERANCE {
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
        iterations: EPA_MAX_ITERATIONS,
    })
}

/// Expand a simplex with fewer than 4 points into a tetrahedron for EPA.
fn epa_with_expanded_simplex(
    shape_a: &CollisionShape,
    pose_a: &Pose,
    shape_b: &CollisionShape,
    pose_b: &Pose,
    simplex: &Simplex,
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

    epa_query(shape_a, pose_a, shape_b, pose_b, &new_simplex)
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
    faces
        .iter()
        .enumerate()
        .min_by(|(_, a), (_, b)| {
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
) -> Option<GjkContact> {
    // Run GJK
    let gjk_result = gjk_query(shape_a, pose_a, shape_b, pose_b);

    if !gjk_result.intersecting {
        return None;
    }

    // Run EPA to get penetration info
    let epa_result = epa_query(shape_a, pose_a, shape_b, pose_b, &gjk_result.simplex)?;

    // Compute contact point
    // The contact point is approximately on the surface of shape A,
    // found by moving from the center of A in the normal direction by the support distance
    let contact_point = support(shape_a, pose_a, &(-epa_result.normal));

    Some(GjkContact {
        point: contact_point,
        normal: epa_result.normal,
        penetration: epa_result.depth.max(0.0),
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

        assert!(gjk_intersection(&shape_a, &pose_a, &shape_b, &pose_b));
    }

    #[test]
    fn test_gjk_spheres_separated() {
        let shape_a = CollisionShape::sphere(1.0);
        let shape_b = CollisionShape::sphere(1.0);

        let pose_a = pose_at(0.0, 0.0, 0.0);
        let pose_b = pose_at(3.0, 0.0, 0.0); // Centers 3 apart, radii sum to 2

        assert!(!gjk_intersection(&shape_a, &pose_a, &shape_b, &pose_b));
    }

    #[test]
    fn test_gjk_sphere_box_intersecting() {
        let shape_a = CollisionShape::sphere(1.0);
        let shape_b = CollisionShape::box_shape(Vector3::new(0.5, 0.5, 0.5));

        let pose_a = pose_at(0.0, 0.0, 0.0);
        let pose_b = pose_at(1.0, 0.0, 0.0); // Should overlap

        assert!(gjk_intersection(&shape_a, &pose_a, &shape_b, &pose_b));
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

        assert!(gjk_intersection(&shape_a, &pose_a, &shape_b, &pose_b));
    }

    #[test]
    fn test_gjk_epa_contact_spheres() {
        let shape_a = CollisionShape::sphere(1.0);
        let shape_b = CollisionShape::sphere(1.0);

        let pose_a = pose_at(0.0, 0.0, 0.0);
        let pose_b = pose_at(1.5, 0.0, 0.0);

        let contact = gjk_epa_contact(&shape_a, &pose_a, &shape_b, &pose_b);
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

        assert!(gjk_intersection(&shape_a, &pose_a, &shape_b, &pose_b));

        // Separated
        let pose_b_far = pose_at(2.0, 0.0, 0.0);
        assert!(!gjk_intersection(&shape_a, &pose_a, &shape_b, &pose_b_far));
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

        assert!(gjk_intersection(&shape_a, &pose_a, &shape_b, &pose_b));
    }

    #[test]
    fn test_gjk_cylinder_sphere_separated() {
        let shape_a = CollisionShape::cylinder(1.0, 0.5);
        let shape_b = CollisionShape::sphere(0.5);

        let pose_a = pose_at(0.0, 0.0, 0.0);
        let pose_b = pose_at(2.0, 0.0, 0.0); // Far away

        assert!(!gjk_intersection(&shape_a, &pose_a, &shape_b, &pose_b));
    }

    #[test]
    fn test_gjk_cylinder_plane_contact() {
        let shape_a = CollisionShape::cylinder(1.0, 0.5);
        let shape_b = CollisionShape::plane(Vector3::z(), 0.0);

        // Cylinder centered at z=0.5, so bottom cap touches ground at z=-0.5
        let pose_a = pose_at(0.0, 0.0, 0.5);
        let pose_b = Pose::identity();

        // Should be penetrating by 0.5
        let contact = gjk_epa_contact(&shape_a, &pose_a, &shape_b, &pose_b);
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

        assert!(gjk_intersection(&shape_a, &pose_a, &shape_b, &pose_b));
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

        assert!(gjk_intersection(&shape_a, &pose_a, &shape_b, &pose_b));
    }

    #[test]
    fn test_gjk_ellipsoid_sphere_separated() {
        let shape_a = CollisionShape::ellipsoid_xyz(2.0, 1.0, 0.5);
        let shape_b = CollisionShape::sphere(0.5);

        let pose_a = pose_at(0.0, 0.0, 0.0);
        let pose_b = pose_at(3.0, 0.0, 0.0); // Far away

        assert!(!gjk_intersection(&shape_a, &pose_a, &shape_b, &pose_b));
    }

    #[test]
    fn test_gjk_ellipsoid_plane_contact() {
        let shape_a = CollisionShape::ellipsoid_xyz(2.0, 1.0, 0.5);
        let shape_b = CollisionShape::plane(Vector3::z(), 0.0);

        // Ellipsoid centered at z=0.3, so bottom (z-radius = 0.5) penetrates
        let pose_a = pose_at(0.0, 0.0, 0.3);
        let pose_b = Pose::identity();

        let contact = gjk_epa_contact(&shape_a, &pose_a, &shape_b, &pose_b);
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

        assert!(gjk_intersection(&shape_a, &pose_a, &shape_b, &pose_b));
    }
}
