//! GJK/EPA bridge — delegates geometric queries to cf-geometry.
//!
//! This module bridges sim-core's physics collision pipeline and cf-geometry's
//! pure geometric algorithms. It provides:
//!
//! - **`PosedShape`**: Wraps `(Shape, Pose)` to implement `cf_geometry::SupportMap`
//!   in world space, with warm-start caching for convex mesh hill-climbing.
//! - **`support()`**: World-space support function with warm-start, used by
//!   heightfield collision and MULTICCD.
//! - **`support_face_points()`**: Flat-face vertex enumeration for MULTICCD.
//! - **`gjk_epa_contact()`**: GJK+EPA penetration → `GjkContact`.
//! - **`gjk_distance()`**: GJK distance → `GjkDistanceResult`.
//!
//! All GJK simplex processing, EPA expansion, and distance query algorithms
//! live in `cf-geometry`. This module contains only the physics-aware wrapping
//! layer (pose transforms, warm-start, non-convex fallbacks).
//!
//! # References
//!
//! - Gilbert, Johnson, Keerthi: "A Fast Procedure for Computing the Distance
//!   Between Complex Objects in Three-Dimensional Space" (1988)
//! - van den Bergen: "Collision Detection in Interactive 3D Environments" (2003)

use nalgebra::{Point3, Vector3};
use sim_types::Pose;
use std::cell::Cell;

use cf_geometry::{Aabb, Bounded, Shape, SupportMap};

/// Minimum vertex count for hill-climbing support.
/// Matches MuJoCo's `mjMESH_HILLCLIMB_MIN = 10`.
const HILL_CLIMB_MIN: usize = 10;

/// Tolerance for numerical comparisons in support functions.
const EPSILON: f64 = 1e-8;

/// Default GJK max iterations (used when no Model context available).
/// MuJoCo default: 35. Historical CortenForge value: 64.
pub const GJK_MAX_ITERATIONS: usize = 35;

/// Default EPA max iterations (used when no Model context available).
/// MuJoCo default: 35. Historical CortenForge value: 64.
pub const EPA_MAX_ITERATIONS: usize = 35;

/// Default EPA convergence tolerance.
pub const EPA_TOLERANCE: f64 = 1e-6;

// =============================================================================
// Contact result type
// =============================================================================

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

// =============================================================================
// PosedShape — world-space SupportMap adapter
// =============================================================================

/// World-space support map adapter.
///
/// Wraps a `(Shape, Pose)` pair to implement `cf_geometry::SupportMap` in world
/// coordinates. Each `PosedShape` carries its own warm-start cache for GJK
/// hill-climbing on convex meshes.
///
/// This is the bridge between sim-core's `(Shape, Pose)` collision pipeline
/// and cf-geometry's local-space `SupportMap` trait.
pub struct PosedShape<'a> {
    shape: &'a Shape,
    pose: &'a Pose,
    warm_start: Cell<usize>,
}

impl<'a> PosedShape<'a> {
    /// Create a new world-space support map for a shape at a given pose.
    #[must_use]
    pub fn new(shape: &'a Shape, pose: &'a Pose) -> Self {
        Self {
            shape,
            pose,
            warm_start: Cell::new(0),
        }
    }
}

impl Bounded for PosedShape<'_> {
    fn aabb(&self) -> Aabb {
        // Compute tight world-space AABB via support queries along ±axes.
        let dirs = [
            Vector3::x(),
            -Vector3::x(),
            Vector3::y(),
            -Vector3::y(),
            Vector3::z(),
            -Vector3::z(),
        ];

        let mut min = Vector3::new(f64::INFINITY, f64::INFINITY, f64::INFINITY);
        let mut max = Vector3::new(f64::NEG_INFINITY, f64::NEG_INFINITY, f64::NEG_INFINITY);

        for dir in &dirs {
            let pt = support(self.shape, self.pose, dir, &self.warm_start);
            min.x = min.x.min(pt.x);
            min.y = min.y.min(pt.y);
            min.z = min.z.min(pt.z);
            max.x = max.x.max(pt.x);
            max.y = max.y.max(pt.y);
            max.z = max.z.max(pt.z);
        }

        Aabb::new(Point3::from(min), Point3::from(max))
    }
}

impl SupportMap for PosedShape<'_> {
    fn support(&self, direction: &Vector3<f64>) -> Point3<f64> {
        support(self.shape, self.pose, direction, &self.warm_start)
    }
}

// =============================================================================
// World-space support functions (physics-aware)
// =============================================================================

/// Compute the support point of a shape at a given pose in a given direction.
///
/// The support point is the point on the shape's surface that is furthest
/// in the given direction. This is the fundamental operation for GJK/EPA.
///
/// # Arguments
///
/// * `shape` - The collision shape
/// * `pose` - The shape's world-space pose (position + orientation)
/// * `direction` - The direction to search in (world-space, need not be unit)
/// * `warm_start` - Cache for convex mesh hill-climbing (pass `Cell::new(0)` if unused)
///
/// # Returns
///
/// The support point in world space.
#[must_use]
pub fn support(
    shape: &Shape,
    pose: &Pose,
    direction: &Vector3<f64>,
    warm_start: &Cell<usize>,
) -> Point3<f64> {
    match shape {
        Shape::Sphere { radius } => support_sphere(pose, *radius, direction),
        Shape::Box { half_extents } => support_box(pose, half_extents, direction),
        Shape::Capsule {
            half_length,
            radius,
        } => support_capsule(pose, *half_length, *radius, direction),
        Shape::ConvexMesh { hull } => {
            let adjacency = if hull.vertices.len() >= HILL_CLIMB_MIN {
                Some(hull.adjacency.as_slice())
            } else {
                None
            };
            support_convex_mesh(pose, &hull.vertices, adjacency, warm_start, direction)
        }
        Shape::Plane { normal, distance } => support_plane(pose, normal, *distance, direction),
        Shape::Cylinder {
            half_length,
            radius,
        } => support_cylinder(pose, *half_length, *radius, direction),
        Shape::Ellipsoid { radii } => support_ellipsoid(pose, radii, direction),
        Shape::HeightField { data } => support_aabb_fallback(pose, &data.aabb(), direction),
        Shape::Sdf { data } => support_aabb_fallback(pose, &data.aabb(), direction),
        Shape::TriangleMesh { mesh, .. } => support_aabb_fallback(pose, &mesh.aabb(), direction),
    }
}

/// AABB-extremes fallback support for non-convex shapes.
fn support_aabb_fallback(
    pose: &Pose,
    local_aabb: &cf_geometry::Aabb,
    direction: &Vector3<f64>,
) -> Point3<f64> {
    let local_dir = pose.rotation.inverse() * direction;
    let local_support = Point3::new(
        if local_dir.x >= 0.0 {
            local_aabb.max.x
        } else {
            local_aabb.min.x
        },
        if local_dir.y >= 0.0 {
            local_aabb.max.y
        } else {
            local_aabb.min.y
        },
        if local_dir.z >= 0.0 {
            local_aabb.max.z
        } else {
            local_aabb.min.z
        },
    );
    pose.transform_point(&local_support)
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
    shape: &Shape,
    pose: &Pose,
    direction: &Vector3<f64>,
) -> Vec<Point3<f64>> {
    match shape {
        Shape::Box { half_extents } => {
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
        Shape::ConvexMesh { hull } => {
            if hull.vertices.is_empty() {
                return vec![pose.position];
            }
            let local_dir = pose.rotation.inverse() * direction;
            // Find max dot product
            let max_dot = hull
                .vertices
                .iter()
                .map(|v| v.coords.dot(&local_dir))
                .fold(f64::NEG_INFINITY, f64::max);
            // Return all vertices within EPSILON of max_dot
            hull.vertices
                .iter()
                .filter(|v| (v.coords.dot(&local_dir) - max_dot).abs() < EPSILON * 10.0)
                .map(|v| pose.transform_point(v))
                .collect()
        }
        // Smooth shapes: single support point
        _ => {
            let warm_start = Cell::new(0);
            vec![support(shape, pose, direction, &warm_start)]
        }
    }
}

// =============================================================================
// Per-shape support functions (local → world)
// =============================================================================

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
/// Selects between hill-climbing (O(√n), when adjacency is available) and
/// exhaustive scan (O(n)). Both strategies warm-start from the cached
/// vertex index.
///
/// MuJoCo ref: `mjc_hillclimbSupport()` and `mjc_meshSupport()` in
/// `engine_collision_convex.c`.
fn support_convex_mesh(
    pose: &Pose,
    vertices: &[Point3<f64>],
    adjacency: Option<&[Vec<u32>]>,
    warm_start: &Cell<usize>,
    direction: &Vector3<f64>,
) -> Point3<f64> {
    if vertices.is_empty() {
        return pose.position;
    }

    let local_dir = pose.rotation.inverse() * direction;

    let best_idx = if let Some(adj) = adjacency {
        // Hill-climbing (steepest-ascent): warm-start from cached vertex
        let start = warm_start.get().min(vertices.len() - 1);
        let idx = hill_climb_support(vertices, adj, &local_dir, start);
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

/// Steepest-ascent hill-climbing support on convex hull adjacency graph.
///
/// Matches MuJoCo's `mjc_hillclimbSupport()`: at each step, examine ALL
/// neighbors of the current vertex, move to the neighbor with the highest
/// dot product, repeat until no neighbor improves.
///
/// Guaranteed to find the global maximum because the dot product
/// is a linear (unimodal) function on a convex surface.
pub(crate) fn hill_climb_support(
    vertices: &[Point3<f64>],
    adjacency: &[Vec<u32>],
    direction: &Vector3<f64>,
    start: usize,
) -> usize {
    let mut current = start;
    let mut max_dot = f64::NEG_INFINITY;

    loop {
        let prev = current;
        // Scan ALL neighbors of current vertex (steepest-ascent)
        for &neighbor in &adjacency[current] {
            let ni = neighbor as usize;
            let dot = vertices[ni].coords.dot(direction);
            if dot > max_dot {
                max_dot = dot;
                current = ni;
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
// GJK/EPA bridge functions — delegate to cf-geometry
// =============================================================================

/// Compute contact information between two shapes using GJK+EPA.
///
/// Returns `None` if the shapes don't intersect or if contact computation fails.
/// Delegates to `cf_geometry::epa_penetration` via `PosedShape` adapters.
#[must_use]
pub fn gjk_epa_contact(
    shape_a: &Shape,
    pose_a: &Pose,
    shape_b: &Shape,
    pose_b: &Pose,
    max_iterations: usize,
    tolerance: f64,
) -> Option<GjkContact> {
    let a = PosedShape::new(shape_a, pose_a);
    let b = PosedShape::new(shape_b, pose_b);

    let pen = cf_geometry::epa_penetration(&a, &b, max_iterations, tolerance)?;

    Some(GjkContact {
        point: pen.point_a,
        normal: pen.normal,
        penetration: pen.depth.max(0.0),
    })
}

/// Check if two shapes intersect using GJK.
///
/// This is a fast intersection test that returns true if the shapes overlap.
/// For penetration information, use [`gjk_epa_contact`].
#[must_use]
pub fn gjk_intersection(
    shape_a: &Shape,
    pose_a: &Pose,
    shape_b: &Shape,
    pose_b: &Pose,
    max_iterations: usize,
) -> bool {
    let _ = max_iterations; // cf-geometry uses its own default
    let a = PosedShape::new(shape_a, pose_a);
    let b = PosedShape::new(shape_b, pose_b);
    cf_geometry::gjk_intersection(&a, &b)
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
}

/// Compute minimum separating distance between two non-overlapping convex shapes.
///
/// Returns `None` if shapes are overlapping (use `gjk_epa_contact` instead).
/// Delegates to `cf_geometry::gjk_distance` via `PosedShape` adapters.
///
/// `max_iterations` and `tolerance` come from `model.ccd_iterations` and
/// `model.ccd_tolerance` respectively.
#[must_use]
pub fn gjk_distance(
    shape_a: &Shape,
    pose_a: &Pose,
    shape_b: &Shape,
    pose_b: &Pose,
    max_iterations: usize,
    tolerance: f64,
) -> Option<GjkDistanceResult> {
    let a = PosedShape::new(shape_a, pose_a);
    let b = PosedShape::new(shape_b, pose_b);

    let d = cf_geometry::gjk_distance(&a, &b, max_iterations, tolerance)?;

    Some(GjkDistanceResult {
        distance: d.distance,
        witness_a: d.point_a,
        witness_b: d.point_b,
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

    /// Test helper: create a regular tetrahedron Shape from half-edge length.
    fn test_tetrahedron(edge_half_length: f64) -> Shape {
        let h = edge_half_length;
        let vertices = vec![
            Point3::new(h, h, h),
            Point3::new(h, -h, -h),
            Point3::new(-h, h, -h),
            Point3::new(-h, -h, h),
        ];
        let hull = cf_geometry::convex_hull(&vertices, None).unwrap();
        Shape::convex_mesh(hull)
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
        let shape_a = Shape::sphere(1.0);
        let shape_b = Shape::sphere(1.0);

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
        let shape_a = Shape::sphere(1.0);
        let shape_b = Shape::sphere(1.0);

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
        let shape_a = Shape::sphere(1.0);
        let shape_b = Shape::box_shape(Vector3::new(0.5, 0.5, 0.5));

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
        let hull = cf_geometry::convex_hull(&vertices, None).unwrap();
        let shape_a = Shape::convex_mesh(hull);
        let shape_b = Shape::sphere(0.5);

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
        let shape_a = Shape::sphere(1.0);
        let shape_b = Shape::sphere(1.0);

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
        let shape_a = test_tetrahedron(0.5);
        let shape_b = test_tetrahedron(0.5);

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
        let shape_a = Shape::cylinder(1.0, 0.5);
        let shape_b = Shape::sphere(0.5);

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
        let shape_a = Shape::cylinder(1.0, 0.5);
        let shape_b = Shape::sphere(0.5);

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
        let shape_a = Shape::cylinder(1.0, 0.5);
        let shape_b = Shape::plane(Vector3::z(), 0.0);

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
        let shape_a = Shape::cylinder(1.0, 0.5);
        let shape_b = Shape::cylinder(1.0, 0.5);

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
        let shape_a = Shape::ellipsoid_xyz(2.0, 1.0, 0.5);
        let shape_b = Shape::sphere(0.5);

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
        let shape_a = Shape::ellipsoid_xyz(2.0, 1.0, 0.5);
        let shape_b = Shape::sphere(0.5);

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
        let shape_a = Shape::ellipsoid_xyz(2.0, 1.0, 0.5);
        let shape_b = Shape::plane(Vector3::z(), 0.0);

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
        let shape_a = Shape::ellipsoid_xyz(1.0, 0.5, 0.5);
        let shape_b = Shape::ellipsoid_xyz(1.0, 0.5, 0.5);

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
        let shape_a = Shape::sphere(1.0);
        let shape_b = Shape::sphere(1.0);
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
        let shape_a = Shape::sphere(1.0);
        let shape_b = Shape::sphere(1.0);
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
        let shape_a = Shape::sphere(1.0);
        let shape_b = Shape::sphere(1.0);
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
        let shape_a = Shape::box_shape(Vector3::new(0.5, 0.5, 0.5));
        let shape_b = Shape::box_shape(Vector3::new(0.5, 0.5, 0.5));
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
        let shape_a = Shape::sphere(1.0);
        let shape_b = Shape::sphere(1.0);
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
        let shape_a = Shape::sphere(1.0);
        let shape_b = Shape::sphere(1.0);
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

    // =========================================================================
    // PosedShape tests
    // =========================================================================

    #[test]
    fn test_posed_shape_aabb() {
        let shape = Shape::sphere(1.0);
        let pose = pose_at(2.0, 3.0, 4.0);
        let posed = PosedShape::new(&shape, &pose);

        let aabb = posed.aabb();
        assert_relative_eq!(aabb.center().x, 2.0, epsilon = 1e-6);
        assert_relative_eq!(aabb.center().y, 3.0, epsilon = 1e-6);
        assert_relative_eq!(aabb.center().z, 4.0, epsilon = 1e-6);
        assert_relative_eq!(aabb.half_extents().x, 1.0, epsilon = 1e-6);
    }

    #[test]
    fn test_posed_shape_support() {
        let shape = Shape::sphere(1.0);
        let pose = pose_at(5.0, 0.0, 0.0);
        let posed = PosedShape::new(&shape, &pose);

        let pt = SupportMap::support(&posed, &Vector3::x());
        assert_relative_eq!(pt.x, 6.0, epsilon = 1e-6);
    }
}
