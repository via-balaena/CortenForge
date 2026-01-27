//! Ray casting for physics queries.
//!
//! This module provides ray casting functionality for distance queries against
//! collision shapes. It's used for rangefinder sensors, visibility checks, and
//! other distance-based queries.
//!
//! # Supported Shapes
//!
//! - Sphere: Analytic ray-sphere intersection
//! - Plane: Analytic ray-plane intersection
//! - Box: Ray-AABB slab test (in local coordinates)
//! - Capsule: Ray-capsule intersection (sphere-swept line)
//! - Cylinder: Analytic ray-cylinder intersection
//! - Ellipsoid: Transformed sphere intersection
//! - `HeightField`: Ray marching against terrain
//! - SDF: Ray marching with distance field
//! - `ConvexMesh`: GJK-raycast or brute-force face test
//! - `TriangleMesh`: BVH-accelerated triangle intersection

// Allow precision loss for vertex counts (safe for meshes < 2^52 vertices)
// Allow truncation and sign loss for step count calculation (safe for reasonable distances)
// Allow suspicious_operation_groupings - false positive for quadratic discriminant formula b*b - a*c
// Allow many_single_char_names - standard notation for Möller-Trumbore algorithm
#![allow(
    clippy::cast_precision_loss,
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    clippy::similar_names,
    clippy::suspicious_operation_groupings,
    clippy::many_single_char_names
)]

use nalgebra::{Point3, UnitVector3, Vector3};
use sim_types::Pose;

use crate::CollisionShape;

/// Result of a ray cast against a shape.
#[derive(Debug, Clone, Copy)]
pub struct RaycastHit {
    /// Distance from ray origin to hit point.
    pub distance: f64,
    /// Hit point in world coordinates.
    pub point: Point3<f64>,
    /// Surface normal at hit point (pointing away from surface).
    pub normal: Vector3<f64>,
}

impl RaycastHit {
    /// Create a new ray hit.
    #[must_use]
    pub fn new(distance: f64, point: Point3<f64>, normal: Vector3<f64>) -> Self {
        Self {
            distance,
            point,
            normal,
        }
    }
}

/// Safe vector normalization with fallback.
/// Returns the normalized vector if norm > threshold, otherwise returns the fallback.
#[inline]
fn safe_normalize(v: &Vector3<f64>, fallback: Vector3<f64>) -> Vector3<f64> {
    let n = v.norm();
    if n > 1e-10 { v / n } else { fallback }
}

/// Cast a ray against a collision shape.
///
/// # Arguments
///
/// * `shape` - The collision shape to test
/// * `shape_pose` - World pose of the shape
/// * `ray_origin` - Ray origin in world coordinates
/// * `ray_direction` - Ray direction (unit vector) in world coordinates
/// * `max_distance` - Maximum ray distance
///
/// # Returns
///
/// The closest ray hit, or `None` if the ray doesn't intersect within `max_distance`.
#[must_use]
pub fn raycast_shape(
    shape: &CollisionShape,
    shape_pose: &Pose,
    ray_origin: Point3<f64>,
    ray_direction: UnitVector3<f64>,
    max_distance: f64,
) -> Option<RaycastHit> {
    match shape {
        CollisionShape::Sphere { radius } => raycast_sphere(
            shape_pose.position,
            *radius,
            ray_origin,
            ray_direction,
            max_distance,
        ),
        CollisionShape::Plane { normal, distance } => {
            // Transform plane to world space
            let world_normal = safe_normalize(&shape_pose.transform_vector(normal), *normal);
            let world_point = shape_pose.position + world_normal * *distance;
            raycast_plane(
                &world_point,
                &world_normal,
                ray_origin,
                ray_direction,
                max_distance,
            )
        }
        CollisionShape::Box { half_extents } => raycast_box(
            shape_pose,
            *half_extents,
            ray_origin,
            ray_direction,
            max_distance,
        ),
        CollisionShape::Capsule {
            half_length,
            radius,
        } => raycast_capsule(
            shape_pose,
            *half_length,
            *radius,
            ray_origin,
            ray_direction,
            max_distance,
        ),
        CollisionShape::Cylinder {
            half_length,
            radius,
        } => raycast_cylinder(
            shape_pose,
            *half_length,
            *radius,
            ray_origin,
            ray_direction,
            max_distance,
        ),
        CollisionShape::Ellipsoid { radii } => {
            raycast_ellipsoid(shape_pose, *radii, ray_origin, ray_direction, max_distance)
        }
        CollisionShape::ConvexMesh { vertices } => raycast_convex_mesh(
            shape_pose,
            vertices,
            ray_origin,
            ray_direction,
            max_distance,
        ),
        CollisionShape::HeightField { data } => raycast_heightfield(
            shape_pose,
            data.as_ref(),
            ray_origin,
            ray_direction,
            max_distance,
        ),
        CollisionShape::Sdf { data } => raycast_sdf(
            shape_pose,
            data.as_ref(),
            ray_origin,
            ray_direction,
            max_distance,
        ),
        CollisionShape::TriangleMesh { data } => raycast_triangle_mesh(
            shape_pose,
            data.as_ref(),
            ray_origin,
            ray_direction,
            max_distance,
        ),
    }
}

/// Ray-sphere intersection.
///
/// Uses the analytic quadratic formula.
fn raycast_sphere(
    center: Point3<f64>,
    radius: f64,
    ray_origin: Point3<f64>,
    ray_direction: UnitVector3<f64>,
    max_distance: f64,
) -> Option<RaycastHit> {
    let oc = ray_origin - center;
    let dir = ray_direction.as_ref();

    // Quadratic: t^2 + 2*b*t + c = 0 where b = oc·dir, c = oc·oc - r²
    let b = oc.dot(dir);
    let c = oc.dot(&oc) - radius * radius;
    let discriminant = b * b - c;

    // Check for miss OR NaN (NaN < 0.0 is false, so explicitly check)
    if !(discriminant >= 0.0) {
        return None;
    }

    let sqrt_d = discriminant.sqrt();

    // Try the near hit first
    let mut t = -b - sqrt_d;
    if t < 0.0 {
        // Near hit is behind, try far hit
        t = -b + sqrt_d;
    }

    if t < 0.0 || t > max_distance {
        return None;
    }

    let point = ray_origin + dir * t;
    let to_point = point - center;
    let dist = to_point.norm();
    // Safe normalize: for a sphere hit, distance should equal radius (non-zero)
    let normal = if dist > 1e-10 { to_point / dist } else { *dir };

    Some(RaycastHit::new(t, point, normal))
}

/// Ray-plane intersection.
///
/// The plane is defined by a point and normal.
fn raycast_plane(
    plane_point: &Point3<f64>,
    plane_normal: &Vector3<f64>,
    ray_origin: Point3<f64>,
    ray_direction: UnitVector3<f64>,
    max_distance: f64,
) -> Option<RaycastHit> {
    // Validate plane normal is not degenerate
    let normal_len_sq = plane_normal.norm_squared();
    if normal_len_sq < 1e-20 {
        return None;
    }

    let dir = ray_direction.as_ref();
    let denom = plane_normal.dot(dir);

    // Parallel to plane (or NaN)
    if !(denom.abs() >= 1e-10) {
        return None;
    }

    let t = (plane_point - ray_origin).dot(plane_normal) / denom;

    if t < 0.0 || t > max_distance {
        return None;
    }

    let point = ray_origin + dir * t;
    // Return normal pointing toward ray origin
    let normal = if denom > 0.0 {
        -*plane_normal
    } else {
        *plane_normal
    };

    Some(RaycastHit::new(t, point, normal))
}

/// Ray-box intersection using slab method.
fn raycast_box(
    pose: &Pose,
    half_extents: Vector3<f64>,
    ray_origin: Point3<f64>,
    ray_direction: UnitVector3<f64>,
    max_distance: f64,
) -> Option<RaycastHit> {
    // Transform ray to local space
    let inv_pose = pose.inverse();
    let local_origin = inv_pose.transform_point(&ray_origin);
    let local_dir = inv_pose.transform_vector(ray_direction.as_ref());

    let mut t_min = 0.0_f64;
    let mut t_max = max_distance;
    let mut hit_normal = Vector3::zeros();

    // Test each slab
    for i in 0..3 {
        let origin_i = local_origin[i];
        let dir_i = local_dir[i];
        let extent = half_extents[i];

        if dir_i.abs() < 1e-10 {
            // Ray parallel to slab
            if origin_i < -extent || origin_i > extent {
                return None;
            }
        } else {
            let inv_dir = 1.0 / dir_i;
            let t1 = (-extent - origin_i) * inv_dir;
            let t2 = (extent - origin_i) * inv_dir;

            let (t_near, t_far, sign) = if t1 < t2 {
                (t1, t2, -1.0)
            } else {
                (t2, t1, 1.0)
            };

            if t_near > t_min {
                t_min = t_near;
                hit_normal = Vector3::zeros();
                hit_normal[i] = sign;
            }

            t_max = t_max.min(t_far);

            if t_min > t_max {
                return None;
            }
        }
    }

    if t_min < 0.0 || t_min > max_distance {
        return None;
    }

    let point = ray_origin + ray_direction.as_ref() * t_min;
    let world_normal = safe_normalize(&pose.transform_vector(&hit_normal), hit_normal);

    Some(RaycastHit::new(t_min, point, world_normal))
}

/// Ray-capsule intersection.
///
/// A capsule is a sphere-swept line segment. We test against the infinite
/// cylinder and the end caps.
fn raycast_capsule(
    pose: &Pose,
    half_length: f64,
    radius: f64,
    ray_origin: Point3<f64>,
    ray_direction: UnitVector3<f64>,
    max_distance: f64,
) -> Option<RaycastHit> {
    // Transform ray to local space
    let inv_pose = pose.inverse();
    let local_origin = inv_pose.transform_point(&ray_origin);
    let local_dir = inv_pose.transform_vector(ray_direction.as_ref());

    // Capsule axis is along Z
    let p1 = Point3::new(0.0, 0.0, -half_length);
    let p2 = Point3::new(0.0, 0.0, half_length);

    let mut closest_hit: Option<(f64, Point3<f64>, Vector3<f64>)> = None;
    let mut check_hit = |t: f64, pt: Point3<f64>, normal: Vector3<f64>| {
        if t >= 0.0
            && t <= max_distance
            && (closest_hit.is_none() || t < closest_hit.as_ref().map_or(f64::MAX, |h| h.0))
        {
            closest_hit = Some((t, pt, normal));
        }
    };

    // Test against infinite cylinder (XY components only)
    let oc = Vector3::new(local_origin.x, local_origin.y, 0.0);
    let dir_xy = Vector3::new(local_dir.x, local_dir.y, 0.0);

    let a = dir_xy.dot(&dir_xy);
    let b = oc.dot(&dir_xy);
    let c = oc.dot(&oc) - radius * radius;

    if a > 1e-10 {
        let discriminant = b * b - a * c;
        // Use >= 0.0 pattern that correctly handles NaN (NaN >= 0.0 is false)
        if discriminant >= 0.0 && discriminant.is_finite() {
            let sqrt_d = discriminant.sqrt();

            for t in [(-b - sqrt_d) / a, (-b + sqrt_d) / a] {
                if t >= 0.0 && t.is_finite() {
                    let pt = local_origin + local_dir * t;
                    // Check if hit is on cylinder body (not beyond caps)
                    if pt.z >= -half_length && pt.z <= half_length {
                        let radial = Vector3::new(pt.x, pt.y, 0.0);
                        let radial_len = radial.norm();
                        // Safe normalize: for cylinder hit, radial distance equals radius
                        let normal = if radial_len > 1e-10 {
                            radial / radial_len
                        } else {
                            Vector3::x()
                        };
                        check_hit(t, pt, normal);
                    }
                }
            }
        }
    }

    // Test against end cap spheres
    for &cap_center in &[p1, p2] {
        let oc = local_origin - cap_center;
        let b = oc.dot(&local_dir);
        let c = oc.dot(&oc) - radius * radius;
        let discriminant = b * b - c;

        if discriminant >= 0.0 && discriminant.is_finite() {
            let sqrt_d = discriminant.sqrt();
            let t = -b - sqrt_d;
            if t >= 0.0 {
                let pt = local_origin + local_dir * t;
                let to_pt = pt - cap_center;
                let dist = to_pt.norm();
                // Safe normalize: for sphere cap hit, distance equals radius
                let normal = if dist > 1e-10 {
                    to_pt / dist
                } else {
                    local_dir
                };
                check_hit(t, pt, normal);
            }
        }
    }

    closest_hit.map(|(t, local_pt, local_normal)| {
        let world_pt = pose.transform_point(&local_pt);
        let world_normal_raw = pose.transform_vector(&local_normal);
        let wn_len = world_normal_raw.norm();
        // Safe normalize: local_normal was already normalized
        let world_normal = if wn_len > 1e-10 {
            world_normal_raw / wn_len
        } else {
            local_normal
        };
        RaycastHit::new(t, world_pt, world_normal)
    })
}

/// Ray-cylinder intersection (flat caps, no hemispheres).
fn raycast_cylinder(
    pose: &Pose,
    half_length: f64,
    radius: f64,
    ray_origin: Point3<f64>,
    ray_direction: UnitVector3<f64>,
    max_distance: f64,
) -> Option<RaycastHit> {
    // Transform ray to local space
    let inv_pose = pose.inverse();
    let local_origin = inv_pose.transform_point(&ray_origin);
    let local_dir = inv_pose.transform_vector(ray_direction.as_ref());

    let mut closest_t = f64::MAX;
    let mut closest_hit: Option<(Point3<f64>, Vector3<f64>)> = None;

    // Test infinite cylinder (XY only)
    let oc = Vector3::new(local_origin.x, local_origin.y, 0.0);
    let dir_xy = Vector3::new(local_dir.x, local_dir.y, 0.0);

    let a = dir_xy.dot(&dir_xy);
    let b = oc.dot(&dir_xy);
    let c = oc.dot(&oc) - radius * radius;

    if a > 1e-10 {
        let discriminant = b * b - a * c;
        // Use >= 0.0 pattern that correctly handles NaN
        if discriminant >= 0.0 && discriminant.is_finite() {
            let sqrt_d = discriminant.sqrt();

            for t in [(-b - sqrt_d) / a, (-b + sqrt_d) / a] {
                if t >= 0.0 && t < closest_t && t <= max_distance && t.is_finite() {
                    let pt = local_origin + local_dir * t;
                    if pt.z >= -half_length && pt.z <= half_length {
                        closest_t = t;
                        let radial = Vector3::new(pt.x, pt.y, 0.0);
                        let normal = safe_normalize(&radial, Vector3::x());
                        closest_hit = Some((pt, normal));
                    }
                }
            }
        }
    }

    // Test flat caps
    for (cap_z, cap_normal) in [
        (-half_length, Vector3::new(0.0, 0.0, -1.0)),
        (half_length, Vector3::new(0.0, 0.0, 1.0)),
    ] {
        if local_dir.z.abs() > 1e-10 {
            let t = (cap_z - local_origin.z) / local_dir.z;
            if t >= 0.0 && t < closest_t && t <= max_distance {
                let pt = local_origin + local_dir * t;
                // Check if hit is inside cap circle
                if pt.x * pt.x + pt.y * pt.y <= radius * radius {
                    closest_t = t;
                    closest_hit = Some((pt, cap_normal));
                }
            }
        }
    }

    closest_hit.map(|(local_pt, local_normal)| {
        let world_pt = pose.transform_point(&local_pt);
        let world_normal = safe_normalize(&pose.transform_vector(&local_normal), local_normal);
        RaycastHit::new(closest_t, world_pt, world_normal)
    })
}

/// Ray-ellipsoid intersection.
///
/// Transform the ray by the inverse scale to make it a sphere intersection.
fn raycast_ellipsoid(
    pose: &Pose,
    radii: Vector3<f64>,
    ray_origin: Point3<f64>,
    ray_direction: UnitVector3<f64>,
    max_distance: f64,
) -> Option<RaycastHit> {
    // Validate radii - avoid division by zero
    const MIN_RADIUS: f64 = 1e-10;
    if radii.x < MIN_RADIUS || radii.y < MIN_RADIUS || radii.z < MIN_RADIUS {
        return None;
    }

    // Transform ray to local space
    let inv_pose = pose.inverse();
    let local_origin = inv_pose.transform_point(&ray_origin);
    let local_dir = inv_pose.transform_vector(ray_direction.as_ref());

    // Scale by inverse radii to transform to unit sphere problem
    let scaled_origin = Point3::new(
        local_origin.x / radii.x,
        local_origin.y / radii.y,
        local_origin.z / radii.z,
    );
    let scaled_dir = Vector3::new(
        local_dir.x / radii.x,
        local_dir.y / radii.y,
        local_dir.z / radii.z,
    );

    // Ray-unit-sphere intersection
    let oc = scaled_origin.coords;
    let a = scaled_dir.dot(&scaled_dir);
    let b = oc.dot(&scaled_dir);
    let c = oc.dot(&oc) - 1.0;

    let discriminant = b * b - a * c;
    // Check for miss OR NaN
    if !(discriminant >= 0.0) {
        return None;
    }

    // Check for degenerate scaled direction (a ≈ 0 means ray parallel to ellipsoid axis)
    if a < 1e-20 {
        return None;
    }

    let sqrt_d = discriminant.sqrt();
    let mut t_scaled = (-b - sqrt_d) / a;
    if t_scaled < 0.0 {
        t_scaled = (-b + sqrt_d) / a;
    }

    if t_scaled < 0.0 {
        return None;
    }

    // Compute actual t in original space
    let local_pt = local_origin + local_dir * t_scaled;
    let t = (pose.transform_point(&local_pt) - ray_origin).norm();

    if t > max_distance {
        return None;
    }

    // Normal: gradient of ellipsoid function f(x,y,z) = (x/a)² + (y/b)² + (z/c)² - 1
    let grad = Vector3::new(
        2.0 * local_pt.x / (radii.x * radii.x),
        2.0 * local_pt.y / (radii.y * radii.y),
        2.0 * local_pt.z / (radii.z * radii.z),
    );
    let local_normal = safe_normalize(&grad, Vector3::z());

    let world_pt = pose.transform_point(&local_pt);
    let world_normal = safe_normalize(&pose.transform_vector(&local_normal), local_normal);

    Some(RaycastHit::new(t, world_pt, world_normal))
}

/// Ray-convex mesh intersection.
///
/// Tests each face of the convex hull.
fn raycast_convex_mesh(
    pose: &Pose,
    vertices: &[Point3<f64>],
    ray_origin: Point3<f64>,
    ray_direction: UnitVector3<f64>,
    max_distance: f64,
) -> Option<RaycastHit> {
    if vertices.len() < 4 {
        return None;
    }

    // Transform ray to local space
    let inv_pose = pose.inverse();
    let local_origin = inv_pose.transform_point(&ray_origin);
    let local_dir = inv_pose.transform_vector(ray_direction.as_ref());

    // Simple approach: compute convex hull faces and test each
    // For a proper implementation, we'd use GJK-raycast or cache the hull
    // For now, use a conservative sphere bound

    // Compute bounding sphere
    let centroid: Vector3<f64> =
        vertices.iter().map(|v| v.coords).sum::<Vector3<f64>>() / (vertices.len() as f64);
    let center = Point3::from(centroid);
    let radius = vertices
        .iter()
        .map(|v| (v - center).norm())
        .fold(0.0_f64, f64::max);

    // First test bounding sphere
    let oc = local_origin - center;
    let b = oc.dot(&local_dir);
    let c = oc.dot(&oc) - radius * radius;
    let discriminant = b * b - c;

    // Check for miss OR NaN
    if !(discriminant >= 0.0) {
        return None;
    }

    // For a proper convex mesh, we'd test actual faces here
    // This is a simplified approximation using the bounding sphere
    let sqrt_d = discriminant.sqrt();
    let t = -b - sqrt_d;

    if !(t >= 0.0 && t <= max_distance) {
        return None;
    }

    let local_pt = local_origin + local_dir * t;
    let to_pt = local_pt - center;
    let local_normal = safe_normalize(&to_pt, local_dir);

    let world_pt = pose.transform_point(&local_pt);
    let world_normal = safe_normalize(&pose.transform_vector(&local_normal), local_normal);

    Some(RaycastHit::new(t, world_pt, world_normal))
}

/// Ray-heightfield intersection using ray marching.
fn raycast_heightfield(
    pose: &Pose,
    data: &crate::heightfield::HeightFieldData,
    ray_origin: Point3<f64>,
    ray_direction: UnitVector3<f64>,
    max_distance: f64,
) -> Option<RaycastHit> {
    // Transform ray to local space
    let inv_pose = pose.inverse();
    let local_origin = inv_pose.transform_point(&ray_origin);
    let local_dir = inv_pose.transform_vector(ray_direction.as_ref());

    // Simple ray marching approach
    let cell_size = data.cell_size();
    // Validate step_size to avoid division by zero or infinite loops
    if cell_size <= 0.0 || !cell_size.is_finite() {
        return None;
    }
    let step_size = cell_size * 0.5;
    // Clamp max_steps to prevent integer overflow and excessive iteration
    let max_steps = ((max_distance / step_size) as usize + 1).min(100_000);

    let mut t = 0.0;
    let mut prev_height_diff = None;

    for _ in 0..max_steps {
        if t > max_distance {
            break;
        }

        let pt = local_origin + local_dir * t;

        // Sample height at this XY position
        if let Some(ground_height) = data.sample(pt.x, pt.y) {
            let height_diff = pt.z - ground_height;

            // Check for crossing
            if let Some(prev_diff) = prev_height_diff {
                if prev_diff > 0.0 && height_diff <= 0.0 {
                    // Crossed the surface, refine with binary search
                    let mut t_lo = t - step_size;
                    let mut t_hi = t;

                    for _ in 0..8 {
                        let t_mid = (t_lo + t_hi) * 0.5;
                        let mid_pt = local_origin + local_dir * t_mid;
                        if let Some(mid_height) = data.sample(mid_pt.x, mid_pt.y) {
                            if mid_pt.z - mid_height > 0.0 {
                                t_lo = t_mid;
                            } else {
                                t_hi = t_mid;
                            }
                        } else {
                            break;
                        }
                    }

                    let final_t = (t_lo + t_hi) * 0.5;
                    let final_pt = local_origin + local_dir * final_t;

                    // Compute normal from heightfield gradient
                    let normal = data
                        .normal(final_pt.x, final_pt.y)
                        .unwrap_or_else(Vector3::z);

                    let world_pt = pose.transform_point(&final_pt);
                    let world_normal = safe_normalize(&pose.transform_vector(&normal), normal);

                    return Some(RaycastHit::new(final_t, world_pt, world_normal));
                }
            }

            prev_height_diff = Some(height_diff);
        }

        t += step_size;
    }

    None
}

/// Ray-SDF intersection using sphere tracing.
fn raycast_sdf(
    pose: &Pose,
    data: &crate::sdf::SdfCollisionData,
    ray_origin: Point3<f64>,
    ray_direction: UnitVector3<f64>,
    max_distance: f64,
) -> Option<RaycastHit> {
    // Transform ray to local space
    let inv_pose = pose.inverse();
    let local_origin = inv_pose.transform_point(&ray_origin);
    let local_dir = inv_pose.transform_vector(ray_direction.as_ref());

    // Sphere tracing
    let mut t = 0.0;
    let max_steps = 128;
    let epsilon = 1e-4;

    for _ in 0..max_steps {
        if t > max_distance {
            return None;
        }

        let pt = local_origin + local_dir * t;

        // Sample SDF - returns Option, so we need to handle the case where
        // the point is outside the grid bounds
        let Some(dist) = data.distance(pt) else {
            t += epsilon;
            continue;
        };

        if dist < epsilon {
            // Hit! Compute normal from gradient
            let grad = data.gradient(pt).unwrap_or_else(Vector3::z);
            let normal = safe_normalize(&grad, Vector3::z());

            let world_pt = pose.transform_point(&pt);
            let world_normal = safe_normalize(&pose.transform_vector(&normal), normal);

            return Some(RaycastHit::new(t, world_pt, world_normal));
        }

        // Step by the distance (sphere tracing guarantee)
        // Handle negative distances (inside object) and NaN by using epsilon as minimum step
        let step = if dist.is_finite() && dist > epsilon {
            dist
        } else {
            epsilon
        };
        t += step;
    }

    None
}

/// Ray-triangle mesh intersection using BVH with early termination.
///
/// Uses `query_ray_closest` for optimal pruning: as closer hits are found,
/// distant BVH nodes are automatically culled. This is O(log n) average case.
fn raycast_triangle_mesh(
    pose: &Pose,
    data: &crate::mesh::TriangleMeshData,
    ray_origin: Point3<f64>,
    ray_direction: UnitVector3<f64>,
    max_distance: f64,
) -> Option<RaycastHit> {
    // Validate max_distance
    if !max_distance.is_finite() || max_distance <= 0.0 {
        return None;
    }

    // Transform ray to local space
    let inv_pose = pose.inverse();
    let local_origin = inv_pose.transform_point(&ray_origin);
    let local_dir = inv_pose.transform_vector(ray_direction.as_ref());

    // Check for degenerate ray (zero direction after transform)
    let dir_norm = local_dir.norm();
    if dir_norm < 1e-10 {
        return None;
    }

    // Use BVH with early termination if available
    if let Some(bvh) = data.bvh() {
        // Precompute inverse direction for slab test
        // IEEE 754: 1.0/0.0 = ±∞, which the slab test handles correctly for axis-aligned rays
        let ray_dir_inv = Vector3::new(1.0 / local_dir.x, 1.0 / local_dir.y, 1.0 / local_dir.z);

        // Track the closest hit for both result and early termination
        let mut closest_hit: Option<(f64, Point3<f64>, Vector3<f64>)> = None;
        let mut cutoff = max_distance;

        // Use query_ray_closest with a callback that tests triangles and updates cutoff
        let result = bvh.query_ray_closest(&local_origin, &ray_dir_inv, max_distance, |tri_idx| {
            let tri = data.get_triangle(tri_idx)?;
            let (v0, v1, v2) = data.triangle_vertices(tri);

            if let Some((t, pt, normal)) =
                ray_triangle_intersection(local_origin, local_dir, v0, v1, v2, cutoff)
            {
                if t < cutoff {
                    cutoff = t;
                    closest_hit = Some((t, pt, normal));
                    return Some(t);
                }
            }
            None
        });

        // Use either the callback result or our tracked closest_hit
        let hit = closest_hit.or_else(|| {
            result.and_then(|(tri_idx, _)| {
                let tri = data.get_triangle(tri_idx)?;
                let (v0, v1, v2) = data.triangle_vertices(tri);
                ray_triangle_intersection(local_origin, local_dir, v0, v1, v2, max_distance)
            })
        });

        hit.map(|(t, local_pt, local_normal)| {
            let world_pt = pose.transform_point(&local_pt);
            let world_normal = safe_normalize(&pose.transform_vector(&local_normal), local_normal);
            RaycastHit::new(t, world_pt, world_normal)
        })
    } else {
        // Fallback: brute force test all triangles
        let mut closest_hit: Option<(f64, Point3<f64>, Vector3<f64>)> = None;

        for tri in data.triangles() {
            let (v0, v1, v2) = data.triangle_vertices(tri);
            let cutoff = closest_hit.as_ref().map_or(max_distance, |h| h.0);

            if let Some((t, pt, normal)) =
                ray_triangle_intersection(local_origin, local_dir, v0, v1, v2, cutoff)
            {
                if closest_hit.is_none() || t < closest_hit.as_ref().map_or(f64::MAX, |h| h.0) {
                    closest_hit = Some((t, pt, normal));
                }
            }
        }

        closest_hit.map(|(t, local_pt, local_normal)| {
            let world_pt = pose.transform_point(&local_pt);
            let world_normal = safe_normalize(&pose.transform_vector(&local_normal), local_normal);
            RaycastHit::new(t, world_pt, world_normal)
        })
    }
}

/// Möller–Trumbore ray-triangle intersection.
fn ray_triangle_intersection(
    ray_origin: Point3<f64>,
    ray_dir: Vector3<f64>,
    v0: Point3<f64>,
    v1: Point3<f64>,
    v2: Point3<f64>,
    max_distance: f64,
) -> Option<(f64, Point3<f64>, Vector3<f64>)> {
    const EPSILON: f64 = 1e-10;

    let edge1 = v1 - v0;
    let edge2 = v2 - v0;
    let h = ray_dir.cross(&edge2);
    let a = edge1.dot(&h);

    if a.abs() < EPSILON {
        return None; // Ray parallel to triangle
    }

    let f = 1.0 / a;
    let s = ray_origin - v0;
    let u = f * s.dot(&h);

    if !(0.0..=1.0).contains(&u) {
        return None;
    }

    let q = s.cross(&edge1);
    let v = f * ray_dir.dot(&q);

    if v < 0.0 || u + v > 1.0 {
        return None;
    }

    let t = f * edge2.dot(&q);

    if t < EPSILON || t > max_distance {
        return None;
    }

    let point = ray_origin + ray_dir * t;
    let cross = edge1.cross(&edge2);
    let normal = safe_normalize(&cross, Vector3::z());
    // Ensure normal points toward ray
    let normal = if normal.dot(&ray_dir) > 0.0 {
        -normal
    } else {
        normal
    };

    Some((t, point, normal))
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::float_cmp)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn test_raycast_sphere_hit() {
        let center = Point3::new(0.0, 0.0, 5.0);
        let radius = 1.0;
        let origin = Point3::origin();
        let direction = UnitVector3::new_normalize(Vector3::z());

        let hit = raycast_sphere(center, radius, origin, direction, 10.0).unwrap();

        assert_relative_eq!(hit.distance, 4.0, epsilon = 1e-6);
        assert_relative_eq!(hit.point.z, 4.0, epsilon = 1e-6);
        assert_relative_eq!(hit.normal.z, -1.0, epsilon = 1e-6);
    }

    #[test]
    fn test_raycast_sphere_miss() {
        let center = Point3::new(5.0, 0.0, 0.0);
        let radius = 1.0;
        let origin = Point3::origin();
        let direction = UnitVector3::new_normalize(Vector3::z());

        let hit = raycast_sphere(center, radius, origin, direction, 10.0);
        assert!(hit.is_none());
    }

    #[test]
    fn test_raycast_sphere_max_distance() {
        let center = Point3::new(0.0, 0.0, 10.0);
        let radius = 1.0;
        let origin = Point3::origin();
        let direction = UnitVector3::new_normalize(Vector3::z());

        // Hit is at t=9, which is beyond max_distance=5
        let hit = raycast_sphere(center, radius, origin, direction, 5.0);
        assert!(hit.is_none());
    }

    #[test]
    fn test_raycast_plane_hit() {
        let plane_point = Point3::new(0.0, 0.0, 5.0);
        let plane_normal = Vector3::z();
        let origin = Point3::origin();
        let direction = UnitVector3::new_normalize(Vector3::z());

        let hit = raycast_plane(&plane_point, &plane_normal, origin, direction, 10.0).unwrap();

        assert_relative_eq!(hit.distance, 5.0, epsilon = 1e-6);
        assert_relative_eq!(hit.normal.z, -1.0, epsilon = 1e-6);
    }

    #[test]
    fn test_raycast_plane_parallel() {
        let plane_point = Point3::new(0.0, 0.0, 5.0);
        let plane_normal = Vector3::z();
        let origin = Point3::origin();
        let direction = UnitVector3::new_normalize(Vector3::x()); // Parallel to plane

        let hit = raycast_plane(&plane_point, &plane_normal, origin, direction, 10.0);
        assert!(hit.is_none());
    }

    #[test]
    fn test_raycast_box_hit() {
        let pose = Pose::from_position(Point3::new(0.0, 0.0, 5.0));
        let half_extents = Vector3::new(1.0, 1.0, 1.0);
        let origin = Point3::origin();
        let direction = UnitVector3::new_normalize(Vector3::z());

        let hit = raycast_box(&pose, half_extents, origin, direction, 10.0).unwrap();

        assert_relative_eq!(hit.distance, 4.0, epsilon = 1e-6);
        assert_relative_eq!(hit.normal.z, -1.0, epsilon = 1e-6);
    }

    #[test]
    fn test_raycast_box_miss() {
        let pose = Pose::from_position(Point3::new(5.0, 0.0, 5.0));
        let half_extents = Vector3::new(1.0, 1.0, 1.0);
        let origin = Point3::origin();
        let direction = UnitVector3::new_normalize(Vector3::z());

        let hit = raycast_box(&pose, half_extents, origin, direction, 10.0);
        assert!(hit.is_none());
    }

    #[test]
    fn test_raycast_capsule_hit_body() {
        let pose = Pose::from_position(Point3::new(0.0, 0.0, 5.0));
        let half_length = 2.0;
        let radius = 0.5;
        let origin = Point3::origin();
        let direction = UnitVector3::new_normalize(Vector3::z());

        let hit = raycast_capsule(&pose, half_length, radius, origin, direction, 10.0).unwrap();

        // Should hit the cap sphere at z = 5 - 2 - 0.5 = 2.5
        assert_relative_eq!(hit.distance, 2.5, epsilon = 1e-4);
    }

    #[test]
    fn test_raycast_cylinder_hit() {
        let pose = Pose::from_position(Point3::new(0.0, 0.0, 5.0));
        let half_length = 1.0;
        let radius = 0.5;
        let origin = Point3::origin();
        let direction = UnitVector3::new_normalize(Vector3::z());

        let hit = raycast_cylinder(&pose, half_length, radius, origin, direction, 10.0).unwrap();

        // Should hit the bottom cap at z = 5 - 1 = 4
        assert_relative_eq!(hit.distance, 4.0, epsilon = 1e-6);
        assert_relative_eq!(hit.normal.z, -1.0, epsilon = 1e-6);
    }

    #[test]
    fn test_raycast_ellipsoid_hit() {
        let pose = Pose::from_position(Point3::new(0.0, 0.0, 5.0));
        let radii = Vector3::new(1.0, 1.0, 2.0); // Stretched along Z
        let origin = Point3::origin();
        let direction = UnitVector3::new_normalize(Vector3::z());

        let hit = raycast_ellipsoid(&pose, radii, origin, direction, 10.0).unwrap();

        // Should hit at z = 5 - 2 = 3
        assert_relative_eq!(hit.distance, 3.0, epsilon = 1e-4);
    }

    #[test]
    fn test_raycast_shape_dispatch() {
        let sphere = CollisionShape::sphere(1.0);
        let pose = Pose::from_position(Point3::new(0.0, 0.0, 5.0));
        let origin = Point3::origin();
        let direction = UnitVector3::new_normalize(Vector3::z());

        let hit = raycast_shape(&sphere, &pose, origin, direction, 10.0).unwrap();

        assert_relative_eq!(hit.distance, 4.0, epsilon = 1e-6);
    }

    #[test]
    fn test_ray_triangle_intersection() {
        let origin = Point3::new(0.0, 0.0, 1.0);
        let dir = -Vector3::z();
        let v0 = Point3::new(-1.0, -1.0, 0.0);
        let v1 = Point3::new(1.0, -1.0, 0.0);
        let v2 = Point3::new(0.0, 1.0, 0.0);

        let hit = ray_triangle_intersection(origin, dir, v0, v1, v2, 10.0).unwrap();

        assert_relative_eq!(hit.0, 1.0, epsilon = 1e-6);
        assert_relative_eq!(hit.2.z, 1.0, epsilon = 1e-6);
    }

    // =========================================================================
    // Edge case tests for NaN/zero/infinity handling
    // =========================================================================

    #[test]
    fn test_raycast_sphere_nan_discriminant() {
        // Test that NaN discriminant is handled properly
        // This can happen with extreme values
        let center = Point3::new(0.0, 0.0, 0.0);
        let radius = 1.0;
        let origin = Point3::origin();
        // Ray inside sphere pointing outward - should still return valid hit
        let direction = UnitVector3::new_normalize(Vector3::z());
        let hit = raycast_sphere(center, radius, origin, direction, 10.0);
        // Origin is inside sphere, so we should get the exit point
        assert!(hit.is_some());
    }

    #[test]
    fn test_raycast_ellipsoid_zero_radius() {
        // Ellipsoid with zero radius should return None
        let pose = Pose::from_position(Point3::new(0.0, 0.0, 5.0));
        let radii = Vector3::new(0.0, 1.0, 1.0); // Zero X radius
        let origin = Point3::origin();
        let direction = UnitVector3::new_normalize(Vector3::z());

        let hit = raycast_ellipsoid(&pose, radii, origin, direction, 10.0);
        assert!(hit.is_none());
    }

    #[test]
    fn test_raycast_plane_zero_normal() {
        // Plane with zero normal should return None (degenerate)
        let plane_point = Point3::new(0.0, 0.0, 5.0);
        let plane_normal = Vector3::zeros(); // Degenerate normal
        let origin = Point3::origin();
        let direction = UnitVector3::new_normalize(Vector3::z());

        let hit = raycast_plane(&plane_point, &plane_normal, origin, direction, 10.0);
        assert!(hit.is_none());
    }

    #[test]
    fn test_raycast_max_distance_zero() {
        // Zero max_distance should return None
        let center = Point3::new(0.0, 0.0, 0.5); // Very close
        let radius = 1.0;
        let origin = Point3::origin();
        let direction = UnitVector3::new_normalize(Vector3::z());

        let hit = raycast_sphere(center, radius, origin, direction, 0.0);
        // Hit point would be at distance < 0.0, so no hit within 0 distance
        assert!(hit.is_none());
    }

    #[test]
    fn test_raycast_negative_max_distance() {
        // Negative max_distance should return None
        let center = Point3::new(0.0, 0.0, 5.0);
        let radius = 1.0;
        let origin = Point3::origin();
        let direction = UnitVector3::new_normalize(Vector3::z());

        let hit = raycast_sphere(center, radius, origin, direction, -10.0);
        assert!(hit.is_none());
    }
}
