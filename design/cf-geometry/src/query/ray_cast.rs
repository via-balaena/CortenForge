//! Ray casting against geometric shapes.
//!
//! All functions operate in **local space** — the shape is centered at the
//! origin with identity orientation. The consuming layer (sim-core) handles
//! the world-space transform via `Pose`.
//!
//! # Supported shapes
//!
//! - Sphere: analytic ray–sphere intersection
//! - Plane: analytic ray–plane intersection
//! - Box: slab test (Kay–Kajiya)
//! - Capsule: cylinder body + sphere caps
//! - Cylinder: infinite cylinder + flat caps
//! - Ellipsoid: transformed sphere intersection
//! - `ConvexMesh`: face-by-face test (Möller–Trumbore)
//! - `TriangleMesh`: BVH-accelerated triangle intersection
//! - `HeightField`: ray marching with binary search refinement
//! - SDF: sphere tracing

#![allow(
    clippy::cast_precision_loss,
    clippy::cast_possible_truncation,
    clippy::cast_sign_loss,
    clippy::similar_names,
    clippy::suspicious_operation_groupings,
    clippy::many_single_char_names,
    clippy::suboptimal_flops,
    clippy::neg_cmp_op_on_partial_ord
)]

use nalgebra::{Point3, Vector3};

use crate::{Ray, RayHit, Shape};

/// Safe vector normalization with fallback.
///
/// Returns the normalized vector if `norm > 1e-10`, otherwise returns the
/// fallback. Used throughout ray casting to avoid division by zero on
/// degenerate normals.
#[inline]
fn safe_normalize(v: &Vector3<f64>, fallback: Vector3<f64>) -> Vector3<f64> {
    let n = v.norm();
    if n > 1e-10 { v / n } else { fallback }
}

/// Cast a ray against a shape in **local space**.
///
/// The shape is assumed to be at the origin with identity orientation.
/// Returns the closest hit within `max_dist`, or `None`.
///
/// # Arguments
///
/// * `ray` — ray with unit-length direction
/// * `shape` — the geometric shape to test
/// * `max_dist` — maximum ray distance
#[must_use]
pub fn ray_cast(ray: &Ray, shape: &Shape, max_dist: f64) -> Option<RayHit> {
    match shape {
        Shape::Sphere { radius } => ray_sphere(ray, *radius, max_dist),
        Shape::Plane { normal, distance } => ray_plane(ray, normal, *distance, max_dist),
        Shape::Box { half_extents } => ray_box(ray, *half_extents, max_dist),
        Shape::Capsule {
            half_length,
            radius,
        } => ray_capsule(ray, *half_length, *radius, max_dist),
        Shape::Cylinder {
            half_length,
            radius,
        } => ray_cylinder(ray, *half_length, *radius, max_dist),
        Shape::Ellipsoid { radii } => ray_ellipsoid(ray, *radii, max_dist),
        Shape::ConvexMesh { hull } => ray_convex_mesh(ray, &hull.vertices, &hull.faces, max_dist),
        Shape::TriangleMesh { mesh, bvh } => ray_indexed_mesh(ray, mesh, bvh, max_dist),
        Shape::HeightField { data } => ray_heightfield(ray, data, max_dist),
        Shape::Sdf { data } => ray_sdf(ray, data, max_dist),
    }
}

// =============================================================================
// Per-shape implementations (all local space)
// =============================================================================

/// Ray–sphere intersection (sphere at origin).
fn ray_sphere(ray: &Ray, radius: f64, max_dist: f64) -> Option<RayHit> {
    let oc = ray.origin.coords;
    let dir = &ray.direction;

    let b = oc.dot(dir);
    let c = oc.dot(&oc) - radius * radius;
    let discriminant = b * b - c;

    if !(discriminant >= 0.0) {
        return None;
    }

    let sqrt_d = discriminant.sqrt();

    let mut t = -b - sqrt_d;
    if t < 0.0 {
        t = -b + sqrt_d;
    }

    if t < 0.0 || t > max_dist {
        return None;
    }

    let point = ray.point_at(t);
    let to_point = point.coords;
    let normal = safe_normalize(&to_point, *dir);

    Some(RayHit::new(t, point, normal))
}

/// Ray–plane intersection (plane at `normal · x = distance`).
fn ray_plane(
    ray: &Ray,
    plane_normal: &Vector3<f64>,
    plane_distance: f64,
    max_dist: f64,
) -> Option<RayHit> {
    let normal_len_sq = plane_normal.norm_squared();
    if normal_len_sq < 1e-20 {
        return None;
    }

    let denom = plane_normal.dot(&ray.direction);

    if !(denom.abs() >= 1e-10) {
        return None;
    }

    // Plane point = normal * distance (point on plane closest to origin)
    let plane_point = Point3::from(*plane_normal * plane_distance);
    let t = (plane_point - ray.origin).dot(plane_normal) / denom;

    if t < 0.0 || t > max_dist {
        return None;
    }

    let point = ray.point_at(t);
    let normal = if denom > 0.0 {
        -*plane_normal
    } else {
        *plane_normal
    };

    Some(RayHit::new(t, point, normal))
}

/// Ray–box intersection using slab method (box at origin).
fn ray_box(ray: &Ray, half_extents: Vector3<f64>, max_dist: f64) -> Option<RayHit> {
    let mut t_min = 0.0_f64;
    let mut t_max = max_dist;
    let mut hit_normal = Vector3::zeros();

    for i in 0..3 {
        let origin_i = ray.origin[i];
        let dir_i = ray.direction[i];
        let extent = half_extents[i];

        if dir_i.abs() < 1e-10 {
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

    if t_min < 0.0 || t_min > max_dist {
        return None;
    }

    let point = ray.point_at(t_min);
    Some(RayHit::new(t_min, point, hit_normal))
}

/// Ray–capsule intersection (capsule along Z at origin).
fn ray_capsule(ray: &Ray, half_length: f64, radius: f64, max_dist: f64) -> Option<RayHit> {
    let local_origin = ray.origin;
    let local_dir = ray.direction;

    let p1 = Point3::new(0.0, 0.0, -half_length);
    let p2 = Point3::new(0.0, 0.0, half_length);

    let mut closest_hit: Option<(f64, Point3<f64>, Vector3<f64>)> = None;
    let mut check_hit = |t: f64, pt: Point3<f64>, normal: Vector3<f64>| {
        if t >= 0.0
            && t <= max_dist
            && (closest_hit.is_none() || t < closest_hit.as_ref().map_or(f64::MAX, |h| h.0))
        {
            closest_hit = Some((t, pt, normal));
        }
    };

    // Infinite cylinder (XY only)
    let oc = Vector3::new(local_origin.x, local_origin.y, 0.0);
    let dir_xy = Vector3::new(local_dir.x, local_dir.y, 0.0);

    let a = dir_xy.dot(&dir_xy);
    let b = oc.dot(&dir_xy);
    let c = oc.dot(&oc) - radius * radius;

    if a > 1e-10 {
        let discriminant = b * b - a * c;
        if discriminant >= 0.0 && discriminant.is_finite() {
            let sqrt_d = discriminant.sqrt();

            for t in [(-b - sqrt_d) / a, (-b + sqrt_d) / a] {
                if t >= 0.0 && t.is_finite() {
                    let pt = Point3::from(local_origin.coords + local_dir * t);
                    if pt.z >= -half_length && pt.z <= half_length {
                        let radial = Vector3::new(pt.x, pt.y, 0.0);
                        let normal = safe_normalize(&radial, Vector3::x());
                        check_hit(t, pt, normal);
                    }
                }
            }
        }
    }

    // Sphere caps
    for &cap_center in &[p1, p2] {
        let oc = local_origin - cap_center;
        let b = oc.dot(&local_dir);
        let c = oc.dot(&oc) - radius * radius;
        let discriminant = b * b - c;

        if discriminant >= 0.0 && discriminant.is_finite() {
            let sqrt_d = discriminant.sqrt();
            let t = -b - sqrt_d;
            if t >= 0.0 {
                let pt = Point3::from(local_origin.coords + local_dir * t);
                let to_pt = pt - cap_center;
                let normal = safe_normalize(&to_pt, local_dir);
                check_hit(t, pt, normal);
            }
        }
    }

    closest_hit.map(|(t, pt, normal)| RayHit::new(t, pt, normal))
}

/// Ray–cylinder intersection with flat caps (cylinder along Z at origin).
fn ray_cylinder(ray: &Ray, half_length: f64, radius: f64, max_dist: f64) -> Option<RayHit> {
    let local_origin = ray.origin;
    let local_dir = ray.direction;

    let mut closest_t = f64::MAX;
    let mut closest_hit: Option<(Point3<f64>, Vector3<f64>)> = None;

    // Infinite cylinder (XY only)
    let oc = Vector3::new(local_origin.x, local_origin.y, 0.0);
    let dir_xy = Vector3::new(local_dir.x, local_dir.y, 0.0);

    let a = dir_xy.dot(&dir_xy);
    let b = oc.dot(&dir_xy);
    let c = oc.dot(&oc) - radius * radius;

    if a > 1e-10 {
        let discriminant = b * b - a * c;
        if discriminant >= 0.0 && discriminant.is_finite() {
            let sqrt_d = discriminant.sqrt();

            for t in [(-b - sqrt_d) / a, (-b + sqrt_d) / a] {
                if t >= 0.0 && t < closest_t && t <= max_dist && t.is_finite() {
                    let pt = Point3::from(local_origin.coords + local_dir * t);
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

    // Flat caps
    for (cap_z, cap_normal) in [
        (-half_length, Vector3::new(0.0, 0.0, -1.0)),
        (half_length, Vector3::new(0.0, 0.0, 1.0)),
    ] {
        if local_dir.z.abs() > 1e-10 {
            let t = (cap_z - local_origin.z) / local_dir.z;
            if t >= 0.0 && t < closest_t && t <= max_dist {
                let pt = Point3::from(local_origin.coords + local_dir * t);
                if pt.x * pt.x + pt.y * pt.y <= radius * radius {
                    closest_t = t;
                    closest_hit = Some((pt, cap_normal));
                }
            }
        }
    }

    closest_hit.map(|(pt, normal)| RayHit::new(closest_t, pt, normal))
}

/// Ray–ellipsoid intersection (ellipsoid at origin).
fn ray_ellipsoid(ray: &Ray, radii: Vector3<f64>, max_dist: f64) -> Option<RayHit> {
    const MIN_RADIUS: f64 = 1e-10;
    if radii.x < MIN_RADIUS || radii.y < MIN_RADIUS || radii.z < MIN_RADIUS {
        return None;
    }

    // Scale to unit sphere
    let scaled_origin = Point3::new(
        ray.origin.x / radii.x,
        ray.origin.y / radii.y,
        ray.origin.z / radii.z,
    );
    let scaled_dir = Vector3::new(
        ray.direction.x / radii.x,
        ray.direction.y / radii.y,
        ray.direction.z / radii.z,
    );

    let oc = scaled_origin.coords;
    let a = scaled_dir.dot(&scaled_dir);
    let b = oc.dot(&scaled_dir);
    let c = oc.dot(&oc) - 1.0;

    let discriminant = b * b - a * c;
    if !(discriminant >= 0.0) {
        return None;
    }
    if a < 1e-20 {
        return None;
    }

    let sqrt_d = discriminant.sqrt();
    let mut t = (-b - sqrt_d) / a;
    if t < 0.0 {
        t = (-b + sqrt_d) / a;
    }
    if t < 0.0 {
        return None;
    }

    let local_pt = ray.point_at(t);

    // For local-space, t IS the distance (no pose transform needed)
    if t > max_dist {
        return None;
    }

    // Normal = gradient of (x/a)² + (y/b)² + (z/c)² − 1
    let grad = Vector3::new(
        2.0 * local_pt.x / (radii.x * radii.x),
        2.0 * local_pt.y / (radii.y * radii.y),
        2.0 * local_pt.z / (radii.z * radii.z),
    );
    let normal = safe_normalize(&grad, Vector3::z());

    Some(RayHit::new(t, local_pt, normal))
}

/// Ray–convex mesh intersection via face-by-face Möller–Trumbore.
fn ray_convex_mesh(
    ray: &Ray,
    vertices: &[Point3<f64>],
    faces: &[[u32; 3]],
    max_dist: f64,
) -> Option<RayHit> {
    if vertices.len() < 4 || faces.is_empty() {
        return None;
    }

    // Bounding sphere quick-reject
    let centroid: Vector3<f64> =
        vertices.iter().map(|v| v.coords).sum::<Vector3<f64>>() / (vertices.len() as f64);
    let center = Point3::from(centroid);
    let bounding_radius = vertices
        .iter()
        .map(|v| (v - center).norm())
        .fold(0.0_f64, f64::max);

    let oc = ray.origin - center;
    let b = oc.dot(&ray.direction);
    let c = oc.dot(&oc) - bounding_radius * bounding_radius;
    let discriminant = b * b - c;
    if !(discriminant >= 0.0) {
        return None;
    }

    // Test every face
    let mut closest: Option<RayHit> = None;
    let mut cutoff = max_dist;

    for face in faces {
        let v0 = vertices[face[0] as usize];
        let v1 = vertices[face[1] as usize];
        let v2 = vertices[face[2] as usize];

        if let Some(hit) = ray_triangle_impl(ray.origin, ray.direction, v0, v1, v2, cutoff) {
            cutoff = hit.distance;
            closest = Some(hit);
        }
    }

    closest
}

/// Ray–indexed mesh intersection using BVH with early termination.
fn ray_indexed_mesh(
    ray: &Ray,
    mesh: &crate::IndexedMesh,
    bvh: &crate::Bvh,
    max_dist: f64,
) -> Option<RayHit> {
    if !max_dist.is_finite() || max_dist <= 0.0 {
        return None;
    }

    let dir_norm = ray.direction.norm();
    if dir_norm < 1e-10 {
        return None;
    }

    let tri_verts = |face_idx: usize| -> Option<(Point3<f64>, Point3<f64>, Point3<f64>)> {
        let face = mesh.faces.get(face_idx)?;
        Some((
            mesh.vertices[face[0] as usize],
            mesh.vertices[face[1] as usize],
            mesh.vertices[face[2] as usize],
        ))
    };

    let ray_dir_inv = Vector3::new(
        1.0 / ray.direction.x,
        1.0 / ray.direction.y,
        1.0 / ray.direction.z,
    );

    let mut closest_hit: Option<RayHit> = None;
    let mut cutoff = max_dist;

    let result = bvh.query_ray_closest(&ray.origin, &ray_dir_inv, max_dist, |tri_idx| {
        let (v0, v1, v2) = tri_verts(tri_idx)?;

        if let Some(hit) = ray_triangle_impl(ray.origin, ray.direction, v0, v1, v2, cutoff) {
            if hit.distance < cutoff {
                cutoff = hit.distance;
                closest_hit = Some(hit);
                return Some(hit.distance);
            }
        }
        None
    });

    closest_hit.or_else(|| {
        result.and_then(|(tri_idx, _)| {
            let (v0, v1, v2) = tri_verts(tri_idx)?;
            ray_triangle_impl(ray.origin, ray.direction, v0, v1, v2, max_dist)
        })
    })
}

/// Ray–height field intersection using ray marching with binary search refinement.
fn ray_heightfield(ray: &Ray, data: &crate::HeightFieldData, max_dist: f64) -> Option<RayHit> {
    let cell_size = data.cell_size();
    if cell_size <= 0.0 || !cell_size.is_finite() {
        return None;
    }

    let step_size = cell_size * 0.5;
    let max_steps = ((max_dist / step_size) as usize + 1).min(100_000);

    let mut t = 0.0;
    let mut prev_height_diff = None;

    for _ in 0..max_steps {
        if t > max_dist {
            break;
        }

        let pt = ray.point_at(t);

        if let Some(ground_height) = data.sample(pt.x, pt.y) {
            let height_diff = pt.z - ground_height;

            if let Some(prev_diff) = prev_height_diff {
                if prev_diff > 0.0 && height_diff <= 0.0 {
                    // Surface crossing — refine with binary search
                    let mut t_lo = t - step_size;
                    let mut t_hi = t;

                    for _ in 0..8 {
                        let t_mid = (t_lo + t_hi) * 0.5;
                        let mid_pt = ray.point_at(t_mid);
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
                    let final_pt = ray.point_at(final_t);

                    let normal = data
                        .normal(final_pt.x, final_pt.y)
                        .unwrap_or_else(Vector3::z);

                    return Some(RayHit::new(final_t, final_pt, normal));
                }
            }

            prev_height_diff = Some(height_diff);
        }

        t += step_size;
    }

    None
}

/// Ray–SDF intersection using sphere tracing.
fn ray_sdf(ray: &Ray, data: &crate::SdfGrid, max_dist: f64) -> Option<RayHit> {
    let mut t = 0.0;
    let max_steps = 128;
    let epsilon = 1e-4;

    for _ in 0..max_steps {
        if t > max_dist {
            return None;
        }

        let pt = ray.point_at(t);

        let Some(dist) = data.distance(pt) else {
            t += epsilon;
            continue;
        };

        if dist < epsilon {
            let grad = data.gradient(pt).unwrap_or_else(Vector3::z);
            let normal = safe_normalize(&grad, Vector3::z());
            return Some(RayHit::new(t, pt, normal));
        }

        let step = if dist.is_finite() && dist > epsilon {
            dist
        } else {
            epsilon
        };
        t += step;
    }

    None
}

// =============================================================================
// Triangle intersection (public + internal)
// =============================================================================

/// Möller–Trumbore ray–triangle intersection.
///
/// Returns the hit as a [`RayHit`] with outward normal pointing toward the ray
/// origin. All coordinates are local-space.
#[must_use]
pub fn ray_triangle(
    ray: &Ray,
    v0: Point3<f64>,
    v1: Point3<f64>,
    v2: Point3<f64>,
    max_dist: f64,
) -> Option<RayHit> {
    ray_triangle_impl(ray.origin, ray.direction, v0, v1, v2, max_dist)
}

/// Internal Möller–Trumbore with origin/direction split (avoids constructing Ray
/// inside hot loops).
fn ray_triangle_impl(
    ray_origin: Point3<f64>,
    ray_dir: Vector3<f64>,
    v0: Point3<f64>,
    v1: Point3<f64>,
    v2: Point3<f64>,
    max_distance: f64,
) -> Option<RayHit> {
    const EPSILON: f64 = 1e-10;

    let edge1 = v1 - v0;
    let edge2 = v2 - v0;
    let h = ray_dir.cross(&edge2);
    let a = edge1.dot(&h);

    if a.abs() < EPSILON {
        return None;
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
    // Normal faces toward ray origin
    let normal = if normal.dot(&ray_dir) > 0.0 {
        -normal
    } else {
        normal
    };

    Some(RayHit::new(t, point, normal))
}

// =============================================================================
// Tests
// =============================================================================

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::float_cmp)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use std::sync::Arc;

    fn ray_z() -> Ray {
        Ray::new(Point3::origin(), Vector3::z())
    }

    fn ray_from(origin: Point3<f64>, dir: Vector3<f64>) -> Ray {
        Ray::new(origin, dir)
    }

    // ── Sphere ────────────────────────────────────────────────────────

    #[test]
    fn sphere_hit_from_outside() {
        let ray = ray_from(Point3::new(0.0, 0.0, -5.0), Vector3::z());
        let hit = ray_sphere(&ray, 1.0, 10.0).unwrap();
        assert_relative_eq!(hit.distance, 4.0, epsilon = 1e-6);
        assert_relative_eq!(hit.normal.z, -1.0, epsilon = 1e-6);
    }

    #[test]
    fn sphere_hit_from_inside() {
        let ray = ray_z();
        let hit = ray_sphere(&ray, 1.0, 10.0).unwrap();
        assert_relative_eq!(hit.distance, 1.0, epsilon = 1e-6);
    }

    #[test]
    fn sphere_miss() {
        let ray = ray_from(Point3::new(5.0, 0.0, -5.0), Vector3::z());
        assert!(ray_sphere(&ray, 1.0, 10.0).is_none());
    }

    #[test]
    fn sphere_beyond_max_dist() {
        let ray = ray_from(Point3::new(0.0, 0.0, -10.0), Vector3::z());
        assert!(ray_sphere(&ray, 1.0, 5.0).is_none());
    }

    // ── Plane ─────────────────────────────────────────────────────────

    #[test]
    fn plane_hit() {
        let ray = ray_from(Point3::new(0.0, 0.0, -5.0), Vector3::z());
        let hit = ray_plane(&ray, &Vector3::z(), 0.0, 10.0).unwrap();
        assert_relative_eq!(hit.distance, 5.0, epsilon = 1e-6);
        assert_relative_eq!(hit.normal.z, -1.0, epsilon = 1e-6);
    }

    #[test]
    fn plane_parallel_miss() {
        let ray = ray_from(Point3::origin(), Vector3::x());
        assert!(ray_plane(&ray, &Vector3::z(), 1.0, 10.0).is_none());
    }

    #[test]
    fn plane_behind_ray() {
        let ray = ray_from(Point3::new(0.0, 0.0, 5.0), Vector3::z());
        assert!(ray_plane(&ray, &Vector3::z(), 0.0, 10.0).is_none());
    }

    // ── Box ───────────────────────────────────────────────────────────

    #[test]
    fn box_hit() {
        let ray = ray_from(Point3::new(0.0, 0.0, -5.0), Vector3::z());
        let hit = ray_box(&ray, Vector3::new(1.0, 1.0, 1.0), 10.0).unwrap();
        assert_relative_eq!(hit.distance, 4.0, epsilon = 1e-6);
        assert_relative_eq!(hit.normal.z, -1.0, epsilon = 1e-6);
    }

    #[test]
    fn box_miss() {
        let ray = ray_from(Point3::new(5.0, 0.0, -5.0), Vector3::z());
        assert!(ray_box(&ray, Vector3::new(1.0, 1.0, 1.0), 10.0).is_none());
    }

    #[test]
    fn box_angled_hit() {
        // Ray from (-5, 0, -5) toward origin — should hit the box at -X face
        let dir = Vector3::new(1.0, 0.0, 1.0).normalize();
        let ray = ray_from(Point3::new(-5.0, 0.0, -5.0), dir);
        let hit = ray_box(&ray, Vector3::new(1.0, 1.0, 1.0), 20.0);
        assert!(hit.is_some());
        let hit = hit.unwrap();
        assert_relative_eq!(hit.normal.x, -1.0, epsilon = 1e-6);
    }

    // ── Capsule ───────────────────────────────────────────────────────

    #[test]
    fn capsule_hit_cap() {
        let ray = ray_from(Point3::new(0.0, 0.0, -5.0), Vector3::z());
        let hit = ray_capsule(&ray, 2.0, 0.5, 10.0).unwrap();
        // Bottom cap center at z = -2.0, radius 0.5 → hit at z = -2.5
        assert_relative_eq!(hit.distance, 2.5, epsilon = 1e-4);
    }

    #[test]
    fn capsule_hit_body() {
        let ray = ray_from(Point3::new(-5.0, 0.0, 0.0), Vector3::x());
        let hit = ray_capsule(&ray, 2.0, 0.5, 10.0).unwrap();
        // Cylinder body at x = -0.5
        assert_relative_eq!(hit.distance, 4.5, epsilon = 1e-4);
    }

    // ── Cylinder ──────────────────────────────────────────────────────

    #[test]
    fn cylinder_hit_cap() {
        let ray = ray_from(Point3::new(0.0, 0.0, -5.0), Vector3::z());
        let hit = ray_cylinder(&ray, 1.0, 0.5, 10.0).unwrap();
        assert_relative_eq!(hit.distance, 4.0, epsilon = 1e-6);
        assert_relative_eq!(hit.normal.z, -1.0, epsilon = 1e-6);
    }

    #[test]
    fn cylinder_hit_body() {
        let ray = ray_from(Point3::new(-5.0, 0.0, 0.0), Vector3::x());
        let hit = ray_cylinder(&ray, 1.0, 0.5, 10.0).unwrap();
        assert_relative_eq!(hit.distance, 4.5, epsilon = 1e-4);
    }

    // ── Ellipsoid ─────────────────────────────────────────────────────

    #[test]
    fn ellipsoid_hit() {
        let ray = ray_from(Point3::new(0.0, 0.0, -5.0), Vector3::z());
        let hit = ray_ellipsoid(&ray, Vector3::new(1.0, 1.0, 2.0), 10.0).unwrap();
        assert_relative_eq!(hit.distance, 3.0, epsilon = 1e-4);
    }

    #[test]
    fn ellipsoid_zero_radius() {
        let ray = ray_z();
        assert!(ray_ellipsoid(&ray, Vector3::new(0.0, 1.0, 1.0), 10.0).is_none());
    }

    // ── Triangle ──────────────────────────────────────────────────────

    #[test]
    fn triangle_hit() {
        let ray = ray_from(Point3::new(0.0, 0.0, 1.0), -Vector3::z());
        let v0 = Point3::new(-1.0, -1.0, 0.0);
        let v1 = Point3::new(1.0, -1.0, 0.0);
        let v2 = Point3::new(0.0, 1.0, 0.0);

        let hit = ray_triangle(&ray, v0, v1, v2, 10.0).unwrap();
        assert_relative_eq!(hit.distance, 1.0, epsilon = 1e-6);
        assert_relative_eq!(hit.normal.z, 1.0, epsilon = 1e-6);
    }

    #[test]
    fn triangle_miss() {
        let ray = ray_from(Point3::new(5.0, 5.0, 1.0), -Vector3::z());
        let v0 = Point3::new(-1.0, -1.0, 0.0);
        let v1 = Point3::new(1.0, -1.0, 0.0);
        let v2 = Point3::new(0.0, 1.0, 0.0);
        assert!(ray_triangle(&ray, v0, v1, v2, 10.0).is_none());
    }

    // ── Shape dispatch ────────────────────────────────────────────────

    #[test]
    fn dispatch_sphere() {
        let shape = Shape::sphere(1.0);
        let ray = ray_from(Point3::new(0.0, 0.0, -5.0), Vector3::z());
        let hit = ray_cast(&ray, &shape, 10.0).unwrap();
        assert_relative_eq!(hit.distance, 4.0, epsilon = 1e-6);
    }

    #[test]
    fn dispatch_plane() {
        let shape = Shape::ground_plane(0.0);
        let ray = ray_from(Point3::new(0.0, 0.0, 5.0), -Vector3::z());
        let hit = ray_cast(&ray, &shape, 10.0).unwrap();
        assert_relative_eq!(hit.distance, 5.0, epsilon = 1e-6);
    }

    #[test]
    fn dispatch_box() {
        let shape = Shape::box_shape(Vector3::new(1.0, 1.0, 1.0));
        let ray = ray_from(Point3::new(0.0, 0.0, -5.0), Vector3::z());
        let hit = ray_cast(&ray, &shape, 10.0).unwrap();
        assert_relative_eq!(hit.distance, 4.0, epsilon = 1e-6);
    }

    #[test]
    fn dispatch_capsule() {
        let shape = Shape::capsule(2.0, 0.5);
        let ray = ray_from(Point3::new(0.0, 0.0, -5.0), Vector3::z());
        let hit = ray_cast(&ray, &shape, 10.0).unwrap();
        assert_relative_eq!(hit.distance, 2.5, epsilon = 1e-4);
    }

    #[test]
    fn dispatch_cylinder() {
        let shape = Shape::cylinder(1.0, 0.5);
        let ray = ray_from(Point3::new(0.0, 0.0, -5.0), Vector3::z());
        let hit = ray_cast(&ray, &shape, 10.0).unwrap();
        assert_relative_eq!(hit.distance, 4.0, epsilon = 1e-6);
    }

    #[test]
    fn dispatch_ellipsoid() {
        let shape = Shape::ellipsoid(Vector3::new(1.0, 1.0, 2.0));
        let ray = ray_from(Point3::new(0.0, 0.0, -5.0), Vector3::z());
        let hit = ray_cast(&ray, &shape, 10.0).unwrap();
        assert_relative_eq!(hit.distance, 3.0, epsilon = 1e-4);
    }

    #[test]
    fn dispatch_triangle_mesh() {
        // Tetrahedron mesh
        let mesh = crate::IndexedMesh::from_parts(
            vec![
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(2.0, 0.0, 0.0),
                Point3::new(1.0, 2.0, 0.0),
                Point3::new(1.0, 1.0, 2.0),
            ],
            vec![[0, 1, 2], [0, 1, 3], [1, 2, 3], [0, 2, 3]],
        );
        let bvh = crate::bvh_from_mesh(&mesh);
        let shape = Shape::triangle_mesh(Arc::new(mesh), Arc::new(bvh));

        let ray = ray_from(Point3::new(1.0, 0.5, 5.0), -Vector3::z());
        let hit = ray_cast(&ray, &shape, 20.0);
        assert!(hit.is_some());
    }

    #[test]
    fn dispatch_heightfield() {
        let data = crate::HeightFieldData::flat(10, 10, 1.0, 0.0);
        let shape = Shape::height_field(Arc::new(data));

        let ray = ray_from(Point3::new(5.0, 5.0, 5.0), -Vector3::z());
        let hit = ray_cast(&ray, &shape, 20.0);
        assert!(hit.is_some());
        let hit = hit.unwrap();
        assert_relative_eq!(hit.distance, 5.0, epsilon = 0.5);
    }

    #[test]
    fn dispatch_sdf() {
        let data = crate::SdfGrid::sphere(Point3::origin(), 1.0, 32, 1.0);
        let shape = Shape::sdf(Arc::new(data));

        let ray = ray_from(Point3::new(0.0, 0.0, -1.5), Vector3::z());
        let hit = ray_cast(&ray, &shape, 20.0);
        // Sphere tracing should find the surface near z = -1.0
        assert!(hit.is_some());
        let hit = hit.unwrap();
        assert_relative_eq!(hit.distance, 0.5, epsilon = 0.15);
    }

    // ── Edge cases ────────────────────────────────────────────────────

    #[test]
    fn zero_max_distance() {
        let ray = ray_from(Point3::new(0.0, 0.0, -0.5), Vector3::z());
        // Sphere at origin radius 1 — hit would be at t=0.5, but max_dist=0
        assert!(ray_sphere(&ray, 1.0, 0.0).is_none());
    }

    #[test]
    fn negative_max_distance() {
        let ray = ray_from(Point3::new(0.0, 0.0, -5.0), Vector3::z());
        assert!(ray_sphere(&ray, 1.0, -10.0).is_none());
    }
}
