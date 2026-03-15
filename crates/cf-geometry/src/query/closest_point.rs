//! Closest-point geometric queries.
//!
//! Pure geometry — all functions operate in local space (no pose transforms).

#![allow(clippy::suboptimal_flops)]

use nalgebra::{Point3, Vector3};

use crate::Shape;

/// Epsilon for degenerate-geometry guards.
const EPSILON: f64 = 1e-10;

/// Safe vector normalization with fallback.
#[inline]
fn safe_normalize(v: &Vector3<f64>, fallback: Vector3<f64>) -> Vector3<f64> {
    let n = v.norm();
    if n > 1e-10 { v / n } else { fallback }
}

/// Closest point on a shape's surface to a query point, in **local space**.
///
/// The shape is assumed to be at the origin with identity orientation.
/// Returns the point on (or nearest to) the shape's surface.
///
/// # Notes
///
/// - For `ConvexMesh`, this uses a bounding-sphere approximation until GJK
///   is available (Session 17).
/// - For `TriangleMesh`, this performs a brute-force search over all triangles
///   (no BVH acceleration for closest-point yet).
#[must_use]
#[allow(clippy::cast_precision_loss)]
pub fn closest_point(shape: &Shape, point: &Point3<f64>) -> Point3<f64> {
    match shape {
        Shape::Sphere { radius } => closest_point_sphere(*radius, point),
        Shape::Plane { normal, distance } => closest_point_plane(normal, *distance, point),
        Shape::Box { half_extents } => closest_point_box(*half_extents, point),
        Shape::Capsule {
            half_length,
            radius,
        } => closest_point_capsule(*half_length, *radius, point),
        Shape::Cylinder {
            half_length,
            radius,
        } => closest_point_cylinder(*half_length, *radius, point),
        Shape::Ellipsoid { radii } => closest_point_ellipsoid(*radii, point),
        Shape::ConvexMesh { hull } => closest_point_convex_mesh(&hull.vertices, point),
        Shape::TriangleMesh { mesh, .. } => closest_point_triangle_mesh(mesh, point),
        Shape::HeightField { data } => closest_point_heightfield(data, point),
        Shape::Sdf { data } => closest_point_sdf(data, point),
    }
}

// =============================================================================
// Per-shape implementations (all local space, shape at origin)
// =============================================================================

/// Closest point on sphere surface to query point.
fn closest_point_sphere(radius: f64, point: &Point3<f64>) -> Point3<f64> {
    let dir = point.coords;
    let dist = dir.norm();
    if dist < EPSILON {
        // Point is at center — return arbitrary surface point
        Point3::new(radius, 0.0, 0.0)
    } else {
        Point3::from(dir * (radius / dist))
    }
}

/// Closest point on infinite plane to query point.
fn closest_point_plane(normal: &Vector3<f64>, distance: f64, point: &Point3<f64>) -> Point3<f64> {
    let n = safe_normalize(normal, Vector3::z());
    let signed_dist = point.coords.dot(&n) - distance;
    Point3::from(point.coords - n * signed_dist)
}

/// Closest point on axis-aligned box surface to query point.
fn closest_point_box(half_extents: Vector3<f64>, point: &Point3<f64>) -> Point3<f64> {
    // Clamp to box interior first
    let clamped = Point3::new(
        point.x.clamp(-half_extents.x, half_extents.x),
        point.y.clamp(-half_extents.y, half_extents.y),
        point.z.clamp(-half_extents.z, half_extents.z),
    );

    // If point was outside (any axis clamped), clamped IS the closest surface point
    let outside = (clamped.x - point.x).abs() > EPSILON
        || (clamped.y - point.y).abs() > EPSILON
        || (clamped.z - point.z).abs() > EPSILON;
    if outside {
        return clamped;
    }

    // Point is inside — project to nearest face.
    // For each axis, compute distance to the positive and negative face.
    // face_dists: [(distance, axis, sign), ...]
    let face_dists = [
        (half_extents.x - point.x, 0usize, 1.0_f64),
        (half_extents.x + point.x, 0, -1.0),
        (half_extents.y - point.y, 1, 1.0),
        (half_extents.y + point.y, 1, -1.0),
        (half_extents.z - point.z, 2, 1.0),
        (half_extents.z + point.z, 2, -1.0),
    ];

    let (_, axis, sign) = face_dists
        .iter()
        .copied()
        .min_by(|a, b| a.0.partial_cmp(&b.0).unwrap_or(std::cmp::Ordering::Equal))
        .unwrap_or((0.0, 0, 1.0));

    let mut result = *point;
    result[axis] = sign * half_extents[axis];
    result
}

/// Closest point on capsule surface to query point.
fn closest_point_capsule(half_length: f64, radius: f64, point: &Point3<f64>) -> Point3<f64> {
    // Capsule axis is Z. Find closest point on axis segment.
    let axis_pt = Point3::new(0.0, 0.0, point.z.clamp(-half_length, half_length));
    let dir = *point - axis_pt;
    let dist = dir.norm();
    if dist < EPSILON {
        // Point is on axis — return arbitrary surface point at this Z
        Point3::new(radius, 0.0, axis_pt.z)
    } else {
        Point3::from(axis_pt.coords + dir * (radius / dist))
    }
}

/// Closest point on cylinder surface to query point.
fn closest_point_cylinder(half_length: f64, radius: f64, point: &Point3<f64>) -> Point3<f64> {
    let z_clamped = point.z.clamp(-half_length, half_length);
    let radial = Vector3::new(point.x, point.y, 0.0);
    let radial_dist = radial.norm();

    let on_cap = point.z.abs() > half_length;
    let outside_barrel = radial_dist > radius;

    if on_cap && outside_barrel {
        // Closest to cap rim
        let dir = if radial_dist > EPSILON {
            radial / radial_dist
        } else {
            Vector3::x()
        };
        Point3::from(dir * radius + Vector3::new(0.0, 0.0, z_clamped))
    } else if on_cap {
        // Closest to flat cap face (inside barrel radius)
        Point3::new(point.x, point.y, z_clamped)
    } else if outside_barrel {
        // Closest to barrel surface
        let dir = if radial_dist > EPSILON {
            radial / radial_dist
        } else {
            Vector3::x()
        };
        Point3::from(dir * radius + Vector3::new(0.0, 0.0, point.z))
    } else {
        // Inside cylinder — project to nearest surface (barrel or cap)
        let to_barrel = radius - radial_dist;
        let to_cap = half_length - point.z.abs();

        if to_barrel < to_cap {
            // Closer to barrel
            let dir = if radial_dist > EPSILON {
                radial / radial_dist
            } else {
                Vector3::x()
            };
            Point3::from(dir * radius + Vector3::new(0.0, 0.0, point.z))
        } else {
            // Closer to cap
            let cap_z = if point.z >= 0.0 {
                half_length
            } else {
                -half_length
            };
            Point3::new(point.x, point.y, cap_z)
        }
    }
}

/// Closest point on ellipsoid surface to query point.
///
/// Uses iterative projection: normalize the scaled point to the unit sphere,
/// then scale back. This converges quickly for moderate aspect ratios.
fn closest_point_ellipsoid(radii: Vector3<f64>, point: &Point3<f64>) -> Point3<f64> {
    // Avoid division by zero for degenerate ellipsoids
    let rx = radii.x.max(EPSILON);
    let ry = radii.y.max(EPSILON);
    let rz = radii.z.max(EPSILON);

    // Scale to unit sphere
    let scaled = Vector3::new(point.x / rx, point.y / ry, point.z / rz);
    let scaled_norm = scaled.norm();

    if scaled_norm < EPSILON {
        // Point is at center — return +X surface point
        return Point3::new(rx, 0.0, 0.0);
    }

    // Project onto unit sphere, scale back
    let unit = scaled / scaled_norm;
    let mut result = Point3::new(unit.x * rx, unit.y * ry, unit.z * rz);

    // Newton refinement (3 iterations is sufficient for good convergence)
    for _ in 0..3 {
        let grad = Vector3::new(
            result.x / (rx * rx),
            result.y / (ry * ry),
            result.z / (rz * rz),
        );
        let grad_norm = grad.norm();
        if grad_norm < EPSILON {
            break;
        }
        let n = grad / grad_norm;
        let diff = *point - result;
        let correction = diff.dot(&n);
        result += n * correction;

        // Re-project onto ellipsoid surface
        let s = Vector3::new(result.x / rx, result.y / ry, result.z / rz);
        let s_norm = s.norm();
        if s_norm > EPSILON {
            let u = s / s_norm;
            result = Point3::new(u.x * rx, u.y * ry, u.z * rz);
        }
    }

    result
}

/// Closest point on convex mesh surface to query point.
///
/// Uses bounding-sphere approximation. Will be replaced with GJK in Session 17.
fn closest_point_convex_mesh(vertices: &[Point3<f64>], point: &Point3<f64>) -> Point3<f64> {
    if vertices.is_empty() {
        return *point;
    }

    // Brute-force: find nearest vertex, then check adjacent faces
    // For now, find the closest vertex as a reasonable approximation
    let mut best = vertices[0];
    let mut best_dist_sq = (vertices[0] - point).norm_squared();

    for v in &vertices[1..] {
        let d = (v - point).norm_squared();
        if d < best_dist_sq {
            best_dist_sq = d;
            best = *v;
        }
    }

    best
}

/// Closest point on triangle mesh surface to query point.
///
/// Brute-force over all triangles (no BVH acceleration for closest-point yet).
fn closest_point_triangle_mesh(mesh: &crate::IndexedMesh, point: &Point3<f64>) -> Point3<f64> {
    let mut best = *point;
    let mut best_dist_sq = f64::MAX;

    for face in &mesh.faces {
        let v0 = mesh.vertices[face[0] as usize];
        let v1 = mesh.vertices[face[1] as usize];
        let v2 = mesh.vertices[face[2] as usize];

        let cp = closest_point_on_triangle(v0, v1, v2, *point);
        let d = (cp - point).norm_squared();
        if d < best_dist_sq {
            best_dist_sq = d;
            best = cp;
        }
    }

    best
}

/// Closest point on height field surface to query point.
fn closest_point_heightfield(data: &crate::HeightFieldData, point: &Point3<f64>) -> Point3<f64> {
    let x = point.x.clamp(0.0, data.extent_x());
    let y = point.y.clamp(0.0, data.extent_y());
    let z = data.sample_clamped(x, y);
    Point3::new(x, y, z)
}

/// Closest point on SDF surface to query point.
fn closest_point_sdf(data: &crate::SdfGrid, point: &Point3<f64>) -> Point3<f64> {
    data.closest_surface_point(*point, 10).unwrap_or(*point)
}

/// Closest point on triangle (v0, v1, v2) to point `p`.
///
/// Uses the Voronoi region method from *Real-Time Collision Detection* §5.1.5.
/// Handles degenerate triangles (zero area) gracefully by falling back to the
/// nearest vertex or edge midpoint.
#[must_use]
pub fn closest_point_on_triangle(
    v0: Point3<f64>,
    v1: Point3<f64>,
    v2: Point3<f64>,
    p: Point3<f64>,
) -> Point3<f64> {
    let ab = v1 - v0;
    let ac = v2 - v0;
    let ap = p - v0;

    let d1 = ab.dot(&ap);
    let d2 = ac.dot(&ap);
    if d1 <= 0.0 && d2 <= 0.0 {
        return v0;
    }

    let bp = p - v1;
    let d3 = ab.dot(&bp);
    let d4 = ac.dot(&bp);
    if d3 >= 0.0 && d4 <= d3 {
        return v1;
    }

    // Edge AB
    let vc = d1 * d4 - d3 * d2;
    if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
        let denom = d1 - d3;
        let v = if denom.abs() > EPSILON {
            d1 / denom
        } else {
            0.5
        };
        return Point3::from(v0.coords + ab * v);
    }

    let cp = p - v2;
    let d5 = ab.dot(&cp);
    let d6 = ac.dot(&cp);
    if d6 >= 0.0 && d5 <= d6 {
        return v2;
    }

    // Edge AC
    let vb = d5 * d2 - d1 * d6;
    if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
        let denom = d2 - d6;
        let w = if denom.abs() > EPSILON {
            d2 / denom
        } else {
            0.5
        };
        return Point3::from(v0.coords + ac * w);
    }

    // Edge BC
    let va = d3 * d6 - d5 * d4;
    if va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0 {
        let num = d4 - d3;
        let denom = num + (d5 - d6);
        let w = if denom.abs() > EPSILON {
            num / denom
        } else {
            0.5
        };
        return Point3::from(v1.coords + (v2 - v1) * w);
    }

    // Interior
    let total = va + vb + vc;
    if total.abs() < EPSILON {
        return v0;
    }
    let denom = 1.0 / total;
    let v = vb * denom;
    let w = vc * denom;
    Point3::from(v0.coords + ab * v + ac * w)
}

/// Closest point on line segment `[a, b]` to point `p`.
///
/// Returns the point on the segment nearest to `p`. For degenerate segments
/// (a ≈ b), returns `a`.
#[inline]
#[must_use]
pub fn closest_point_segment(a: Point3<f64>, b: Point3<f64>, p: Point3<f64>) -> Point3<f64> {
    let ab = b - a;
    let ap = p - a;
    let ab_len_sq = ab.dot(&ab);

    if ab_len_sq < EPSILON {
        return a;
    }

    let t = (ap.dot(&ab) / ab_len_sq).clamp(0.0, 1.0);
    Point3::from(a.coords + ab * t)
}

/// Closest points between two line segments `[p1, q1]` and `[p2, q2]`.
///
/// Returns `(point_on_seg1, point_on_seg2)`. Uses the standard
/// segment–segment distance algorithm with proper clamping for all
/// degenerate cases (zero-length segments, parallel segments).
#[must_use]
#[allow(clippy::many_single_char_names)]
pub fn closest_points_segments(
    p1: Point3<f64>,
    q1: Point3<f64>,
    p2: Point3<f64>,
    q2: Point3<f64>,
) -> (Point3<f64>, Point3<f64>) {
    let d1 = q1 - p1;
    let d2 = q2 - p2;
    let r = p1 - p2;

    let a = d1.dot(&d1);
    let e = d2.dot(&d2);
    let f = d2.dot(&r);

    // Both segments degenerate
    if a < EPSILON && e < EPSILON {
        return (p1, p2);
    }
    // First segment degenerate
    if a < EPSILON {
        let t = (f / e).clamp(0.0, 1.0);
        return (p1, Point3::from(p2.coords + d2 * t));
    }
    // Second segment degenerate
    if e < EPSILON {
        let s = (-d1.dot(&r) / a).clamp(0.0, 1.0);
        return (Point3::from(p1.coords + d1 * s), p2);
    }

    let b = d1.dot(&d2);
    let c = d1.dot(&r);
    // Determinant of the 2×2 system: denom = a*e − b²
    #[allow(clippy::suspicious_operation_groupings)]
    let denom = a * e - b * b;

    let (mut s, mut t) = if denom.abs() < EPSILON {
        // Parallel segments
        (0.0, f / e)
    } else {
        let s_val = (b * f - c * e) / denom;
        let t_val = (b * s_val + f) / e;
        (s_val, t_val)
    };

    // Clamp and recompute
    if s < 0.0 {
        s = 0.0;
        t = (f / e).clamp(0.0, 1.0);
    } else if s > 1.0 {
        s = 1.0;
        t = ((b + f) / e).clamp(0.0, 1.0);
    }

    if t < 0.0 {
        t = 0.0;
        s = (-c / a).clamp(0.0, 1.0);
    } else if t > 1.0 {
        t = 1.0;
        s = ((b - c) / a).clamp(0.0, 1.0);
    }

    (
        Point3::from(p1.coords + d1 * s),
        Point3::from(p2.coords + d2 * t),
    )
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

    // ── closest_point (Shape dispatch) ────────────────────────────────

    #[test]
    fn cp_sphere_outside() {
        let shape = Shape::sphere(1.0);
        let p = Point3::new(3.0, 0.0, 0.0);
        let cp = closest_point(&shape, &p);
        assert_relative_eq!(cp, Point3::new(1.0, 0.0, 0.0), epsilon = 1e-10);
    }

    #[test]
    fn cp_sphere_inside() {
        let shape = Shape::sphere(2.0);
        let p = Point3::new(0.5, 0.0, 0.0);
        let cp = closest_point(&shape, &p);
        // Should project to surface
        assert_relative_eq!(cp, Point3::new(2.0, 0.0, 0.0), epsilon = 1e-10);
    }

    #[test]
    fn cp_sphere_at_center() {
        let shape = Shape::sphere(1.0);
        let p = Point3::origin();
        let cp = closest_point(&shape, &p);
        // Should return some point on the surface
        assert_relative_eq!(cp.coords.norm(), 1.0, epsilon = 1e-10);
    }

    #[test]
    fn cp_plane() {
        let shape = Shape::ground_plane(0.0);
        let p = Point3::new(3.0, 4.0, 5.0);
        let cp = closest_point(&shape, &p);
        assert_relative_eq!(cp, Point3::new(3.0, 4.0, 0.0), epsilon = 1e-10);
    }

    #[test]
    fn cp_box_outside() {
        let shape = Shape::box_shape(Vector3::new(1.0, 1.0, 1.0));
        let p = Point3::new(3.0, 0.0, 0.0);
        let cp = closest_point(&shape, &p);
        assert_relative_eq!(cp, Point3::new(1.0, 0.0, 0.0), epsilon = 1e-10);
    }

    #[test]
    fn cp_box_inside() {
        let shape = Shape::box_shape(Vector3::new(2.0, 2.0, 2.0));
        let p = Point3::new(0.5, 0.0, 0.0);
        let cp = closest_point(&shape, &p);
        // Should project to nearest face (+X face at x=2)
        assert_relative_eq!(cp.x, 2.0, epsilon = 1e-10);
        assert_relative_eq!(cp.y, 0.0, epsilon = 1e-10);
        assert_relative_eq!(cp.z, 0.0, epsilon = 1e-10);
    }

    #[test]
    fn cp_box_corner() {
        let shape = Shape::box_shape(Vector3::new(1.0, 1.0, 1.0));
        let p = Point3::new(5.0, 5.0, 5.0);
        let cp = closest_point(&shape, &p);
        assert_relative_eq!(cp, Point3::new(1.0, 1.0, 1.0), epsilon = 1e-10);
    }

    #[test]
    fn cp_capsule_body() {
        let shape = Shape::capsule(2.0, 0.5);
        let p = Point3::new(3.0, 0.0, 0.0);
        let cp = closest_point(&shape, &p);
        assert_relative_eq!(cp, Point3::new(0.5, 0.0, 0.0), epsilon = 1e-10);
    }

    #[test]
    fn cp_capsule_cap() {
        let shape = Shape::capsule(2.0, 0.5);
        let p = Point3::new(0.0, 0.0, 5.0);
        let cp = closest_point(&shape, &p);
        // Closest to top cap center at (0,0,2), radius 0.5 → (0,0,2.5)
        assert_relative_eq!(cp, Point3::new(0.0, 0.0, 2.5), epsilon = 1e-10);
    }

    #[test]
    fn cp_cylinder_barrel() {
        let shape = Shape::cylinder(1.0, 0.5);
        let p = Point3::new(3.0, 0.0, 0.0);
        let cp = closest_point(&shape, &p);
        assert_relative_eq!(cp, Point3::new(0.5, 0.0, 0.0), epsilon = 1e-10);
    }

    #[test]
    fn cp_cylinder_cap() {
        let shape = Shape::cylinder(1.0, 0.5);
        let p = Point3::new(0.0, 0.0, 5.0);
        let cp = closest_point(&shape, &p);
        assert_relative_eq!(cp, Point3::new(0.0, 0.0, 1.0), epsilon = 1e-10);
    }

    #[test]
    fn cp_ellipsoid() {
        let shape = Shape::ellipsoid(Vector3::new(2.0, 1.0, 1.0));
        let p = Point3::new(10.0, 0.0, 0.0);
        let cp = closest_point(&shape, &p);
        // Should be near (2, 0, 0) — the +X pole
        assert_relative_eq!(cp.x, 2.0, epsilon = 0.1);
        assert_relative_eq!(cp.y, 0.0, epsilon = 0.1);
        assert_relative_eq!(cp.z, 0.0, epsilon = 0.1);
    }

    #[test]
    fn cp_heightfield() {
        let data = crate::HeightFieldData::flat(10, 10, 1.0, 0.0);
        let shape = Shape::height_field(Arc::new(data));
        let p = Point3::new(5.0, 5.0, 10.0);
        let cp = closest_point(&shape, &p);
        assert_relative_eq!(cp.x, 5.0, epsilon = 1e-10);
        assert_relative_eq!(cp.y, 5.0, epsilon = 1e-10);
        assert_relative_eq!(cp.z, 0.0, epsilon = 1e-10);
    }

    #[test]
    fn cp_sdf() {
        let data = crate::SdfGrid::sphere(Point3::origin(), 1.0, 32, 1.0);
        let shape = Shape::sdf(Arc::new(data));
        let p = Point3::new(0.5, 0.0, 0.0);
        let cp = closest_point(&shape, &p);
        // Should converge near the surface at (1, 0, 0)
        assert_relative_eq!(cp.coords.norm(), 1.0, epsilon = 0.15);
    }

    #[test]
    fn cp_triangle_mesh() {
        let mesh = crate::IndexedMesh::from_parts(
            vec![
                Point3::new(-1.0, -1.0, 0.0),
                Point3::new(1.0, -1.0, 0.0),
                Point3::new(0.0, 1.0, 0.0),
            ],
            vec![[0, 1, 2]],
        );
        let bvh = crate::bvh_from_mesh(&mesh);
        let shape = Shape::triangle_mesh(Arc::new(mesh), Arc::new(bvh));
        let p = Point3::new(0.0, 0.0, 5.0);
        let cp = closest_point(&shape, &p);
        assert_relative_eq!(cp, Point3::new(0.0, 0.0, 0.0), epsilon = 1e-10);
    }

    // ── closest_point_on_triangle ─────────────────────────────────────

    #[test]
    fn triangle_point_on_face() {
        let v0 = Point3::new(0.0, 0.0, 0.0);
        let v1 = Point3::new(4.0, 0.0, 0.0);
        let v2 = Point3::new(0.0, 4.0, 0.0);

        // Point directly above centroid
        let p = Point3::new(1.0, 1.0, 5.0);
        let c = closest_point_on_triangle(v0, v1, v2, p);
        assert_relative_eq!(c.x, 1.0, epsilon = 1e-10);
        assert_relative_eq!(c.y, 1.0, epsilon = 1e-10);
        assert_relative_eq!(c.z, 0.0, epsilon = 1e-10);
    }

    #[test]
    fn triangle_point_at_vertex() {
        let v0 = Point3::new(0.0, 0.0, 0.0);
        let v1 = Point3::new(1.0, 0.0, 0.0);
        let v2 = Point3::new(0.0, 1.0, 0.0);

        // Nearest to v0
        let p = Point3::new(-1.0, -1.0, 0.0);
        let c = closest_point_on_triangle(v0, v1, v2, p);
        assert_relative_eq!(c, v0, epsilon = 1e-10);

        // Nearest to v1
        let p = Point3::new(2.0, -1.0, 0.0);
        let c = closest_point_on_triangle(v0, v1, v2, p);
        assert_relative_eq!(c, v1, epsilon = 1e-10);

        // Nearest to v2
        let p = Point3::new(-1.0, 2.0, 0.0);
        let c = closest_point_on_triangle(v0, v1, v2, p);
        assert_relative_eq!(c, v2, epsilon = 1e-10);
    }

    #[test]
    fn triangle_point_on_edge() {
        let v0 = Point3::new(0.0, 0.0, 0.0);
        let v1 = Point3::new(2.0, 0.0, 0.0);
        let v2 = Point3::new(0.0, 2.0, 0.0);

        // Nearest to edge AB (below triangle)
        let p = Point3::new(1.0, -1.0, 0.0);
        let c = closest_point_on_triangle(v0, v1, v2, p);
        assert_relative_eq!(c.x, 1.0, epsilon = 1e-10);
        assert_relative_eq!(c.y, 0.0, epsilon = 1e-10);
    }

    #[test]
    fn triangle_degenerate_zero_area() {
        // Collinear vertices
        let v0 = Point3::new(0.0, 0.0, 0.0);
        let v1 = Point3::new(1.0, 0.0, 0.0);
        let v2 = Point3::new(2.0, 0.0, 0.0);

        let p = Point3::new(0.5, 1.0, 0.0);
        let c = closest_point_on_triangle(v0, v1, v2, p);
        // Should return some valid point on/near the degenerate triangle
        assert!(c.x.is_finite() && c.y.is_finite() && c.z.is_finite());
    }

    // ── closest_point_segment ─────────────────────────────────────────

    #[test]
    fn segment_midpoint() {
        let a = Point3::new(0.0, 0.0, 0.0);
        let b = Point3::new(2.0, 0.0, 0.0);
        let p = Point3::new(1.0, 1.0, 0.0);
        let c = closest_point_segment(a, b, p);
        assert_relative_eq!(c, Point3::new(1.0, 0.0, 0.0), epsilon = 1e-10);
    }

    #[test]
    fn segment_clamp_start() {
        let a = Point3::new(0.0, 0.0, 0.0);
        let b = Point3::new(2.0, 0.0, 0.0);
        let p = Point3::new(-1.0, 0.0, 0.0);
        let c = closest_point_segment(a, b, p);
        assert_relative_eq!(c, a, epsilon = 1e-10);
    }

    #[test]
    fn segment_clamp_end() {
        let a = Point3::new(0.0, 0.0, 0.0);
        let b = Point3::new(2.0, 0.0, 0.0);
        let p = Point3::new(5.0, 0.0, 0.0);
        let c = closest_point_segment(a, b, p);
        assert_relative_eq!(c, b, epsilon = 1e-10);
    }

    #[test]
    fn segment_degenerate_zero_length() {
        let a = Point3::new(1.0, 2.0, 3.0);
        let b = a;
        let p = Point3::new(5.0, 5.0, 5.0);
        let c = closest_point_segment(a, b, p);
        assert_relative_eq!(c, a, epsilon = 1e-10);
    }

    // ── closest_points_segments ───────────────────────────────────────

    #[test]
    fn segments_crossing() {
        // Two segments crossing at origin
        let p1 = Point3::new(-1.0, 0.0, 0.0);
        let q1 = Point3::new(1.0, 0.0, 0.0);
        let p2 = Point3::new(0.0, -1.0, 0.0);
        let q2 = Point3::new(0.0, 1.0, 0.0);

        let (c1, c2) = closest_points_segments(p1, q1, p2, q2);
        assert_relative_eq!(c1, Point3::origin(), epsilon = 1e-10);
        assert_relative_eq!(c2, Point3::origin(), epsilon = 1e-10);
    }

    #[test]
    fn segments_parallel() {
        let p1 = Point3::new(0.0, 0.0, 0.0);
        let q1 = Point3::new(2.0, 0.0, 0.0);
        let p2 = Point3::new(0.0, 1.0, 0.0);
        let q2 = Point3::new(2.0, 1.0, 0.0);

        let (c1, c2) = closest_points_segments(p1, q1, p2, q2);
        // Distance should be 1.0 (in Y)
        assert_relative_eq!((c1 - c2).norm(), 1.0, epsilon = 1e-6);
    }

    #[test]
    fn segments_skew() {
        // Z-axis segment and X-axis segment offset in Y
        let p1 = Point3::new(0.0, 0.0, -1.0);
        let q1 = Point3::new(0.0, 0.0, 1.0);
        let p2 = Point3::new(-1.0, 1.0, 0.0);
        let q2 = Point3::new(1.0, 1.0, 0.0);

        let (c1, c2) = closest_points_segments(p1, q1, p2, q2);
        assert_relative_eq!(c1, Point3::new(0.0, 0.0, 0.0), epsilon = 1e-10);
        assert_relative_eq!(c2, Point3::new(0.0, 1.0, 0.0), epsilon = 1e-10);
    }

    #[test]
    fn segments_both_degenerate() {
        let p1 = Point3::new(1.0, 0.0, 0.0);
        let q1 = p1;
        let p2 = Point3::new(0.0, 1.0, 0.0);
        let q2 = p2;

        let (c1, c2) = closest_points_segments(p1, q1, p2, q2);
        assert_relative_eq!(c1, p1, epsilon = 1e-10);
        assert_relative_eq!(c2, p2, epsilon = 1e-10);
    }

    #[test]
    fn segments_one_degenerate() {
        let p1 = Point3::new(0.0, 0.0, 0.0);
        let q1 = p1;
        let p2 = Point3::new(-1.0, 1.0, 0.0);
        let q2 = Point3::new(1.0, 1.0, 0.0);

        let (c1, c2) = closest_points_segments(p1, q1, p2, q2);
        assert_relative_eq!(c1, p1, epsilon = 1e-10);
        assert_relative_eq!(c2, Point3::new(0.0, 1.0, 0.0), epsilon = 1e-10);
    }
}
