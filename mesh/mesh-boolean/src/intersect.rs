//! Triangle-triangle and edge-triangle intersection tests.
//!
//! This module provides geometric intersection tests used for detecting
//! where two meshes overlap during boolean operations.

// Allow this pattern - it's correct for barycentric coordinate determinant calculation
#![allow(clippy::suspicious_operation_groupings)]

use mesh_types::{Point3, Vector3};

/// Result of an edge-triangle intersection test.
#[derive(Debug, Clone)]
pub struct EdgeTriangleIntersection {
    /// Parameter t along the edge (0.0 = start, 1.0 = end).
    pub t: f64,
    /// The intersection point in 3D space.
    pub point: Point3<f64>,
    /// Barycentric coordinates (u, v) on the triangle.
    /// The third coordinate w = 1 - u - v.
    pub barycentric: (f64, f64),
}

/// Result of a triangle-triangle intersection test.
#[derive(Debug, Clone)]
pub struct TriangleTriangleIntersection {
    /// Start point of intersection segment (if any).
    pub start: Point3<f64>,
    /// End point of intersection segment (if any).
    pub end: Point3<f64>,
}

/// Ray-triangle intersection using Möller-Trumbore algorithm.
///
/// Tests if a ray intersects a triangle and returns the intersection parameter.
///
/// # Arguments
///
/// * `origin` - Ray origin
/// * `direction` - Ray direction (does not need to be normalized)
/// * `v0`, `v1`, `v2` - Triangle vertices
/// * `epsilon` - Tolerance for parallel ray detection
///
/// # Returns
///
/// `Some(t)` where t is the ray parameter at intersection, or `None` if no intersection.
/// The intersection point is `origin + t * direction`.
#[must_use]
pub fn ray_triangle_intersect(
    origin: &Point3<f64>,
    direction: &Vector3<f64>,
    v0: &Point3<f64>,
    v1: &Point3<f64>,
    v2: &Point3<f64>,
    epsilon: f64,
) -> Option<f64> {
    let edge1 = v1 - v0;
    let edge2 = v2 - v0;
    let h = direction.cross(&edge2);
    let a = edge1.dot(&h);

    // Ray is parallel to triangle
    if a.abs() < epsilon {
        return None;
    }

    let f = 1.0 / a;
    let s = origin - v0;
    let u = f * s.dot(&h);

    // Intersection is outside triangle
    if !(0.0..=1.0).contains(&u) {
        return None;
    }

    let q = s.cross(&edge1);
    let v = f * direction.dot(&q);

    // Intersection is outside triangle
    if v < 0.0 || u + v > 1.0 {
        return None;
    }

    let t = f * edge2.dot(&q);

    // Intersection is behind ray origin
    if t > epsilon {
        Some(t)
    } else {
        None
    }
}

/// Test if an edge intersects a triangle.
///
/// # Arguments
///
/// * `e0`, `e1` - Edge endpoints
/// * `v0`, `v1`, `v2` - Triangle vertices
/// * `epsilon` - Tolerance for intersection detection
///
/// # Returns
///
/// `Some(EdgeTriangleIntersection)` if the edge intersects the triangle interior,
/// or `None` if there is no intersection.
#[must_use]
pub fn edge_triangle_intersect(
    e0: &Point3<f64>,
    e1: &Point3<f64>,
    v0: &Point3<f64>,
    v1: &Point3<f64>,
    v2: &Point3<f64>,
    epsilon: f64,
) -> Option<EdgeTriangleIntersection> {
    let direction = e1 - e0;
    let edge_length_sq = direction.norm_squared();

    // Degenerate edge
    if edge_length_sq < epsilon * epsilon {
        return None;
    }

    let edge1 = v1 - v0;
    let edge2 = v2 - v0;
    let h = direction.cross(&edge2);
    let a = edge1.dot(&h);

    // Edge is parallel to triangle
    if a.abs() < epsilon {
        return None;
    }

    let f = 1.0 / a;
    let s = e0 - v0;
    let u = f * s.dot(&h);

    // Intersection is outside triangle
    if !(0.0..=1.0).contains(&u) {
        return None;
    }

    let q = s.cross(&edge1);
    let v = f * direction.dot(&q);

    // Intersection is outside triangle
    if v < 0.0 || u + v > 1.0 {
        return None;
    }

    let t = f * edge2.dot(&q);

    // Intersection must be within edge bounds [0, 1]
    if t < -epsilon || t > 1.0 + epsilon {
        return None;
    }

    // Clamp t to [0, 1]
    let t_clamped = t.clamp(0.0, 1.0);
    let point = Point3::from(e0.coords + direction * t_clamped);

    Some(EdgeTriangleIntersection {
        t: t_clamped,
        point,
        barycentric: (u, v),
    })
}

/// Test if two triangles intersect.
///
/// Uses edge-triangle intersection tests on all 6 edges (3 from each triangle).
///
/// # Arguments
///
/// * `a0`, `a1`, `a2` - First triangle vertices
/// * `b0`, `b1`, `b2` - Second triangle vertices
/// * `epsilon` - Tolerance for intersection detection
///
/// # Returns
///
/// `true` if the triangles intersect, `false` otherwise.
#[must_use]
pub fn triangles_intersect(
    a0: &Point3<f64>,
    a1: &Point3<f64>,
    a2: &Point3<f64>,
    b0: &Point3<f64>,
    b1: &Point3<f64>,
    b2: &Point3<f64>,
    epsilon: f64,
) -> bool {
    // Check if any edge of A intersects triangle B
    let edges_a = [(a0, a1), (a1, a2), (a2, a0)];
    for (e0, e1) in &edges_a {
        if edge_triangle_intersect(e0, e1, b0, b1, b2, epsilon).is_some() {
            return true;
        }
    }

    // Check if any edge of B intersects triangle A
    let edges_b = [(b0, b1), (b1, b2), (b2, b0)];
    for (e0, e1) in &edges_b {
        if edge_triangle_intersect(e0, e1, a0, a1, a2, epsilon).is_some() {
            return true;
        }
    }

    false
}

/// Compute the intersection segment between two triangles.
///
/// If the triangles intersect, returns the line segment where they meet.
///
/// # Arguments
///
/// * `a0`, `a1`, `a2` - First triangle vertices
/// * `b0`, `b1`, `b2` - Second triangle vertices
/// * `epsilon` - Tolerance for intersection detection
///
/// # Returns
///
/// `Some(TriangleTriangleIntersection)` containing the intersection segment,
/// or `None` if the triangles don't intersect.
#[must_use]
pub fn compute_triangle_intersection(
    a0: &Point3<f64>,
    a1: &Point3<f64>,
    a2: &Point3<f64>,
    b0: &Point3<f64>,
    b1: &Point3<f64>,
    b2: &Point3<f64>,
    epsilon: f64,
) -> Option<TriangleTriangleIntersection> {
    // Collect all intersection points
    let mut points: Vec<Point3<f64>> = Vec::with_capacity(6);

    // Check edges of A against triangle B
    let edges_a = [(a0, a1), (a1, a2), (a2, a0)];
    for (e0, e1) in &edges_a {
        if let Some(intersection) = edge_triangle_intersect(e0, e1, b0, b1, b2, epsilon) {
            points.push(intersection.point);
        }
    }

    // Check edges of B against triangle A
    let edges_b = [(b0, b1), (b1, b2), (b2, b0)];
    for (e0, e1) in &edges_b {
        if let Some(intersection) = edge_triangle_intersect(e0, e1, a0, a1, a2, epsilon) {
            points.push(intersection.point);
        }
    }

    // Need at least 2 points for a segment
    if points.len() < 2 {
        return None;
    }

    // Remove duplicate points
    let mut unique_points: Vec<Point3<f64>> = Vec::with_capacity(points.len());
    for p in points {
        let is_duplicate = unique_points
            .iter()
            .any(|q| (p - q).norm_squared() < epsilon * epsilon);
        if !is_duplicate {
            unique_points.push(p);
        }
    }

    if unique_points.len() < 2 {
        return None;
    }

    // Find the two points that are furthest apart (the segment endpoints)
    let mut max_dist_sq = 0.0;
    let mut start_idx = 0;
    let mut end_idx = 1;

    for (i, p1) in unique_points.iter().enumerate() {
        for (j, p2) in unique_points.iter().enumerate().skip(i + 1) {
            let dist_sq = (p2 - p1).norm_squared();
            if dist_sq > max_dist_sq {
                max_dist_sq = dist_sq;
                start_idx = i;
                end_idx = j;
            }
        }
    }

    Some(TriangleTriangleIntersection {
        start: unique_points[start_idx],
        end: unique_points[end_idx],
    })
}

/// Compute the signed distance from a point to a plane.
///
/// # Arguments
///
/// * `point` - The point to test
/// * `plane_point` - A point on the plane
/// * `plane_normal` - The plane normal (should be unit length)
///
/// # Returns
///
/// Positive distance if point is on the same side as the normal,
/// negative if on the opposite side.
#[inline]
#[must_use]
pub fn signed_distance_to_plane(
    point: &Point3<f64>,
    plane_point: &Point3<f64>,
    plane_normal: &Vector3<f64>,
) -> f64 {
    (point - plane_point).dot(plane_normal)
}

/// Compute the normal of a triangle.
///
/// # Arguments
///
/// * `v0`, `v1`, `v2` - Triangle vertices in counter-clockwise order
///
/// # Returns
///
/// The triangle normal (not normalized) computed as (v1-v0) × (v2-v0).
#[inline]
#[must_use]
pub fn triangle_normal(v0: &Point3<f64>, v1: &Point3<f64>, v2: &Point3<f64>) -> Vector3<f64> {
    let edge1 = v1 - v0;
    let edge2 = v2 - v0;
    edge1.cross(&edge2)
}

/// Compute the unit normal of a triangle.
///
/// # Arguments
///
/// * `v0`, `v1`, `v2` - Triangle vertices in counter-clockwise order
/// * `epsilon` - Tolerance for degenerate triangle detection
///
/// # Returns
///
/// `Some(normal)` if the triangle has non-zero area, `None` if degenerate.
#[must_use]
pub fn triangle_unit_normal(
    v0: &Point3<f64>,
    v1: &Point3<f64>,
    v2: &Point3<f64>,
    epsilon: f64,
) -> Option<Vector3<f64>> {
    let normal = triangle_normal(v0, v1, v2);
    let len = normal.norm();

    if len < epsilon {
        None
    } else {
        Some(normal / len)
    }
}

/// Compute the centroid of a triangle.
///
/// # Arguments
///
/// * `v0`, `v1`, `v2` - Triangle vertices
///
/// # Returns
///
/// The centroid (average of the three vertices).
#[inline]
#[must_use]
pub fn triangle_centroid(v0: &Point3<f64>, v1: &Point3<f64>, v2: &Point3<f64>) -> Point3<f64> {
    Point3::from((v0.coords + v1.coords + v2.coords) / 3.0)
}

/// Point-in-triangle test using barycentric coordinates.
///
/// # Arguments
///
/// * `point` - The point to test (assumed to be in the triangle's plane)
/// * `v0`, `v1`, `v2` - Triangle vertices
/// * `epsilon` - Tolerance for edge cases
///
/// # Returns
///
/// `true` if the point is inside or on the boundary of the triangle.
#[must_use]
pub fn point_in_triangle(
    point: &Point3<f64>,
    v0: &Point3<f64>,
    v1: &Point3<f64>,
    v2: &Point3<f64>,
    epsilon: f64,
) -> bool {
    let v0v1 = v1 - v0;
    let v0v2 = v2 - v0;
    let v0p = point - v0;

    let dot00 = v0v2.dot(&v0v2);
    let dot01 = v0v2.dot(&v0v1);
    let dot02 = v0v2.dot(&v0p);
    let dot11 = v0v1.dot(&v0v1);
    let dot12 = v0v1.dot(&v0p);

    let inv_denom = dot00 * dot11 - dot01 * dot01;
    if inv_denom.abs() < epsilon {
        return false; // Degenerate triangle
    }
    let inv_denom = 1.0 / inv_denom;

    let u = (dot11 * dot02 - dot01 * dot12) * inv_denom;
    let v = (dot00 * dot12 - dot01 * dot02) * inv_denom;

    // Check if point is in triangle
    u >= -epsilon && v >= -epsilon && u + v <= 1.0 + epsilon
}

/// Compute the closest point on a triangle to a given point.
///
/// # Arguments
///
/// * `point` - The query point
/// * `v0`, `v1`, `v2` - Triangle vertices
///
/// # Returns
///
/// The closest point on the triangle surface to the query point.
#[must_use]
pub fn closest_point_on_triangle(
    point: &Point3<f64>,
    v0: &Point3<f64>,
    v1: &Point3<f64>,
    v2: &Point3<f64>,
) -> Point3<f64> {
    let ab = v1 - v0;
    let ac = v2 - v0;
    let ap = point - v0;

    let d1 = ab.dot(&ap);
    let d2 = ac.dot(&ap);

    // Check if P is in vertex region outside A
    if d1 <= 0.0 && d2 <= 0.0 {
        return *v0;
    }

    let bp = point - v1;
    let d3 = ab.dot(&bp);
    let d4 = ac.dot(&bp);

    // Check if P is in vertex region outside B
    if d3 >= 0.0 && d4 <= d3 {
        return *v1;
    }

    // Check if P is in edge region of AB
    let vc = d1 * d4 - d3 * d2;
    if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
        let v = d1 / (d1 - d3);
        return Point3::from(v0.coords + ab * v);
    }

    let cp = point - v2;
    let d5 = ab.dot(&cp);
    let d6 = ac.dot(&cp);

    // Check if P is in vertex region outside C
    if d6 >= 0.0 && d5 <= d6 {
        return *v2;
    }

    // Check if P is in edge region of AC
    let vb = d5 * d2 - d1 * d6;
    if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
        let w = d2 / (d2 - d6);
        return Point3::from(v0.coords + ac * w);
    }

    // Check if P is in edge region of BC
    let va = d3 * d6 - d5 * d4;
    if va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0 {
        let w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return Point3::from(v1.coords + (v2 - v1) * w);
    }

    // P is inside face region
    let denom = 1.0 / (va + vb + vc);
    let v = vb * denom;
    let w = vc * denom;
    Point3::from(v0.coords + ab * v + ac * w)
}

#[cfg(test)]
mod tests {
    use super::*;

    const EPSILON: f64 = 1e-10;

    #[test]
    fn test_ray_triangle_intersect_hit() {
        let origin = Point3::new(0.5, 0.5, -1.0);
        let direction = Vector3::new(0.0, 0.0, 1.0);

        let v0 = Point3::new(0.0, 0.0, 0.0);
        let v1 = Point3::new(1.0, 0.0, 0.0);
        let v2 = Point3::new(0.5, 1.0, 0.0);

        let result = ray_triangle_intersect(&origin, &direction, &v0, &v1, &v2, EPSILON);
        assert!(result.is_some());
        let t = result.unwrap();
        assert!((t - 1.0).abs() < 1e-6);
    }

    #[test]
    fn test_ray_triangle_intersect_miss() {
        let origin = Point3::new(2.0, 2.0, -1.0); // Outside triangle
        let direction = Vector3::new(0.0, 0.0, 1.0);

        let v0 = Point3::new(0.0, 0.0, 0.0);
        let v1 = Point3::new(1.0, 0.0, 0.0);
        let v2 = Point3::new(0.5, 1.0, 0.0);

        let result = ray_triangle_intersect(&origin, &direction, &v0, &v1, &v2, EPSILON);
        assert!(result.is_none());
    }

    #[test]
    fn test_ray_triangle_intersect_parallel() {
        let origin = Point3::new(0.5, 0.5, 0.0);
        let direction = Vector3::new(1.0, 0.0, 0.0); // Parallel to triangle

        let v0 = Point3::new(0.0, 0.0, 0.0);
        let v1 = Point3::new(1.0, 0.0, 0.0);
        let v2 = Point3::new(0.5, 1.0, 0.0);

        let result = ray_triangle_intersect(&origin, &direction, &v0, &v1, &v2, EPSILON);
        assert!(result.is_none());
    }

    #[test]
    fn test_edge_triangle_intersect_through() {
        let e0 = Point3::new(0.5, 0.5, -1.0);
        let e1 = Point3::new(0.5, 0.5, 1.0);

        let v0 = Point3::new(0.0, 0.0, 0.0);
        let v1 = Point3::new(1.0, 0.0, 0.0);
        let v2 = Point3::new(0.5, 1.0, 0.0);

        let result = edge_triangle_intersect(&e0, &e1, &v0, &v1, &v2, EPSILON);
        assert!(result.is_some());

        let intersection = result.unwrap();
        assert!((intersection.t - 0.5).abs() < 1e-6);
        assert!((intersection.point.z).abs() < 1e-6);
    }

    #[test]
    fn test_edge_triangle_intersect_miss() {
        let e0 = Point3::new(2.0, 2.0, -1.0);
        let e1 = Point3::new(2.0, 2.0, 1.0);

        let v0 = Point3::new(0.0, 0.0, 0.0);
        let v1 = Point3::new(1.0, 0.0, 0.0);
        let v2 = Point3::new(0.5, 1.0, 0.0);

        let result = edge_triangle_intersect(&e0, &e1, &v0, &v1, &v2, EPSILON);
        assert!(result.is_none());
    }

    #[test]
    fn test_triangles_intersect_yes() {
        // Two triangles that intersect
        let a0 = Point3::new(0.0, 0.0, 0.0);
        let a1 = Point3::new(2.0, 0.0, 0.0);
        let a2 = Point3::new(1.0, 2.0, 0.0);

        let b0 = Point3::new(1.0, 0.5, -1.0);
        let b1 = Point3::new(1.0, 0.5, 1.0);
        let b2 = Point3::new(1.0, 1.5, 0.0);

        assert!(triangles_intersect(&a0, &a1, &a2, &b0, &b1, &b2, EPSILON));
    }

    #[test]
    fn test_triangles_intersect_no() {
        // Two triangles that don't intersect
        let a0 = Point3::new(0.0, 0.0, 0.0);
        let a1 = Point3::new(1.0, 0.0, 0.0);
        let a2 = Point3::new(0.5, 1.0, 0.0);

        let b0 = Point3::new(0.0, 0.0, 5.0);
        let b1 = Point3::new(1.0, 0.0, 5.0);
        let b2 = Point3::new(0.5, 1.0, 5.0);

        assert!(!triangles_intersect(&a0, &a1, &a2, &b0, &b1, &b2, EPSILON));
    }

    #[test]
    fn test_triangle_normal() {
        let v0 = Point3::new(0.0, 0.0, 0.0);
        let v1 = Point3::new(1.0, 0.0, 0.0);
        let v2 = Point3::new(0.0, 1.0, 0.0);

        let normal = triangle_normal(&v0, &v1, &v2);
        // Cross product of (1,0,0) and (0,1,0) is (0,0,1)
        assert!((normal.z - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_triangle_unit_normal() {
        let v0 = Point3::new(0.0, 0.0, 0.0);
        let v1 = Point3::new(2.0, 0.0, 0.0);
        let v2 = Point3::new(0.0, 2.0, 0.0);

        let normal = triangle_unit_normal(&v0, &v1, &v2, EPSILON).unwrap();
        assert!((normal.norm() - 1.0).abs() < 1e-10);
        assert!((normal.z - 1.0).abs() < 1e-10);
    }

    #[test]
    fn test_triangle_unit_normal_degenerate() {
        let v0 = Point3::new(0.0, 0.0, 0.0);
        let v1 = Point3::new(1.0, 0.0, 0.0);
        let v2 = Point3::new(2.0, 0.0, 0.0); // Collinear

        let normal = triangle_unit_normal(&v0, &v1, &v2, EPSILON);
        assert!(normal.is_none());
    }

    #[test]
    fn test_triangle_centroid() {
        let v0 = Point3::new(0.0, 0.0, 0.0);
        let v1 = Point3::new(3.0, 0.0, 0.0);
        let v2 = Point3::new(0.0, 3.0, 0.0);

        let centroid = triangle_centroid(&v0, &v1, &v2);
        assert!((centroid.x - 1.0).abs() < 1e-10);
        assert!((centroid.y - 1.0).abs() < 1e-10);
        assert!((centroid.z - 0.0).abs() < 1e-10);
    }

    #[test]
    fn test_point_in_triangle_inside() {
        let v0 = Point3::new(0.0, 0.0, 0.0);
        let v1 = Point3::new(1.0, 0.0, 0.0);
        let v2 = Point3::new(0.5, 1.0, 0.0);

        let point = Point3::new(0.5, 0.3, 0.0);
        assert!(point_in_triangle(&point, &v0, &v1, &v2, EPSILON));
    }

    #[test]
    fn test_point_in_triangle_outside() {
        let v0 = Point3::new(0.0, 0.0, 0.0);
        let v1 = Point3::new(1.0, 0.0, 0.0);
        let v2 = Point3::new(0.5, 1.0, 0.0);

        let point = Point3::new(2.0, 2.0, 0.0);
        assert!(!point_in_triangle(&point, &v0, &v1, &v2, EPSILON));
    }

    #[test]
    fn test_point_in_triangle_on_edge() {
        let v0 = Point3::new(0.0, 0.0, 0.0);
        let v1 = Point3::new(1.0, 0.0, 0.0);
        let v2 = Point3::new(0.5, 1.0, 0.0);

        let point = Point3::new(0.5, 0.0, 0.0); // On edge v0-v1
        assert!(point_in_triangle(&point, &v0, &v1, &v2, 1e-6));
    }

    #[test]
    fn test_signed_distance_to_plane() {
        let plane_point = Point3::new(0.0, 0.0, 0.0);
        let plane_normal = Vector3::new(0.0, 0.0, 1.0);

        let above = Point3::new(0.0, 0.0, 5.0);
        let below = Point3::new(0.0, 0.0, -3.0);
        let on_plane = Point3::new(1.0, 2.0, 0.0);

        assert!((signed_distance_to_plane(&above, &plane_point, &plane_normal) - 5.0).abs() < 1e-10);
        assert!(
            (signed_distance_to_plane(&below, &plane_point, &plane_normal) - (-3.0)).abs() < 1e-10
        );
        assert!(signed_distance_to_plane(&on_plane, &plane_point, &plane_normal).abs() < 1e-10);
    }

    #[test]
    fn test_closest_point_on_triangle_inside() {
        let v0 = Point3::new(0.0, 0.0, 0.0);
        let v1 = Point3::new(1.0, 0.0, 0.0);
        let v2 = Point3::new(0.5, 1.0, 0.0);

        let point = Point3::new(0.5, 0.3, 1.0);
        let closest = closest_point_on_triangle(&point, &v0, &v1, &v2);

        // Should project to (0.5, 0.3, 0.0)
        assert!((closest.x - 0.5).abs() < 1e-6);
        assert!((closest.y - 0.3).abs() < 1e-6);
        assert!(closest.z.abs() < 1e-6);
    }

    #[test]
    fn test_closest_point_on_triangle_vertex() {
        let v0 = Point3::new(0.0, 0.0, 0.0);
        let v1 = Point3::new(1.0, 0.0, 0.0);
        let v2 = Point3::new(0.5, 1.0, 0.0);

        let point = Point3::new(-1.0, -1.0, 0.0);
        let closest = closest_point_on_triangle(&point, &v0, &v1, &v2);

        // Should be vertex v0
        assert!((closest.x - v0.x).abs() < 1e-6);
        assert!((closest.y - v0.y).abs() < 1e-6);
    }

    #[test]
    fn test_compute_triangle_intersection() {
        // Two triangles that intersect
        let a0 = Point3::new(0.0, 0.0, 0.0);
        let a1 = Point3::new(2.0, 0.0, 0.0);
        let a2 = Point3::new(1.0, 2.0, 0.0);

        let b0 = Point3::new(1.0, 0.5, -1.0);
        let b1 = Point3::new(1.0, 0.5, 1.0);
        let b2 = Point3::new(1.0, 1.5, 0.0);

        let result = compute_triangle_intersection(&a0, &a1, &a2, &b0, &b1, &b2, 1e-8);
        assert!(result.is_some());
    }
}
