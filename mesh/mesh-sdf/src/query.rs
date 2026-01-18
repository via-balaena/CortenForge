//! Geometric query utilities for SDF computation.
//!
//! Provides functions for computing closest points on triangles
//! and ray-mesh intersection tests.

use mesh_types::IndexedMesh;
use nalgebra::{Point3, Vector3};

/// Compute the closest point on a triangle to a query point.
///
/// This implements the algorithm from "Real-Time Collision Detection" by Christer Ericson.
///
/// # Arguments
///
/// * `point` - The query point
/// * `v0`, `v1`, `v2` - The triangle vertices
///
/// # Returns
///
/// The closest point on the triangle.
#[must_use]
pub fn closest_point_on_triangle(
    point: Point3<f64>,
    v0: Point3<f64>,
    v1: Point3<f64>,
    v2: Point3<f64>,
) -> Point3<f64> {
    let ab = v1 - v0;
    let ac = v2 - v0;
    let ap = point - v0;

    let d1 = ab.dot(&ap);
    let d2 = ac.dot(&ap);

    // Check if P is in vertex region outside A
    if d1 <= 0.0 && d2 <= 0.0 {
        return v0;
    }

    let bp = point - v1;
    let d3 = ab.dot(&bp);
    let d4 = ac.dot(&bp);

    // Check if P is in vertex region outside B
    if d3 >= 0.0 && d4 <= d3 {
        return v1;
    }

    // Check if P is in edge region of AB
    let vc = d1 * d4 - d3 * d2;
    if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
        let v = d1 / (d1 - d3);
        return v0 + ab * v;
    }

    let cp = point - v2;
    let d5 = ab.dot(&cp);
    let d6 = ac.dot(&cp);

    // Check if P is in vertex region outside C
    if d6 >= 0.0 && d5 <= d6 {
        return v2;
    }

    // Check if P is in edge region of AC
    let vb = d5 * d2 - d1 * d6;
    if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
        let w = d2 / (d2 - d6);
        return v0 + ac * w;
    }

    // Check if P is in edge region of BC
    let va = d3 * d6 - d5 * d4;
    if va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0 {
        let w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return v1 + (v2 - v1) * w;
    }

    // P is inside the face region
    let denom = 1.0 / (va + vb + vc);
    let v = vb * denom;
    let w = vc * denom;

    v0 + ab * v + ac * w
}

/// Test if a ray intersects a triangle.
///
/// Uses the Möller–Trumbore algorithm.
///
/// # Arguments
///
/// * `ray_origin` - Origin of the ray
/// * `ray_dir` - Direction of the ray (should be normalized)
/// * `v0`, `v1`, `v2` - Triangle vertices
///
/// # Returns
///
/// `Some(t)` where `t` is the ray parameter at intersection, or `None` if no intersection.
#[must_use]
pub fn ray_triangle_intersect(
    ray_origin: Point3<f64>,
    ray_dir: Vector3<f64>,
    v0: Point3<f64>,
    v1: Point3<f64>,
    v2: Point3<f64>,
) -> Option<f64> {
    const EPSILON: f64 = 1e-10;

    let edge1 = v1 - v0;
    let edge2 = v2 - v0;

    let h = ray_dir.cross(&edge2);
    let a = edge1.dot(&h);

    // Ray is parallel to triangle
    if a.abs() < EPSILON {
        return None;
    }

    let f = 1.0 / a;
    let s = ray_origin - v0;
    let u = f * s.dot(&h);

    // Check barycentric coordinate u
    if !(0.0..=1.0).contains(&u) {
        return None;
    }

    let q = s.cross(&edge1);
    let v = f * ray_dir.dot(&q);

    // Check barycentric coordinate v
    if v < 0.0 || u + v > 1.0 {
        return None;
    }

    // Compute t (distance along ray)
    let t = f * edge2.dot(&q);

    if t > EPSILON {
        Some(t)
    } else {
        None
    }
}

/// Check if a point is inside a mesh using ray casting.
///
/// Casts a ray in the +X direction and counts intersections.
/// Odd number of intersections means inside.
///
/// # Arguments
///
/// * `point` - The point to test
/// * `mesh` - The mesh
///
/// # Returns
///
/// `true` if the point is inside the mesh.
#[must_use]
pub fn point_in_mesh(point: Point3<f64>, mesh: &IndexedMesh) -> bool {
    let ray_dir = Vector3::new(1.0, 0.0, 0.0);
    let mut count = 0;

    for face in &mesh.faces {
        let v0 = mesh.vertices[face[0] as usize].position;
        let v1 = mesh.vertices[face[1] as usize].position;
        let v2 = mesh.vertices[face[2] as usize].position;

        if ray_triangle_intersect(point, ray_dir, v0, v1, v2).is_some() {
            count += 1;
        }
    }

    // Odd count means inside
    count % 2 == 1
}

/// Compute the squared distance from a point to a line segment.
///
/// # Arguments
///
/// * `point` - The query point
/// * `a`, `b` - Line segment endpoints
///
/// # Returns
///
/// The squared distance to the segment.
#[must_use]
pub fn point_segment_distance_squared(
    point: Point3<f64>,
    a: Point3<f64>,
    b: Point3<f64>,
) -> f64 {
    let ab = b - a;
    let ap = point - a;

    let t = ap.dot(&ab) / ab.norm_squared().max(f64::EPSILON);
    let t_clamped = t.clamp(0.0, 1.0);

    let closest = a + ab * t_clamped;
    (point - closest).norm_squared()
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use mesh_types::Vertex;

    fn simple_triangle() -> (Point3<f64>, Point3<f64>, Point3<f64>) {
        (
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(10.0, 0.0, 0.0),
            Point3::new(5.0, 10.0, 0.0),
        )
    }

    #[test]
    fn closest_point_inside_triangle() {
        let (v0, v1, v2) = simple_triangle();
        let point = Point3::new(5.0, 3.0, 5.0);

        let closest = closest_point_on_triangle(point, v0, v1, v2);

        // Closest should be on the triangle plane (z=0)
        assert_relative_eq!(closest.z, 0.0, epsilon = 1e-10);
        // And inside the triangle bounds
        assert!(closest.x >= 0.0 && closest.x <= 10.0);
        assert!(closest.y >= 0.0 && closest.y <= 10.0);
    }

    #[test]
    fn closest_point_vertex_region() {
        let (v0, v1, v2) = simple_triangle();

        // Point near vertex 0
        let point = Point3::new(-5.0, -5.0, 0.0);
        let closest = closest_point_on_triangle(point, v0, v1, v2);

        assert_relative_eq!(closest.x, 0.0, epsilon = 1e-10);
        assert_relative_eq!(closest.y, 0.0, epsilon = 1e-10);
    }

    #[test]
    fn closest_point_edge_region() {
        let (v0, v1, v2) = simple_triangle();

        // Point near edge v0-v1
        let point = Point3::new(5.0, -5.0, 0.0);
        let closest = closest_point_on_triangle(point, v0, v1, v2);

        // Should be on the edge (y = 0)
        assert_relative_eq!(closest.y, 0.0, epsilon = 1e-10);
        assert!(closest.x >= 0.0 && closest.x <= 10.0);
    }

    #[test]
    fn ray_hits_triangle() {
        let (v0, v1, v2) = simple_triangle();
        let ray_origin = Point3::new(5.0, 3.0, 5.0);
        let ray_dir = Vector3::new(0.0, 0.0, -1.0);

        let hit = ray_triangle_intersect(ray_origin, ray_dir, v0, v1, v2);

        assert!(hit.is_some());
        assert_relative_eq!(hit.expect("should hit"), 5.0, epsilon = 1e-10);
    }

    #[test]
    fn ray_misses_triangle() {
        let (v0, v1, v2) = simple_triangle();
        let ray_origin = Point3::new(100.0, 100.0, 5.0);
        let ray_dir = Vector3::new(0.0, 0.0, -1.0);

        let hit = ray_triangle_intersect(ray_origin, ray_dir, v0, v1, v2);

        assert!(hit.is_none());
    }

    #[test]
    fn ray_parallel_to_triangle() {
        let (v0, v1, v2) = simple_triangle();
        let ray_origin = Point3::new(5.0, 3.0, 5.0);
        let ray_dir = Vector3::new(1.0, 0.0, 0.0); // Parallel to XY plane

        let hit = ray_triangle_intersect(ray_origin, ray_dir, v0, v1, v2);

        assert!(hit.is_none());
    }

    #[test]
    fn point_inside_tetrahedron() {
        let mut mesh = IndexedMesh::new();
        mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(1.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(0.5, 0.866, 0.0));
        mesh.vertices.push(Vertex::from_coords(0.5, 0.289, 0.816));

        mesh.faces.push([0, 2, 1]);
        mesh.faces.push([0, 1, 3]);
        mesh.faces.push([1, 2, 3]);
        mesh.faces.push([2, 0, 3]);

        // Centroid should be inside
        let centroid = Point3::new(0.5, 0.385, 0.204);
        assert!(point_in_mesh(centroid, &mesh));
    }

    #[test]
    fn point_outside_tetrahedron() {
        let mut mesh = IndexedMesh::new();
        mesh.vertices.push(Vertex::from_coords(0.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(1.0, 0.0, 0.0));
        mesh.vertices.push(Vertex::from_coords(0.5, 0.866, 0.0));
        mesh.vertices.push(Vertex::from_coords(0.5, 0.289, 0.816));

        mesh.faces.push([0, 2, 1]);
        mesh.faces.push([0, 1, 3]);
        mesh.faces.push([1, 2, 3]);
        mesh.faces.push([2, 0, 3]);

        // Far outside
        let outside = Point3::new(10.0, 10.0, 10.0);
        assert!(!point_in_mesh(outside, &mesh));
    }

    #[test]
    fn segment_distance_midpoint() {
        let a = Point3::new(0.0, 0.0, 0.0);
        let b = Point3::new(10.0, 0.0, 0.0);
        let point = Point3::new(5.0, 5.0, 0.0);

        let dist_sq = point_segment_distance_squared(point, a, b);

        // Distance should be 5 (perpendicular to midpoint)
        assert_relative_eq!(dist_sq, 25.0, epsilon = 1e-10);
    }

    #[test]
    fn segment_distance_endpoint() {
        let a = Point3::new(0.0, 0.0, 0.0);
        let b = Point3::new(10.0, 0.0, 0.0);
        let point = Point3::new(-5.0, 0.0, 0.0);

        let dist_sq = point_segment_distance_squared(point, a, b);

        // Distance should be 5 (to endpoint a)
        assert_relative_eq!(dist_sq, 25.0, epsilon = 1e-10);
    }
}
