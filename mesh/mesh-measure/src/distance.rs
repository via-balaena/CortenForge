//! Distance measurement utilities.
//!
//! Provides point-to-point and point-to-mesh distance calculations.

use mesh_types::{IndexedMesh, MeshTopology, Point3};

/// Distance measurement result between two points.
///
/// # Example
///
/// ```
/// use mesh_measure::measure_distance;
/// use nalgebra::Point3;
///
/// let from = Point3::origin();
/// let to = Point3::new(3.0, 4.0, 0.0);
/// let result = measure_distance(from, to);
///
/// assert!((result.distance - 5.0).abs() < 1e-10); // 3-4-5 triangle
/// ```
#[derive(Debug, Clone)]
pub struct DistanceMeasurement {
    /// Start point.
    pub from: Point3<f64>,
    /// End point.
    pub to: Point3<f64>,
    /// Euclidean distance.
    pub distance: f64,
    /// Absolute distance along X axis.
    pub dx: f64,
    /// Absolute distance along Y axis.
    pub dy: f64,
    /// Absolute distance along Z axis.
    pub dz: f64,
}

impl DistanceMeasurement {
    /// Get the direction vector (not normalized).
    #[must_use]
    pub fn direction(&self) -> nalgebra::Vector3<f64> {
        self.to - self.from
    }

    /// Get the normalized direction vector.
    ///
    /// Returns `None` if the distance is zero.
    #[must_use]
    pub fn direction_normalized(&self) -> Option<nalgebra::Vector3<f64>> {
        if self.distance.abs() < f64::EPSILON {
            None
        } else {
            Some(self.direction() / self.distance)
        }
    }

    /// Get the midpoint between from and to.
    #[must_use]
    pub fn midpoint(&self) -> Point3<f64> {
        Point3::from((self.from.coords + self.to.coords) / 2.0)
    }
}

/// Measure distance between two points.
///
/// # Arguments
///
/// * `from` - Start point
/// * `to` - End point
///
/// # Returns
///
/// A [`DistanceMeasurement`] with the distance and component distances.
///
/// # Example
///
/// ```
/// use mesh_measure::measure_distance;
/// use nalgebra::Point3;
///
/// let result = measure_distance(
///     Point3::new(0.0, 0.0, 0.0),
///     Point3::new(10.0, 0.0, 0.0)
/// );
///
/// assert!((result.distance - 10.0).abs() < 1e-10);
/// assert!((result.dx - 10.0).abs() < 1e-10);
/// assert!((result.dy).abs() < 1e-10);
/// assert!((result.dz).abs() < 1e-10);
/// ```
#[must_use]
pub fn measure_distance(from: Point3<f64>, to: Point3<f64>) -> DistanceMeasurement {
    let diff = to - from;
    DistanceMeasurement {
        from,
        to,
        distance: diff.norm(),
        dx: diff.x.abs(),
        dy: diff.y.abs(),
        dz: diff.z.abs(),
    }
}

/// Find the closest point on a mesh surface to a given point.
///
/// Iterates through all triangles and finds the point on the mesh
/// surface that is closest to the query point.
///
/// # Arguments
///
/// * `mesh` - The mesh to search
/// * `point` - The query point
///
/// # Returns
///
/// The closest point on the mesh surface, or `None` if the mesh is empty.
///
/// # Example
///
/// ```
/// use mesh_types::unit_cube;
/// use mesh_measure::closest_point_on_mesh;
/// use nalgebra::Point3;
///
/// let cube = unit_cube();
/// let closest = closest_point_on_mesh(&cube, Point3::new(10.0, 0.5, 0.5));
///
/// assert!(closest.is_some());
/// let p = closest.unwrap();
/// // Closest point should be on the +X face at x=1.0
/// assert!((p.x - 1.0).abs() < 0.01);
/// ```
#[must_use]
pub fn closest_point_on_mesh(mesh: &IndexedMesh, point: Point3<f64>) -> Option<Point3<f64>> {
    if mesh.faces.is_empty() {
        return None;
    }

    let mut closest = None;
    let mut min_dist_sq = f64::INFINITY;

    for triangle in mesh.triangles() {
        let p = closest_point_on_triangle(point, triangle.v0, triangle.v1, triangle.v2);
        let dist_sq = (p - point).norm_squared();

        if dist_sq < min_dist_sq {
            min_dist_sq = dist_sq;
            closest = Some(p);
        }
    }

    closest
}

/// Compute the distance from a point to the mesh surface.
///
/// # Arguments
///
/// * `mesh` - The mesh to measure distance to
/// * `point` - The query point
///
/// # Returns
///
/// The minimum distance from the point to any triangle on the mesh,
/// or `None` if the mesh is empty.
///
/// # Example
///
/// ```
/// use mesh_types::unit_cube;
/// use mesh_measure::distance_to_mesh;
/// use nalgebra::Point3;
///
/// let cube = unit_cube();
/// let dist = distance_to_mesh(&cube, Point3::new(2.0, 0.5, 0.5));
///
/// assert!(dist.is_some());
/// // Distance from (2, 0.5, 0.5) to unit cube face at x=1 is 1.0
/// assert!((dist.unwrap() - 1.0).abs() < 0.01);
/// ```
#[must_use]
pub fn distance_to_mesh(mesh: &IndexedMesh, point: Point3<f64>) -> Option<f64> {
    closest_point_on_mesh(mesh, point).map(|closest| (point - closest).norm())
}

/// Compute the closest point on a triangle to a query point.
///
/// Uses the barycentric coordinate method for robust computation.
#[allow(clippy::many_single_char_names)]
fn closest_point_on_triangle(
    p: Point3<f64>,
    a: Point3<f64>,
    b: Point3<f64>,
    c: Point3<f64>,
) -> Point3<f64> {
    let ab = b - a;
    let ac = c - a;
    let ap = p - a;

    let d1 = ab.dot(&ap);
    let d2 = ac.dot(&ap);
    if d1 <= 0.0 && d2 <= 0.0 {
        return a;
    }

    let bp = p - b;
    let d3 = ab.dot(&bp);
    let d4 = ac.dot(&bp);
    if d3 >= 0.0 && d4 <= d3 {
        return b;
    }

    let vc = d1.mul_add(d4, -(d3 * d2));
    if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
        let v = d1 / (d1 - d3);
        return Point3::from(a.coords + ab * v);
    }

    let cp = p - c;
    let d5 = ab.dot(&cp);
    let d6 = ac.dot(&cp);
    if d6 >= 0.0 && d5 <= d6 {
        return c;
    }

    let vb = d5.mul_add(d2, -(d1 * d6));
    if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
        let w = d2 / (d2 - d6);
        return Point3::from(a.coords + ac * w);
    }

    let va = d3.mul_add(d6, -(d5 * d4));
    if va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0 {
        let w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return Point3::from(b.coords + (c - b) * w);
    }

    let denom = 1.0 / (va + vb + vc);
    let v = vb * denom;
    let w = vc * denom;
    Point3::from(a.coords + ab * v + ac * w)
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used, clippy::float_cmp)]
mod tests {
    use super::*;
    use mesh_types::unit_cube;

    #[test]
    fn test_measure_distance_basic() {
        let from = Point3::origin();
        let to = Point3::new(3.0, 4.0, 0.0);
        let result = measure_distance(from, to);

        assert!((result.distance - 5.0).abs() < 1e-10);
        assert!((result.dx - 3.0).abs() < 1e-10);
        assert!((result.dy - 4.0).abs() < 1e-10);
        assert!((result.dz).abs() < 1e-10);
    }

    #[test]
    fn test_measure_distance_3d() {
        let from = Point3::origin();
        let to = Point3::new(1.0, 1.0, 1.0);
        let result = measure_distance(from, to);

        let expected = (3.0_f64).sqrt();
        assert!((result.distance - expected).abs() < 1e-10);
    }

    #[test]
    fn test_measure_distance_zero() {
        let p = Point3::new(5.0, 5.0, 5.0);
        let result = measure_distance(p, p);

        assert!((result.distance).abs() < 1e-10);
    }

    #[test]
    fn test_distance_midpoint() {
        let from = Point3::origin();
        let to = Point3::new(10.0, 0.0, 0.0);
        let result = measure_distance(from, to);

        let mid = result.midpoint();
        assert!((mid.x - 5.0).abs() < 1e-10);
        assert!((mid.y).abs() < 1e-10);
        assert!((mid.z).abs() < 1e-10);
    }

    #[test]
    fn test_distance_direction() {
        let from = Point3::origin();
        let to = Point3::new(10.0, 0.0, 0.0);
        let result = measure_distance(from, to);

        let dir = result.direction_normalized();
        assert!(dir.is_some());
        let d = dir.unwrap();
        assert!((d.x - 1.0).abs() < 1e-10);
        assert!((d.y).abs() < 1e-10);
        assert!((d.z).abs() < 1e-10);
    }

    #[test]
    fn test_closest_point_on_mesh() {
        let cube = unit_cube();
        let closest = closest_point_on_mesh(&cube, Point3::new(10.0, 0.5, 0.5));

        assert!(closest.is_some());
        let p = closest.unwrap();
        // Should be on +X face
        assert!((p.x - 1.0).abs() < 0.01);
    }

    #[test]
    fn test_closest_point_inside_mesh() {
        let cube = unit_cube();
        let closest = closest_point_on_mesh(&cube, Point3::new(0.5, 0.5, 0.5));

        assert!(closest.is_some());
        // Point is inside, closest should be on a face
    }

    #[test]
    fn test_closest_point_empty_mesh() {
        let mesh = IndexedMesh::new();
        let closest = closest_point_on_mesh(&mesh, Point3::origin());

        assert!(closest.is_none());
    }

    #[test]
    fn test_distance_to_mesh() {
        let cube = unit_cube();
        let dist = distance_to_mesh(&cube, Point3::new(2.0, 0.5, 0.5));

        assert!(dist.is_some());
        assert!((dist.unwrap() - 1.0).abs() < 0.01);
    }

    #[test]
    fn test_distance_to_empty_mesh() {
        let mesh = IndexedMesh::new();
        let dist = distance_to_mesh(&mesh, Point3::origin());

        assert!(dist.is_none());
    }

    #[test]
    fn test_closest_point_on_triangle_vertex() {
        let a = Point3::origin();
        let b = Point3::new(1.0, 0.0, 0.0);
        let c = Point3::new(0.0, 1.0, 0.0);

        // Point closest to vertex A
        let p = Point3::new(-1.0, -1.0, 0.0);
        let closest = closest_point_on_triangle(p, a, b, c);
        assert!((closest - a).norm() < 1e-10);
    }

    #[test]
    fn test_closest_point_on_triangle_edge() {
        let a = Point3::origin();
        let b = Point3::new(1.0, 0.0, 0.0);
        let c = Point3::new(0.0, 1.0, 0.0);

        // Point closest to edge AB
        let p = Point3::new(0.5, -1.0, 0.0);
        let closest = closest_point_on_triangle(p, a, b, c);
        assert!((closest.y).abs() < 1e-10);
        assert!(closest.x >= 0.0 && closest.x <= 1.0);
    }

    #[test]
    fn test_closest_point_on_triangle_interior() {
        let a = Point3::origin();
        let b = Point3::new(1.0, 0.0, 0.0);
        let c = Point3::new(0.0, 1.0, 0.0);

        // Point above center of triangle
        let p = Point3::new(0.25, 0.25, 1.0);
        let closest = closest_point_on_triangle(p, a, b, c);
        // Should be in the interior with z=0
        assert!((closest.z).abs() < 1e-10);
        assert!(closest.x > 0.0 && closest.y > 0.0);
    }
}
