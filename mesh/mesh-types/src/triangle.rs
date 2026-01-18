//! Triangle type for geometric calculations.

use nalgebra::{Point3, Vector3};

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

/// A triangle with concrete vertex positions.
///
/// This is a utility type for geometric calculations. It stores the actual
/// vertex positions rather than indices.
///
/// Winding is **counter-clockwise (CCW) when viewed from the front**
/// (normal points toward viewer).
///
/// # Example
///
/// ```
/// use mesh_types::{Triangle, Point3};
///
/// let tri = Triangle::new(
///     Point3::new(0.0, 0.0, 0.0),
///     Point3::new(1.0, 0.0, 0.0),
///     Point3::new(0.0, 1.0, 0.0),
/// );
///
/// // Area of a right triangle with legs 1 and 1
/// assert!((tri.area() - 0.5).abs() < 1e-10);
///
/// // Normal points in +Z direction
/// let normal = tri.normal().unwrap();
/// assert!((normal.z - 1.0).abs() < 1e-10);
/// ```
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct Triangle {
    /// First vertex.
    pub v0: Point3<f64>,
    /// Second vertex.
    pub v1: Point3<f64>,
    /// Third vertex.
    pub v2: Point3<f64>,
}

impl Triangle {
    /// Create a new triangle from three points.
    ///
    /// # Example
    ///
    /// ```
    /// use mesh_types::{Triangle, Point3};
    ///
    /// let tri = Triangle::new(
    ///     Point3::new(0.0, 0.0, 0.0),
    ///     Point3::new(1.0, 0.0, 0.0),
    ///     Point3::new(0.0, 1.0, 0.0),
    /// );
    /// ```
    #[inline]
    #[must_use]
    pub const fn new(v0: Point3<f64>, v1: Point3<f64>, v2: Point3<f64>) -> Self {
        Self { v0, v1, v2 }
    }

    /// Create a triangle from coordinate arrays.
    ///
    /// # Example
    ///
    /// ```
    /// use mesh_types::Triangle;
    ///
    /// let tri = Triangle::from_arrays(
    ///     [0.0, 0.0, 0.0],
    ///     [1.0, 0.0, 0.0],
    ///     [0.0, 1.0, 0.0],
    /// );
    /// ```
    #[inline]
    #[must_use]
    #[allow(clippy::missing_const_for_fn)] // Point3::new is not const in nalgebra
    pub fn from_arrays(v0: [f64; 3], v1: [f64; 3], v2: [f64; 3]) -> Self {
        Self {
            v0: Point3::new(v0[0], v0[1], v0[2]),
            v1: Point3::new(v1[0], v1[1], v1[2]),
            v2: Point3::new(v2[0], v2[1], v2[2]),
        }
    }

    /// Compute the (unnormalized) face normal via cross product.
    ///
    /// The direction follows the right-hand rule with CCW winding.
    /// The magnitude equals twice the triangle's area.
    ///
    /// # Example
    ///
    /// ```
    /// use mesh_types::{Triangle, Point3};
    ///
    /// let tri = Triangle::new(
    ///     Point3::new(0.0, 0.0, 0.0),
    ///     Point3::new(2.0, 0.0, 0.0),
    ///     Point3::new(0.0, 2.0, 0.0),
    /// );
    ///
    /// let n = tri.normal_unnormalized();
    /// // Magnitude = 2 * area = 2 * 2 = 4
    /// assert!((n.norm() - 4.0).abs() < 1e-10);
    /// ```
    #[inline]
    #[must_use]
    pub fn normal_unnormalized(&self) -> Vector3<f64> {
        let e1 = self.v1 - self.v0;
        let e2 = self.v2 - self.v0;
        e1.cross(&e2)
    }

    /// Compute the unit face normal.
    ///
    /// Returns `None` for degenerate triangles (zero area).
    ///
    /// # Example
    ///
    /// ```
    /// use mesh_types::{Triangle, Point3};
    ///
    /// // Valid triangle
    /// let tri = Triangle::new(
    ///     Point3::new(0.0, 0.0, 0.0),
    ///     Point3::new(1.0, 0.0, 0.0),
    ///     Point3::new(0.0, 1.0, 0.0),
    /// );
    /// let normal = tri.normal().unwrap();
    /// assert!((normal.z - 1.0).abs() < 1e-10);
    ///
    /// // Degenerate triangle (collinear points)
    /// let degen = Triangle::new(
    ///     Point3::new(0.0, 0.0, 0.0),
    ///     Point3::new(1.0, 0.0, 0.0),
    ///     Point3::new(2.0, 0.0, 0.0),
    /// );
    /// assert!(degen.normal().is_none());
    /// ```
    #[must_use]
    pub fn normal(&self) -> Option<Vector3<f64>> {
        let n = self.normal_unnormalized();
        let len_sq = n.norm_squared();
        if len_sq > f64::EPSILON {
            Some(n / len_sq.sqrt())
        } else {
            None
        }
    }

    /// Compute the area of the triangle.
    ///
    /// # Example
    ///
    /// ```
    /// use mesh_types::{Triangle, Point3};
    ///
    /// // Right triangle with legs 3 and 4
    /// let tri = Triangle::new(
    ///     Point3::new(0.0, 0.0, 0.0),
    ///     Point3::new(3.0, 0.0, 0.0),
    ///     Point3::new(0.0, 4.0, 0.0),
    /// );
    /// assert!((tri.area() - 6.0).abs() < 1e-10);
    /// ```
    #[inline]
    #[must_use]
    pub fn area(&self) -> f64 {
        self.normal_unnormalized().norm() * 0.5
    }

    /// Compute the centroid (center of mass).
    ///
    /// # Example
    ///
    /// ```
    /// use mesh_types::{Triangle, Point3};
    ///
    /// let tri = Triangle::new(
    ///     Point3::new(0.0, 0.0, 0.0),
    ///     Point3::new(3.0, 0.0, 0.0),
    ///     Point3::new(0.0, 3.0, 0.0),
    /// );
    /// let c = tri.centroid();
    /// assert!((c.x - 1.0).abs() < 1e-10);
    /// assert!((c.y - 1.0).abs() < 1e-10);
    /// ```
    #[inline]
    #[must_use]
    pub fn centroid(&self) -> Point3<f64> {
        Point3::new(
            (self.v0.x + self.v1.x + self.v2.x) / 3.0,
            (self.v0.y + self.v1.y + self.v2.y) / 3.0,
            (self.v0.z + self.v1.z + self.v2.z) / 3.0,
        )
    }

    /// Get the three edges as (start, end) pairs.
    ///
    /// Returns edges in order: v0→v1, v1→v2, v2→v0.
    #[must_use]
    pub const fn edges(&self) -> [(Point3<f64>, Point3<f64>); 3] {
        [(self.v0, self.v1), (self.v1, self.v2), (self.v2, self.v0)]
    }

    /// Compute the lengths of the three edges.
    ///
    /// Returns `[len01, len12, len20]` where `lenXY` is the distance from vX to vY.
    ///
    /// # Example
    ///
    /// ```
    /// use mesh_types::{Triangle, Point3};
    ///
    /// // 3-4-5 right triangle
    /// let tri = Triangle::new(
    ///     Point3::new(0.0, 0.0, 0.0),
    ///     Point3::new(3.0, 0.0, 0.0),
    ///     Point3::new(0.0, 4.0, 0.0),
    /// );
    /// let lengths = tri.edge_lengths();
    /// assert!((lengths[0] - 3.0).abs() < 1e-10);  // v0 -> v1
    /// assert!((lengths[1] - 5.0).abs() < 1e-10);  // v1 -> v2 (hypotenuse)
    /// assert!((lengths[2] - 4.0).abs() < 1e-10);  // v2 -> v0
    /// ```
    #[inline]
    #[must_use]
    pub fn edge_lengths(&self) -> [f64; 3] {
        [
            (self.v1 - self.v0).norm(),
            (self.v2 - self.v1).norm(),
            (self.v0 - self.v2).norm(),
        ]
    }

    /// Get the length of the shortest edge.
    #[inline]
    #[must_use]
    pub fn min_edge_length(&self) -> f64 {
        let [a, b, c] = self.edge_lengths();
        a.min(b).min(c)
    }

    /// Get the length of the longest edge.
    #[inline]
    #[must_use]
    pub fn max_edge_length(&self) -> f64 {
        let [a, b, c] = self.edge_lengths();
        a.max(b).max(c)
    }

    /// Compute the aspect ratio of the triangle.
    ///
    /// Aspect ratio is defined as `longest_edge / shortest_altitude`.
    /// - A well-shaped equilateral triangle has aspect ratio ≈ 1.15
    /// - Very thin/needle triangles have high aspect ratios (>10 is problematic)
    ///
    /// Returns `f64::INFINITY` for degenerate triangles (zero area).
    ///
    /// # Example
    ///
    /// ```
    /// use mesh_types::{Triangle, Point3};
    ///
    /// // Equilateral triangle
    /// let sqrt3 = 3.0_f64.sqrt();
    /// let tri = Triangle::new(
    ///     Point3::new(0.0, 0.0, 0.0),
    ///     Point3::new(2.0, 0.0, 0.0),
    ///     Point3::new(1.0, sqrt3, 0.0),
    /// );
    /// let ar = tri.aspect_ratio();
    /// assert!(ar > 1.1 && ar < 1.2);
    /// ```
    #[must_use]
    pub fn aspect_ratio(&self) -> f64 {
        let area = self.area();
        if area < f64::EPSILON {
            return f64::INFINITY;
        }

        let max_edge = self.max_edge_length();

        // Altitude = 2 * area / base
        // Shortest altitude corresponds to longest edge as base
        let shortest_altitude = 2.0 * area / max_edge;

        if shortest_altitude < f64::EPSILON {
            return f64::INFINITY;
        }

        max_edge / shortest_altitude
    }

    /// Check if triangle vertices are nearly collinear.
    ///
    /// Uses the cross product magnitude relative to edge lengths.
    ///
    /// # Arguments
    ///
    /// * `epsilon` - Threshold for the sine of the angle between edges.
    ///   Smaller values detect more triangles as collinear.
    ///
    /// # Example
    ///
    /// ```
    /// use mesh_types::{Triangle, Point3};
    ///
    /// // Collinear points
    /// let degen = Triangle::new(
    ///     Point3::new(0.0, 0.0, 0.0),
    ///     Point3::new(1.0, 0.0, 0.0),
    ///     Point3::new(2.0, 0.0, 0.0),
    /// );
    /// assert!(degen.is_nearly_collinear(0.01));
    ///
    /// // Well-formed triangle
    /// let good = Triangle::new(
    ///     Point3::new(0.0, 0.0, 0.0),
    ///     Point3::new(1.0, 0.0, 0.0),
    ///     Point3::new(0.5, 1.0, 0.0),
    /// );
    /// assert!(!good.is_nearly_collinear(0.01));
    /// ```
    #[must_use]
    pub fn is_nearly_collinear(&self, epsilon: f64) -> bool {
        let e1 = self.v1 - self.v0;
        let e2 = self.v2 - self.v0;

        let cross_magnitude = e1.cross(&e2).norm();
        let edge_product = e1.norm() * e2.norm();

        if edge_product < f64::EPSILON {
            return true; // Degenerate edges
        }

        // sin(angle) = |cross| / (|e1| * |e2|)
        cross_magnitude / edge_product < epsilon
    }

    /// Check if the triangle is degenerate (zero or near-zero area).
    ///
    /// # Arguments
    ///
    /// * `epsilon` - Area threshold below which the triangle is degenerate.
    #[inline]
    #[must_use]
    pub fn is_degenerate(&self, epsilon: f64) -> bool {
        self.area() < epsilon
    }

    /// Check if the triangle is degenerate using multiple criteria.
    ///
    /// A triangle is considered degenerate if any of these conditions are met:
    /// - Area is below `area_threshold`
    /// - Aspect ratio exceeds `max_aspect_ratio`
    /// - Shortest edge is below `min_edge_length`
    ///
    /// # Arguments
    ///
    /// * `area_threshold` - Minimum acceptable area (e.g., 1e-9)
    /// * `max_aspect_ratio` - Maximum acceptable aspect ratio (e.g., 1000.0)
    /// * `min_edge_length` - Minimum acceptable edge length (e.g., 1e-9)
    #[must_use]
    pub fn is_degenerate_enhanced(
        &self,
        area_threshold: f64,
        max_aspect_ratio: f64,
        min_edge_length: f64,
    ) -> bool {
        if self.area() < area_threshold {
            return true;
        }

        if self.aspect_ratio() > max_aspect_ratio {
            return true;
        }

        if self.min_edge_length() < min_edge_length {
            return true;
        }

        false
    }

    /// Get vertices as an array.
    #[inline]
    #[must_use]
    pub const fn vertices(&self) -> [Point3<f64>; 3] {
        [self.v0, self.v1, self.v2]
    }

    /// Create a new triangle with reversed winding (flipped normal).
    #[inline]
    #[must_use]
    pub const fn reversed(&self) -> Self {
        Self {
            v0: self.v0,
            v1: self.v2,
            v2: self.v1,
        }
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn triangle_normal() {
        let tri = Triangle::new(
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        );

        let normal = tri.normal();
        assert!(normal.is_some());
        let n = normal.map(|n| (n.x, n.y, n.z));
        assert!(n.is_some());
        let (x, y, z) = n.map_or((0.0, 0.0, 0.0), |n| n);
        assert!(x.abs() < 1e-10);
        assert!(y.abs() < 1e-10);
        assert!((z - 1.0).abs() < 1e-10);
    }

    #[test]
    fn triangle_area() {
        let tri = Triangle::new(
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        );
        assert!((tri.area() - 0.5).abs() < 1e-10);
    }

    #[test]
    fn triangle_centroid() {
        let tri = Triangle::new(
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(3.0, 0.0, 0.0),
            Point3::new(0.0, 3.0, 0.0),
        );
        let c = tri.centroid();
        assert!((c.x - 1.0).abs() < 1e-10);
        assert!((c.y - 1.0).abs() < 1e-10);
        assert!(c.z.abs() < 1e-10);
    }

    #[test]
    fn degenerate_triangle_normal() {
        let tri = Triangle::new(
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(2.0, 0.0, 0.0),
        );
        assert!(tri.normal().is_none());
    }

    #[test]
    fn triangle_edge_lengths() {
        // 3-4-5 right triangle
        let tri = Triangle::new(
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(3.0, 0.0, 0.0),
            Point3::new(0.0, 4.0, 0.0),
        );
        let lengths = tri.edge_lengths();
        assert!((lengths[0] - 3.0).abs() < 1e-10);
        assert!((lengths[1] - 5.0).abs() < 1e-10);
        assert!((lengths[2] - 4.0).abs() < 1e-10);
    }

    #[test]
    fn triangle_aspect_ratio_equilateral() {
        let sqrt3 = 3.0_f64.sqrt();
        let tri = Triangle::new(
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(2.0, 0.0, 0.0),
            Point3::new(1.0, sqrt3, 0.0),
        );
        let ar = tri.aspect_ratio();
        assert!(ar > 1.1 && ar < 1.2);
    }

    #[test]
    fn triangle_aspect_ratio_degenerate() {
        let tri = Triangle::new(
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(2.0, 0.0, 0.0),
        );
        assert!(tri.aspect_ratio().is_infinite());
    }

    #[test]
    fn triangle_reversed() {
        let tri = Triangle::new(
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        );
        let rev = tri.reversed();
        let n1 = tri.normal();
        let n2 = rev.normal();
        assert!(n1.is_some());
        assert!(n2.is_some());
        // Normals should be opposite
        let (n1, n2) = (n1.map(|n| n.z), n2.map(|n| n.z));
        assert!((n1.unwrap_or(0.0) + n2.unwrap_or(0.0)).abs() < 1e-10);
    }
}
