//! Triangle defined by three concrete vertex positions.
//!
//! Replaces `mesh_types::Triangle`. Winding is **counter-clockwise (CCW)
//! when viewed from the front** (normal points toward viewer).

use nalgebra::{Point3, Vector3};

use crate::Aabb;
use crate::bounded::Bounded;

/// A triangle with concrete vertex positions.
///
/// Stores the actual vertex positions rather than indices. Winding is
/// **counter-clockwise (CCW) when viewed from the front** — the normal
/// points toward the viewer by the right-hand rule.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Triangle {
    /// First vertex.
    pub v0: Point3<f64>,
    /// Second vertex.
    pub v1: Point3<f64>,
    /// Third vertex.
    pub v2: Point3<f64>,
}

impl Triangle {
    /// Creates a new triangle from three points.
    #[inline]
    #[must_use]
    pub const fn new(v0: Point3<f64>, v1: Point3<f64>, v2: Point3<f64>) -> Self {
        Self { v0, v1, v2 }
    }

    /// Creates a triangle from coordinate arrays.
    #[inline]
    #[must_use]
    // Body uses float ops / trait methods that are not yet const-stable.
    #[allow(clippy::missing_const_for_fn)]
    pub fn from_arrays(v0: [f64; 3], v1: [f64; 3], v2: [f64; 3]) -> Self {
        Self {
            v0: Point3::new(v0[0], v0[1], v0[2]),
            v1: Point3::new(v1[0], v1[1], v1[2]),
            v2: Point3::new(v2[0], v2[1], v2[2]),
        }
    }

    /// Computes the (unnormalized) face normal via cross product.
    ///
    /// The direction follows the right-hand rule with CCW winding.
    /// The magnitude equals twice the triangle's area.
    #[inline]
    #[must_use]
    pub fn normal_unnormalized(&self) -> Vector3<f64> {
        let e1 = self.v1 - self.v0;
        let e2 = self.v2 - self.v0;
        e1.cross(&e2)
    }

    /// Computes the unit face normal.
    ///
    /// Returns `None` for degenerate triangles (zero area).
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

    /// Computes the area of the triangle.
    #[inline]
    #[must_use]
    pub fn area(&self) -> f64 {
        self.normal_unnormalized().norm() * 0.5
    }

    /// Computes the centroid (center of mass).
    #[inline]
    #[must_use]
    pub fn centroid(&self) -> Point3<f64> {
        Point3::new(
            (self.v0.x + self.v1.x + self.v2.x) / 3.0,
            (self.v0.y + self.v1.y + self.v2.y) / 3.0,
            (self.v0.z + self.v1.z + self.v2.z) / 3.0,
        )
    }

    /// Returns the three edges as (start, end) pairs.
    ///
    /// Order: v0→v1, v1→v2, v2→v0.
    #[must_use]
    pub const fn edges(&self) -> [(Point3<f64>, Point3<f64>); 3] {
        [(self.v0, self.v1), (self.v1, self.v2), (self.v2, self.v0)]
    }

    /// Computes the lengths of the three edges.
    ///
    /// Returns `[len01, len12, len20]`.
    #[inline]
    #[must_use]
    pub fn edge_lengths(&self) -> [f64; 3] {
        [
            (self.v1 - self.v0).norm(),
            (self.v2 - self.v1).norm(),
            (self.v0 - self.v2).norm(),
        ]
    }

    /// Returns the length of the shortest edge.
    #[inline]
    #[must_use]
    pub fn min_edge_length(&self) -> f64 {
        let [a, b, c] = self.edge_lengths();
        a.min(b).min(c)
    }

    /// Returns the length of the longest edge.
    #[inline]
    #[must_use]
    pub fn max_edge_length(&self) -> f64 {
        let [a, b, c] = self.edge_lengths();
        a.max(b).max(c)
    }

    /// Computes the aspect ratio of the triangle.
    ///
    /// Defined as `longest_edge / shortest_altitude`. An equilateral triangle
    /// has aspect ratio ~1.15. Very thin triangles have high ratios (>10 is
    /// problematic). Returns `f64::INFINITY` for degenerate triangles.
    #[must_use]
    pub fn aspect_ratio(&self) -> f64 {
        let area = self.area();
        if area < f64::EPSILON {
            return f64::INFINITY;
        }

        let max_edge = self.max_edge_length();
        let shortest_altitude = 2.0 * area / max_edge;

        if shortest_altitude < f64::EPSILON {
            return f64::INFINITY;
        }

        max_edge / shortest_altitude
    }

    /// Checks if triangle vertices are nearly collinear.
    ///
    /// Uses the cross product magnitude relative to edge lengths.
    /// `epsilon` is the threshold for the sine of the angle between edges.
    #[must_use]
    pub fn is_nearly_collinear(&self, epsilon: f64) -> bool {
        let e1 = self.v1 - self.v0;
        let e2 = self.v2 - self.v0;

        let cross_magnitude = e1.cross(&e2).norm();
        let edge_product = e1.norm() * e2.norm();

        if edge_product < f64::EPSILON {
            return true;
        }

        cross_magnitude / edge_product < epsilon
    }

    /// Checks if the triangle is degenerate (area below `epsilon`).
    #[inline]
    #[must_use]
    pub fn is_degenerate(&self, epsilon: f64) -> bool {
        self.area() < epsilon
    }

    /// Checks if the triangle is degenerate using multiple criteria.
    ///
    /// A triangle is degenerate if any of:
    /// - Area is below `area_threshold`
    /// - Aspect ratio exceeds `max_aspect_ratio`
    /// - Shortest edge is below `min_edge_length`
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

    /// Returns vertices as an array.
    #[inline]
    #[must_use]
    pub const fn vertices(&self) -> [Point3<f64>; 3] {
        [self.v0, self.v1, self.v2]
    }

    /// Creates a new triangle with reversed winding (flipped normal).
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

impl Bounded for Triangle {
    fn aabb(&self) -> Aabb {
        Aabb::from_triangle(&self.v0, &self.v1, &self.v2)
    }
}
