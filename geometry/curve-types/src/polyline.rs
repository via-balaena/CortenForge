//! Polyline (piecewise linear) curves.
//!
//! A polyline is a sequence of connected line segments defined by vertices.
//! It's the simplest curve type and is commonly used for:
//!
//! - Path representations
//! - Curve approximations
//! - Wire routing
//! - Boundary definitions

use crate::{Curve, CurveError, Result, TubularCurve};
use nalgebra::{Point3, Vector3};

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

/// A piecewise linear curve defined by a sequence of vertices.
///
/// The curve passes through all vertices in order, with linear
/// interpolation between consecutive points.
///
/// # Parameterization
///
/// The parameter `t ∈ [0, 1]` maps to the polyline based on arc length.
/// - `t = 0`: First vertex
/// - `t = 1`: Last vertex
/// - `t = 0.5`: Point at half the total arc length
///
/// # Example
///
/// ```
/// use curve_types::{Polyline, Curve};
/// use nalgebra::Point3;
///
/// let polyline = Polyline::new(vec![
///     Point3::new(0.0, 0.0, 0.0),
///     Point3::new(1.0, 0.0, 0.0),
///     Point3::new(1.0, 1.0, 0.0),
/// ]);
///
/// // Length is 2 (two unit segments)
/// assert!((polyline.arc_length() - 2.0).abs() < 1e-10);
///
/// // Midpoint is at the corner
/// let mid = polyline.point_at(0.5);
/// assert!((mid.x - 1.0).abs() < 1e-10);
/// assert!(mid.y.abs() < 1e-10);
/// ```
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct Polyline {
    /// The vertices of the polyline.
    vertices: Vec<Point3<f64>>,
    /// Cumulative arc lengths at each vertex (precomputed).
    cumulative_lengths: Vec<f64>,
    /// Total arc length (cached).
    total_length: f64,
}

impl Polyline {
    /// Create a new polyline from vertices.
    ///
    /// # Panics
    ///
    /// Panics if fewer than 2 vertices are provided.
    ///
    /// # Example
    ///
    /// ```
    /// use curve_types::Polyline;
    /// use nalgebra::Point3;
    ///
    /// let polyline = Polyline::new(vec![
    ///     Point3::origin(),
    ///     Point3::new(1.0, 0.0, 0.0),
    ///     Point3::new(2.0, 1.0, 0.0),
    /// ]);
    /// ```
    #[must_use]
    pub fn new(vertices: Vec<Point3<f64>>) -> Self {
        assert!(vertices.len() >= 2, "Polyline requires at least 2 vertices");

        let (cumulative_lengths, total_length) = compute_cumulative_lengths(&vertices);

        Self {
            vertices,
            cumulative_lengths,
            total_length,
        }
    }

    /// Try to create a new polyline, returning an error if invalid.
    ///
    /// # Errors
    ///
    /// Returns [`CurveError::InsufficientPoints`] if fewer than 2 vertices.
    pub fn try_new(vertices: Vec<Point3<f64>>) -> Result<Self> {
        if vertices.len() < 2 {
            return Err(CurveError::insufficient_points(2, vertices.len()));
        }

        let (cumulative_lengths, total_length) = compute_cumulative_lengths(&vertices);

        Ok(Self {
            vertices,
            cumulative_lengths,
            total_length,
        })
    }

    /// Create a polyline from a single line segment.
    #[must_use]
    pub fn from_segment(start: Point3<f64>, end: Point3<f64>) -> Self {
        Self::new(vec![start, end])
    }

    /// Get the vertices of the polyline.
    #[must_use]
    pub fn vertices(&self) -> &[Point3<f64>] {
        &self.vertices
    }

    /// Get the number of vertices.
    #[must_use]
    pub fn len(&self) -> usize {
        self.vertices.len()
    }

    /// Check if the polyline is empty (should never be true for valid polyline).
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.vertices.is_empty()
    }

    /// Get the number of segments (edges).
    #[must_use]
    pub fn num_segments(&self) -> usize {
        self.vertices.len().saturating_sub(1)
    }

    /// Get a specific vertex by index.
    #[must_use]
    pub fn vertex(&self, index: usize) -> Option<&Point3<f64>> {
        self.vertices.get(index)
    }

    /// Get a segment as a pair of points.
    #[must_use]
    pub fn segment(&self, index: usize) -> Option<(&Point3<f64>, &Point3<f64>)> {
        if index < self.num_segments() {
            Some((&self.vertices[index], &self.vertices[index + 1]))
        } else {
            None
        }
    }

    /// Get the direction vector for a segment.
    #[must_use]
    pub fn segment_direction(&self, index: usize) -> Option<Vector3<f64>> {
        self.segment(index).map(|(a, b)| (b - a).normalize())
    }

    /// Get the length of a specific segment.
    #[must_use]
    pub fn segment_length(&self, index: usize) -> Option<f64> {
        self.segment(index).map(|(a, b)| (b - a).norm())
    }

    /// Find which segment contains the given arc length.
    ///
    /// Returns `(segment_index, local_t)` where `local_t ∈ [0, 1]`
    /// is the parameter within that segment.
    fn segment_at_arc(&self, arc: f64) -> (usize, f64) {
        if arc <= 0.0 {
            return (0, 0.0);
        }
        if arc >= self.total_length {
            return (self.num_segments() - 1, 1.0);
        }

        // Binary search for segment
        let mut lo = 0;
        let mut hi = self.cumulative_lengths.len() - 1;

        while lo < hi {
            let mid = (lo + hi) / 2;
            if self.cumulative_lengths[mid] < arc {
                lo = mid + 1;
            } else {
                hi = mid;
            }
        }

        // lo is now the index of the first cumulative length >= arc
        let seg_idx = lo.saturating_sub(1);
        let seg_start = self.cumulative_lengths[seg_idx];
        let seg_end = self.cumulative_lengths[seg_idx + 1];
        let seg_len = seg_end - seg_start;

        let local_t = if seg_len > 1e-10 {
            (arc - seg_start) / seg_len
        } else {
            0.0
        };

        (seg_idx, local_t)
    }

    /// Append a vertex to the polyline.
    pub fn push(&mut self, vertex: Point3<f64>) {
        if let Some(last) = self.vertices.last() {
            let seg_len = (vertex - last).norm();
            self.total_length += seg_len;
            self.cumulative_lengths.push(self.total_length);
        }
        self.vertices.push(vertex);
    }

    /// Create a reversed copy of this polyline.
    #[must_use]
    pub fn reversed(&self) -> Self {
        let mut vertices = self.vertices.clone();
        vertices.reverse();
        Self::new(vertices)
    }

    /// Simplify the polyline by removing collinear points.
    ///
    /// Points are considered collinear if the angle deviation is less
    /// than `angle_tolerance` radians.
    #[must_use]
    pub fn simplified(&self, angle_tolerance: f64) -> Self {
        if self.vertices.len() <= 2 {
            return self.clone();
        }

        let cos_tol = angle_tolerance.cos();
        let mut result = vec![self.vertices[0]];

        for i in 1..self.vertices.len() - 1 {
            let prev = result.last().copied().unwrap_or(self.vertices[0]);
            let curr = self.vertices[i];
            let next = self.vertices[i + 1];

            let d1 = (curr - prev).normalize();
            let d2 = (next - curr).normalize();

            // Keep point if direction change exceeds tolerance
            if d1.dot(&d2) < cos_tol {
                result.push(curr);
            }
        }

        result.push(*self.vertices.last().unwrap_or(&self.vertices[0]));

        Self::new(result)
    }

    /// Resample the polyline to have uniform arc length spacing.
    ///
    /// # Parameters
    ///
    /// - `n`: Number of vertices in the resampled polyline
    #[must_use]
    pub fn resampled(&self, n: usize) -> Self {
        let points = self.sample_arc_length(n.max(2));
        Self::new(points)
    }

    /// Compute the bending angle at each interior vertex.
    ///
    /// Returns angles in radians. The angle is 0 for collinear points
    /// and π for a complete reversal.
    #[must_use]
    pub fn bend_angles(&self) -> Vec<f64> {
        if self.vertices.len() < 3 {
            return Vec::new();
        }

        (1..self.vertices.len() - 1)
            .map(|i| {
                let d1 = (self.vertices[i] - self.vertices[i - 1]).normalize();
                let d2 = (self.vertices[i + 1] - self.vertices[i]).normalize();
                d1.dot(&d2).clamp(-1.0, 1.0).acos()
            })
            .collect()
    }

    /// Count the number of bends exceeding a threshold angle.
    #[must_use]
    pub fn count_bends(&self, threshold_radians: f64) -> usize {
        self.bend_angles()
            .iter()
            .filter(|&&angle| angle > threshold_radians)
            .count()
    }
}

impl Curve for Polyline {
    fn point_at(&self, t: f64) -> Point3<f64> {
        let t = t.clamp(0.0, 1.0);
        let arc = t * self.total_length;
        let (seg_idx, local_t) = self.segment_at_arc(arc);

        let p0 = self.vertices[seg_idx];
        let p1 = self.vertices[seg_idx + 1];

        p0 + (p1 - p0) * local_t
    }

    fn tangent_at(&self, t: f64) -> Vector3<f64> {
        let t = t.clamp(0.0, 1.0);
        let arc = t * self.total_length;
        let (seg_idx, _) = self.segment_at_arc(arc);

        let p0 = self.vertices[seg_idx];
        let p1 = self.vertices[seg_idx + 1];
        let dir = p1 - p0;

        if dir.norm() > 1e-10 {
            dir.normalize()
        } else if seg_idx + 2 < self.vertices.len() {
            // Degenerate segment, try next
            (self.vertices[seg_idx + 2] - p1).normalize()
        } else if seg_idx > 0 {
            // Try previous
            (p0 - self.vertices[seg_idx - 1]).normalize()
        } else {
            Vector3::x()
        }
    }

    fn derivative_at(&self, t: f64) -> Vector3<f64> {
        let t = t.clamp(0.0, 1.0);
        let arc = t * self.total_length;
        let (seg_idx, _) = self.segment_at_arc(arc);

        let p0 = self.vertices[seg_idx];
        let p1 = self.vertices[seg_idx + 1];

        // Derivative is constant per segment (scaled by total length for parameterization)
        (p1 - p0) * (self.num_segments() as f64)
    }

    fn second_derivative_at(&self, _t: f64) -> Vector3<f64> {
        // Piecewise linear has zero second derivative everywhere
        // (technically undefined at vertices, but we return zero)
        Vector3::zeros()
    }

    fn arc_length(&self) -> f64 {
        self.total_length
    }

    fn arc_length_between(&self, t0: f64, t1: f64) -> f64 {
        let t0 = t0.clamp(0.0, 1.0);
        let t1 = t1.clamp(0.0, 1.0);
        (t1 - t0).abs() * self.total_length
    }

    fn arc_to_t(&self, s: f64) -> f64 {
        if self.total_length <= 0.0 {
            return 0.0;
        }
        (s / self.total_length).clamp(0.0, 1.0)
    }

    fn t_to_arc(&self, t: f64) -> f64 {
        t.clamp(0.0, 1.0) * self.total_length
    }

    fn is_closed(&self) -> bool {
        if self.vertices.len() < 2 {
            return false;
        }
        let tolerance = 1e-10;
        (self.vertices[0] - self.vertices[self.vertices.len() - 1]).norm() < tolerance
    }

    fn bounding_box(&self) -> (Point3<f64>, Point3<f64>) {
        let mut min = self.vertices[0];
        let mut max = self.vertices[0];

        for p in &self.vertices[1..] {
            min.x = min.x.min(p.x);
            min.y = min.y.min(p.y);
            min.z = min.z.min(p.z);
            max.x = max.x.max(p.x);
            max.y = max.y.max(p.y);
            max.z = max.z.max(p.z);
        }

        (min, max)
    }
}

/// A polyline with radius information at each vertex.
///
/// This represents a tube-like structure where the radius can vary
/// along the length of the polyline.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct TubularPolyline {
    /// The underlying polyline.
    polyline: Polyline,
    /// Radius at each vertex.
    radii: Vec<f64>,
}

impl TubularPolyline {
    /// Create a new tubular polyline.
    ///
    /// # Panics
    ///
    /// Panics if vertices and radii have different lengths,
    /// or if fewer than 2 vertices are provided.
    #[must_use]
    pub fn new(vertices: Vec<Point3<f64>>, radii: Vec<f64>) -> Self {
        assert_eq!(
            vertices.len(),
            radii.len(),
            "Vertices and radii must have same length"
        );
        let polyline = Polyline::new(vertices);
        Self { polyline, radii }
    }

    /// Create a tubular polyline with constant radius.
    #[must_use]
    pub fn with_constant_radius(vertices: Vec<Point3<f64>>, radius: f64) -> Self {
        let radii = vec![radius; vertices.len()];
        Self::new(vertices, radii)
    }

    /// Get the underlying polyline.
    #[must_use]
    pub fn polyline(&self) -> &Polyline {
        &self.polyline
    }

    /// Get the radii at each vertex.
    #[must_use]
    pub fn radii(&self) -> &[f64] {
        &self.radii
    }
}

impl Curve for TubularPolyline {
    fn point_at(&self, t: f64) -> Point3<f64> {
        self.polyline.point_at(t)
    }

    fn tangent_at(&self, t: f64) -> Vector3<f64> {
        self.polyline.tangent_at(t)
    }

    fn derivative_at(&self, t: f64) -> Vector3<f64> {
        self.polyline.derivative_at(t)
    }

    fn second_derivative_at(&self, t: f64) -> Vector3<f64> {
        self.polyline.second_derivative_at(t)
    }

    fn arc_length(&self) -> f64 {
        self.polyline.arc_length()
    }
}

impl TubularCurve for TubularPolyline {
    fn radius_at(&self, t: f64) -> f64 {
        let t = t.clamp(0.0, 1.0);
        let arc = t * self.polyline.total_length;
        let (seg_idx, local_t) = self.polyline.segment_at_arc(arc);

        let r0 = self.radii[seg_idx];
        let r1 = self.radii[seg_idx + 1];

        r0 + (r1 - r0) * local_t
    }
}

/// Compute cumulative arc lengths for a vertex list.
fn compute_cumulative_lengths(vertices: &[Point3<f64>]) -> (Vec<f64>, f64) {
    let mut cumulative = Vec::with_capacity(vertices.len());
    let mut total = 0.0;

    cumulative.push(0.0);
    for i in 1..vertices.len() {
        let seg_len = (vertices[i] - vertices[i - 1]).norm();
        total += seg_len;
        cumulative.push(total);
    }

    (cumulative, total)
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn test_polyline_creation() {
        let polyline = Polyline::new(vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
        ]);

        assert_eq!(polyline.len(), 3);
        assert_eq!(polyline.num_segments(), 2);
        assert_relative_eq!(polyline.arc_length(), 2.0, epsilon = 1e-10);
    }

    #[test]
    fn test_polyline_point_at() {
        let polyline = Polyline::new(vec![Point3::new(0.0, 0.0, 0.0), Point3::new(2.0, 0.0, 0.0)]);

        let start = polyline.point_at(0.0);
        assert_relative_eq!(start.x, 0.0, epsilon = 1e-10);

        let mid = polyline.point_at(0.5);
        assert_relative_eq!(mid.x, 1.0, epsilon = 1e-10);

        let end = polyline.point_at(1.0);
        assert_relative_eq!(end.x, 2.0, epsilon = 1e-10);
    }

    #[test]
    fn test_polyline_arc_length_param() {
        let polyline = Polyline::new(vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(3.0, 0.0, 0.0),
            Point3::new(3.0, 4.0, 0.0),
        ]);

        // Total length: 3 + 4 = 7
        assert_relative_eq!(polyline.arc_length(), 7.0, epsilon = 1e-10);

        // Arc to t conversion
        let t = polyline.arc_to_t(3.5);
        assert_relative_eq!(t, 0.5, epsilon = 1e-10);
    }

    #[test]
    fn test_polyline_tangent() {
        let polyline = Polyline::new(vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
        ]);

        let t1 = polyline.tangent_at(0.25);
        assert_relative_eq!(t1, Vector3::x(), epsilon = 1e-10);

        let t2 = polyline.tangent_at(0.75);
        assert_relative_eq!(t2, Vector3::y(), epsilon = 1e-10);
    }

    #[test]
    fn test_polyline_bend_angles() {
        let polyline = Polyline::new(vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
        ]);

        let angles = polyline.bend_angles();
        assert_eq!(angles.len(), 1);
        assert_relative_eq!(angles[0], std::f64::consts::FRAC_PI_2, epsilon = 1e-10);
    }

    #[test]
    fn test_polyline_simplify() {
        let polyline = Polyline::new(vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(2.0, 0.0, 0.0), // Collinear
            Point3::new(3.0, 0.0, 0.0), // Collinear
            Point3::new(3.0, 1.0, 0.0), // 90° turn
        ]);

        let simplified = polyline.simplified(0.1);
        // Should remove collinear points
        assert!(simplified.len() < polyline.len());
        assert_eq!(simplified.len(), 3); // Start, corner, end
    }

    #[test]
    fn test_polyline_resample() {
        let polyline = Polyline::new(vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(10.0, 0.0, 0.0),
        ]);

        let resampled = polyline.resampled(11);
        assert_eq!(resampled.len(), 11);

        // Check uniform spacing
        for i in 0..11 {
            let expected_x = i as f64;
            assert_relative_eq!(resampled.vertices()[i].x, expected_x, epsilon = 1e-10);
        }
    }

    #[test]
    fn test_polyline_reversed() {
        let polyline = Polyline::new(vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(2.0, 0.0, 0.0),
        ]);

        let reversed = polyline.reversed();
        assert_relative_eq!(reversed.vertices()[0].x, 2.0, epsilon = 1e-10);
        assert_relative_eq!(reversed.vertices()[2].x, 0.0, epsilon = 1e-10);
    }

    #[test]
    fn test_tubular_polyline() {
        let tubular = TubularPolyline::new(
            vec![Point3::new(0.0, 0.0, 0.0), Point3::new(10.0, 0.0, 0.0)],
            vec![1.0, 2.0],
        );

        assert_relative_eq!(tubular.radius_at(0.0), 1.0, epsilon = 1e-10);
        assert_relative_eq!(tubular.radius_at(0.5), 1.5, epsilon = 1e-10);
        assert_relative_eq!(tubular.radius_at(1.0), 2.0, epsilon = 1e-10);
    }

    #[test]
    fn test_bounding_box() {
        let polyline = Polyline::new(vec![
            Point3::new(1.0, 2.0, 3.0),
            Point3::new(4.0, 5.0, 6.0),
            Point3::new(-1.0, 0.0, 4.0),
        ]);

        let (min, max) = polyline.bounding_box();
        assert_relative_eq!(min.x, -1.0, epsilon = 1e-10);
        assert_relative_eq!(min.y, 0.0, epsilon = 1e-10);
        assert_relative_eq!(min.z, 3.0, epsilon = 1e-10);
        assert_relative_eq!(max.x, 4.0, epsilon = 1e-10);
        assert_relative_eq!(max.y, 5.0, epsilon = 1e-10);
        assert_relative_eq!(max.z, 6.0, epsilon = 1e-10);
    }
}
