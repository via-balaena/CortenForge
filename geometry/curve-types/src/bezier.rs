//! Bézier curve types.
//!
//! This module provides quadratic and cubic Bézier curves, as well as
//! splines composed of multiple Bézier segments.

use crate::{Curve, CurveError, Result};
use nalgebra::{Point3, Vector3};

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

/// Continuity level between curve segments.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Hash, Default)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub enum Continuity {
    /// C0: Position continuity only (curves meet at endpoints).
    C0,
    /// C1: First derivative (tangent direction and magnitude) is continuous.
    #[default]
    C1,
    /// G1: Geometric continuity (tangent direction is continuous, magnitude may differ).
    G1,
    /// C2: Second derivative (curvature) is continuous.
    C2,
}

/// A quadratic Bézier curve defined by 3 control points.
///
/// The curve passes through the first and last control points,
/// while the middle control point "pulls" the curve toward it.
///
/// # Equation
///
/// ```text
/// B(t) = (1-t)²P₀ + 2(1-t)tP₁ + t²P₂
/// ```
///
/// # Example
///
/// ```
/// use curve_types::{QuadraticBezier, Curve};
/// use nalgebra::Point3;
///
/// let curve = QuadraticBezier::new(
///     Point3::new(0.0, 0.0, 0.0),
///     Point3::new(1.0, 2.0, 0.0),
///     Point3::new(2.0, 0.0, 0.0),
/// );
///
/// let mid = curve.point_at(0.5);
/// // Midpoint is pulled toward control point
/// assert!(mid.y > 0.0);
/// ```
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct QuadraticBezier {
    /// Start point.
    pub p0: Point3<f64>,
    /// Control point.
    pub p1: Point3<f64>,
    /// End point.
    pub p2: Point3<f64>,
}

impl QuadraticBezier {
    /// Create a new quadratic Bézier curve.
    #[must_use]
    pub const fn new(p0: Point3<f64>, p1: Point3<f64>, p2: Point3<f64>) -> Self {
        Self { p0, p1, p2 }
    }

    /// Get the control points as an array.
    #[must_use]
    pub fn control_points(&self) -> [Point3<f64>; 3] {
        [self.p0, self.p1, self.p2]
    }

    /// Elevate the degree to produce an equivalent cubic Bézier.
    ///
    /// This is useful when you need to combine quadratic and cubic curves.
    #[must_use]
    pub fn elevate(&self) -> CubicBezier {
        // Degree elevation formulas
        let q0 = self.p0;
        let q1 = Point3::from(self.p0.coords * (1.0 / 3.0) + self.p1.coords * (2.0 / 3.0));
        let q2 = Point3::from(self.p1.coords * (2.0 / 3.0) + self.p2.coords * (1.0 / 3.0));
        let q3 = self.p2;

        CubicBezier::new(q0, q1, q2, q3)
    }

    /// Split the curve at parameter `t`, returning two quadratic curves.
    #[must_use]
    pub fn split(&self, t: f64) -> (Self, Self) {
        let t = t.clamp(0.0, 1.0);

        // De Casteljau's algorithm
        let p01 = lerp_point(self.p0, self.p1, t);
        let p12 = lerp_point(self.p1, self.p2, t);
        let p012 = lerp_point(p01, p12, t);

        let left = Self::new(self.p0, p01, p012);
        let right = Self::new(p012, p12, self.p2);

        (left, right)
    }
}

impl Curve for QuadraticBezier {
    fn point_at(&self, t: f64) -> Point3<f64> {
        let t = t.clamp(0.0, 1.0);
        let s = 1.0 - t;

        Point3::from(
            self.p0.coords * (s * s) + self.p1.coords * (2.0 * s * t) + self.p2.coords * (t * t),
        )
    }

    fn tangent_at(&self, t: f64) -> Vector3<f64> {
        self.derivative_at(t).normalize()
    }

    fn derivative_at(&self, t: f64) -> Vector3<f64> {
        let t = t.clamp(0.0, 1.0);
        let s = 1.0 - t;

        // B'(t) = 2(1-t)(P₁-P₀) + 2t(P₂-P₁)
        (self.p1 - self.p0) * (2.0 * s) + (self.p2 - self.p1) * (2.0 * t)
    }

    fn second_derivative_at(&self, _t: f64) -> Vector3<f64> {
        // B''(t) = 2(P₂ - 2P₁ + P₀) (constant)
        (self.p2.coords - self.p1.coords * 2.0 + self.p0.coords) * 2.0
    }
}

/// A cubic Bézier curve defined by 4 control points.
///
/// This is the most commonly used Bézier curve in CAD and graphics.
/// The curve passes through P₀ and P₃, and is tangent to P₀P₁ at the
/// start and P₂P₃ at the end.
///
/// # Equation
///
/// ```text
/// B(t) = (1-t)³P₀ + 3(1-t)²tP₁ + 3(1-t)t²P₂ + t³P₃
/// ```
///
/// # Example
///
/// ```
/// use curve_types::{CubicBezier, Curve};
/// use nalgebra::Point3;
///
/// let curve = CubicBezier::new(
///     Point3::new(0.0, 0.0, 0.0),
///     Point3::new(1.0, 2.0, 0.0),
///     Point3::new(3.0, 2.0, 0.0),
///     Point3::new(4.0, 0.0, 0.0),
/// );
///
/// // The curve starts and ends at P0 and P3
/// let start = curve.point_at(0.0);
/// assert!((start.x - 0.0).abs() < 1e-10);
///
/// let end = curve.point_at(1.0);
/// assert!((end.x - 4.0).abs() < 1e-10);
/// ```
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct CubicBezier {
    /// Start point.
    pub p0: Point3<f64>,
    /// First control point (affects start tangent).
    pub p1: Point3<f64>,
    /// Second control point (affects end tangent).
    pub p2: Point3<f64>,
    /// End point.
    pub p3: Point3<f64>,
}

impl CubicBezier {
    /// Create a new cubic Bézier curve.
    #[must_use]
    pub const fn new(p0: Point3<f64>, p1: Point3<f64>, p2: Point3<f64>, p3: Point3<f64>) -> Self {
        Self { p0, p1, p2, p3 }
    }

    /// Create a cubic Bézier from endpoints and tangent vectors.
    ///
    /// The control points are placed to achieve the specified tangent
    /// directions at the endpoints. The tangent magnitudes control
    /// how strongly the curve is "pulled" in each direction.
    ///
    /// # Parameters
    ///
    /// - `start`: Start point
    /// - `start_tangent`: Tangent direction at start (not necessarily unit)
    /// - `end`: End point
    /// - `end_tangent`: Tangent direction at end (not necessarily unit)
    #[must_use]
    pub fn from_hermite(
        start: Point3<f64>,
        start_tangent: Vector3<f64>,
        end: Point3<f64>,
        end_tangent: Vector3<f64>,
    ) -> Self {
        // Hermite to Bézier conversion
        let p0 = start;
        let p1 = start + start_tangent / 3.0;
        let p2 = end - end_tangent / 3.0;
        let p3 = end;

        Self::new(p0, p1, p2, p3)
    }

    /// Get the control points as an array.
    #[must_use]
    pub fn control_points(&self) -> [Point3<f64>; 4] {
        [self.p0, self.p1, self.p2, self.p3]
    }

    /// Split the curve at parameter `t`, returning two cubic curves.
    #[must_use]
    pub fn split(&self, t: f64) -> (Self, Self) {
        let t = t.clamp(0.0, 1.0);

        // De Casteljau's algorithm
        let p01 = lerp_point(self.p0, self.p1, t);
        let p12 = lerp_point(self.p1, self.p2, t);
        let p23 = lerp_point(self.p2, self.p3, t);

        let p012 = lerp_point(p01, p12, t);
        let p123 = lerp_point(p12, p23, t);

        let p0123 = lerp_point(p012, p123, t);

        let left = Self::new(self.p0, p01, p012, p0123);
        let right = Self::new(p0123, p123, p23, self.p3);

        (left, right)
    }

    /// Compute the convex hull of the control points.
    ///
    /// The curve is guaranteed to lie within this convex hull.
    #[must_use]
    pub fn control_polygon_length(&self) -> f64 {
        (self.p1 - self.p0).norm() + (self.p2 - self.p1).norm() + (self.p3 - self.p2).norm()
    }

    /// Check if the curve is approximately flat within tolerance.
    ///
    /// This is useful for adaptive rendering/tessellation.
    #[must_use]
    pub fn is_flat(&self, tolerance: f64) -> bool {
        // Check if control points are close to the line P0-P3
        let chord = self.p3 - self.p0;
        let chord_len = chord.norm();

        if chord_len < 1e-10 {
            // Degenerate case
            return (self.p1 - self.p0).norm() < tolerance
                && (self.p2 - self.p0).norm() < tolerance;
        }

        let chord_dir = chord / chord_len;

        // Distance from P1 to line P0-P3
        let v1 = self.p1 - self.p0;
        let d1 = (v1 - chord_dir * v1.dot(&chord_dir)).norm();

        // Distance from P2 to line P0-P3
        let v2 = self.p2 - self.p0;
        let d2 = (v2 - chord_dir * v2.dot(&chord_dir)).norm();

        d1 < tolerance && d2 < tolerance
    }

    /// Approximate the curve with a polyline using adaptive subdivision.
    #[must_use]
    pub fn to_polyline(&self, tolerance: f64) -> Vec<Point3<f64>> {
        let mut result = Vec::new();
        result.push(self.p0);
        self.subdivide_to_polyline(tolerance, &mut result);
        result
    }

    fn subdivide_to_polyline(&self, tolerance: f64, result: &mut Vec<Point3<f64>>) {
        if self.is_flat(tolerance) {
            result.push(self.p3);
        } else {
            let (left, right) = self.split(0.5);
            left.subdivide_to_polyline(tolerance, result);
            right.subdivide_to_polyline(tolerance, result);
        }
    }
}

impl Curve for CubicBezier {
    fn point_at(&self, t: f64) -> Point3<f64> {
        let t = t.clamp(0.0, 1.0);
        let s = 1.0 - t;
        let s2 = s * s;
        let t2 = t * t;

        Point3::from(
            self.p0.coords * (s2 * s)
                + self.p1.coords * (3.0 * s2 * t)
                + self.p2.coords * (3.0 * s * t2)
                + self.p3.coords * (t2 * t),
        )
    }

    fn tangent_at(&self, t: f64) -> Vector3<f64> {
        let d = self.derivative_at(t);
        let norm = d.norm();
        if norm > 1e-10 {
            d / norm
        } else {
            // Degenerate case: try second derivative
            let d2 = self.second_derivative_at(t);
            if d2.norm() > 1e-10 {
                d2.normalize()
            } else {
                Vector3::x()
            }
        }
    }

    fn derivative_at(&self, t: f64) -> Vector3<f64> {
        let t = t.clamp(0.0, 1.0);
        let s = 1.0 - t;

        // B'(t) = 3(1-t)²(P₁-P₀) + 6(1-t)t(P₂-P₁) + 3t²(P₃-P₂)
        (self.p1 - self.p0) * (3.0 * s * s)
            + (self.p2 - self.p1) * (6.0 * s * t)
            + (self.p3 - self.p2) * (3.0 * t * t)
    }

    fn second_derivative_at(&self, t: f64) -> Vector3<f64> {
        let t = t.clamp(0.0, 1.0);
        let s = 1.0 - t;

        // B''(t) = 6(1-t)(P₂ - 2P₁ + P₀) + 6t(P₃ - 2P₂ + P₁)
        let a = self.p2.coords - self.p1.coords * 2.0 + self.p0.coords;
        let b = self.p3.coords - self.p2.coords * 2.0 + self.p1.coords;

        a * (6.0 * s) + b * (6.0 * t)
    }
}

/// A spline composed of multiple Bézier segments.
///
/// The segments can have different continuity levels at the joints.
/// By default, C1 continuity is maintained.
///
/// # Example
///
/// ```
/// use curve_types::{BezierSpline, CubicBezier, Continuity, Curve};
/// use nalgebra::Point3;
///
/// // Create a spline with two cubic segments
/// let spline = BezierSpline::from_segments(vec![
///     CubicBezier::new(
///         Point3::new(0.0, 0.0, 0.0),
///         Point3::new(1.0, 1.0, 0.0),
///         Point3::new(2.0, 1.0, 0.0),
///         Point3::new(3.0, 0.0, 0.0),
///     ),
///     CubicBezier::new(
///         Point3::new(3.0, 0.0, 0.0),
///         Point3::new(4.0, -1.0, 0.0),
///         Point3::new(5.0, -1.0, 0.0),
///         Point3::new(6.0, 0.0, 0.0),
///     ),
/// ]);
///
/// assert_eq!(spline.num_segments(), 2);
/// ```
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct BezierSpline {
    /// The cubic Bézier segments.
    segments: Vec<CubicBezier>,
    /// Cumulative parameter ranges for each segment.
    /// segment i covers t in [param_ranges[i], param_ranges[i+1])
    param_ranges: Vec<f64>,
    /// Total arc length (cached).
    total_length: f64,
}

impl BezierSpline {
    /// Create a spline from pre-made cubic Bézier segments.
    ///
    /// # Panics
    ///
    /// Panics if no segments are provided.
    #[must_use]
    pub fn from_segments(segments: Vec<CubicBezier>) -> Self {
        assert!(
            !segments.is_empty(),
            "BezierSpline requires at least one segment"
        );

        // Compute arc lengths for each segment
        let mut arc_lengths: Vec<f64> = segments.iter().map(|seg| seg.arc_length()).collect();

        let total_length: f64 = arc_lengths.iter().sum();

        // Compute parameter ranges based on arc length
        let mut param_ranges = Vec::with_capacity(segments.len() + 1);
        param_ranges.push(0.0);

        let mut cumulative = 0.0;
        for len in &mut arc_lengths {
            cumulative += *len;
            param_ranges.push(cumulative / total_length);
        }
        // Ensure last value is exactly 1.0
        if let Some(last) = param_ranges.last_mut() {
            *last = 1.0;
        }

        Self {
            segments,
            param_ranges,
            total_length,
        }
    }

    /// Create a spline that passes through the given points with C1 continuity.
    ///
    /// Uses Catmull-Rom-like interpolation to compute control points.
    ///
    /// # Errors
    ///
    /// Returns error if fewer than 2 points are provided.
    pub fn through_points(points: &[Point3<f64>]) -> Result<Self> {
        if points.len() < 2 {
            return Err(CurveError::insufficient_points(2, points.len()));
        }

        if points.len() == 2 {
            // Simple linear segment
            let seg = CubicBezier::new(
                points[0],
                lerp_point(points[0], points[1], 1.0 / 3.0),
                lerp_point(points[0], points[1], 2.0 / 3.0),
                points[1],
            );
            return Ok(Self::from_segments(vec![seg]));
        }

        let mut segments = Vec::with_capacity(points.len() - 1);

        for i in 0..points.len() - 1 {
            let p0 = points[i];
            let p3 = points[i + 1];

            // Compute tangents using Catmull-Rom approach
            let tangent_start = if i == 0 {
                p3 - p0
            } else {
                (points[i + 1] - points[i - 1]) * 0.5
            };

            let tangent_end = if i == points.len() - 2 {
                p3 - p0
            } else {
                (points[i + 2] - points[i]) * 0.5
            };

            let p1 = p0 + tangent_start / 3.0;
            let p2 = p3 - tangent_end / 3.0;

            segments.push(CubicBezier::new(p0, p1, p2, p3));
        }

        Ok(Self::from_segments(segments))
    }

    /// Get the number of segments.
    #[must_use]
    pub fn num_segments(&self) -> usize {
        self.segments.len()
    }

    /// Get a specific segment.
    #[must_use]
    pub fn segment(&self, index: usize) -> Option<&CubicBezier> {
        self.segments.get(index)
    }

    /// Get all segments.
    #[must_use]
    pub fn segments(&self) -> &[CubicBezier] {
        &self.segments
    }

    /// Find which segment contains parameter `t` and compute the local parameter.
    fn segment_at(&self, t: f64) -> (usize, f64) {
        if t <= 0.0 {
            return (0, 0.0);
        }
        if t >= 1.0 {
            return (self.segments.len() - 1, 1.0);
        }

        // Binary search for segment
        let mut lo = 0;
        let mut hi = self.segments.len();

        while lo < hi {
            let mid = (lo + hi) / 2;
            if self.param_ranges[mid + 1] <= t {
                lo = mid + 1;
            } else {
                hi = mid;
            }
        }

        let seg_idx = lo.min(self.segments.len() - 1);
        let t_start = self.param_ranges[seg_idx];
        let t_end = self.param_ranges[seg_idx + 1];

        let local_t = if (t_end - t_start).abs() > 1e-10 {
            (t - t_start) / (t_end - t_start)
        } else {
            0.0
        };

        (seg_idx, local_t)
    }

    /// Check continuity at all joints.
    #[must_use]
    pub fn verify_continuity(&self, level: Continuity, tolerance: f64) -> Vec<bool> {
        if self.segments.len() < 2 {
            return Vec::new();
        }

        (0..self.segments.len() - 1)
            .map(|i| {
                let seg_a = &self.segments[i];
                let seg_b = &self.segments[i + 1];

                match level {
                    Continuity::C0 => (seg_a.p3 - seg_b.p0).norm() < tolerance,
                    Continuity::G1 => {
                        let pos_ok = (seg_a.p3 - seg_b.p0).norm() < tolerance;
                        let tan_a = seg_a.tangent_at(1.0);
                        let tan_b = seg_b.tangent_at(0.0);
                        let tan_ok = (tan_a - tan_b).norm() < tolerance
                            || (tan_a + tan_b).norm() < tolerance;
                        pos_ok && tan_ok
                    }
                    Continuity::C1 => {
                        let pos_ok = (seg_a.p3 - seg_b.p0).norm() < tolerance;
                        let deriv_a = seg_a.derivative_at(1.0);
                        let deriv_b = seg_b.derivative_at(0.0);
                        let deriv_ok = (deriv_a - deriv_b).norm() < tolerance;
                        pos_ok && deriv_ok
                    }
                    Continuity::C2 => {
                        let c1_ok = {
                            let pos_ok = (seg_a.p3 - seg_b.p0).norm() < tolerance;
                            let deriv_a = seg_a.derivative_at(1.0);
                            let deriv_b = seg_b.derivative_at(0.0);
                            let deriv_ok = (deriv_a - deriv_b).norm() < tolerance;
                            pos_ok && deriv_ok
                        };
                        let d2_a = seg_a.second_derivative_at(1.0);
                        let d2_b = seg_b.second_derivative_at(0.0);
                        let d2_ok = (d2_a - d2_b).norm() < tolerance;
                        c1_ok && d2_ok
                    }
                }
            })
            .collect()
    }
}

impl Curve for BezierSpline {
    fn point_at(&self, t: f64) -> Point3<f64> {
        let (seg_idx, local_t) = self.segment_at(t);
        self.segments[seg_idx].point_at(local_t)
    }

    fn tangent_at(&self, t: f64) -> Vector3<f64> {
        let (seg_idx, local_t) = self.segment_at(t);
        self.segments[seg_idx].tangent_at(local_t)
    }

    fn derivative_at(&self, t: f64) -> Vector3<f64> {
        let (seg_idx, local_t) = self.segment_at(t);
        // Scale by segment duration for correct global derivative
        let seg_duration = self.param_ranges[seg_idx + 1] - self.param_ranges[seg_idx];
        self.segments[seg_idx].derivative_at(local_t) / seg_duration
    }

    fn second_derivative_at(&self, t: f64) -> Vector3<f64> {
        let (seg_idx, local_t) = self.segment_at(t);
        let seg_duration = self.param_ranges[seg_idx + 1] - self.param_ranges[seg_idx];
        self.segments[seg_idx].second_derivative_at(local_t) / (seg_duration * seg_duration)
    }

    fn arc_length(&self) -> f64 {
        self.total_length
    }
}

/// Linear interpolation between two points.
#[inline]
fn lerp_point(a: Point3<f64>, b: Point3<f64>, t: f64) -> Point3<f64> {
    Point3::from(a.coords * (1.0 - t) + b.coords * t)
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn test_quadratic_bezier() {
        let curve = QuadraticBezier::new(
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 2.0, 0.0),
            Point3::new(2.0, 0.0, 0.0),
        );

        // Endpoints
        assert_relative_eq!(curve.point_at(0.0).coords, curve.p0.coords, epsilon = 1e-10);
        assert_relative_eq!(curve.point_at(1.0).coords, curve.p2.coords, epsilon = 1e-10);

        // Midpoint (should be pulled toward control point)
        let mid = curve.point_at(0.5);
        assert_relative_eq!(mid.x, 1.0, epsilon = 1e-10);
        assert!(mid.y > 0.0); // Pulled up by control point
    }

    #[test]
    fn test_quadratic_elevation() {
        let quad = QuadraticBezier::new(
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
            Point3::new(2.0, 0.0, 0.0),
        );

        let cubic = quad.elevate();

        // They should produce the same curve
        for i in 0..=10 {
            let t = i as f64 / 10.0;
            let p_quad = quad.point_at(t);
            let p_cubic = cubic.point_at(t);
            assert_relative_eq!(p_quad.coords, p_cubic.coords, epsilon = 1e-10);
        }
    }

    #[test]
    fn test_cubic_bezier() {
        let curve = CubicBezier::new(
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 2.0, 0.0),
            Point3::new(3.0, 2.0, 0.0),
            Point3::new(4.0, 0.0, 0.0),
        );

        // Endpoints
        assert_relative_eq!(curve.point_at(0.0).coords, curve.p0.coords, epsilon = 1e-10);
        assert_relative_eq!(curve.point_at(1.0).coords, curve.p3.coords, epsilon = 1e-10);

        // Tangent at start should point toward P1
        let tan_start = curve.tangent_at(0.0);
        let expected = (curve.p1 - curve.p0).normalize();
        assert_relative_eq!(tan_start, expected, epsilon = 1e-10);
    }

    #[test]
    fn test_cubic_from_hermite() {
        let start = Point3::new(0.0, 0.0, 0.0);
        let end = Point3::new(4.0, 0.0, 0.0);
        let start_tan = Vector3::new(3.0, 3.0, 0.0);
        let end_tan = Vector3::new(3.0, -3.0, 0.0);

        let curve = CubicBezier::from_hermite(start, start_tan, end, end_tan);

        assert_relative_eq!(curve.point_at(0.0).coords, start.coords, epsilon = 1e-10);
        assert_relative_eq!(curve.point_at(1.0).coords, end.coords, epsilon = 1e-10);
    }

    #[test]
    fn test_cubic_split() {
        let curve = CubicBezier::new(
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 2.0, 0.0),
            Point3::new(3.0, 2.0, 0.0),
            Point3::new(4.0, 0.0, 0.0),
        );

        let (left, right) = curve.split(0.5);

        // Split point should match
        let split_point = curve.point_at(0.5);
        assert_relative_eq!(
            left.point_at(1.0).coords,
            split_point.coords,
            epsilon = 1e-10
        );
        assert_relative_eq!(
            right.point_at(0.0).coords,
            split_point.coords,
            epsilon = 1e-10
        );

        // Original endpoints preserved
        assert_relative_eq!(left.point_at(0.0).coords, curve.p0.coords, epsilon = 1e-10);
        assert_relative_eq!(right.point_at(1.0).coords, curve.p3.coords, epsilon = 1e-10);
    }

    #[test]
    fn test_cubic_is_flat() {
        // A nearly straight curve
        let flat = CubicBezier::new(
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(2.0, 0.0, 0.0),
            Point3::new(3.0, 0.0, 0.0),
        );
        assert!(flat.is_flat(0.01));

        // A curved segment
        let curved = CubicBezier::new(
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(0.0, 2.0, 0.0),
            Point3::new(3.0, 2.0, 0.0),
            Point3::new(3.0, 0.0, 0.0),
        );
        assert!(!curved.is_flat(0.1));
    }

    #[test]
    #[allow(clippy::unwrap_used)]
    fn test_bezier_spline() {
        let spline = BezierSpline::from_segments(vec![
            CubicBezier::new(
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(1.0, 1.0, 0.0),
                Point3::new(2.0, 1.0, 0.0),
                Point3::new(3.0, 0.0, 0.0),
            ),
            CubicBezier::new(
                Point3::new(3.0, 0.0, 0.0),
                Point3::new(4.0, -1.0, 0.0),
                Point3::new(5.0, -1.0, 0.0),
                Point3::new(6.0, 0.0, 0.0),
            ),
        ]);

        assert_eq!(spline.num_segments(), 2);

        // Endpoints
        assert_relative_eq!(spline.point_at(0.0).x, 0.0, epsilon = 1e-10);
        assert_relative_eq!(spline.point_at(1.0).x, 6.0, epsilon = 1e-10);

        // C0 continuity
        let continuity = spline.verify_continuity(Continuity::C0, 1e-10);
        assert_eq!(continuity.len(), 1);
        assert!(continuity[0]);
    }

    #[test]
    #[allow(clippy::unwrap_used)]
    fn test_bezier_spline_through_points() {
        let points = vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
            Point3::new(2.0, 0.0, 0.0),
            Point3::new(3.0, 1.0, 0.0),
        ];

        let spline = BezierSpline::through_points(&points).unwrap();

        // Should pass through all points
        assert_eq!(spline.num_segments(), 3);

        // Check interpolation at knots (approximately)
        assert_relative_eq!(
            spline.point_at(0.0).coords,
            points[0].coords,
            epsilon = 1e-10
        );
        assert_relative_eq!(
            spline.point_at(1.0).coords,
            points[3].coords,
            epsilon = 1e-10
        );
    }
}
