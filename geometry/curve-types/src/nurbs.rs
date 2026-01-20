//! Non-uniform rational B-spline (NURBS) curves.
//!
//! NURBS are a generalization of B-splines that can exactly represent
//! conic sections (circles, ellipses, parabolas, hyperbolas) and other
//! curves that B-splines cannot.

use crate::{Curve, CurveError, Result};
use nalgebra::{Point3, Point4, Vector3};

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

/// A non-uniform rational B-spline (NURBS) curve.
///
/// NURBS extend B-splines by adding weights to control points, which allows
/// exact representation of conic sections. A NURBS curve is defined as:
///
/// ```text
/// C(u) = Σ N_i,p(u) * w_i * P_i / Σ N_i,p(u) * w_i
/// ```
///
/// where:
/// - `P_i` are control points
/// - `w_i` are weights (positive values)
/// - `N_i,p(u)` are B-spline basis functions of degree `p`
///
/// # Weights
///
/// - Weight = 1.0: Standard B-spline behavior
/// - Weight > 1.0: Curve is pulled toward the control point
/// - Weight < 1.0: Curve is pushed away from the control point
/// - Weight = 0.0: Invalid (would cause division by zero)
///
/// # Example
///
/// ```
/// use curve_types::{Nurbs, Curve};
/// use nalgebra::Point3;
///
/// // Create a NURBS curve
/// let control_points = vec![
///     Point3::new(0.0, 0.0, 0.0),
///     Point3::new(1.0, 2.0, 0.0),
///     Point3::new(3.0, 2.0, 0.0),
///     Point3::new(4.0, 0.0, 0.0),
/// ];
/// let weights = vec![1.0, 1.0, 1.0, 1.0]; // Unit weights = B-spline
///
/// let nurbs = Nurbs::clamped(control_points, weights, 3).unwrap();
/// ```
///
/// # Representing Circles
///
/// NURBS can exactly represent circles using quadratic basis with specific
/// weights. See [`Nurbs::circle`] for constructing circular arcs.
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct Nurbs {
    /// Control points in homogeneous coordinates (weighted).
    /// Each point is stored as (w*x, w*y, w*z, w).
    homogeneous_points: Vec<Point4<f64>>,
    /// Knot vector.
    knots: Vec<f64>,
    /// Degree of the NURBS.
    degree: usize,
}

impl Nurbs {
    /// Create a NURBS curve with explicit knot vector.
    ///
    /// # Parameters
    ///
    /// - `control_points`: The control points
    /// - `weights`: Weights for each control point (must be positive)
    /// - `knots`: The knot vector
    /// - `degree`: The polynomial degree
    ///
    /// # Errors
    ///
    /// Returns error if:
    /// - Control points and weights have different lengths
    /// - Fewer than `degree + 1` control points
    /// - Knot vector has incorrect length
    /// - Knot vector is not non-decreasing
    /// - Any weight is non-positive
    pub fn new(
        control_points: Vec<Point3<f64>>,
        weights: Vec<f64>,
        knots: Vec<f64>,
        degree: usize,
    ) -> Result<Self> {
        let n = control_points.len();

        if weights.len() != n {
            return Err(CurveError::invalid_knot_vector(format!(
                "control points ({}) and weights ({}) must have same length",
                n,
                weights.len()
            )));
        }

        // Validate weights
        for (i, &w) in weights.iter().enumerate() {
            if w <= 0.0 {
                return Err(CurveError::InvalidWeight { index: i, value: w });
            }
        }

        // Need at least degree + 1 control points
        if n < degree + 1 {
            return Err(CurveError::insufficient_points(degree + 1, n));
        }

        // Knot vector length must be n + degree + 1
        let expected_knots = n + degree + 1;
        if knots.len() != expected_knots {
            return Err(CurveError::invalid_knot_vector(format!(
                "expected {} knots, got {}",
                expected_knots,
                knots.len()
            )));
        }

        // Knots must be non-decreasing
        for i in 1..knots.len() {
            if knots[i] < knots[i - 1] {
                return Err(CurveError::invalid_knot_vector(format!(
                    "knot vector not non-decreasing at index {}",
                    i
                )));
            }
        }

        // Convert to homogeneous coordinates
        let homogeneous_points: Vec<Point4<f64>> = control_points
            .iter()
            .zip(weights.iter())
            .map(|(p, &w)| Point4::new(p.x * w, p.y * w, p.z * w, w))
            .collect();

        Ok(Self {
            homogeneous_points,
            knots,
            degree,
        })
    }

    /// Create a clamped NURBS with uniform interior knots.
    ///
    /// # Parameters
    ///
    /// - `control_points`: The control points
    /// - `weights`: Weights for each control point
    /// - `degree`: The polynomial degree
    pub fn clamped(
        control_points: Vec<Point3<f64>>,
        weights: Vec<f64>,
        degree: usize,
    ) -> Result<Self> {
        let n = control_points.len();

        if n < degree + 1 {
            return Err(CurveError::insufficient_points(degree + 1, n));
        }

        // Generate clamped knot vector
        let mut knots = Vec::with_capacity(n + degree + 1);

        for _ in 0..=degree {
            knots.push(0.0);
        }

        let num_interior = n - degree - 1;
        for i in 1..=num_interior {
            knots.push(i as f64 / (num_interior + 1) as f64);
        }

        for _ in 0..=degree {
            knots.push(1.0);
        }

        Self::new(control_points, weights, knots, degree)
    }

    /// Create a circular arc using NURBS.
    ///
    /// Uses a quadratic NURBS with appropriate weights to exactly represent
    /// a circular arc.
    ///
    /// # Parameters
    ///
    /// - `center`: Center of the circle
    /// - `radius`: Radius of the circle
    /// - `start_angle`: Start angle in radians
    /// - `end_angle`: End angle in radians
    /// - `normal`: Normal vector defining the plane of the circle
    ///
    /// # Example
    ///
    /// ```
    /// use curve_types::{Nurbs, Curve};
    /// use nalgebra::{Point3, Vector3};
    /// use std::f64::consts::PI;
    ///
    /// // Create a semicircle
    /// let arc = Nurbs::circular_arc(
    ///     Point3::new(0.0, 0.0, 0.0),
    ///     1.0,
    ///     0.0,
    ///     PI,
    ///     Vector3::z(),
    /// ).unwrap();
    ///
    /// // Points on the arc are exactly on the circle
    /// let mid = arc.point_at(0.5);
    /// let dist = (mid.coords - Point3::new(0.0, 1.0, 0.0).coords).norm();
    /// assert!(dist < 1e-10);
    /// ```
    pub fn circular_arc(
        center: Point3<f64>,
        radius: f64,
        start_angle: f64,
        end_angle: f64,
        normal: Vector3<f64>,
    ) -> Result<Self> {
        if radius <= 0.0 {
            return Err(CurveError::InvalidRadius(radius));
        }

        let mut angle = end_angle - start_angle;
        if angle.abs() < 1e-10 {
            return Err(CurveError::InvalidAngle {
                reason: "start and end angles are equal".to_string(),
            });
        }

        // Normalize angle to [0, 2π]
        while angle < 0.0 {
            angle += 2.0 * std::f64::consts::PI;
        }
        while angle > 2.0 * std::f64::consts::PI {
            angle -= 2.0 * std::f64::consts::PI;
        }

        // Determine number of segments needed (max 90° per segment for good weights)
        let num_arcs = ((angle / std::f64::consts::FRAC_PI_2).ceil() as usize).max(1);
        let arc_angle = angle / num_arcs as f64;

        // Build local coordinate system (same as arc.rs)
        let normal = normal.normalize();
        let reference = if normal.z.abs() < 0.9 {
            Vector3::z()
        } else {
            Vector3::x()
        };
        let v_axis = normal.cross(&reference).normalize();
        let u_axis = v_axis.cross(&normal);

        // Weight for arc control points
        let w1 = (arc_angle / 2.0).cos();

        let mut control_points = Vec::new();
        let mut weights = Vec::new();

        let mut current_angle = start_angle;

        for i in 0..num_arcs {
            let a0 = current_angle;
            let a1 = current_angle + arc_angle / 2.0;
            let a2 = current_angle + arc_angle;

            let p0 = center + (u_axis * a0.cos() + v_axis * a0.sin()) * radius;
            let p2 = center + (u_axis * a2.cos() + v_axis * a2.sin()) * radius;

            // Middle control point (intersection of tangent lines)
            let p1 = center + (u_axis * a1.cos() + v_axis * a1.sin()) * (radius / w1);

            if i == 0 {
                control_points.push(p0);
                weights.push(1.0);
            }

            control_points.push(p1);
            weights.push(w1);

            control_points.push(p2);
            weights.push(1.0);

            current_angle = a2;
        }

        // Generate clamped quadratic knot vector
        let n = control_points.len();
        let degree = 2;
        let mut knots = Vec::with_capacity(n + degree + 1);

        for _ in 0..=degree {
            knots.push(0.0);
        }

        // Interior knots (2 per arc segment, except last)
        for i in 1..num_arcs {
            let u = i as f64 / num_arcs as f64;
            knots.push(u);
            knots.push(u);
        }

        for _ in 0..=degree {
            knots.push(1.0);
        }

        Self::new(control_points, weights, knots, degree)
    }

    /// Create a full circle using NURBS.
    ///
    /// This creates a closed NURBS curve that exactly represents a circle.
    pub fn circle(center: Point3<f64>, radius: f64, normal: Vector3<f64>) -> Result<Self> {
        Self::circular_arc(
            center,
            radius,
            0.0,
            2.0 * std::f64::consts::PI - 1e-10,
            normal,
        )
    }

    /// Get the control points (in Cartesian coordinates).
    #[must_use]
    pub fn control_points(&self) -> Vec<Point3<f64>> {
        self.homogeneous_points
            .iter()
            .map(|hp| Point3::new(hp.x / hp.w, hp.y / hp.w, hp.z / hp.w))
            .collect()
    }

    /// Get the weights.
    #[must_use]
    pub fn weights(&self) -> Vec<f64> {
        self.homogeneous_points.iter().map(|hp| hp.w).collect()
    }

    /// Get the knot vector.
    #[must_use]
    pub fn knots(&self) -> &[f64] {
        &self.knots
    }

    /// Get the degree.
    #[must_use]
    pub fn degree(&self) -> usize {
        self.degree
    }

    /// Get the number of control points.
    #[must_use]
    pub fn num_control_points(&self) -> usize {
        self.homogeneous_points.len()
    }

    /// Get the valid parameter domain.
    #[must_use]
    pub fn domain(&self) -> (f64, f64) {
        let p = self.degree;
        (self.knots[p], self.knots[self.knots.len() - p - 1])
    }

    /// Find the knot span for parameter `u`.
    fn find_span(&self, u: f64) -> usize {
        let n = self.homogeneous_points.len();
        let p = self.degree;

        if u >= self.knots[n] {
            return n - 1;
        }

        let mut low = p;
        let mut high = n;

        while low < high {
            let mid = (low + high) / 2;
            if self.knots[mid] > u {
                high = mid;
            } else {
                low = mid + 1;
            }
        }

        low - 1
    }

    /// Compute basis functions at parameter `u`.
    fn basis_functions(&self, span: usize, u: f64) -> Vec<f64> {
        let p = self.degree;
        let mut n_basis = vec![0.0; p + 1];
        let mut left = vec![0.0; p + 1];
        let mut right = vec![0.0; p + 1];

        n_basis[0] = 1.0;

        for j in 1..=p {
            left[j] = u - self.knots[span + 1 - j];
            right[j] = self.knots[span + j] - u;

            let mut saved = 0.0;
            for r in 0..j {
                let denom = right[r + 1] + left[j - r];
                if denom.abs() > 1e-15 {
                    let temp = n_basis[r] / denom;
                    n_basis[r] = saved + right[r + 1] * temp;
                    saved = left[j - r] * temp;
                } else {
                    n_basis[r] = saved;
                    saved = 0.0;
                }
            }
            n_basis[j] = saved;
        }

        n_basis
    }

    /// Normalize parameter from [0,1] to domain.
    fn normalize_param(&self, t: f64) -> f64 {
        let (u_min, u_max) = self.domain();
        u_min + t.clamp(0.0, 1.0) * (u_max - u_min)
    }
}

impl Curve for Nurbs {
    fn point_at(&self, t: f64) -> Point3<f64> {
        let u = self.normalize_param(t);
        let span = self.find_span(u);
        let basis = self.basis_functions(span, u);

        // Evaluate in homogeneous coordinates
        let mut hpoint = Point4::new(0.0, 0.0, 0.0, 0.0);
        for i in 0..=self.degree {
            let idx = span - self.degree + i;
            let hp = &self.homogeneous_points[idx];
            hpoint.x += hp.x * basis[i];
            hpoint.y += hp.y * basis[i];
            hpoint.z += hp.z * basis[i];
            hpoint.w += hp.w * basis[i];
        }

        // Project to Cartesian
        Point3::new(
            hpoint.x / hpoint.w,
            hpoint.y / hpoint.w,
            hpoint.z / hpoint.w,
        )
    }

    fn tangent_at(&self, t: f64) -> Vector3<f64> {
        self.derivative_at(t).normalize()
    }

    fn derivative_at(&self, t: f64) -> Vector3<f64> {
        // Numerical derivative
        let h = 1e-7;
        let p1 = self.point_at((t + h).min(1.0));
        let p0 = self.point_at((t - h).max(0.0));
        (p1 - p0) / (2.0 * h)
    }

    fn second_derivative_at(&self, t: f64) -> Vector3<f64> {
        let h = 1e-6;
        let d1 = self.derivative_at((t + h).min(1.0));
        let d0 = self.derivative_at((t - h).max(0.0));
        (d1 - d0) / (2.0 * h)
    }
}

#[cfg(test)]
#[allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::float_cmp,
    clippy::redundant_clone
)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use std::f64::consts::PI;

    #[test]
    #[allow(clippy::unwrap_used)]
    fn test_nurbs_unit_weights() {
        // With unit weights, NURBS should behave like B-spline
        let points = vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 2.0, 0.0),
            Point3::new(3.0, 2.0, 0.0),
            Point3::new(4.0, 0.0, 0.0),
        ];
        let weights = vec![1.0, 1.0, 1.0, 1.0];

        let nurbs = Nurbs::clamped(points.clone(), weights, 3).unwrap();

        // Should pass through endpoints
        assert_relative_eq!(
            nurbs.point_at(0.0).coords,
            points[0].coords,
            epsilon = 1e-10
        );
        assert_relative_eq!(
            nurbs.point_at(1.0).coords,
            points[3].coords,
            epsilon = 1e-10
        );
    }

    #[test]
    #[allow(clippy::unwrap_used)]
    fn test_nurbs_weight_effect() {
        let points = vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 2.0, 0.0),
            Point3::new(2.0, 0.0, 0.0),
        ];

        // Unit weights
        let nurbs1 = Nurbs::clamped(points.clone(), vec![1.0, 1.0, 1.0], 2).unwrap();
        let mid1 = nurbs1.point_at(0.5);

        // Higher middle weight pulls curve toward control point
        let nurbs2 = Nurbs::clamped(points.clone(), vec![1.0, 10.0, 1.0], 2).unwrap();
        let mid2 = nurbs2.point_at(0.5);

        // mid2 should be higher (pulled toward P1)
        assert!(mid2.y > mid1.y);
    }

    #[test]
    #[allow(clippy::unwrap_used)]
    fn test_nurbs_circular_arc() {
        let arc = Nurbs::circular_arc(Point3::origin(), 1.0, 0.0, PI / 2.0, Vector3::z()).unwrap();

        // All points should be on the unit circle
        for i in 0..=10 {
            let t = i as f64 / 10.0;
            let p = arc.point_at(t);
            let dist = (p.x * p.x + p.y * p.y).sqrt();
            assert_relative_eq!(dist, 1.0, epsilon = 1e-6);
        }

        // Start at (1, 0)
        let start = arc.point_at(0.0);
        assert_relative_eq!(start.x, 1.0, epsilon = 1e-6);
        assert_relative_eq!(start.y, 0.0, epsilon = 1e-6);

        // End at (0, 1)
        let end = arc.point_at(1.0);
        assert_relative_eq!(end.x, 0.0, epsilon = 1e-6);
        assert_relative_eq!(end.y, 1.0, epsilon = 1e-6);
    }

    #[test]
    #[allow(clippy::unwrap_used)]
    fn test_nurbs_semicircle() {
        let arc = Nurbs::circular_arc(Point3::origin(), 2.0, 0.0, PI, Vector3::z()).unwrap();

        // All points on circle of radius 2
        for i in 0..=20 {
            let t = i as f64 / 20.0;
            let p = arc.point_at(t);
            let dist = (p.x * p.x + p.y * p.y).sqrt();
            assert_relative_eq!(dist, 2.0, epsilon = 1e-5);
        }
    }

    #[test]
    fn test_nurbs_invalid_weight() {
        let points = vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
            Point3::new(2.0, 0.0, 0.0),
        ];

        let result = Nurbs::clamped(points, vec![1.0, 0.0, 1.0], 2);
        assert!(result.is_err());
    }

    #[test]
    fn test_nurbs_invalid_radius() {
        let result = Nurbs::circular_arc(Point3::origin(), -1.0, 0.0, PI, Vector3::z());
        assert!(result.is_err());
    }
}
