//! B-spline curves.
//!
//! B-splines are a generalization of Bézier curves that provide local control
//! through a knot vector. Changes to a control point only affect a local
//! portion of the curve, making them ideal for interactive design.

use crate::{Curve, CurveError, Result};
use nalgebra::{Point3, Vector3};

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

/// A B-spline curve of arbitrary degree.
///
/// B-splines are defined by:
/// - **Control points**: Define the shape of the curve
/// - **Knot vector**: Defines the parameterization and local influence of control points
/// - **Degree**: Determines smoothness (degree 1 = linear, 2 = quadratic, 3 = cubic)
///
/// The curve generally does not pass through the control points (except at
/// endpoints with suitable knot multiplicity).
///
/// # Knot Vector
///
/// The knot vector must be non-decreasing and have length `n + p + 1`, where
/// `n` is the number of control points and `p` is the degree.
///
/// Common knot vector configurations:
/// - **Uniform**: Equally spaced knots
/// - **Open/Clamped**: Knots repeated at endpoints so curve passes through end control points
/// - **Non-uniform**: Variable spacing for more control
///
/// # Example
///
/// ```
/// use curve_types::{BSpline, Curve};
/// use nalgebra::Point3;
///
/// // Create a cubic B-spline (degree 3)
/// let control_points = vec![
///     Point3::new(0.0, 0.0, 0.0),
///     Point3::new(1.0, 2.0, 0.0),
///     Point3::new(3.0, 2.0, 0.0),
///     Point3::new(4.0, 0.0, 0.0),
///     Point3::new(5.0, -1.0, 0.0),
/// ];
///
/// let spline = BSpline::clamped(control_points, 3).unwrap();
///
/// // Evaluate the curve
/// let mid = spline.point_at(0.5);
/// ```
#[derive(Debug, Clone, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct BSpline {
    /// Control points.
    control_points: Vec<Point3<f64>>,
    /// Knot vector.
    knots: Vec<f64>,
    /// Degree of the B-spline.
    degree: usize,
    /// Cached arc length.
    arc_length_cache: Option<f64>,
}

impl BSpline {
    /// Create a B-spline with explicit knot vector.
    ///
    /// # Parameters
    ///
    /// - `control_points`: The control points
    /// - `knots`: The knot vector (must have length `n + degree + 1`)
    /// - `degree`: The polynomial degree
    ///
    /// # Errors
    ///
    /// Returns error if:
    /// - Fewer than `degree + 1` control points
    /// - Knot vector has incorrect length
    /// - Knot vector is not non-decreasing
    pub fn new(control_points: Vec<Point3<f64>>, knots: Vec<f64>, degree: usize) -> Result<Self> {
        let n = control_points.len();

        // Need at least degree + 1 control points
        if n < degree + 1 {
            return Err(CurveError::insufficient_points(degree + 1, n));
        }

        // Knot vector length must be n + degree + 1
        let expected_knots = n + degree + 1;
        if knots.len() != expected_knots {
            return Err(CurveError::invalid_knot_vector(format!(
                "expected {} knots for {} control points and degree {}, got {}",
                expected_knots,
                n,
                degree,
                knots.len()
            )));
        }

        // Knots must be non-decreasing
        for i in 1..knots.len() {
            if knots[i] < knots[i - 1] {
                return Err(CurveError::invalid_knot_vector(format!(
                    "knot vector is not non-decreasing at index {} ({} < {})",
                    i,
                    knots[i],
                    knots[i - 1]
                )));
            }
        }

        Ok(Self {
            control_points,
            knots,
            degree,
            arc_length_cache: None,
        })
    }

    /// Create a clamped (open) B-spline with uniform interior knots.
    ///
    /// Clamped B-splines pass through their first and last control points,
    /// making them more intuitive for most applications.
    ///
    /// # Parameters
    ///
    /// - `control_points`: The control points
    /// - `degree`: The polynomial degree
    ///
    /// # Errors
    ///
    /// Returns error if fewer than `degree + 1` control points.
    ///
    /// # Example
    ///
    /// ```
    /// use curve_types::{BSpline, Curve};
    /// use nalgebra::Point3;
    ///
    /// let points = vec![
    ///     Point3::new(0.0, 0.0, 0.0),
    ///     Point3::new(1.0, 1.0, 0.0),
    ///     Point3::new(2.0, 0.0, 0.0),
    ///     Point3::new(3.0, 1.0, 0.0),
    /// ];
    ///
    /// let spline = BSpline::clamped(points, 3).unwrap();
    ///
    /// // Curve passes through endpoints
    /// let start = spline.point_at(0.0);
    /// assert!((start.x - 0.0).abs() < 1e-10);
    /// ```
    pub fn clamped(control_points: Vec<Point3<f64>>, degree: usize) -> Result<Self> {
        let n = control_points.len();

        if n < degree + 1 {
            return Err(CurveError::insufficient_points(degree + 1, n));
        }

        // Generate clamped knot vector
        let mut knots = Vec::with_capacity(n + degree + 1);

        // Repeat first knot (degree + 1) times
        for _ in 0..=degree {
            knots.push(0.0);
        }

        // Interior knots (uniform)
        let num_interior = n - degree - 1;
        for i in 1..=num_interior {
            knots.push(i as f64 / (num_interior + 1) as f64);
        }

        // Repeat last knot (degree + 1) times
        for _ in 0..=degree {
            knots.push(1.0);
        }

        Self::new(control_points, knots, degree)
    }

    /// Create a B-spline with uniform knot vector.
    ///
    /// Uniform B-splines have equally-spaced knots throughout.
    /// The curve generally does not pass through the endpoints.
    ///
    /// # Parameters
    ///
    /// - `control_points`: The control points
    /// - `degree`: The polynomial degree
    ///
    /// # Errors
    ///
    /// Returns error if fewer than `degree + 1` control points.
    pub fn uniform(control_points: Vec<Point3<f64>>, degree: usize) -> Result<Self> {
        let n = control_points.len();

        if n < degree + 1 {
            return Err(CurveError::insufficient_points(degree + 1, n));
        }

        // Generate uniform knot vector
        let num_knots = n + degree + 1;
        let knots: Vec<f64> = (0..num_knots)
            .map(|i| i as f64 / (num_knots - 1) as f64)
            .collect();

        Self::new(control_points, knots, degree)
    }

    /// Get the control points.
    #[must_use]
    pub fn control_points(&self) -> &[Point3<f64>] {
        &self.control_points
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

    /// Get the order (degree + 1).
    #[must_use]
    pub fn order(&self) -> usize {
        self.degree + 1
    }

    /// Get the number of control points.
    #[must_use]
    pub fn num_control_points(&self) -> usize {
        self.control_points.len()
    }

    /// Get the valid parameter domain [u_min, u_max].
    ///
    /// For clamped splines, this is typically [0, 1].
    #[must_use]
    pub fn domain(&self) -> (f64, f64) {
        let p = self.degree;
        (self.knots[p], self.knots[self.knots.len() - p - 1])
    }

    /// Find the knot span index for parameter `u`.
    ///
    /// Returns `i` such that `knots[i] <= u < knots[i+1]`.
    fn find_span(&self, u: f64) -> usize {
        let n = self.control_points.len();
        let p = self.degree;

        // Special case: u at or beyond the end
        if u >= self.knots[n] {
            return n - 1;
        }

        // Binary search
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

    /// Compute the non-zero basis functions at parameter `u`.
    ///
    /// Returns an array of `degree + 1` basis function values.
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

    /// Compute basis functions and their first derivatives.
    fn basis_functions_derivs(&self, span: usize, u: f64) -> (Vec<f64>, Vec<f64>) {
        let p = self.degree;

        // For degree 0, no derivatives
        if p == 0 {
            return (vec![1.0], vec![0.0]);
        }

        let mut n_basis = vec![vec![0.0; p + 1]; 2];
        let mut left = vec![0.0; p + 1];
        let mut right = vec![0.0; p + 1];

        // Lower triangular basis functions (for derivative computation)
        let mut ndu = vec![vec![0.0; p + 1]; p + 1];
        ndu[0][0] = 1.0;

        for j in 1..=p {
            left[j] = u - self.knots[span + 1 - j];
            right[j] = self.knots[span + j] - u;

            let mut saved = 0.0;
            for r in 0..j {
                // Lower triangle
                let denom = right[r + 1] + left[j - r];
                ndu[j][r] = if denom.abs() > 1e-15 { denom } else { 1.0 };
                let temp = ndu[r][j - 1] / ndu[j][r];

                // Upper triangle
                ndu[r][j] = saved + right[r + 1] * temp;
                saved = left[j - r] * temp;
            }
            ndu[j][j] = saved;
        }

        // Load basis functions
        for j in 0..=p {
            n_basis[0][j] = ndu[j][p];
        }

        // Compute derivative
        let mut a = vec![vec![0.0; p + 1]; 2];
        for r in 0..=p {
            let mut s1 = 0;
            let mut s2 = 1;
            a[0][0] = 1.0;

            // Compute first derivative
            let rk = r as i32 - 1;
            let pk = (p - 1) as i32;

            if r >= 1 {
                a[s2][0] = a[s1][0] / ndu[pk as usize + 1][rk as usize];
                n_basis[1][r] = a[s2][0];
            }

            let j1 = if rk >= -1 { 1 } else { -rk as usize };
            let j2 = if (r as i32 - 1) <= pk { 1 } else { p - r + 1 };

            for j in j1..j2 {
                let idx = (rk + j as i32) as usize;
                a[s2][j] = (a[s1][j] - a[s1][j - 1]) / ndu[pk as usize + 1][idx];
                n_basis[1][r] += a[s2][j];
            }

            if r <= p - 1 {
                a[s2][1] = -a[s1][0] / ndu[pk as usize + 1][r];
                n_basis[1][r] += a[s2][1];
            }

            // Swap rows
            std::mem::swap(&mut s1, &mut s2);
        }

        // Multiply by factorial
        let factor = p as f64;
        for j in 0..=p {
            n_basis[1][j] *= factor;
        }

        (n_basis[0].clone(), n_basis[1].clone())
    }

    /// Convert the B-spline domain parameter to [0, 1].
    fn normalize_param(&self, t: f64) -> f64 {
        let (u_min, u_max) = self.domain();
        u_min + t.clamp(0.0, 1.0) * (u_max - u_min)
    }

    /// Insert a knot into the B-spline.
    ///
    /// This refines the curve without changing its shape.
    #[must_use]
    pub fn insert_knot(&self, u: f64) -> Self {
        let n = self.control_points.len();
        let p = self.degree;

        // Find span
        let k = self.find_span(u);

        // Compute new control points
        let mut new_points = Vec::with_capacity(n + 1);

        for i in 0..=k - p {
            new_points.push(self.control_points[i]);
        }

        for i in (k - p + 1)..=k {
            let alpha = (u - self.knots[i]) / (self.knots[i + p] - self.knots[i]);
            let pt = Point3::from(
                self.control_points[i - 1].coords * (1.0 - alpha)
                    + self.control_points[i].coords * alpha,
            );
            new_points.push(pt);
        }

        for i in k..n {
            new_points.push(self.control_points[i]);
        }

        // Create new knot vector
        let mut new_knots = Vec::with_capacity(self.knots.len() + 1);
        new_knots.extend_from_slice(&self.knots[..=k]);
        new_knots.push(u);
        new_knots.extend_from_slice(&self.knots[k + 1..]);

        // Unwrap is safe because we're preserving validity
        Self::new(new_points, new_knots, p).unwrap_or_else(|_| self.clone())
    }
}

impl Curve for BSpline {
    fn point_at(&self, t: f64) -> Point3<f64> {
        let u = self.normalize_param(t);
        let span = self.find_span(u);
        let basis = self.basis_functions(span, u);

        let mut point = Vector3::zeros();
        for i in 0..=self.degree {
            let idx = span - self.degree + i;
            point += self.control_points[idx].coords * basis[i];
        }

        Point3::from(point)
    }

    fn tangent_at(&self, t: f64) -> Vector3<f64> {
        self.derivative_at(t).normalize()
    }

    fn derivative_at(&self, t: f64) -> Vector3<f64> {
        let u = self.normalize_param(t);
        let span = self.find_span(u);
        let (_, basis_derivs) = self.basis_functions_derivs(span, u);

        let mut deriv = Vector3::zeros();
        for i in 0..=self.degree {
            let idx = span - self.degree + i;
            deriv += self.control_points[idx].coords * basis_derivs[i];
        }

        // Scale by domain transformation
        let (u_min, u_max) = self.domain();
        deriv * (u_max - u_min)
    }

    fn second_derivative_at(&self, t: f64) -> Vector3<f64> {
        // Numerical approximation for second derivative
        let h = 1e-6;
        let d1 = if t + h <= 1.0 {
            self.derivative_at(t + h)
        } else {
            self.derivative_at(t)
        };
        let d0 = if t - h >= 0.0 {
            self.derivative_at(t - h)
        } else {
            self.derivative_at(t)
        };

        (d1 - d0) / (2.0 * h)
    }

    fn arc_length(&self) -> f64 {
        // Use adaptive Simpson's rule (from trait default, but we cache)
        if let Some(cached) = self.arc_length_cache {
            return cached;
        }
        self.arc_length_between(0.0, 1.0)
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    #[allow(clippy::unwrap_used)]
    fn test_bspline_clamped_cubic() {
        let points = vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 2.0, 0.0),
            Point3::new(3.0, 2.0, 0.0),
            Point3::new(4.0, 0.0, 0.0),
        ];

        let spline = BSpline::clamped(points.clone(), 3).unwrap();

        // Clamped spline should pass through endpoints
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

    #[test]
    #[allow(clippy::unwrap_used)]
    fn test_bspline_clamped_quadratic() {
        let points = vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
            Point3::new(2.0, 0.0, 0.0),
        ];

        let spline = BSpline::clamped(points.clone(), 2).unwrap();

        assert_relative_eq!(
            spline.point_at(0.0).coords,
            points[0].coords,
            epsilon = 1e-10
        );
        assert_relative_eq!(
            spline.point_at(1.0).coords,
            points[2].coords,
            epsilon = 1e-10
        );
    }

    #[test]
    #[allow(clippy::unwrap_used)]
    fn test_bspline_linear() {
        let points = vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
            Point3::new(2.0, 0.0, 0.0),
        ];

        let spline = BSpline::clamped(points, 1).unwrap();

        // Linear B-spline should be piecewise linear through control points
        let mid = spline.point_at(0.5);
        // Midpoint should be at (1, 1)
        assert_relative_eq!(mid.x, 1.0, epsilon = 0.1);
    }

    #[test]
    #[allow(clippy::unwrap_used)]
    fn test_bspline_domain() {
        let points = vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 1.0, 0.0),
            Point3::new(2.0, 0.0, 0.0),
            Point3::new(3.0, 1.0, 0.0),
        ];

        let spline = BSpline::clamped(points, 3).unwrap();
        let (u_min, u_max) = spline.domain();

        assert_relative_eq!(u_min, 0.0, epsilon = 1e-10);
        assert_relative_eq!(u_max, 1.0, epsilon = 1e-10);
    }

    #[test]
    fn test_bspline_insufficient_points() {
        let points = vec![Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 1.0, 0.0)];

        // Cubic needs at least 4 points
        let result = BSpline::clamped(points, 3);
        assert!(result.is_err());
    }

    #[test]
    #[allow(clippy::unwrap_used)]
    fn test_bspline_knot_insertion() {
        let points = vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 2.0, 0.0),
            Point3::new(3.0, 2.0, 0.0),
            Point3::new(4.0, 0.0, 0.0),
        ];

        let spline = BSpline::clamped(points, 3).unwrap();
        let refined = spline.insert_knot(0.5);

        // Refined spline should have one more control point
        assert_eq!(
            refined.num_control_points(),
            spline.num_control_points() + 1
        );

        // Should evaluate to same curve
        for i in 0..=10 {
            let t = i as f64 / 10.0;
            assert_relative_eq!(
                spline.point_at(t).coords,
                refined.point_at(t).coords,
                epsilon = 1e-6
            );
        }
    }

    #[test]
    #[allow(clippy::unwrap_used)]
    fn test_bspline_tangent() {
        let points = vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(2.0, 0.0, 0.0),
            Point3::new(3.0, 0.0, 0.0),
        ];

        // Straight line should have tangent along X
        let spline = BSpline::clamped(points, 3).unwrap();
        let tangent = spline.tangent_at(0.5);

        assert_relative_eq!(tangent.x, 1.0, epsilon = 1e-6);
        assert_relative_eq!(tangent.y, 0.0, epsilon = 1e-6);
        assert_relative_eq!(tangent.z, 0.0, epsilon = 1e-6);
    }
}
