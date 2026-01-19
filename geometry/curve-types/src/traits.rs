//! Core curve traits.
//!
//! This module defines the fundamental traits that all curve types implement.

use nalgebra::{Point2, Point3, Vector2, Vector3};

/// A parametric curve in 3D space.
///
/// All curves are parameterized over `t ∈ [0, 1]`, where `t=0` is the start
/// and `t=1` is the end of the curve.
///
/// # Implementors
///
/// - [`Polyline`](crate::Polyline) - Piecewise linear
/// - [`CubicBezier`](crate::CubicBezier) - Single cubic Bézier segment
/// - [`BezierSpline`](crate::BezierSpline) - Multiple Bézier segments
/// - [`BSpline`](crate::BSpline) - B-spline curves
/// - [`Nurbs`](crate::Nurbs) - NURBS curves
/// - [`Arc`](crate::Arc) - Circular arcs
/// - [`Circle`](crate::Circle) - Full circles
/// - [`Helix`](crate::Helix) - Helical curves
pub trait Curve {
    /// Evaluate the curve position at parameter `t ∈ [0, 1]`.
    ///
    /// # Parameters
    ///
    /// - `t`: Parameter value in range [0, 1]
    ///
    /// # Returns
    ///
    /// The 3D point on the curve at parameter `t`.
    ///
    /// # Panics
    ///
    /// May panic if `t` is outside [0, 1]. Use [`Self::point_at_clamped`] for
    /// safe evaluation.
    fn point_at(&self, t: f64) -> Point3<f64>;

    /// Evaluate the curve position, clamping `t` to [0, 1].
    ///
    /// This is a safe version of [`Self::point_at`] that clamps out-of-range
    /// parameters instead of panicking.
    fn point_at_clamped(&self, t: f64) -> Point3<f64> {
        self.point_at(t.clamp(0.0, 1.0))
    }

    /// Compute the unit tangent vector at parameter `t`.
    ///
    /// The tangent points in the direction of increasing `t`.
    ///
    /// # Returns
    ///
    /// A normalized (unit length) vector tangent to the curve.
    fn tangent_at(&self, t: f64) -> Vector3<f64>;

    /// Compute the first derivative (velocity) at parameter `t`.
    ///
    /// Unlike [`Self::tangent_at`], this returns the non-normalized derivative,
    /// which encodes both direction and speed.
    fn derivative_at(&self, t: f64) -> Vector3<f64>;

    /// Compute the second derivative (acceleration) at parameter `t`.
    ///
    /// This is useful for computing curvature and for physics simulations.
    fn second_derivative_at(&self, t: f64) -> Vector3<f64>;

    /// Compute the unit normal vector at parameter `t`.
    ///
    /// The normal is perpendicular to the tangent and points toward the
    /// center of curvature. For straight segments, a consistent arbitrary
    /// normal is chosen.
    fn normal_at(&self, t: f64) -> Vector3<f64> {
        let tangent = self.tangent_at(t);
        let d2 = self.second_derivative_at(t);

        // Project out the tangent component
        let normal = d2 - tangent * tangent.dot(&d2);
        let norm = normal.norm();

        if norm > 1e-10 {
            normal / norm
        } else {
            // Straight segment: choose arbitrary perpendicular
            arbitrary_perpendicular(&tangent)
        }
    }

    /// Compute the unit binormal vector at parameter `t`.
    ///
    /// The binormal is `tangent × normal`, completing the Frenet-Serret frame.
    fn binormal_at(&self, t: f64) -> Vector3<f64> {
        self.tangent_at(t).cross(&self.normal_at(t))
    }

    /// Compute the curvature at parameter `t`.
    ///
    /// Curvature is the reciprocal of the radius of the osculating circle.
    /// Higher curvature means sharper bends.
    ///
    /// # Returns
    ///
    /// Non-negative curvature value. Returns 0 for straight segments.
    fn curvature_at(&self, t: f64) -> f64 {
        let d1 = self.derivative_at(t);
        let d2 = self.second_derivative_at(t);
        let cross = d1.cross(&d2);
        let d1_norm = d1.norm();

        if d1_norm > 1e-10 {
            cross.norm() / d1_norm.powi(3)
        } else {
            0.0
        }
    }

    /// Compute the torsion at parameter `t`.
    ///
    /// Torsion measures how much the curve twists out of the osculating plane.
    ///
    /// # Returns
    ///
    /// Torsion value. Positive torsion indicates right-handed twist.
    fn torsion_at(&self, t: f64) -> f64 {
        let d1 = self.derivative_at(t);
        let d2 = self.second_derivative_at(t);
        let cross = d1.cross(&d2);
        let cross_norm_sq = cross.norm_squared();

        if cross_norm_sq > 1e-20 {
            // Torsion requires third derivative
            // For most curve types, we approximate numerically
            let h = 1e-6;
            let d2_plus = if t + h <= 1.0 {
                self.second_derivative_at(t + h)
            } else {
                self.second_derivative_at(t)
            };
            let d2_minus = if t - h >= 0.0 {
                self.second_derivative_at(t - h)
            } else {
                self.second_derivative_at(t)
            };
            let d3 = (d2_plus - d2_minus) / (2.0 * h);

            cross.dot(&d3) / cross_norm_sq
        } else {
            0.0
        }
    }

    /// Compute the total arc length of the curve.
    ///
    /// This integrates the curve length from `t=0` to `t=1` using adaptive
    /// Gauss-Legendre quadrature.
    fn arc_length(&self) -> f64 {
        self.arc_length_between(0.0, 1.0)
    }

    /// Compute the arc length between two parameter values.
    ///
    /// # Parameters
    ///
    /// - `t0`: Start parameter
    /// - `t1`: End parameter
    ///
    /// # Returns
    ///
    /// The arc length from `t0` to `t1`. Always non-negative.
    fn arc_length_between(&self, t0: f64, t1: f64) -> f64 {
        // Adaptive Simpson's rule for arc length integration
        let (start, end) = if t0 < t1 { (t0, t1) } else { (t1, t0) };
        adaptive_arc_length(self, start, end, 1e-10, 20)
    }

    /// Convert an arc length to a parameter value.
    ///
    /// Given a distance `s` along the curve from the start, find the
    /// parameter `t` such that the arc length from 0 to `t` equals `s`.
    ///
    /// # Parameters
    ///
    /// - `s`: Arc length from start of curve
    ///
    /// # Returns
    ///
    /// Parameter `t ∈ [0, 1]` corresponding to arc length `s`.
    /// Clamps to [0, 1] if `s` is outside [0, total_length].
    fn arc_to_t(&self, s: f64) -> f64 {
        if s <= 0.0 {
            return 0.0;
        }

        let total = self.arc_length();
        if s >= total {
            return 1.0;
        }

        // Binary search for the parameter
        let mut lo = 0.0;
        let mut hi = 1.0;
        let tolerance = 1e-10;
        let max_iterations = 50;

        for _ in 0..max_iterations {
            let mid = (lo + hi) / 2.0;
            let arc = self.arc_length_between(0.0, mid);

            if (arc - s).abs() < tolerance {
                return mid;
            }

            if arc < s {
                lo = mid;
            } else {
                hi = mid;
            }
        }

        (lo + hi) / 2.0
    }

    /// Convert a parameter value to arc length.
    ///
    /// # Parameters
    ///
    /// - `t`: Parameter value in [0, 1]
    ///
    /// # Returns
    ///
    /// The arc length from the start of the curve to parameter `t`.
    fn t_to_arc(&self, t: f64) -> f64 {
        self.arc_length_between(0.0, t.clamp(0.0, 1.0))
    }

    /// Sample the curve at uniform parameter intervals.
    ///
    /// # Parameters
    ///
    /// - `n`: Number of samples (must be >= 2)
    ///
    /// # Returns
    ///
    /// Vector of `n` points evenly spaced in parameter space.
    fn sample_uniform(&self, n: usize) -> Vec<Point3<f64>> {
        let n = n.max(2);
        (0..n)
            .map(|i| {
                let t = i as f64 / (n - 1) as f64;
                self.point_at(t)
            })
            .collect()
    }

    /// Sample the curve at uniform arc length intervals.
    ///
    /// This produces points that are approximately evenly spaced along the
    /// actual curve length, unlike [`Self::sample_uniform`] which spaces
    /// evenly in parameter space.
    ///
    /// # Parameters
    ///
    /// - `n`: Number of samples (must be >= 2)
    ///
    /// # Returns
    ///
    /// Vector of `n` points approximately evenly spaced by arc length.
    fn sample_arc_length(&self, n: usize) -> Vec<Point3<f64>> {
        let n = n.max(2);
        let total = self.arc_length();

        (0..n)
            .map(|i| {
                let s = i as f64 / (n - 1) as f64 * total;
                let t = self.arc_to_t(s);
                self.point_at(t)
            })
            .collect()
    }

    /// Get the start point of the curve (`t=0`).
    fn start(&self) -> Point3<f64> {
        self.point_at(0.0)
    }

    /// Get the end point of the curve (`t=1`).
    fn end(&self) -> Point3<f64> {
        self.point_at(1.0)
    }

    /// Check if the curve is closed (start equals end within tolerance).
    fn is_closed(&self) -> bool {
        let tolerance = 1e-10;
        (self.start() - self.end()).norm() < tolerance
    }

    /// Compute the bounding box of the curve.
    ///
    /// Returns `(min, max)` corners of the axis-aligned bounding box.
    fn bounding_box(&self) -> (Point3<f64>, Point3<f64>) {
        // Sample-based approximation
        let samples = self.sample_uniform(100);
        let mut min = samples[0];
        let mut max = samples[0];

        for p in &samples[1..] {
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

/// A parametric curve in 2D space.
///
/// Similar to [`Curve`] but operates in 2D. Useful for planar curves,
/// cross-sections, and profile curves.
pub trait Curve2D {
    /// Evaluate the curve position at parameter `t ∈ [0, 1]`.
    fn point_at(&self, t: f64) -> Point2<f64>;

    /// Compute the unit tangent vector at parameter `t`.
    fn tangent_at(&self, t: f64) -> Vector2<f64>;

    /// Compute the first derivative at parameter `t`.
    fn derivative_at(&self, t: f64) -> Vector2<f64>;

    /// Compute the second derivative at parameter `t`.
    fn second_derivative_at(&self, t: f64) -> Vector2<f64>;

    /// Compute the unit normal vector at parameter `t`.
    ///
    /// The 2D normal is perpendicular to the tangent (rotated 90° counter-clockwise).
    fn normal_at(&self, t: f64) -> Vector2<f64> {
        let t = self.tangent_at(t);
        Vector2::new(-t.y, t.x)
    }

    /// Compute the signed curvature at parameter `t`.
    ///
    /// Positive curvature indicates counter-clockwise bending.
    fn curvature_at(&self, t: f64) -> f64 {
        let d1 = self.derivative_at(t);
        let d2 = self.second_derivative_at(t);
        let cross = d1.x * d2.y - d1.y * d2.x;
        let d1_norm = d1.norm();

        if d1_norm > 1e-10 {
            cross / d1_norm.powi(3)
        } else {
            0.0
        }
    }

    /// Compute the total arc length.
    fn arc_length(&self) -> f64;

    /// Sample the curve at uniform parameter intervals.
    fn sample_uniform(&self, n: usize) -> Vec<Point2<f64>> {
        let n = n.max(2);
        (0..n)
            .map(|i| {
                let t = i as f64 / (n - 1) as f64;
                self.point_at(t)
            })
            .collect()
    }

    /// Get the start point.
    fn start(&self) -> Point2<f64> {
        self.point_at(0.0)
    }

    /// Get the end point.
    fn end(&self) -> Point2<f64> {
        self.point_at(1.0)
    }
}

/// A curve with variable radius (tube-like).
///
/// This trait extends [`Curve`] for curves that represent the centerline
/// of a tube or vessel with varying cross-section.
pub trait TubularCurve: Curve {
    /// Get the radius at parameter `t`.
    ///
    /// # Parameters
    ///
    /// - `t`: Parameter value in [0, 1]
    ///
    /// # Returns
    ///
    /// The radius of the tube cross-section at parameter `t`.
    fn radius_at(&self, t: f64) -> f64;

    /// Get the minimum radius along the curve.
    fn min_radius(&self) -> f64 {
        (0..=100)
            .map(|i| self.radius_at(i as f64 / 100.0))
            .fold(f64::INFINITY, f64::min)
    }

    /// Get the maximum radius along the curve.
    fn max_radius(&self) -> f64 {
        (0..=100)
            .map(|i| self.radius_at(i as f64 / 100.0))
            .fold(0.0, f64::max)
    }

    /// Sample the curve with radius information.
    ///
    /// # Returns
    ///
    /// Vector of `(point, radius)` pairs.
    fn sample_with_radius(&self, n: usize) -> Vec<(Point3<f64>, f64)> {
        let n = n.max(2);
        (0..n)
            .map(|i| {
                let t = i as f64 / (n - 1) as f64;
                (self.point_at(t), self.radius_at(t))
            })
            .collect()
    }
}

/// Choose an arbitrary perpendicular vector to the given vector.
fn arbitrary_perpendicular(v: &Vector3<f64>) -> Vector3<f64> {
    let abs_x = v.x.abs();
    let abs_y = v.y.abs();
    let abs_z = v.z.abs();

    // Choose the axis most perpendicular to v
    let reference = if abs_x <= abs_y && abs_x <= abs_z {
        Vector3::x()
    } else if abs_y <= abs_z {
        Vector3::y()
    } else {
        Vector3::z()
    };

    let perp = v.cross(&reference);
    perp.normalize()
}

/// Adaptive Simpson's rule for arc length integration.
fn adaptive_arc_length<C: Curve + ?Sized>(
    curve: &C,
    a: f64,
    b: f64,
    tolerance: f64,
    max_depth: usize,
) -> f64 {
    fn simpson_step<C: Curve + ?Sized>(curve: &C, a: f64, b: f64) -> f64 {
        let mid = (a + b) / 2.0;
        let h = (b - a) / 6.0;

        let fa = curve.derivative_at(a).norm();
        let fm = curve.derivative_at(mid).norm();
        let fb = curve.derivative_at(b).norm();

        h * (fa + 4.0 * fm + fb)
    }

    fn adaptive_helper<C: Curve + ?Sized>(
        curve: &C,
        a: f64,
        b: f64,
        tolerance: f64,
        whole: f64,
        depth: usize,
    ) -> f64 {
        let mid = (a + b) / 2.0;
        let left = simpson_step(curve, a, mid);
        let right = simpson_step(curve, mid, b);
        let combined = left + right;

        if depth == 0 || (combined - whole).abs() < 15.0 * tolerance {
            combined + (combined - whole) / 15.0
        } else {
            let new_tol = tolerance / 2.0;
            adaptive_helper(curve, a, mid, new_tol, left, depth - 1)
                + adaptive_helper(curve, mid, b, new_tol, right, depth - 1)
        }
    }

    let whole = simpson_step(curve, a, b);
    adaptive_helper(curve, a, b, tolerance, whole, max_depth)
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    // A simple test curve: a straight line
    struct LineSegment {
        start: Point3<f64>,
        end: Point3<f64>,
    }

    impl Curve for LineSegment {
        fn point_at(&self, t: f64) -> Point3<f64> {
            self.start + (self.end - self.start) * t
        }

        fn tangent_at(&self, _t: f64) -> Vector3<f64> {
            (self.end - self.start).normalize()
        }

        fn derivative_at(&self, _t: f64) -> Vector3<f64> {
            self.end - self.start
        }

        fn second_derivative_at(&self, _t: f64) -> Vector3<f64> {
            Vector3::zeros()
        }
    }

    #[test]
    fn test_line_segment() {
        let line = LineSegment {
            start: Point3::new(0.0, 0.0, 0.0),
            end: Point3::new(3.0, 4.0, 0.0),
        };

        // Endpoints
        assert_relative_eq!(line.start().coords, line.point_at(0.0).coords);
        assert_relative_eq!(line.end().coords, line.point_at(1.0).coords);

        // Midpoint
        let mid = line.point_at(0.5);
        assert_relative_eq!(mid.x, 1.5, epsilon = 1e-10);
        assert_relative_eq!(mid.y, 2.0, epsilon = 1e-10);

        // Arc length (3-4-5 triangle)
        let length = line.arc_length();
        assert_relative_eq!(length, 5.0, epsilon = 1e-10);

        // Curvature is zero for straight line
        assert_relative_eq!(line.curvature_at(0.5), 0.0, epsilon = 1e-10);
    }

    #[test]
    fn test_arc_length_parameterization() {
        let line = LineSegment {
            start: Point3::new(0.0, 0.0, 0.0),
            end: Point3::new(10.0, 0.0, 0.0),
        };

        // arc_to_t should return t for uniform line
        let t = line.arc_to_t(5.0);
        assert_relative_eq!(t, 0.5, epsilon = 1e-6);

        // t_to_arc inverse
        let s = line.t_to_arc(0.5);
        assert_relative_eq!(s, 5.0, epsilon = 1e-6);
    }

    #[test]
    fn test_arbitrary_perpendicular() {
        let v = Vector3::new(1.0, 0.0, 0.0);
        let perp = arbitrary_perpendicular(&v);
        assert_relative_eq!(v.dot(&perp), 0.0, epsilon = 1e-10);
        assert_relative_eq!(perp.norm(), 1.0, epsilon = 1e-10);

        let v = Vector3::new(1.0, 1.0, 1.0).normalize();
        let perp = arbitrary_perpendicular(&v);
        assert_relative_eq!(v.dot(&perp), 0.0, epsilon = 1e-10);
    }

    #[test]
    fn test_sampling() {
        let line = LineSegment {
            start: Point3::new(0.0, 0.0, 0.0),
            end: Point3::new(10.0, 0.0, 0.0),
        };

        let samples = line.sample_uniform(11);
        assert_eq!(samples.len(), 11);
        assert_relative_eq!(samples[0].x, 0.0, epsilon = 1e-10);
        assert_relative_eq!(samples[5].x, 5.0, epsilon = 1e-10);
        assert_relative_eq!(samples[10].x, 10.0, epsilon = 1e-10);
    }
}
