//! Curve operations: split, join, reverse, and other manipulations.
//!
//! This module provides common operations that work on any curve type
//! that implements the [`Curve`] trait.

use crate::{Curve, CurveError, Polyline, Result};
use nalgebra::Point3;

/// Trait for curves that support splitting and joining.
pub trait CurveOps: Curve + Sized {
    /// Split the curve at parameter `t`, returning two curves.
    ///
    /// # Errors
    ///
    /// Returns error if `t` is at or very close to 0 or 1.
    fn split(&self, t: f64) -> Result<(Self, Self)>;

    /// Reverse the parameterization direction.
    ///
    /// Creates a new curve where `t=0` corresponds to the original `t=1`
    /// and vice versa.
    fn reverse(&self) -> Self;

    /// Join this curve with another, creating a combined curve.
    ///
    /// # Errors
    ///
    /// Returns error if the curves don't connect (gap > tolerance).
    fn join(&self, other: &Self, tolerance: f64) -> Result<Self>;
}

/// Reverse a curve, creating a polyline approximation going in the opposite direction.
///
/// This works for any curve type by sampling and reversing the samples.
///
/// # Parameters
///
/// - `curve`: The curve to reverse
///
/// # Returns
///
/// A [`Polyline`] approximation going in the reverse direction.
///
/// # Example
///
/// ```
/// use curve_types::{Polyline, Curve, reverse};
/// use nalgebra::Point3;
///
/// let curve = Polyline::new(vec![
///     Point3::new(0.0, 0.0, 0.0),
///     Point3::new(1.0, 0.0, 0.0),
///     Point3::new(2.0, 0.0, 0.0),
/// ]);
///
/// let reversed = reverse(&curve);
///
/// // Start and end are swapped
/// assert!((reversed.point_at(0.0).x - 2.0).abs() < 1e-10);
/// assert!((reversed.point_at(1.0).x - 0.0).abs() < 1e-10);
/// ```
#[must_use]
pub fn reverse<C: Curve>(curve: &C) -> Polyline {
    // Sample the curve
    let n = 100; // Default sampling resolution
    let mut points: Vec<Point3<f64>> = curve.sample_uniform(n);
    points.reverse();
    Polyline::new(points)
}

/// Split a curve at parameter `t`, returning two polyline approximations.
///
/// # Parameters
///
/// - `curve`: The curve to split
/// - `t`: Split parameter in (0, 1)
///
/// # Errors
///
/// Returns error if `t` is not in the open interval (0, 1).
///
/// # Example
///
/// ```
/// use curve_types::{Polyline, Curve, split_at};
/// use nalgebra::Point3;
///
/// let curve = Polyline::new(vec![
///     Point3::new(0.0, 0.0, 0.0),
///     Point3::new(1.0, 0.0, 0.0),
///     Point3::new(2.0, 0.0, 0.0),
/// ]);
///
/// let (left, right) = split_at(&curve, 0.5).unwrap();
///
/// // Left ends where right begins
/// assert!((left.point_at(1.0) - right.point_at(0.0)).norm() < 1e-10);
/// ```
pub fn split_at<C: Curve>(curve: &C, t: f64) -> Result<(Polyline, Polyline)> {
    if t <= 0.0 || t >= 1.0 {
        return Err(CurveError::invalid_split(
            t,
            "split parameter must be in open interval (0, 1)",
        ));
    }

    // Sample the curve at higher resolution
    let n_left = ((t * 100.0) as usize).max(2);
    let n_right = (((1.0 - t) * 100.0) as usize).max(2);

    let mut left_points = Vec::with_capacity(n_left);
    let mut right_points = Vec::with_capacity(n_right);

    // Sample left portion
    for i in 0..n_left {
        let s = i as f64 / (n_left - 1) as f64 * t;
        left_points.push(curve.point_at(s));
    }

    // Sample right portion
    for i in 0..n_right {
        let s = t + i as f64 / (n_right - 1) as f64 * (1.0 - t);
        right_points.push(curve.point_at(s));
    }

    Ok((Polyline::new(left_points), Polyline::new(right_points)))
}

/// Join two curves into a single polyline.
///
/// If there's a gap between the curves, it will be connected with a straight
/// segment.
///
/// # Parameters
///
/// - `curve1`: First curve
/// - `curve2`: Second curve
///
/// # Returns
///
/// A [`Polyline`] representing the combined curve.
///
/// # Example
///
/// ```
/// use curve_types::{Polyline, Curve, join};
/// use nalgebra::Point3;
///
/// let curve1 = Polyline::new(vec![
///     Point3::new(0.0, 0.0, 0.0),
///     Point3::new(1.0, 0.0, 0.0),
/// ]);
///
/// let curve2 = Polyline::new(vec![
///     Point3::new(1.0, 0.0, 0.0),
///     Point3::new(2.0, 1.0, 0.0),
/// ]);
///
/// let joined = join(&curve1, &curve2);
/// // Joined curve has many vertices due to sampling
/// assert!(joined.vertices().len() > 2);
/// // Endpoints are preserved
/// assert!((joined.point_at(0.0) - curve1.point_at(0.0)).norm() < 1e-10);
/// assert!((joined.point_at(1.0) - curve2.point_at(1.0)).norm() < 1e-10);
/// ```
#[must_use]
pub fn join<C1: Curve, C2: Curve>(curve1: &C1, curve2: &C2) -> Polyline {
    let n1 = 50;
    let n2 = 50;

    let mut points = curve1.sample_uniform(n1);

    // Check if we need to add a connecting segment
    let end1 = curve1.end();
    let start2 = curve2.start();
    let gap = (end1 - start2).norm();

    if gap > 1e-10 {
        // Add explicit connection point
        points.push(start2);
    }

    // Add second curve samples (skip first if it overlaps)
    let samples2 = curve2.sample_uniform(n2);
    let skip_first = gap < 1e-6;
    points.extend(samples2.into_iter().skip(if skip_first { 1 } else { 0 }));

    Polyline::new(points)
}

/// Join two curves, returning an error if the gap exceeds tolerance.
///
/// # Parameters
///
/// - `curve1`: First curve
/// - `curve2`: Second curve
/// - `tolerance`: Maximum allowed gap between curves
///
/// # Errors
///
/// Returns [`CurveError::CannotJoin`] if the gap exceeds tolerance.
pub fn join_strict<C1: Curve, C2: Curve>(
    curve1: &C1,
    curve2: &C2,
    tolerance: f64,
) -> Result<Polyline> {
    let end1 = curve1.end();
    let start2 = curve2.start();
    let gap = (end1 - start2).norm();

    if gap > tolerance {
        return Err(CurveError::CannotJoin { gap });
    }

    Ok(join(curve1, curve2))
}

/// Subdivide a curve into multiple segments at the given parameters.
///
/// # Parameters
///
/// - `curve`: The curve to subdivide
/// - `params`: Parameter values where to split (must be sorted, in (0, 1))
///
/// # Returns
///
/// Vector of polyline segments.
pub fn subdivide<C: Curve>(curve: &C, params: &[f64]) -> Vec<Polyline> {
    if params.is_empty() {
        return vec![Polyline::new(curve.sample_uniform(100))];
    }

    let mut segments = Vec::with_capacity(params.len() + 1);
    let mut prev_t = 0.0;

    for &t in params {
        if t > prev_t && t < 1.0 {
            let n = ((t - prev_t) * 100.0) as usize;
            let n = n.max(2);
            let mut points = Vec::with_capacity(n);

            for i in 0..n {
                let s = prev_t + i as f64 / (n - 1) as f64 * (t - prev_t);
                points.push(curve.point_at(s));
            }

            segments.push(Polyline::new(points));
            prev_t = t;
        }
    }

    // Final segment
    if prev_t < 1.0 {
        let n = ((1.0 - prev_t) * 100.0) as usize;
        let n = n.max(2);
        let mut points = Vec::with_capacity(n);

        for i in 0..n {
            let s = prev_t + i as f64 / (n - 1) as f64 * (1.0 - prev_t);
            points.push(curve.point_at(s));
        }

        segments.push(Polyline::new(points));
    }

    segments
}

/// Extract a portion of a curve between two parameter values.
///
/// # Parameters
///
/// - `curve`: The source curve
/// - `t0`: Start parameter
/// - `t1`: End parameter
///
/// # Returns
///
/// A [`Polyline`] representing the portion of the curve.
#[must_use]
pub fn extract<C: Curve>(curve: &C, t0: f64, t1: f64) -> Polyline {
    let t0 = t0.clamp(0.0, 1.0);
    let t1 = t1.clamp(0.0, 1.0);

    if (t1 - t0).abs() < 1e-10 {
        return Polyline::new(vec![curve.point_at(t0), curve.point_at(t0)]);
    }

    let (start, end) = if t0 < t1 { (t0, t1) } else { (t1, t0) };
    let n = ((end - start) * 100.0) as usize;
    let n = n.max(2);

    let mut points = Vec::with_capacity(n);
    for i in 0..n {
        let t = start + i as f64 / (n - 1) as f64 * (end - start);
        points.push(curve.point_at(t));
    }

    let mut polyline = Polyline::new(points);

    // If original parameters were reversed, reverse the result
    if t0 > t1 {
        polyline = polyline.reversed();
    }

    polyline
}

/// Find the parameter on a curve closest to a given point.
///
/// Uses a combination of sampling and binary refinement.
///
/// # Parameters
///
/// - `curve`: The curve to search
/// - `point`: The query point
///
/// # Returns
///
/// `(t, distance)` where `t` is the closest parameter and `distance`
/// is the distance from the point to the curve at that parameter.
#[must_use]
pub fn closest_point<C: Curve>(curve: &C, point: Point3<f64>) -> (f64, f64) {
    // Initial sampling
    let n = 100;
    let mut best_t = 0.0;
    let mut best_dist = f64::MAX;

    for i in 0..=n {
        let t = i as f64 / n as f64;
        let p = curve.point_at(t);
        let dist = (p - point).norm();

        if dist < best_dist {
            best_dist = dist;
            best_t = t;
        }
    }

    // Refine with golden section search
    let golden = (5.0_f64.sqrt() - 1.0) / 2.0;
    let tol = 1e-8;

    let mut a = (best_t - 0.02).max(0.0);
    let mut b = (best_t + 0.02).min(1.0);

    let mut c = b - golden * (b - a);
    let mut d = a + golden * (b - a);

    while (b - a) > tol {
        let fc = (curve.point_at(c) - point).norm();
        let fd = (curve.point_at(d) - point).norm();

        if fc < fd {
            b = d;
            d = c;
            c = b - golden * (b - a);
        } else {
            a = c;
            c = d;
            d = a + golden * (b - a);
        }
    }

    let t = (a + b) / 2.0;
    let dist = (curve.point_at(t) - point).norm();

    (t, dist)
}

/// Compute the intersection parameters between two curves.
///
/// Uses a numerical approach with subdivision.
///
/// # Parameters
///
/// - `curve1`: First curve
/// - `curve2`: Second curve
/// - `tolerance`: Distance tolerance for intersection
///
/// # Returns
///
/// Vector of `(t1, t2)` pairs where the curves intersect.
#[must_use]
pub fn intersections<C1: Curve, C2: Curve>(
    curve1: &C1,
    curve2: &C2,
    tolerance: f64,
) -> Vec<(f64, f64)> {
    let mut results = Vec::new();

    // Coarse sampling to find candidate regions
    let n = 50;

    for i in 0..n {
        let t1_start = i as f64 / n as f64;
        let t1_end = (i + 1) as f64 / n as f64;

        for j in 0..n {
            let t2_start = j as f64 / n as f64;
            let t2_end = (j + 1) as f64 / n as f64;

            // Check if bounding boxes might intersect
            let p1a = curve1.point_at(t1_start);
            let p1b = curve1.point_at(t1_end);
            let p2a = curve2.point_at(t2_start);
            let p2b = curve2.point_at(t2_end);

            // Simple distance check (more sophisticated BB test possible)
            let min_dist = [
                (p1a - p2a).norm(),
                (p1a - p2b).norm(),
                (p1b - p2a).norm(),
                (p1b - p2b).norm(),
            ]
            .into_iter()
            .fold(f64::MAX, f64::min);

            if min_dist < tolerance * 10.0 {
                // Refine this cell
                if let Some((t1, t2)) = refine_intersection(
                    curve1, curve2, t1_start, t1_end, t2_start, t2_end, tolerance,
                ) {
                    // Avoid duplicates
                    let is_dup = results.iter().any(|&(r1, r2): &(f64, f64)| {
                        (r1 - t1).abs() < 1e-6 && (r2 - t2).abs() < 1e-6
                    });

                    if !is_dup {
                        results.push((t1, t2));
                    }
                }
            }
        }
    }

    results
}

/// Refine intersection search in a cell.
fn refine_intersection<C1: Curve, C2: Curve>(
    curve1: &C1,
    curve2: &C2,
    t1_min: f64,
    t1_max: f64,
    t2_min: f64,
    t2_max: f64,
    tolerance: f64,
) -> Option<(f64, f64)> {
    let max_iter = 20;
    let mut t1 = (t1_min + t1_max) / 2.0;
    let mut t2 = (t2_min + t2_max) / 2.0;

    for _ in 0..max_iter {
        let p1 = curve1.point_at(t1);
        let p2 = curve2.point_at(t2);
        let dist = (p1 - p2).norm();

        if dist < tolerance {
            return Some((t1, t2));
        }

        // Newton-like iteration
        let d1 = curve1.derivative_at(t1);
        let d2 = curve2.derivative_at(t2);

        let diff = p2 - p1;

        // Solve linearized system: diff + d1*dt1 - d2*dt2 ≈ 0
        // Project onto tangent directions
        let dt1 = diff.dot(&d1) / d1.norm_squared().max(1e-10);
        let dt2 = -diff.dot(&d2) / d2.norm_squared().max(1e-10);

        t1 = (t1 + dt1 * 0.5).clamp(t1_min, t1_max);
        t2 = (t2 + dt2 * 0.5).clamp(t2_min, t2_max);
    }

    None
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn test_reverse() {
        let curve = Polyline::new(vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(2.0, 0.0, 0.0),
        ]);

        let reversed = reverse(&curve);

        assert_relative_eq!(reversed.point_at(0.0).x, 2.0, epsilon = 1e-6);
        assert_relative_eq!(reversed.point_at(1.0).x, 0.0, epsilon = 1e-6);
    }

    #[test]
    #[allow(clippy::unwrap_used)]
    fn test_split_at() {
        let curve = Polyline::new(vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(10.0, 0.0, 0.0),
        ]);

        let (left, right) = split_at(&curve, 0.5).unwrap();

        assert_relative_eq!(left.point_at(0.0).x, 0.0, epsilon = 1e-6);
        assert_relative_eq!(left.point_at(1.0).x, 5.0, epsilon = 1e-6);
        assert_relative_eq!(right.point_at(0.0).x, 5.0, epsilon = 1e-6);
        assert_relative_eq!(right.point_at(1.0).x, 10.0, epsilon = 1e-6);
    }

    #[test]
    fn test_split_invalid() {
        let curve = Polyline::new(vec![Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0)]);

        assert!(split_at(&curve, 0.0).is_err());
        assert!(split_at(&curve, 1.0).is_err());
        assert!(split_at(&curve, -0.5).is_err());
        assert!(split_at(&curve, 1.5).is_err());
    }

    #[test]
    fn test_join() {
        let curve1 = Polyline::new(vec![Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0)]);

        let curve2 = Polyline::new(vec![Point3::new(1.0, 0.0, 0.0), Point3::new(2.0, 0.0, 0.0)]);

        let joined = join(&curve1, &curve2);

        assert_relative_eq!(joined.point_at(0.0).x, 0.0, epsilon = 1e-6);
        assert_relative_eq!(joined.point_at(1.0).x, 2.0, epsilon = 1e-6);
    }

    #[test]
    fn test_join_with_gap() {
        let curve1 = Polyline::new(vec![Point3::new(0.0, 0.0, 0.0), Point3::new(1.0, 0.0, 0.0)]);

        let curve2 = Polyline::new(vec![Point3::new(2.0, 0.0, 0.0), Point3::new(3.0, 0.0, 0.0)]);

        // join() should work even with gap
        let joined = join(&curve1, &curve2);
        assert_relative_eq!(joined.point_at(0.0).x, 0.0, epsilon = 1e-6);
        assert_relative_eq!(joined.point_at(1.0).x, 3.0, epsilon = 1e-6);

        // join_strict() should fail with small tolerance
        assert!(join_strict(&curve1, &curve2, 0.5).is_err());

        // join_strict() should succeed with large tolerance
        assert!(join_strict(&curve1, &curve2, 2.0).is_ok());
    }

    #[test]
    fn test_extract() {
        let curve = Polyline::new(vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(10.0, 0.0, 0.0),
        ]);

        let portion = extract(&curve, 0.25, 0.75);

        assert_relative_eq!(portion.point_at(0.0).x, 2.5, epsilon = 1e-6);
        assert_relative_eq!(portion.point_at(1.0).x, 7.5, epsilon = 1e-6);
    }

    #[test]
    fn test_closest_point() {
        let curve = Polyline::new(vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(10.0, 0.0, 0.0),
        ]);

        // Point on line
        let (t, dist) = closest_point(&curve, Point3::new(5.0, 0.0, 0.0));
        assert_relative_eq!(t, 0.5, epsilon = 1e-3);
        assert_relative_eq!(dist, 0.0, epsilon = 1e-6);

        // Point off line
        let (t, dist) = closest_point(&curve, Point3::new(5.0, 3.0, 0.0));
        assert_relative_eq!(t, 0.5, epsilon = 1e-3);
        assert_relative_eq!(dist, 3.0, epsilon = 1e-6);
    }

    #[test]
    fn test_subdivide() {
        let curve = Polyline::new(vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(10.0, 0.0, 0.0),
        ]);

        let segments = subdivide(&curve, &[0.25, 0.5, 0.75]);

        assert_eq!(segments.len(), 4);
    }
}
