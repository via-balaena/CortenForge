//! Catmull-Rom curve — the canonical differentiable centerline.
//!
//! A [`CatmullRomCurve`] is a C¹ interpolating spline through a sequence of
//! control points. It is the single source of "the curve" in CortenForge:
//! anything that realizes a centerline against matter (a swept tube bored out
//! of a solid, a raised rib, a toolpath) reads its geometry from here, so a
//! realization and its route can never describe *different* curves.
//!
//! # Parameterization
//!
//! [`sample`](CatmullRomCurve::sample) and [`tangent`](CatmullRomCurve::tangent)
//! take a global parameter `t ∈ [0, 1]` spanning the whole curve. With `n`
//! control points the curve has `n - 1` spans of equal parameter width; span
//! boundaries land exactly on the interior control points. The curve is clamped
//! (open): the first and last spans duplicate the endpoint control points, so
//! the curve passes through *every* control point, endpoints included.

use nalgebra::{Point3, Vector3};

/// An interpolating Catmull-Rom spline through a sequence of control points.
///
/// Construct with [`CatmullRomCurve::new`], which requires at least two control
/// points. The curve interpolates each control point and is C¹ everywhere.
///
/// ```
/// use cf_geometry::CatmullRomCurve;
/// use nalgebra::Point3;
///
/// let cps = vec![
///     Point3::new(0.0, 0.0, 0.0),
///     Point3::new(1.0, 1.0, 0.0),
///     Point3::new(2.0, 0.0, 0.0),
/// ];
/// let curve = CatmullRomCurve::new(cps).expect("two or more control points");
///
/// // The curve passes through its control points at the span boundaries.
/// let mid = curve.sample(0.5);
/// assert!((mid - Point3::new(1.0, 1.0, 0.0)).norm() < 1e-12);
/// ```
#[derive(Debug, Clone, PartialEq)]
pub struct CatmullRomCurve {
    control_points: Vec<Point3<f64>>,
}

impl CatmullRomCurve {
    /// Creates a curve through `control_points`.
    ///
    /// Returns [`None`] when fewer than two control points are supplied — a
    /// curve is undefined without at least a start and an end.
    #[must_use]
    pub fn new(control_points: Vec<Point3<f64>>) -> Option<Self> {
        if control_points.len() < 2 {
            return None;
        }
        Some(Self { control_points })
    }

    /// Returns the control points the curve interpolates.
    #[must_use]
    pub fn control_points(&self) -> &[Point3<f64>] {
        &self.control_points
    }

    /// Evaluates the curve position at global parameter `t ∈ [0, 1]`.
    ///
    /// `t` is clamped to `[0, 1]`. `t = 0` returns the first control point and
    /// `t = 1` the last; interior control points sit at `t = i / (n - 1)`.
    #[must_use]
    pub fn sample(&self, t: f64) -> Point3<f64> {
        let (span, u) = self.locate(t);
        let (p0, p1, p2, p3) = span_points(&self.control_points, span);
        cr_point(p0, p1, p2, p3, u)
    }

    /// Evaluates the curve tangent `dC/dt` at global parameter `t ∈ [0, 1]`.
    ///
    /// The tangent is taken with respect to the *global* parameter, so it is
    /// consistent with a finite difference of [`sample`](Self::sample): the
    /// per-span derivative is scaled by the number of spans. The result is not
    /// normalized; its magnitude is the local rate of travel along the curve.
    #[must_use]
    pub fn tangent(&self, t: f64) -> Vector3<f64> {
        let (span, u) = self.locate(t);
        let (p0, p1, p2, p3) = span_points(&self.control_points, span);
        let num_spans = self.control_points.len() - 1;
        // Span count is a small, exact-in-f64 integer (control-point count).
        #[allow(clippy::cast_precision_loss)]
        let scale = num_spans as f64;
        cr_deriv(p0, p1, p2, p3, u) * scale
    }

    /// Returns the point on the curve closest to `p`.
    ///
    /// Convenience wrapper over [`nearest_point_on_catmull_rom`]; see that
    /// function for the algorithm.
    #[must_use]
    pub fn nearest_point(&self, p: &Point3<f64>) -> Point3<f64> {
        nearest_point_on_catmull_rom(&self.control_points, p)
    }

    /// Maps a global `t ∈ [0, 1]` to its span index and local parameter `u`.
    //
    // Casts are between small, non-negative integers and f64: `num_spans` is the
    // control-point count (exact in f64); `tt.floor()` is clamped to `[0, 1]·
    // num_spans` so it is non-negative and below `num_spans` before truncation.
    #[allow(
        clippy::cast_precision_loss,
        clippy::cast_possible_truncation,
        clippy::cast_sign_loss
    )]
    fn locate(&self, t: f64) -> (usize, f64) {
        let num_spans = self.control_points.len() - 1;
        let tt = t.clamp(0.0, 1.0) * num_spans as f64;
        let span = (tt.floor() as usize).min(num_spans - 1);
        let u = tt - span as f64;
        (span, u)
    }
}

/// Returns the point on the clamped Catmull-Rom curve through `control_points`
/// closest to `query`.
///
/// Walks every span with a coarse 16-way sampling followed by four Newton
/// iterations on the squared distance. The two-control-point case degenerates
/// to the closest point on the single line segment.
///
/// This is the borrowing form of [`CatmullRomCurve::nearest_point`]: it takes
/// the control points directly so callers on a hot path (e.g. evaluating a
/// swept-tube SDF while meshing) need not allocate a curve per query.
///
/// With fewer than two control points the curve is undefined; the origin is
/// returned. Prefer [`CatmullRomCurve::new`], which rejects that case.
#[must_use]
pub fn nearest_point_on_catmull_rom(
    control_points: &[Point3<f64>],
    query: &Point3<f64>,
) -> Point3<f64> {
    let n = control_points.len();

    // A curve needs at least two control points; below that it is undefined
    // (see the doc contract). Returning origin keeps this public entry point
    // panic-free — callers that need validation use `CatmullRomCurve::new`.
    if n < 2 {
        return Point3::origin();
    }

    if n == 2 {
        return crate::closest_point_segment(control_points[0], control_points[1], *query);
    }

    let mut min_dist_sq = f64::INFINITY;
    let mut nearest = Point3::origin();

    // n >= 3 here, so `n - 1` cannot underflow.
    let num_spans = n - 1;
    for span in 0..num_spans {
        let (p0, p1, p2, p3) = span_points(control_points, span);

        // Coarse search: subdivide the span into 16 linear segments.
        let subdivs: u32 = 16;
        let mut best_t = 0.0_f64;
        let mut best_d_sq = f64::INFINITY;

        for i in 0..=subdivs {
            let t = f64::from(i) / f64::from(subdivs);
            let q = cr_point(p0, p1, p2, p3, t);
            let d_sq = nalgebra::distance_squared(query, &q);
            if d_sq < best_d_sq {
                best_d_sq = d_sq;
                best_t = t;
            }
        }

        // Newton refinement on distance²(t): minimize f(t) = |C(t) - query|².
        let mut t = best_t;
        for _ in 0..4 {
            let c = cr_point(p0, p1, p2, p3, t);
            let c_d = cr_deriv(p0, p1, p2, p3, t);
            let c_dd = cr_deriv2(p0, p1, p2, p3, t);
            let diff = c - query;
            let fp = 2.0 * diff.dot(&c_d);
            let fpp = 2.0 * (c_d.dot(&c_d) + diff.dot(&c_dd));
            if fpp.abs() < 1e-20 {
                break;
            }
            t -= fp / fpp;
            t = t.clamp(0.0, 1.0);
        }

        let c = cr_point(p0, p1, p2, p3, t);
        let d_sq = nalgebra::distance_squared(query, &c);
        if d_sq < min_dist_sq {
            min_dist_sq = d_sq;
            nearest = c;
        }
    }

    nearest
}

/// Returns the four control points `(p0, p1, p2, p3)` governing `span`,
/// duplicating the endpoints for the clamped (open) convention.
fn span_points(
    control_points: &[Point3<f64>],
    span: usize,
) -> (Point3<f64>, Point3<f64>, Point3<f64>, Point3<f64>) {
    let n = control_points.len();
    let p0 = control_points[if span == 0 { 0 } else { span - 1 }];
    let p1 = control_points[span];
    let p2 = control_points[span + 1];
    let p3 = control_points[if span + 2 < n { span + 2 } else { n - 1 }];
    (p0, p1, p2, p3)
}

/// Catmull-Rom spline position on a single span at local `t ∈ [0, 1]`.
fn cr_point(
    p0: Point3<f64>,
    p1: Point3<f64>,
    p2: Point3<f64>,
    p3: Point3<f64>,
    t: f64,
) -> Point3<f64> {
    let t2 = t * t;
    let t3 = t2 * t;
    let c = (p1.coords * 2.0
        + (-p0.coords + p2.coords) * t
        + (p0.coords * 2.0 - p1.coords * 5.0 + p2.coords * 4.0 - p3.coords) * t2
        + (-p0.coords + p1.coords * 3.0 - p2.coords * 3.0 + p3.coords) * t3)
        * 0.5;
    Point3::from(c)
}

/// First derivative of the Catmull-Rom spline on a single span.
fn cr_deriv(
    p0: Point3<f64>,
    p1: Point3<f64>,
    p2: Point3<f64>,
    p3: Point3<f64>,
    t: f64,
) -> Vector3<f64> {
    let t2 = t * t;
    ((-p0.coords + p2.coords)
        + (p0.coords * 4.0 - p1.coords * 10.0 + p2.coords * 8.0 - p3.coords * 2.0) * t
        + (-p0.coords * 3.0 + p1.coords * 9.0 - p2.coords * 9.0 + p3.coords * 3.0) * t2)
        * 0.5
}

/// Second derivative of the Catmull-Rom spline on a single span.
fn cr_deriv2(
    p0: Point3<f64>,
    p1: Point3<f64>,
    p2: Point3<f64>,
    p3: Point3<f64>,
    t: f64,
) -> Vector3<f64> {
    ((p0.coords * 4.0 - p1.coords * 10.0 + p2.coords * 8.0 - p3.coords * 2.0)
        + (-p0.coords * 6.0 + p1.coords * 18.0 - p2.coords * 18.0 + p3.coords * 6.0) * t)
        * 0.5
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::float_cmp, clippy::cast_precision_loss)]
mod tests {
    use super::*;
    use approx::assert_abs_diff_eq;

    fn sample_curve() -> CatmullRomCurve {
        CatmullRomCurve::new(vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 2.0, 0.0),
            Point3::new(3.0, 2.0, 1.0),
            Point3::new(4.0, 0.0, 1.0),
            Point3::new(5.0, -1.0, 0.0),
        ])
        .unwrap()
    }

    #[test]
    fn new_rejects_fewer_than_two_points() {
        assert!(CatmullRomCurve::new(vec![]).is_none());
        assert!(CatmullRomCurve::new(vec![Point3::origin()]).is_none());
        assert!(CatmullRomCurve::new(vec![Point3::origin(), Point3::new(1.0, 0.0, 0.0)]).is_some());
    }

    #[test]
    fn sample_passes_through_control_points() {
        let curve = sample_curve();
        let cps = curve.control_points();
        let num_spans = cps.len() - 1;
        for (i, cp) in cps.iter().enumerate() {
            let t = i as f64 / num_spans as f64;
            let got = curve.sample(t);
            assert_abs_diff_eq!(got, cp, epsilon = 1e-12);
        }
    }

    #[test]
    fn sample_clamps_out_of_range_t() {
        let curve = sample_curve();
        let cps = curve.control_points();
        assert_abs_diff_eq!(curve.sample(-0.5), curve.sample(0.0), epsilon = 1e-15);
        assert_abs_diff_eq!(curve.sample(1.5), curve.sample(1.0), epsilon = 1e-15);
        assert_abs_diff_eq!(curve.sample(0.0), cps[0], epsilon = 1e-12);
        assert_abs_diff_eq!(curve.sample(1.0), cps[cps.len() - 1], epsilon = 1e-12);
    }

    #[test]
    fn tangent_matches_finite_difference_of_sample() {
        let curve = sample_curve();
        let h = 1e-6;
        // Denominator 21 is coprime to the 4 spans, so no sample lands on a span
        // boundary — the curve is C¹ but not C² at the joints, where a central
        // difference straddling the joint is only O(h)-accurate.
        for k in 1..21 {
            let t = f64::from(k) / 21.0;
            let fd = (curve.sample(t + h) - curve.sample(t - h)) / (2.0 * h);
            let analytic = curve.tangent(t);
            assert_abs_diff_eq!(analytic, fd, epsilon = 1e-6);
        }
    }

    #[test]
    fn nearest_point_recovers_small_offset() {
        let curve = sample_curve();
        // Offset a known on-curve point along the surface normal-ish direction
        // (perpendicular to the tangent) by a small amount; nearest_point must
        // return approximately the original on-curve point.
        let t = 0.37;
        let on_curve = curve.sample(t);
        let tan = curve.tangent(t).normalize();
        // Any vector not parallel to the tangent, projected off it.
        let arbitrary = Vector3::new(0.0, 0.0, 1.0);
        let perp = (arbitrary - tan * arbitrary.dot(&tan)).normalize();
        let probe = on_curve + perp * 1e-3;
        let nearest = curve.nearest_point(&probe);
        assert_abs_diff_eq!(nearest, on_curve, epsilon = 1e-5);
    }

    #[test]
    fn nearest_point_on_curve_is_itself() {
        let curve = sample_curve();
        for k in 0..=10 {
            let t = f64::from(k) / 10.0;
            let on_curve = curve.sample(t);
            let nearest = curve.nearest_point(&on_curve);
            assert_abs_diff_eq!(nearest, on_curve, epsilon = 1e-9);
        }
    }

    #[test]
    fn two_points_degenerate_to_segment() {
        let a = Point3::new(-1.0, 0.5, 2.0);
        let b = Point3::new(3.0, -2.0, 1.0);
        let curve = CatmullRomCurve::new(vec![a, b]).unwrap();

        // sample is the straight line between the two points.
        assert_abs_diff_eq!(curve.sample(0.0), a, epsilon = 1e-12);
        assert_abs_diff_eq!(curve.sample(1.0), b, epsilon = 1e-12);

        // nearest_point matches closest_point_segment byte-for-byte.
        for probe in [
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(5.0, 5.0, 5.0),
            Point3::new(-3.0, 0.0, 2.0),
        ] {
            let got = curve.nearest_point(&probe);
            let expected = crate::closest_point_segment(a, b, probe);
            assert_eq!(got, expected);
        }
    }

    #[test]
    fn nearest_point_on_fewer_than_two_points_is_origin() {
        // The free fn is public and unguarded; below two control points the
        // curve is undefined and it must return origin, not panic.
        let probe = Point3::new(1.0, 2.0, 3.0);
        assert_eq!(nearest_point_on_catmull_rom(&[], &probe), Point3::origin());
        assert_eq!(
            nearest_point_on_catmull_rom(&[Point3::new(4.0, 5.0, 6.0)], &probe),
            Point3::origin()
        );
    }
}
