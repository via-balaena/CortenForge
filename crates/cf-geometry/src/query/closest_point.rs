//! Closest-point geometric queries.
//!
//! Pure geometry — all functions operate in local space (no pose transforms).

#![allow(clippy::suboptimal_flops)]

use nalgebra::Point3;

/// Epsilon for degenerate-geometry guards.
const EPSILON: f64 = 1e-10;

/// Closest point on triangle (v0, v1, v2) to point `p`.
///
/// Uses the Voronoi region method from *Real-Time Collision Detection* §5.1.5.
/// Handles degenerate triangles (zero area) gracefully by falling back to the
/// nearest vertex or edge midpoint.
#[must_use]
pub fn closest_point_on_triangle(
    v0: Point3<f64>,
    v1: Point3<f64>,
    v2: Point3<f64>,
    p: Point3<f64>,
) -> Point3<f64> {
    let ab = v1 - v0;
    let ac = v2 - v0;
    let ap = p - v0;

    let d1 = ab.dot(&ap);
    let d2 = ac.dot(&ap);
    if d1 <= 0.0 && d2 <= 0.0 {
        return v0;
    }

    let bp = p - v1;
    let d3 = ab.dot(&bp);
    let d4 = ac.dot(&bp);
    if d3 >= 0.0 && d4 <= d3 {
        return v1;
    }

    // Edge AB
    let vc = d1 * d4 - d3 * d2;
    if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
        let denom = d1 - d3;
        let v = if denom.abs() > EPSILON {
            d1 / denom
        } else {
            0.5
        };
        return Point3::from(v0.coords + ab * v);
    }

    let cp = p - v2;
    let d5 = ab.dot(&cp);
    let d6 = ac.dot(&cp);
    if d6 >= 0.0 && d5 <= d6 {
        return v2;
    }

    // Edge AC
    let vb = d5 * d2 - d1 * d6;
    if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
        let denom = d2 - d6;
        let w = if denom.abs() > EPSILON {
            d2 / denom
        } else {
            0.5
        };
        return Point3::from(v0.coords + ac * w);
    }

    // Edge BC
    let va = d3 * d6 - d5 * d4;
    if va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0 {
        let num = d4 - d3;
        let denom = num + (d5 - d6);
        let w = if denom.abs() > EPSILON {
            num / denom
        } else {
            0.5
        };
        return Point3::from(v1.coords + (v2 - v1) * w);
    }

    // Interior
    let total = va + vb + vc;
    if total.abs() < EPSILON {
        return v0;
    }
    let denom = 1.0 / total;
    let v = vb * denom;
    let w = vc * denom;
    Point3::from(v0.coords + ab * v + ac * w)
}

/// Closest point on line segment `[a, b]` to point `p`.
///
/// Returns the point on the segment nearest to `p`. For degenerate segments
/// (a ≈ b), returns `a`.
#[inline]
#[must_use]
pub fn closest_point_segment(a: Point3<f64>, b: Point3<f64>, p: Point3<f64>) -> Point3<f64> {
    let ab = b - a;
    let ap = p - a;
    let ab_len_sq = ab.dot(&ab);

    if ab_len_sq < EPSILON {
        return a;
    }

    let t = (ap.dot(&ab) / ab_len_sq).clamp(0.0, 1.0);
    Point3::from(a.coords + ab * t)
}

/// Closest points between two line segments `[p1, q1]` and `[p2, q2]`.
///
/// Returns `(point_on_seg1, point_on_seg2)`. Uses the standard
/// segment–segment distance algorithm with proper clamping for all
/// degenerate cases (zero-length segments, parallel segments).
#[must_use]
#[allow(clippy::many_single_char_names)]
pub fn closest_points_segments(
    p1: Point3<f64>,
    q1: Point3<f64>,
    p2: Point3<f64>,
    q2: Point3<f64>,
) -> (Point3<f64>, Point3<f64>) {
    let d1 = q1 - p1;
    let d2 = q2 - p2;
    let r = p1 - p2;

    let a = d1.dot(&d1);
    let e = d2.dot(&d2);
    let f = d2.dot(&r);

    // Both segments degenerate
    if a < EPSILON && e < EPSILON {
        return (p1, p2);
    }
    // First segment degenerate
    if a < EPSILON {
        let t = (f / e).clamp(0.0, 1.0);
        return (p1, Point3::from(p2.coords + d2 * t));
    }
    // Second segment degenerate
    if e < EPSILON {
        let s = (-d1.dot(&r) / a).clamp(0.0, 1.0);
        return (Point3::from(p1.coords + d1 * s), p2);
    }

    let b = d1.dot(&d2);
    let c = d1.dot(&r);
    // Determinant of the 2×2 system: denom = a*e − b²
    #[allow(clippy::suspicious_operation_groupings)]
    let denom = a * e - b * b;

    let (mut s, mut t) = if denom.abs() < EPSILON {
        // Parallel segments
        (0.0, f / e)
    } else {
        let s_val = (b * f - c * e) / denom;
        let t_val = (b * s_val + f) / e;
        (s_val, t_val)
    };

    // Clamp and recompute
    if s < 0.0 {
        s = 0.0;
        t = (f / e).clamp(0.0, 1.0);
    } else if s > 1.0 {
        s = 1.0;
        t = ((b + f) / e).clamp(0.0, 1.0);
    }

    if t < 0.0 {
        t = 0.0;
        s = (-c / a).clamp(0.0, 1.0);
    } else if t > 1.0 {
        t = 1.0;
        s = ((b - c) / a).clamp(0.0, 1.0);
    }

    (
        Point3::from(p1.coords + d1 * s),
        Point3::from(p2.coords + d2 * t),
    )
}

// =============================================================================
// Tests
// =============================================================================

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::float_cmp)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    // ── closest_point_on_triangle ─────────────────────────────────────

    #[test]
    fn triangle_point_on_face() {
        let v0 = Point3::new(0.0, 0.0, 0.0);
        let v1 = Point3::new(4.0, 0.0, 0.0);
        let v2 = Point3::new(0.0, 4.0, 0.0);

        // Point directly above centroid
        let p = Point3::new(1.0, 1.0, 5.0);
        let c = closest_point_on_triangle(v0, v1, v2, p);
        assert_relative_eq!(c.x, 1.0, epsilon = 1e-10);
        assert_relative_eq!(c.y, 1.0, epsilon = 1e-10);
        assert_relative_eq!(c.z, 0.0, epsilon = 1e-10);
    }

    #[test]
    fn triangle_point_at_vertex() {
        let v0 = Point3::new(0.0, 0.0, 0.0);
        let v1 = Point3::new(1.0, 0.0, 0.0);
        let v2 = Point3::new(0.0, 1.0, 0.0);

        // Nearest to v0
        let p = Point3::new(-1.0, -1.0, 0.0);
        let c = closest_point_on_triangle(v0, v1, v2, p);
        assert_relative_eq!(c, v0, epsilon = 1e-10);

        // Nearest to v1
        let p = Point3::new(2.0, -1.0, 0.0);
        let c = closest_point_on_triangle(v0, v1, v2, p);
        assert_relative_eq!(c, v1, epsilon = 1e-10);

        // Nearest to v2
        let p = Point3::new(-1.0, 2.0, 0.0);
        let c = closest_point_on_triangle(v0, v1, v2, p);
        assert_relative_eq!(c, v2, epsilon = 1e-10);
    }

    #[test]
    fn triangle_point_on_edge() {
        let v0 = Point3::new(0.0, 0.0, 0.0);
        let v1 = Point3::new(2.0, 0.0, 0.0);
        let v2 = Point3::new(0.0, 2.0, 0.0);

        // Nearest to edge AB (below triangle)
        let p = Point3::new(1.0, -1.0, 0.0);
        let c = closest_point_on_triangle(v0, v1, v2, p);
        assert_relative_eq!(c.x, 1.0, epsilon = 1e-10);
        assert_relative_eq!(c.y, 0.0, epsilon = 1e-10);
    }

    #[test]
    fn triangle_degenerate_zero_area() {
        // Collinear vertices
        let v0 = Point3::new(0.0, 0.0, 0.0);
        let v1 = Point3::new(1.0, 0.0, 0.0);
        let v2 = Point3::new(2.0, 0.0, 0.0);

        let p = Point3::new(0.5, 1.0, 0.0);
        let c = closest_point_on_triangle(v0, v1, v2, p);
        // Should return some valid point on/near the degenerate triangle
        assert!(c.x.is_finite() && c.y.is_finite() && c.z.is_finite());
    }

    // ── closest_point_segment ─────────────────────────────────────────

    #[test]
    fn segment_midpoint() {
        let a = Point3::new(0.0, 0.0, 0.0);
        let b = Point3::new(2.0, 0.0, 0.0);
        let p = Point3::new(1.0, 1.0, 0.0);
        let c = closest_point_segment(a, b, p);
        assert_relative_eq!(c, Point3::new(1.0, 0.0, 0.0), epsilon = 1e-10);
    }

    #[test]
    fn segment_clamp_start() {
        let a = Point3::new(0.0, 0.0, 0.0);
        let b = Point3::new(2.0, 0.0, 0.0);
        let p = Point3::new(-1.0, 0.0, 0.0);
        let c = closest_point_segment(a, b, p);
        assert_relative_eq!(c, a, epsilon = 1e-10);
    }

    #[test]
    fn segment_clamp_end() {
        let a = Point3::new(0.0, 0.0, 0.0);
        let b = Point3::new(2.0, 0.0, 0.0);
        let p = Point3::new(5.0, 0.0, 0.0);
        let c = closest_point_segment(a, b, p);
        assert_relative_eq!(c, b, epsilon = 1e-10);
    }

    #[test]
    fn segment_degenerate_zero_length() {
        let a = Point3::new(1.0, 2.0, 3.0);
        let b = a;
        let p = Point3::new(5.0, 5.0, 5.0);
        let c = closest_point_segment(a, b, p);
        assert_relative_eq!(c, a, epsilon = 1e-10);
    }

    // ── closest_points_segments ───────────────────────────────────────

    #[test]
    fn segments_crossing() {
        // Two segments crossing at origin
        let p1 = Point3::new(-1.0, 0.0, 0.0);
        let q1 = Point3::new(1.0, 0.0, 0.0);
        let p2 = Point3::new(0.0, -1.0, 0.0);
        let q2 = Point3::new(0.0, 1.0, 0.0);

        let (c1, c2) = closest_points_segments(p1, q1, p2, q2);
        assert_relative_eq!(c1, Point3::origin(), epsilon = 1e-10);
        assert_relative_eq!(c2, Point3::origin(), epsilon = 1e-10);
    }

    #[test]
    fn segments_parallel() {
        let p1 = Point3::new(0.0, 0.0, 0.0);
        let q1 = Point3::new(2.0, 0.0, 0.0);
        let p2 = Point3::new(0.0, 1.0, 0.0);
        let q2 = Point3::new(2.0, 1.0, 0.0);

        let (c1, c2) = closest_points_segments(p1, q1, p2, q2);
        // Distance should be 1.0 (in Y)
        assert_relative_eq!((c1 - c2).norm(), 1.0, epsilon = 1e-6);
    }

    #[test]
    fn segments_skew() {
        // Z-axis segment and X-axis segment offset in Y
        let p1 = Point3::new(0.0, 0.0, -1.0);
        let q1 = Point3::new(0.0, 0.0, 1.0);
        let p2 = Point3::new(-1.0, 1.0, 0.0);
        let q2 = Point3::new(1.0, 1.0, 0.0);

        let (c1, c2) = closest_points_segments(p1, q1, p2, q2);
        assert_relative_eq!(c1, Point3::new(0.0, 0.0, 0.0), epsilon = 1e-10);
        assert_relative_eq!(c2, Point3::new(0.0, 1.0, 0.0), epsilon = 1e-10);
    }

    #[test]
    fn segments_both_degenerate() {
        let p1 = Point3::new(1.0, 0.0, 0.0);
        let q1 = p1;
        let p2 = Point3::new(0.0, 1.0, 0.0);
        let q2 = p2;

        let (c1, c2) = closest_points_segments(p1, q1, p2, q2);
        assert_relative_eq!(c1, p1, epsilon = 1e-10);
        assert_relative_eq!(c2, p2, epsilon = 1e-10);
    }

    #[test]
    fn segments_one_degenerate() {
        let p1 = Point3::new(0.0, 0.0, 0.0);
        let q1 = p1;
        let p2 = Point3::new(-1.0, 1.0, 0.0);
        let q2 = Point3::new(1.0, 1.0, 0.0);

        let (c1, c2) = closest_points_segments(p1, q1, p2, q2);
        assert_relative_eq!(c1, p1, epsilon = 1e-10);
        assert_relative_eq!(c2, Point3::new(0.0, 1.0, 0.0), epsilon = 1e-10);
    }
}
