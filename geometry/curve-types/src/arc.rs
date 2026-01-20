//! Arc and circle curves.
//!
//! This module provides parametric representations of circular arcs and
//! full circles. Unlike NURBS circles, these use direct trigonometric
//! evaluation which is more efficient for simple circular geometry.

use crate::{Curve, CurveError, Result};
use nalgebra::{Point3, Vector3};
use std::f64::consts::{PI, TAU};

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

/// A circular arc in 3D space.
///
/// The arc is defined by its center, radius, start and end angles, and
/// the plane normal. The arc is traced counter-clockwise when viewed
/// from the direction of the normal.
///
/// # Example
///
/// ```
/// use curve_types::{Arc, Curve};
/// use nalgebra::{Point3, Vector3};
/// use std::f64::consts::PI;
///
/// // Create a quarter circle in the XY plane
/// let arc = Arc::from_center_radius_angles(
///     Point3::origin(),
///     1.0,
///     0.0,
///     PI / 2.0,
///     Vector3::z(),
/// );
///
/// // Start at (1, 0, 0)
/// let start = arc.point_at(0.0);
/// assert!((start.x - 1.0).abs() < 1e-10);
///
/// // End at (0, 1, 0)
/// let end = arc.point_at(1.0);
/// assert!((end.y - 1.0).abs() < 1e-10);
/// ```
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct Arc {
    /// Center of the arc.
    center: Point3<f64>,
    /// Radius of the arc.
    radius: f64,
    /// Start angle in radians.
    start_angle: f64,
    /// Sweep angle in radians (positive = CCW, negative = CW).
    sweep_angle: f64,
    /// Normal vector of the arc plane.
    normal: Vector3<f64>,
    /// X-axis in the arc plane (direction of angle 0).
    x_axis: Vector3<f64>,
    /// Y-axis in the arc plane.
    y_axis: Vector3<f64>,
}

impl Arc {
    /// Create an arc from center, radius, start/end angles, and plane normal.
    ///
    /// # Parameters
    ///
    /// - `center`: Center point of the arc
    /// - `radius`: Radius (must be positive)
    /// - `start_angle`: Start angle in radians
    /// - `end_angle`: End angle in radians
    /// - `normal`: Normal vector defining the arc plane
    ///
    /// The arc goes counter-clockwise from start to end when viewed from
    /// the direction of the normal.
    #[must_use]
    pub fn from_center_radius_angles(
        center: Point3<f64>,
        radius: f64,
        start_angle: f64,
        end_angle: f64,
        normal: Vector3<f64>,
    ) -> Self {
        let normal = normal.normalize();
        let sweep_angle = end_angle - start_angle;

        // Build local coordinate system
        let (x_axis, y_axis) = build_plane_axes(&normal);

        Self {
            center,
            radius: radius.abs(),
            start_angle,
            sweep_angle,
            normal,
            x_axis,
            y_axis,
        }
    }

    /// Create an arc through three points.
    ///
    /// # Errors
    ///
    /// Returns error if the points are collinear.
    pub fn through_points(p1: Point3<f64>, p2: Point3<f64>, p3: Point3<f64>) -> Result<Self> {
        // Compute the plane normal
        let v1 = p2 - p1;
        let v2 = p3 - p1;
        let normal = v1.cross(&v2);

        if normal.norm() < 1e-10 {
            return Err(CurveError::degenerate("points are collinear"));
        }

        let normal = normal.normalize();

        // Find circle center using perpendicular bisectors
        let mid1 = p1 + v1 / 2.0;
        let mid2 = p1 + v2 / 2.0;

        let bisect1 = normal.cross(&v1).normalize();
        let bisect2 = normal.cross(&v2).normalize();

        // Solve for intersection of bisector lines
        // mid1 + s*bisect1 = mid2 + t*bisect2
        // This is a 2D problem in the plane
        let d = mid2 - mid1;
        let denom = bisect1.x * bisect2.y - bisect1.y * bisect2.x + bisect1.y * bisect2.z
            - bisect1.z * bisect2.y
            + bisect1.z * bisect2.x
            - bisect1.x * bisect2.z;

        if denom.abs() < 1e-10 {
            return Err(CurveError::degenerate("cannot find circle center"));
        }

        // Use least squares approach for robustness
        let a = nalgebra::Matrix3x2::new(
            bisect1.x, -bisect2.x, bisect1.y, -bisect2.y, bisect1.z, -bisect2.z,
        );
        let b = nalgebra::Vector3::new(d.x, d.y, d.z);

        let ata = a.transpose() * a;
        let atb = a.transpose() * b;

        let det = ata[(0, 0)] * ata[(1, 1)] - ata[(0, 1)] * ata[(1, 0)];
        if det.abs() < 1e-10 {
            return Err(CurveError::degenerate("singular system"));
        }

        let s = (ata[(1, 1)] * atb[0] - ata[(0, 1)] * atb[1]) / det;
        let center = mid1 + bisect1 * s;
        let radius = (p1 - center).norm();

        // Compute angles
        let (x_axis, y_axis) = build_plane_axes(&normal);
        let to_angle = |p: Point3<f64>| -> f64 {
            let v = p - center;
            let x = v.dot(&x_axis);
            let y = v.dot(&y_axis);
            y.atan2(x)
        };

        let angle1 = to_angle(p1);
        let angle2 = to_angle(p2);
        let angle3 = to_angle(p3);

        // Determine arc direction
        let mut sweep = angle3 - angle1;
        let mid_angle = angle2 - angle1;

        // Normalize angles
        let normalize = |a: f64| -> f64 {
            let mut a = a;
            while a < 0.0 {
                a += TAU;
            }
            while a > TAU {
                a -= TAU;
            }
            a
        };

        let sweep_norm = normalize(sweep);
        let mid_norm = normalize(mid_angle);

        // Check if p2 is on the short or long arc
        let on_short = mid_norm < sweep_norm || (sweep_norm < PI && mid_norm < PI);

        if !on_short {
            // Take the long way around
            sweep = if sweep > 0.0 {
                sweep - TAU
            } else {
                sweep + TAU
            };
        }

        Ok(Self {
            center,
            radius,
            start_angle: angle1,
            sweep_angle: sweep,
            normal,
            x_axis,
            y_axis,
        })
    }

    /// Get the center of the arc.
    #[must_use]
    pub fn center(&self) -> Point3<f64> {
        self.center
    }

    /// Get the radius.
    #[must_use]
    pub fn radius(&self) -> f64 {
        self.radius
    }

    /// Get the start angle in radians.
    #[must_use]
    pub fn start_angle(&self) -> f64 {
        self.start_angle
    }

    /// Get the end angle in radians.
    #[must_use]
    pub fn end_angle(&self) -> f64 {
        self.start_angle + self.sweep_angle
    }

    /// Get the sweep angle in radians.
    #[must_use]
    pub fn sweep_angle(&self) -> f64 {
        self.sweep_angle
    }

    /// Get the plane normal.
    #[must_use]
    pub fn normal(&self) -> Vector3<f64> {
        self.normal
    }

    /// Convert angle to position on the arc.
    fn angle_to_point(&self, angle: f64) -> Point3<f64> {
        let x = angle.cos();
        let y = angle.sin();
        self.center + (self.x_axis * x + self.y_axis * y) * self.radius
    }

    /// Convert position to angle.
    #[allow(dead_code)]
    fn point_to_angle(&self, p: Point3<f64>) -> f64 {
        let v = p - self.center;
        let x = v.dot(&self.x_axis);
        let y = v.dot(&self.y_axis);
        y.atan2(x)
    }
}

impl Curve for Arc {
    fn point_at(&self, t: f64) -> Point3<f64> {
        let t = t.clamp(0.0, 1.0);
        let angle = self.start_angle + t * self.sweep_angle;
        self.angle_to_point(angle)
    }

    fn tangent_at(&self, t: f64) -> Vector3<f64> {
        let t = t.clamp(0.0, 1.0);
        let angle = self.start_angle + t * self.sweep_angle;

        // Tangent is perpendicular to radius direction
        let dir = if self.sweep_angle >= 0.0 { 1.0 } else { -1.0 };
        (-self.x_axis * angle.sin() + self.y_axis * angle.cos()) * dir
    }

    fn derivative_at(&self, t: f64) -> Vector3<f64> {
        self.tangent_at(t) * (self.radius * self.sweep_angle.abs())
    }

    fn second_derivative_at(&self, t: f64) -> Vector3<f64> {
        let t = t.clamp(0.0, 1.0);
        let angle = self.start_angle + t * self.sweep_angle;

        // Second derivative points toward center
        let radial = -(self.x_axis * angle.cos() + self.y_axis * angle.sin());
        radial * (self.radius * self.sweep_angle * self.sweep_angle)
    }

    fn arc_length(&self) -> f64 {
        self.radius * self.sweep_angle.abs()
    }

    fn arc_length_between(&self, t0: f64, t1: f64) -> f64 {
        self.radius * self.sweep_angle.abs() * (t1 - t0).abs()
    }

    fn arc_to_t(&self, s: f64) -> f64 {
        let total = self.arc_length();
        if total < 1e-10 {
            return 0.0;
        }
        (s / total).clamp(0.0, 1.0)
    }

    fn t_to_arc(&self, t: f64) -> f64 {
        self.radius * self.sweep_angle.abs() * t.clamp(0.0, 1.0)
    }

    fn curvature_at(&self, _t: f64) -> f64 {
        1.0 / self.radius
    }

    fn normal_at(&self, t: f64) -> Vector3<f64> {
        let t = t.clamp(0.0, 1.0);
        let angle = self.start_angle + t * self.sweep_angle;

        // Normal points toward center
        -(self.x_axis * angle.cos() + self.y_axis * angle.sin())
    }

    fn bounding_box(&self) -> (Point3<f64>, Point3<f64>) {
        // Sample the arc and include extreme points
        let mut min = self.point_at(0.0);
        let mut max = min;

        let update = |p: Point3<f64>, min: &mut Point3<f64>, max: &mut Point3<f64>| {
            min.x = min.x.min(p.x);
            min.y = min.y.min(p.y);
            min.z = min.z.min(p.z);
            max.x = max.x.max(p.x);
            max.y = max.y.max(p.y);
            max.z = max.z.max(p.z);
        };

        // Sample along the arc
        for i in 1..=32 {
            let t = i as f64 / 32.0;
            update(self.point_at(t), &mut min, &mut max);
        }

        // Check cardinal directions if they're in the arc range
        for angle_offset in [0.0, PI / 2.0, PI, 3.0 * PI / 2.0] {
            let angle = angle_offset;
            let t = (angle - self.start_angle) / self.sweep_angle;
            if (0.0..=1.0).contains(&t) {
                update(self.angle_to_point(angle), &mut min, &mut max);
            }
        }

        (min, max)
    }
}

/// A full circle in 3D space.
///
/// This is a closed curve representing a complete circle.
///
/// # Example
///
/// ```
/// use curve_types::{Circle, Curve};
/// use nalgebra::{Point3, Vector3};
///
/// let circle = Circle::new(Point3::origin(), 2.0, Vector3::z());
///
/// // Circle is closed
/// assert!(circle.is_closed());
///
/// // Circumference
/// assert!((circle.arc_length() - 4.0 * std::f64::consts::PI).abs() < 1e-10);
/// ```
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct Circle {
    /// The underlying arc (full circle).
    arc: Arc,
}

impl Circle {
    /// Create a circle from center, radius, and plane normal.
    #[must_use]
    pub fn new(center: Point3<f64>, radius: f64, normal: Vector3<f64>) -> Self {
        let arc = Arc::from_center_radius_angles(center, radius, 0.0, TAU, normal);
        Self { arc }
    }

    /// Create a circle through three points.
    ///
    /// # Errors
    ///
    /// Returns error if the points are collinear.
    pub fn through_points(p1: Point3<f64>, p2: Point3<f64>, p3: Point3<f64>) -> Result<Self> {
        let arc = Arc::through_points(p1, p2, p3)?;
        Ok(Self::new(arc.center(), arc.radius(), arc.normal()))
    }

    /// Get the center.
    #[must_use]
    pub fn center(&self) -> Point3<f64> {
        self.arc.center()
    }

    /// Get the radius.
    #[must_use]
    pub fn radius(&self) -> f64 {
        self.arc.radius()
    }

    /// Get the plane normal.
    #[must_use]
    pub fn normal(&self) -> Vector3<f64> {
        self.arc.normal()
    }

    /// Get the diameter.
    #[must_use]
    pub fn diameter(&self) -> f64 {
        self.arc.radius() * 2.0
    }

    /// Get the circumference.
    #[must_use]
    pub fn circumference(&self) -> f64 {
        TAU * self.arc.radius()
    }

    /// Get the area of the disk.
    #[must_use]
    pub fn area(&self) -> f64 {
        PI * self.arc.radius() * self.arc.radius()
    }

    /// Check if a point lies on the circle (within tolerance).
    #[must_use]
    pub fn contains_point(&self, p: Point3<f64>, tolerance: f64) -> bool {
        let v = p - self.center();

        // Check if in plane
        if v.dot(&self.normal()).abs() > tolerance {
            return false;
        }

        // Check if at correct radius
        let dist = v.norm();
        (dist - self.radius()).abs() < tolerance
    }
}

impl Curve for Circle {
    fn point_at(&self, t: f64) -> Point3<f64> {
        self.arc.point_at(t)
    }

    fn tangent_at(&self, t: f64) -> Vector3<f64> {
        self.arc.tangent_at(t)
    }

    fn derivative_at(&self, t: f64) -> Vector3<f64> {
        self.arc.derivative_at(t)
    }

    fn second_derivative_at(&self, t: f64) -> Vector3<f64> {
        self.arc.second_derivative_at(t)
    }

    fn arc_length(&self) -> f64 {
        self.arc.arc_length()
    }

    fn curvature_at(&self, t: f64) -> f64 {
        self.arc.curvature_at(t)
    }

    fn normal_at(&self, t: f64) -> Vector3<f64> {
        self.arc.normal_at(t)
    }

    fn is_closed(&self) -> bool {
        true
    }

    fn bounding_box(&self) -> (Point3<f64>, Point3<f64>) {
        self.arc.bounding_box()
    }
}

/// Build orthonormal axes for a plane given its normal.
///
/// For Z-up normal, returns X=(1,0,0), Y=(0,1,0).
fn build_plane_axes(normal: &Vector3<f64>) -> (Vector3<f64>, Vector3<f64>) {
    let n = normal.normalize();

    // Choose a reference vector not parallel to normal
    let reference = if n.z.abs() < 0.9 {
        Vector3::z()
    } else {
        Vector3::x()
    };

    // x_axis = n × reference (perpendicular to both)
    let y_axis = n.cross(&reference).normalize();
    let x_axis = y_axis.cross(&n);

    (x_axis, y_axis)
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn test_arc_quarter_circle() {
        let arc =
            Arc::from_center_radius_angles(Point3::origin(), 1.0, 0.0, PI / 2.0, Vector3::z());

        // Start at (1, 0)
        let start = arc.point_at(0.0);
        assert_relative_eq!(start.x, 1.0, epsilon = 1e-10);
        assert_relative_eq!(start.y, 0.0, epsilon = 1e-10);

        // End at (0, 1)
        let end = arc.point_at(1.0);
        assert_relative_eq!(end.x, 0.0, epsilon = 1e-10);
        assert_relative_eq!(end.y, 1.0, epsilon = 1e-10);

        // Arc length
        assert_relative_eq!(arc.arc_length(), PI / 2.0, epsilon = 1e-10);
    }

    #[test]
    fn test_arc_semicircle() {
        let arc =
            Arc::from_center_radius_angles(Point3::new(1.0, 1.0, 0.0), 2.0, 0.0, PI, Vector3::z());

        // All points at radius 2 from center
        for i in 0..=10 {
            let t = i as f64 / 10.0;
            let p = arc.point_at(t);
            let dist = (p - arc.center()).norm();
            assert_relative_eq!(dist, 2.0, epsilon = 1e-10);
        }
    }

    #[test]
    fn test_arc_clockwise() {
        let arc = Arc::from_center_radius_angles(
            Point3::origin(),
            1.0,
            0.0,
            -PI / 2.0, // Negative = clockwise
            Vector3::z(),
        );

        // End at (0, -1)
        let end = arc.point_at(1.0);
        assert_relative_eq!(end.x, 0.0, epsilon = 1e-10);
        assert_relative_eq!(end.y, -1.0, epsilon = 1e-10);
    }

    #[test]
    fn test_arc_curvature() {
        let arc = Arc::from_center_radius_angles(Point3::origin(), 2.0, 0.0, PI, Vector3::z());

        // Curvature = 1/radius
        assert_relative_eq!(arc.curvature_at(0.5), 0.5, epsilon = 1e-10);
    }

    #[test]
    fn test_arc_tangent() {
        let arc =
            Arc::from_center_radius_angles(Point3::origin(), 1.0, 0.0, PI / 2.0, Vector3::z());

        // At start, tangent is +Y
        let tan_start = arc.tangent_at(0.0);
        assert_relative_eq!(tan_start.x, 0.0, epsilon = 1e-10);
        assert_relative_eq!(tan_start.y, 1.0, epsilon = 1e-10);

        // At end, tangent is -X
        let tan_end = arc.tangent_at(1.0);
        assert_relative_eq!(tan_end.x, -1.0, epsilon = 1e-10);
        assert_relative_eq!(tan_end.y, 0.0, epsilon = 1e-10);
    }

    #[test]
    #[allow(clippy::unwrap_used)]
    fn test_arc_through_points() {
        let p1 = Point3::new(1.0, 0.0, 0.0);
        let p2 = Point3::new(0.0, 1.0, 0.0);
        let p3 = Point3::new(-1.0, 0.0, 0.0);

        let arc = Arc::through_points(p1, p2, p3).unwrap();

        // Center should be at origin
        assert_relative_eq!(arc.center().x, 0.0, epsilon = 1e-6);
        assert_relative_eq!(arc.center().y, 0.0, epsilon = 1e-6);

        // Radius should be 1
        assert_relative_eq!(arc.radius(), 1.0, epsilon = 1e-6);

        // Arc should pass through all three points
        let start = arc.point_at(0.0);
        assert_relative_eq!((start - p1).norm(), 0.0, epsilon = 1e-6);

        let end = arc.point_at(1.0);
        assert_relative_eq!((end - p3).norm(), 0.0, epsilon = 1e-6);
    }

    #[test]
    fn test_circle() {
        let circle = Circle::new(Point3::origin(), 3.0, Vector3::z());

        assert!(circle.is_closed());
        assert_relative_eq!(circle.circumference(), TAU * 3.0, epsilon = 1e-10);
        assert_relative_eq!(circle.area(), PI * 9.0, epsilon = 1e-10);
        assert_relative_eq!(circle.diameter(), 6.0, epsilon = 1e-10);
    }

    #[test]
    fn test_circle_contains_point() {
        let circle = Circle::new(Point3::origin(), 1.0, Vector3::z());

        // Point on circle
        assert!(circle.contains_point(Point3::new(1.0, 0.0, 0.0), 1e-6));
        assert!(circle.contains_point(Point3::new(0.0, 1.0, 0.0), 1e-6));

        // Point not on circle
        assert!(!circle.contains_point(Point3::new(0.5, 0.0, 0.0), 1e-6));
        assert!(!circle.contains_point(Point3::new(1.0, 0.0, 1.0), 1e-6)); // Out of plane
    }

    #[test]
    #[allow(clippy::unwrap_used)]
    fn test_circle_through_points() {
        let p1 = Point3::new(1.0, 0.0, 0.0);
        let p2 = Point3::new(0.0, 1.0, 0.0);
        let p3 = Point3::new(-1.0, 0.0, 0.0);

        let circle = Circle::through_points(p1, p2, p3).unwrap();

        assert_relative_eq!(circle.center().x, 0.0, epsilon = 1e-6);
        assert_relative_eq!(circle.center().y, 0.0, epsilon = 1e-6);
        assert_relative_eq!(circle.radius(), 1.0, epsilon = 1e-6);
    }

    #[test]
    fn test_arc_collinear_error() {
        let p1 = Point3::new(0.0, 0.0, 0.0);
        let p2 = Point3::new(1.0, 0.0, 0.0);
        let p3 = Point3::new(2.0, 0.0, 0.0);

        let result = Arc::through_points(p1, p2, p3);
        assert!(result.is_err());
    }
}
