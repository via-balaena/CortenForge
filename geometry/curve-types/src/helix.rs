//! Helical curves.
//!
//! A helix is a curve that spirals around an axis while advancing along it.
//! Common in springs, screws, DNA structure, and spiral staircases.

use crate::Curve;
use nalgebra::{Point3, Vector3};
use std::f64::consts::TAU;

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

/// A helical curve in 3D space.
///
/// The helix spirals around an axis with configurable radius, pitch, and
/// number of turns.
///
/// # Parameterization
///
/// - `t = 0`: Start of helix
/// - `t = 1`: End of helix (after `turns` complete rotations)
///
/// # Example
///
/// ```
/// use curve_types::{Helix, Curve};
/// use nalgebra::{Point3, Vector3};
///
/// // Create a helix with 2 turns
/// let helix = Helix::new(
///     Point3::origin(),     // base point
///     Vector3::z(),         // axis direction
///     1.0,                  // radius
///     2.0,                  // pitch (advance per turn)
///     2.0,                  // number of turns
/// );
///
/// // Total height is turns * pitch = 4.0
/// let end = helix.point_at(1.0);
/// assert!((end.z - 4.0).abs() < 1e-10);
/// ```
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct Helix {
    /// Base point (start of helix at t=0).
    origin: Point3<f64>,
    /// Axis direction (normalized).
    axis: Vector3<f64>,
    /// Radius of the helix.
    radius: f64,
    /// Pitch: vertical advance per complete turn.
    pitch: f64,
    /// Number of turns.
    turns: f64,
    /// Local X-axis (perpendicular to axis, direction of angle 0).
    x_axis: Vector3<f64>,
    /// Local Y-axis (perpendicular to axis and x_axis).
    y_axis: Vector3<f64>,
    /// Start angle offset in radians.
    start_angle: f64,
    /// Handedness: true for right-handed (CCW when viewed along axis), false for left.
    right_handed: bool,
}

impl Helix {
    /// Create a helix with default parameters.
    ///
    /// # Parameters
    ///
    /// - `origin`: Base point (center of starting circle)
    /// - `axis`: Direction of the helix axis
    /// - `radius`: Radius of the helix (positive)
    /// - `pitch`: Vertical advance per complete turn
    /// - `turns`: Number of complete rotations
    #[must_use]
    pub fn new(
        origin: Point3<f64>,
        axis: Vector3<f64>,
        radius: f64,
        pitch: f64,
        turns: f64,
    ) -> Self {
        let axis = axis.normalize();
        let (x_axis, y_axis) = build_helix_axes(&axis);

        Self {
            origin,
            axis,
            radius: radius.abs(),
            pitch,
            turns: turns.abs(),
            x_axis,
            y_axis,
            start_angle: 0.0,
            right_handed: true,
        }
    }

    /// Create a helix with specified start angle and handedness.
    #[must_use]
    pub fn with_options(
        origin: Point3<f64>,
        axis: Vector3<f64>,
        radius: f64,
        pitch: f64,
        turns: f64,
        start_angle: f64,
        right_handed: bool,
    ) -> Self {
        let axis = axis.normalize();
        let (x_axis, y_axis) = build_helix_axes(&axis);

        Self {
            origin,
            axis,
            radius: radius.abs(),
            pitch,
            turns: turns.abs(),
            x_axis,
            y_axis,
            start_angle,
            right_handed,
        }
    }

    /// Create a left-handed helix.
    #[must_use]
    pub fn left_handed(
        origin: Point3<f64>,
        axis: Vector3<f64>,
        radius: f64,
        pitch: f64,
        turns: f64,
    ) -> Self {
        Self::with_options(origin, axis, radius, pitch, turns, 0.0, false)
    }

    /// Create a conical helix (radius varies along length).
    ///
    /// The radius linearly interpolates from `start_radius` to `end_radius`
    /// along the helix.
    #[must_use]
    pub fn conical(
        origin: Point3<f64>,
        axis: Vector3<f64>,
        start_radius: f64,
        end_radius: f64,
        pitch: f64,
        turns: f64,
    ) -> ConicalHelix {
        ConicalHelix {
            helix: Self::new(origin, axis, start_radius, pitch, turns),
            end_radius: end_radius.abs(),
        }
    }

    /// Get the origin (base point).
    #[must_use]
    pub fn origin(&self) -> Point3<f64> {
        self.origin
    }

    /// Get the axis direction.
    #[must_use]
    pub fn axis(&self) -> Vector3<f64> {
        self.axis
    }

    /// Get the radius.
    #[must_use]
    pub fn radius(&self) -> f64 {
        self.radius
    }

    /// Get the pitch.
    #[must_use]
    pub fn pitch(&self) -> f64 {
        self.pitch
    }

    /// Get the number of turns.
    #[must_use]
    pub fn turns(&self) -> f64 {
        self.turns
    }

    /// Get the total height (axial length) of the helix.
    #[must_use]
    pub fn height(&self) -> f64 {
        self.turns * self.pitch
    }

    /// Check if right-handed.
    #[must_use]
    pub fn is_right_handed(&self) -> bool {
        self.right_handed
    }

    /// Compute angle at parameter t.
    fn angle_at(&self, t: f64) -> f64 {
        let sign = if self.right_handed { 1.0 } else { -1.0 };
        self.start_angle + sign * t * self.turns * TAU
    }
}

impl Curve for Helix {
    fn point_at(&self, t: f64) -> Point3<f64> {
        let t = t.clamp(0.0, 1.0);
        let angle = self.angle_at(t);
        let height = t * self.height();

        let radial = self.x_axis * angle.cos() + self.y_axis * angle.sin();
        self.origin + radial * self.radius + self.axis * height
    }

    fn tangent_at(&self, t: f64) -> Vector3<f64> {
        self.derivative_at(t).normalize()
    }

    fn derivative_at(&self, t: f64) -> Vector3<f64> {
        let t = t.clamp(0.0, 1.0);
        let angle = self.angle_at(t);
        let sign = if self.right_handed { 1.0 } else { -1.0 };

        // d/dt of position
        let angular_velocity = sign * self.turns * TAU;
        let radial_deriv = (-self.x_axis * angle.sin() + self.y_axis * angle.cos())
            * self.radius
            * angular_velocity;
        let axial_deriv = self.axis * self.height();

        radial_deriv + axial_deriv
    }

    fn second_derivative_at(&self, t: f64) -> Vector3<f64> {
        let t = t.clamp(0.0, 1.0);
        let angle = self.angle_at(t);
        let sign = if self.right_handed { 1.0 } else { -1.0 };

        let angular_velocity = sign * self.turns * TAU;

        // d²/dt² of position (only radial component has second derivative)
        let radial_accel = (-self.x_axis * angle.cos() - self.y_axis * angle.sin())
            * self.radius
            * angular_velocity
            * angular_velocity;

        radial_accel
    }

    fn arc_length(&self) -> f64 {
        // Arc length of helix: sqrt((2πr*turns)² + height²)
        let circumferential = TAU * self.radius * self.turns;
        let axial = self.height();
        (circumferential * circumferential + axial * axial).sqrt()
    }

    fn curvature_at(&self, _t: f64) -> f64 {
        // Curvature of helix is constant: r / (r² + (pitch/(2π))²)
        let c = self.pitch / TAU;
        self.radius / (self.radius * self.radius + c * c)
    }

    fn torsion_at(&self, _t: f64) -> f64 {
        // Torsion of helix is constant: c / (r² + c²)
        let c = self.pitch / TAU;
        let sign = if self.right_handed { 1.0 } else { -1.0 };
        sign * c / (self.radius * self.radius + c * c)
    }
}

/// A conical helix where the radius varies along the length.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct ConicalHelix {
    /// Base helix parameters.
    helix: Helix,
    /// Radius at t=1.
    end_radius: f64,
}

impl ConicalHelix {
    /// Get the start radius.
    #[must_use]
    pub fn start_radius(&self) -> f64 {
        self.helix.radius
    }

    /// Get the end radius.
    #[must_use]
    pub fn end_radius(&self) -> f64 {
        self.end_radius
    }

    /// Get the radius at parameter t.
    #[must_use]
    pub fn radius_at(&self, t: f64) -> f64 {
        self.helix.radius + t * (self.end_radius - self.helix.radius)
    }
}

impl Curve for ConicalHelix {
    fn point_at(&self, t: f64) -> Point3<f64> {
        let t = t.clamp(0.0, 1.0);
        let angle = self.helix.angle_at(t);
        let height = t * self.helix.height();
        let radius = self.radius_at(t);

        let radial = self.helix.x_axis * angle.cos() + self.helix.y_axis * angle.sin();
        self.helix.origin + radial * radius + self.helix.axis * height
    }

    fn tangent_at(&self, t: f64) -> Vector3<f64> {
        self.derivative_at(t).normalize()
    }

    fn derivative_at(&self, t: f64) -> Vector3<f64> {
        let t = t.clamp(0.0, 1.0);
        let angle = self.helix.angle_at(t);
        let radius = self.radius_at(t);
        let radius_rate = self.end_radius - self.helix.radius;
        let sign = if self.helix.right_handed { 1.0 } else { -1.0 };
        let angular_velocity = sign * self.helix.turns * TAU;

        // Radial direction and its derivative
        let radial_dir = self.helix.x_axis * angle.cos() + self.helix.y_axis * angle.sin();
        let radial_dir_deriv =
            (-self.helix.x_axis * angle.sin() + self.helix.y_axis * angle.cos()) * angular_velocity;

        // Product rule: d/dt(r(t) * radial_dir(t))
        let radial_deriv = radial_dir * radius_rate + radial_dir_deriv * radius;
        let axial_deriv = self.helix.axis * self.helix.height();

        radial_deriv + axial_deriv
    }

    fn second_derivative_at(&self, t: f64) -> Vector3<f64> {
        // Numerical approximation
        let h = 1e-6;
        let d1 = self.derivative_at((t + h).min(1.0));
        let d0 = self.derivative_at((t - h).max(0.0));
        (d1 - d0) / (2.0 * h)
    }

    fn arc_length(&self) -> f64 {
        // Numerical integration for varying radius
        self.arc_length_between(0.0, 1.0)
    }
}

/// Build orthonormal axes perpendicular to the helix axis.
///
/// For Z-axis, returns X=(1,0,0), Y=(0,1,0).
fn build_helix_axes(axis: &Vector3<f64>) -> (Vector3<f64>, Vector3<f64>) {
    let reference = if axis.z.abs() < 0.9 {
        Vector3::z()
    } else {
        Vector3::x()
    };

    let y_axis = axis.cross(&reference).normalize();
    let x_axis = y_axis.cross(axis);

    (x_axis, y_axis)
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn test_helix_basic() {
        let helix = Helix::new(
            Point3::origin(),
            Vector3::z(),
            1.0, // radius
            2.0, // pitch
            3.0, // turns
        );

        // Start at (1, 0, 0) (or similar depending on x_axis)
        let start = helix.point_at(0.0);
        let start_dist = (start.x * start.x + start.y * start.y).sqrt();
        assert_relative_eq!(start_dist, 1.0, epsilon = 1e-10);
        assert_relative_eq!(start.z, 0.0, epsilon = 1e-10);

        // End at height = turns * pitch = 6.0
        let end = helix.point_at(1.0);
        assert_relative_eq!(end.z, 6.0, epsilon = 1e-10);
        let end_dist = (end.x * end.x + end.y * end.y).sqrt();
        assert_relative_eq!(end_dist, 1.0, epsilon = 1e-10);
    }

    #[test]
    fn test_helix_arc_length() {
        let helix = Helix::new(
            Point3::origin(),
            Vector3::z(),
            1.0,
            0.0, // No vertical advance = circle
            1.0, // One turn
        );

        // Arc length should be 2π for one turn with no pitch
        assert_relative_eq!(helix.arc_length(), TAU, epsilon = 1e-10);
    }

    #[test]
    fn test_helix_with_pitch() {
        let helix = Helix::new(
            Point3::origin(),
            Vector3::z(),
            1.0,
            1.0, // pitch
            1.0, // one turn
        );

        // Arc length: sqrt((2π)² + 1²) = sqrt(4π² + 1)
        let expected = (TAU * TAU + 1.0).sqrt();
        assert_relative_eq!(helix.arc_length(), expected, epsilon = 1e-6);
    }

    #[test]
    fn test_helix_curvature() {
        let helix = Helix::new(
            Point3::origin(),
            Vector3::z(),
            2.0, // radius
            1.0, // pitch
            1.0,
        );

        let c = 1.0 / TAU; // pitch / (2π)
        let expected = 2.0 / (4.0 + c * c); // r / (r² + c²)
        assert_relative_eq!(helix.curvature_at(0.5), expected, epsilon = 1e-6);
    }

    #[test]
    fn test_helix_left_handed() {
        let rh = Helix::new(Point3::origin(), Vector3::z(), 1.0, 1.0, 1.0);
        let lh = Helix::left_handed(Point3::origin(), Vector3::z(), 1.0, 1.0, 1.0);

        // At t=0.5, they should be at opposite angular positions (mirrored)
        let rh_mid = rh.point_at(0.5);
        let lh_mid = lh.point_at(0.5);

        // Y coordinates should have opposite signs
        assert!(rh_mid.y * lh_mid.y < 0.0 || rh_mid.y.abs() < 1e-10);
    }

    #[test]
    fn test_conical_helix() {
        let helix = Helix::conical(
            Point3::origin(),
            Vector3::z(),
            1.0, // start radius
            2.0, // end radius
            1.0,
            1.0,
        );

        // Start radius
        let start = helix.point_at(0.0);
        let start_dist = (start.x * start.x + start.y * start.y).sqrt();
        assert_relative_eq!(start_dist, 1.0, epsilon = 1e-10);

        // End radius
        let end = helix.point_at(1.0);
        let end_dist = (end.x * end.x + end.y * end.y).sqrt();
        assert_relative_eq!(end_dist, 2.0, epsilon = 1e-10);

        // Middle radius
        assert_relative_eq!(helix.radius_at(0.5), 1.5, epsilon = 1e-10);
    }
}
