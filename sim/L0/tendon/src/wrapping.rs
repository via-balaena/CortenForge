//! Wrapping geometry for tendons around obstacles.
//!
//! Tendons often wrap around bones or other obstacles in their path.
//! This module provides geometric primitives for computing wrap paths.
//!
//! # Wrapping Types
//!
//! - **Sphere wrapping**: Tendon wraps over a spherical surface (e.g., joint)
//! - **Cylinder wrapping**: Tendon wraps around a cylindrical surface (e.g., pulley)
//!
//! # Algorithm
//!
//! For each wrap geometry, we compute:
//! 1. The tangent points where the tendon contacts the surface
//! 2. The geodesic (shortest path) along the surface
//! 3. The total path length including the wrap
//!
//! # MuJoCo Compatibility
//!
//! MuJoCo supports sphere and cylinder wrapping surfaces. This implementation
//! follows the same geometric approach.

use nalgebra::{Isometry3, Point3, Vector3};

#[cfg(feature = "serde")]
use serde::{Deserialize, Serialize};

/// Result of computing a wrap path.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub enum WrapResult {
    /// No wrapping - straight path is shorter.
    NoWrap,

    /// Tendon wraps around the obstacle.
    Wrapped {
        /// First tangent point (where tendon contacts surface).
        tangent_point_1: Point3<f64>,

        /// Second tangent point (where tendon leaves surface).
        tangent_point_2: Point3<f64>,

        /// Length along the surface (arc length).
        arc_length: f64,

        /// Normal at center of wrap.
        wrap_normal: Vector3<f64>,
    },
}

impl WrapResult {
    /// Check if wrapping occurred.
    #[must_use]
    pub fn is_wrapped(&self) -> bool {
        matches!(self, Self::Wrapped { .. })
    }

    /// Get the total additional length from wrapping.
    ///
    /// This is the arc length minus the chord length.
    #[must_use]
    pub fn additional_length(&self) -> f64 {
        match self {
            Self::NoWrap => 0.0,
            Self::Wrapped {
                tangent_point_1,
                tangent_point_2,
                arc_length,
                ..
            } => {
                let chord = (tangent_point_2 - tangent_point_1).norm();
                arc_length - chord
            }
        }
    }
}

/// Sphere wrapping surface.
///
/// Tendons wrap over spheres at joints (e.g., shoulder, hip).
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct SphereWrap {
    /// Center of the sphere in local coordinates.
    pub center: Point3<f64>,

    /// Radius of the sphere.
    pub radius: f64,

    /// Which side to wrap on (positive or negative relative to center).
    ///
    /// Some wrapping surfaces only allow wrapping on one side.
    pub side: WrapSide,
}

/// Which side of the wrapping surface to use.
#[derive(Debug, Clone, Copy, PartialEq, Eq, Default)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub enum WrapSide {
    /// Wrap on either side (closest).
    #[default]
    Both,

    /// Only wrap on positive side (relative to some axis).
    Positive,

    /// Only wrap on negative side.
    Negative,
}

impl SphereWrap {
    /// Create a new sphere wrap.
    #[must_use]
    pub fn new(center: Point3<f64>, radius: f64) -> Self {
        Self {
            center,
            radius: radius.abs(),
            side: WrapSide::Both,
        }
    }

    /// Create a sphere wrap at the origin.
    #[must_use]
    pub fn at_origin(radius: f64) -> Self {
        Self::new(Point3::origin(), radius)
    }

    /// Set the wrapping side.
    #[must_use]
    pub fn with_side(mut self, side: WrapSide) -> Self {
        self.side = side;
        self
    }

    /// Compute the wrap path from point A to point B.
    ///
    /// # Arguments
    ///
    /// * `p1` - Start point
    /// * `p2` - End point
    ///
    /// # Returns
    ///
    /// The wrap result containing tangent points and arc length.
    #[must_use]
    pub fn compute_wrap(&self, p1: &Point3<f64>, p2: &Point3<f64>) -> WrapResult {
        // Vector from center to each point
        let v1 = p1 - self.center;
        let v2 = p2 - self.center;

        let d1 = v1.norm();
        let d2 = v2.norm();

        // If either point is inside the sphere, no valid wrap
        if d1 < self.radius || d2 < self.radius {
            return WrapResult::NoWrap;
        }

        // Check if straight line misses the sphere
        let line = p2 - p1;
        let line_length = line.norm();

        if line_length < 1e-10 {
            return WrapResult::NoWrap;
        }

        let line_dir = line / line_length;

        // Project center onto line
        let to_center = self.center - p1;
        let proj_dist = to_center.dot(&line_dir);

        // Closest point on line to sphere center
        let closest = p1 + line_dir * proj_dist.clamp(0.0, line_length);
        let dist_to_line = (self.center - closest).norm();

        // If line doesn't intersect sphere, no wrap needed
        if dist_to_line > self.radius * 1.01 {
            // 1% margin for numerical stability
            return WrapResult::NoWrap;
        }

        // Compute tangent points using the tangent line formula
        // For a point P outside a sphere with center C and radius r,
        // the tangent point T satisfies: |CT| = r and CT ⊥ PT

        // Tangent point 1 (from p1)
        let t1 = self.compute_tangent_point(p1);

        // Tangent point 2 (from p2)
        let t2 = self.compute_tangent_point(p2);

        // Compute arc length on the sphere
        let arc_angle = self.angle_between_tangents(&t1, &t2);
        let arc_length = self.radius * arc_angle;

        // Compute wrap normal (average of normals at tangent points)
        let n1 = (t1 - self.center).normalize();
        let n2 = (t2 - self.center).normalize();
        let wrap_normal = (n1 + n2).normalize();

        WrapResult::Wrapped {
            tangent_point_1: t1,
            tangent_point_2: t2,
            arc_length,
            wrap_normal,
        }
    }

    /// Compute tangent point from an external point.
    fn compute_tangent_point(&self, external: &Point3<f64>) -> Point3<f64> {
        let v = external - self.center;
        let d = v.norm();

        if d <= self.radius {
            // Point inside sphere, return closest surface point
            return self.center + v.normalize() * self.radius;
        }

        // Distance from external point to tangent point along the line to center
        // Using right triangle: d² = r² + t²  =>  t = sqrt(d² - r²)
        // The tangent point is at distance sqrt(d² - r²) from the external point
        // along a direction perpendicular to the radius at the tangent point

        // Simpler: tangent point is at angle θ from line to center where cos(θ) = r/d
        let cos_theta = self.radius / d;
        let sin_theta = (1.0 - cos_theta * cos_theta).sqrt();

        // Distance from center to tangent point along v direction
        let along_v = self.radius * cos_theta;

        // Find a perpendicular direction
        let v_norm = v.normalize();
        let perp = find_perpendicular(&v_norm);

        // Tangent point
        self.center + v_norm * along_v + perp * (self.radius * sin_theta)
    }

    /// Compute angle between two tangent points on the sphere.
    fn angle_between_tangents(&self, t1: &Point3<f64>, t2: &Point3<f64>) -> f64 {
        let v1 = (t1 - self.center).normalize();
        let v2 = (t2 - self.center).normalize();

        let cos_angle = v1.dot(&v2).clamp(-1.0, 1.0);
        cos_angle.acos()
    }
}

/// Cylinder wrapping surface.
///
/// Tendons wrap around cylinders for pulleys and cable guides.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub struct CylinderWrap {
    /// Center of the cylinder (on the axis).
    pub center: Point3<f64>,

    /// Axis direction (normalized).
    pub axis: Vector3<f64>,

    /// Radius of the cylinder.
    pub radius: f64,

    /// Half-height of the cylinder (length = 2 * half_height).
    pub half_height: f64,

    /// Which side to wrap on.
    pub side: WrapSide,
}

impl CylinderWrap {
    /// Create a new cylinder wrap.
    #[must_use]
    pub fn new(center: Point3<f64>, axis: Vector3<f64>, radius: f64, half_height: f64) -> Self {
        Self {
            center,
            axis: axis.normalize(),
            radius: radius.abs(),
            half_height: half_height.abs(),
            side: WrapSide::Both,
        }
    }

    /// Create a cylinder wrap aligned with Z axis at origin.
    #[must_use]
    pub fn z_aligned(radius: f64, half_height: f64) -> Self {
        Self::new(Point3::origin(), Vector3::z(), radius, half_height)
    }

    /// Set the wrapping side.
    #[must_use]
    pub fn with_side(mut self, side: WrapSide) -> Self {
        self.side = side;
        self
    }

    /// Compute the wrap path from point A to point B.
    #[must_use]
    pub fn compute_wrap(&self, p1: &Point3<f64>, p2: &Point3<f64>) -> WrapResult {
        // Project points onto plane perpendicular to axis
        let v1 = p1 - self.center;
        let v2 = p2 - self.center;

        let v1_axial = self.axis * v1.dot(&self.axis);
        let v1_radial = v1 - v1_axial;

        let v2_axial = self.axis * v2.dot(&self.axis);
        let v2_radial = v2 - v2_axial;

        let d1 = v1_radial.norm();
        let d2 = v2_radial.norm();

        // If either point is inside the cylinder, no valid wrap
        if d1 < self.radius || d2 < self.radius {
            return WrapResult::NoWrap;
        }

        // Check if straight line misses the cylinder
        // (2D problem in the plane perpendicular to axis)
        let line_2d = v2_radial - v1_radial;
        let line_len_2d = line_2d.norm();

        if line_len_2d < 1e-10 {
            return WrapResult::NoWrap;
        }

        let line_dir_2d = line_2d / line_len_2d;

        // Perpendicular distance from cylinder axis to the line (2D calculation)
        // The perpendicular in 2D (XY plane since we removed axial component)
        let perp_2d = Vector3::new(-line_dir_2d.y, line_dir_2d.x, 0.0);
        let dist_to_line = v1_radial.dot(&perp_2d).abs();

        if dist_to_line > self.radius * 1.01 {
            return WrapResult::NoWrap;
        }

        // Compute tangent points (in the radial plane)
        let t1_radial = self.compute_tangent_2d(&v1_radial);
        let t2_radial = self.compute_tangent_2d(&v2_radial);

        // Compute the axial positions for tangent points
        // Interpolate based on radial progress
        let angle1 = v1_radial.y.atan2(v1_radial.x);
        let angle2 = v2_radial.y.atan2(v2_radial.x);
        let angle_t1 = t1_radial.y.atan2(t1_radial.x);
        let angle_t2 = t2_radial.y.atan2(t2_radial.x);

        let total_angle = angle_span(angle1, angle2);
        let frac_t1 = if total_angle.abs() > 1e-10 {
            angle_span(angle1, angle_t1) / total_angle
        } else {
            0.5
        };
        let frac_t2 = if total_angle.abs() > 1e-10 {
            angle_span(angle1, angle_t2) / total_angle
        } else {
            0.5
        };

        let z1 = v1_axial.dot(&self.axis);
        let z2 = v2_axial.dot(&self.axis);
        let z_t1 = z1 + frac_t1 * (z2 - z1);
        let z_t2 = z1 + frac_t2 * (z2 - z1);

        // Full 3D tangent points
        let t1 = self.center + t1_radial + self.axis * z_t1;
        let t2 = self.center + t2_radial + self.axis * z_t2;

        // Compute arc length (in radial plane)
        let wrap_angle = angle_span(angle_t1, angle_t2).abs();
        let radial_arc = self.radius * wrap_angle;

        // Add helical component
        let axial_dist = (z_t2 - z_t1).abs();
        let arc_length = (radial_arc * radial_arc + axial_dist * axial_dist).sqrt();

        // Wrap normal (radial direction at center of wrap)
        let mid_angle = angle_t1 + wrap_angle / 2.0;
        let wrap_normal = Vector3::new(mid_angle.cos(), mid_angle.sin(), 0.0);

        WrapResult::Wrapped {
            tangent_point_1: t1,
            tangent_point_2: t2,
            arc_length,
            wrap_normal,
        }
    }

    /// Compute tangent point in 2D (radial plane).
    fn compute_tangent_2d(&self, radial: &Vector3<f64>) -> Vector3<f64> {
        let d = radial.norm();

        if d <= self.radius {
            return radial.normalize() * self.radius;
        }

        let cos_theta = self.radius / d;
        let sin_theta = (1.0 - cos_theta * cos_theta).sqrt();

        let radial_norm = radial.normalize();
        let perp = Vector3::new(-radial_norm.y, radial_norm.x, 0.0);

        radial_norm * (self.radius * cos_theta) + perp * (self.radius * sin_theta)
    }
}

/// General wrapping geometry enum.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(Serialize, Deserialize))]
pub enum WrappingGeometry {
    /// Sphere wrapping surface.
    Sphere(SphereWrap),

    /// Cylinder wrapping surface.
    Cylinder(CylinderWrap),
}

impl WrappingGeometry {
    /// Create a sphere wrapping geometry.
    #[must_use]
    pub fn sphere(radius: f64) -> Self {
        Self::Sphere(SphereWrap::at_origin(radius))
    }

    /// Create a sphere wrapping geometry with center.
    #[must_use]
    pub fn sphere_at(center: Point3<f64>, radius: f64) -> Self {
        Self::Sphere(SphereWrap::new(center, radius))
    }

    /// Create a cylinder wrapping geometry.
    #[must_use]
    pub fn cylinder(radius: f64, half_height: f64) -> Self {
        Self::Cylinder(CylinderWrap::z_aligned(radius, half_height))
    }

    /// Create a cylinder wrapping geometry with position and axis.
    #[must_use]
    pub fn cylinder_at(
        center: Point3<f64>,
        axis: Vector3<f64>,
        radius: f64,
        half_height: f64,
    ) -> Self {
        Self::Cylinder(CylinderWrap::new(center, axis, radius, half_height))
    }

    /// Compute the wrap path.
    #[must_use]
    pub fn compute_wrap(&self, p1: &Point3<f64>, p2: &Point3<f64>) -> WrapResult {
        match self {
            Self::Sphere(s) => s.compute_wrap(p1, p2),
            Self::Cylinder(c) => c.compute_wrap(p1, p2),
        }
    }

    /// Transform the geometry by an isometry.
    #[must_use]
    pub fn transform(&self, transform: &Isometry3<f64>) -> Self {
        match self {
            Self::Sphere(s) => Self::Sphere(SphereWrap {
                center: transform * s.center,
                radius: s.radius,
                side: s.side,
            }),
            Self::Cylinder(c) => Self::Cylinder(CylinderWrap {
                center: transform * c.center,
                axis: transform.rotation * c.axis,
                radius: c.radius,
                half_height: c.half_height,
                side: c.side,
            }),
        }
    }

    /// Get the radius of the wrapping surface.
    #[must_use]
    pub fn radius(&self) -> f64 {
        match self {
            Self::Sphere(s) => s.radius,
            Self::Cylinder(c) => c.radius,
        }
    }
}

/// Find a vector perpendicular to the given vector.
fn find_perpendicular(v: &Vector3<f64>) -> Vector3<f64> {
    // Find the component with smallest magnitude and use that axis
    let abs_x = v.x.abs();
    let abs_y = v.y.abs();
    let abs_z = v.z.abs();

    let helper = if abs_x <= abs_y && abs_x <= abs_z {
        Vector3::x()
    } else if abs_y <= abs_z {
        Vector3::y()
    } else {
        Vector3::z()
    };

    v.cross(&helper).normalize()
}

/// Compute the signed angle span between two angles.
fn angle_span(a1: f64, a2: f64) -> f64 {
    let mut diff = a2 - a1;

    // Normalize to [-π, π]
    while diff > std::f64::consts::PI {
        diff -= 2.0 * std::f64::consts::PI;
    }
    while diff < -std::f64::consts::PI {
        diff += 2.0 * std::f64::consts::PI;
    }

    diff
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn test_sphere_wrap_no_intersection() {
        let wrap = SphereWrap::new(Point3::origin(), 0.1);

        // Points far from sphere - no wrap needed
        let result = wrap.compute_wrap(&Point3::new(-1.0, 1.0, 0.0), &Point3::new(1.0, 1.0, 0.0));

        assert!(!result.is_wrapped());
    }

    #[test]
    fn test_sphere_wrap_intersection() {
        let wrap = SphereWrap::new(Point3::origin(), 0.1);

        // Points that would intersect sphere if connected directly
        let result = wrap.compute_wrap(&Point3::new(-0.5, 0.0, 0.0), &Point3::new(0.5, 0.0, 0.0));

        assert!(result.is_wrapped());

        if let WrapResult::Wrapped { arc_length, .. } = result {
            // Arc should be approximately half circle = π * r
            let expected = std::f64::consts::PI * 0.1;
            assert_relative_eq!(arc_length, expected, epsilon = 0.02);
        }
    }

    #[test]
    fn test_cylinder_wrap_no_intersection() {
        let wrap = CylinderWrap::z_aligned(0.1, 1.0);

        // Points far from cylinder
        let result = wrap.compute_wrap(&Point3::new(-1.0, 1.0, 0.0), &Point3::new(1.0, 1.0, 0.0));

        assert!(!result.is_wrapped());
    }

    #[test]
    fn test_cylinder_wrap_intersection() {
        let wrap = CylinderWrap::z_aligned(0.1, 1.0);

        // Points that would intersect cylinder
        let result = wrap.compute_wrap(&Point3::new(-0.5, 0.0, 0.0), &Point3::new(0.5, 0.0, 0.0));

        assert!(result.is_wrapped());
    }

    #[test]
    fn test_wrapping_geometry_enum() {
        let sphere = WrappingGeometry::sphere(0.1);
        let cylinder = WrappingGeometry::cylinder(0.1, 1.0);

        assert_relative_eq!(sphere.radius(), 0.1, epsilon = 1e-10);
        assert_relative_eq!(cylinder.radius(), 0.1, epsilon = 1e-10);
    }

    #[test]
    fn test_geometry_transform() {
        let sphere = WrappingGeometry::sphere_at(Point3::origin(), 0.1);

        let transform = Isometry3::translation(1.0, 0.0, 0.0);
        let transformed = sphere.transform(&transform);

        if let WrappingGeometry::Sphere(s) = transformed {
            assert_relative_eq!(s.center.x, 1.0, epsilon = 1e-10);
        }
    }

    #[test]
    fn test_additional_length() {
        let result = WrapResult::Wrapped {
            tangent_point_1: Point3::new(-0.1, 0.0, 0.0),
            tangent_point_2: Point3::new(0.1, 0.0, 0.0),
            arc_length: 0.5,
            wrap_normal: Vector3::y(),
        };

        let additional = result.additional_length();
        // Arc = 0.5, chord = 0.2, additional = 0.3
        assert_relative_eq!(additional, 0.3, epsilon = 1e-10);
    }

    #[test]
    fn test_find_perpendicular() {
        let v = Vector3::new(1.0, 0.0, 0.0);
        let perp = find_perpendicular(&v);

        assert_relative_eq!(v.dot(&perp), 0.0, epsilon = 1e-10);
        assert_relative_eq!(perp.norm(), 1.0, epsilon = 1e-10);
    }

    #[test]
    fn test_angle_span() {
        assert_relative_eq!(angle_span(0.0, 1.0), 1.0, epsilon = 1e-10);
        assert_relative_eq!(angle_span(1.0, 0.0), -1.0, epsilon = 1e-10);

        // Wrap around
        let span = angle_span(3.0, -3.0);
        assert!(span.abs() < std::f64::consts::PI);
    }
}
