//! Plane representation for surface fitting.

use nalgebra::Vector3;

/// A plane in 3D space defined by a point and normal.
///
/// The plane equation is: `normal · (p - point) = 0`
/// or equivalently: `normal · p = d` where `d = normal · point`
#[derive(Debug, Clone, Copy)]
pub struct Plane {
    /// A point on the plane.
    pub point: Vector3<f64>,
    /// The plane normal (unit vector).
    pub normal: Vector3<f64>,
}

impl Plane {
    /// Create a new plane from a point and normal.
    ///
    /// The normal is automatically normalized.
    ///
    /// # Arguments
    ///
    /// * `point` - A point on the plane
    /// * `normal` - The plane normal (will be normalized)
    ///
    /// # Returns
    ///
    /// `Some(Plane)` if the normal is non-zero, `None` otherwise.
    #[must_use]
    pub fn new(point: Vector3<f64>, normal: Vector3<f64>) -> Option<Self> {
        let norm = normal.norm();
        if norm < f64::EPSILON {
            return None;
        }
        Some(Self {
            point,
            normal: normal / norm,
        })
    }

    /// Create a plane from three non-collinear points.
    ///
    /// The normal is computed as `(p1 - p0) × (p2 - p0)`, normalized.
    ///
    /// # Arguments
    ///
    /// * `p0`, `p1`, `p2` - Three points defining the plane
    ///
    /// # Returns
    ///
    /// `Some(Plane)` if the points are non-collinear, `None` otherwise.
    #[must_use]
    pub fn from_points(p0: Vector3<f64>, p1: Vector3<f64>, p2: Vector3<f64>) -> Option<Self> {
        let v1 = p1 - p0;
        let v2 = p2 - p0;
        let normal = v1.cross(&v2);
        Self::new(p0, normal)
    }

    /// Compute the signed distance from a point to the plane.
    ///
    /// Positive distance means the point is on the side the normal points to.
    /// Negative distance means the point is on the opposite side.
    #[must_use]
    pub fn signed_distance(&self, point: Vector3<f64>) -> f64 {
        self.normal.dot(&(point - self.point))
    }

    /// Compute the absolute distance from a point to the plane.
    #[must_use]
    pub fn distance(&self, point: Vector3<f64>) -> f64 {
        self.signed_distance(point).abs()
    }

    /// Project a point onto the plane.
    #[must_use]
    pub fn project(&self, point: Vector3<f64>) -> Vector3<f64> {
        point - self.signed_distance(point) * self.normal
    }

    /// Check if a point is within a given distance threshold of the plane.
    #[must_use]
    pub fn is_inlier(&self, point: Vector3<f64>, threshold: f64) -> bool {
        self.distance(point) <= threshold
    }

    /// Get the plane constant `d` where the plane equation is `n · p = d`.
    #[must_use]
    pub fn d(&self) -> f64 {
        self.normal.dot(&self.point)
    }
}

/// Create a default plane (Z=0) for testing fallback scenarios.
#[cfg(test)]
fn default_plane() -> Plane {
    Plane {
        point: Vector3::zeros(),
        normal: Vector3::z(),
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn plane_from_point_normal() {
        let plane = Plane::new(
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(0.0, 0.0, 2.0), // Not normalized
        );

        assert!(plane.is_some());
        let plane = plane.unwrap_or_else(default_plane);
        assert_relative_eq!(plane.normal.norm(), 1.0, epsilon = 1e-10);
        assert_relative_eq!(plane.normal.z, 1.0, epsilon = 1e-10);
    }

    #[test]
    fn plane_from_three_points() {
        let p0 = Vector3::new(0.0, 0.0, 0.0);
        let p1 = Vector3::new(1.0, 0.0, 0.0);
        let p2 = Vector3::new(0.0, 1.0, 0.0);

        let plane = Plane::from_points(p0, p1, p2);
        assert!(plane.is_some());
        let plane = plane.unwrap_or_else(default_plane);

        // Normal should be ±Z
        assert_relative_eq!(plane.normal.x, 0.0, epsilon = 1e-10);
        assert_relative_eq!(plane.normal.y, 0.0, epsilon = 1e-10);
        assert_relative_eq!(plane.normal.z.abs(), 1.0, epsilon = 1e-10);
    }

    #[test]
    fn collinear_points_no_plane() {
        let p0 = Vector3::new(0.0, 0.0, 0.0);
        let p1 = Vector3::new(1.0, 0.0, 0.0);
        let p2 = Vector3::new(2.0, 0.0, 0.0);

        let plane = Plane::from_points(p0, p1, p2);
        assert!(plane.is_none());
    }

    #[test]
    fn signed_distance() {
        let plane = Plane::new(Vector3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0));
        let plane = plane.unwrap_or_else(default_plane);

        // Point above plane
        let above = Vector3::new(0.0, 0.0, 5.0);
        assert_relative_eq!(plane.signed_distance(above), 5.0, epsilon = 1e-10);

        // Point below plane
        let below = Vector3::new(0.0, 0.0, -3.0);
        assert_relative_eq!(plane.signed_distance(below), -3.0, epsilon = 1e-10);

        // Point on plane
        let on = Vector3::new(10.0, 20.0, 0.0);
        assert_relative_eq!(plane.signed_distance(on), 0.0, epsilon = 1e-10);
    }

    #[test]
    fn project_point() {
        let plane = Plane::new(Vector3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0));
        let plane = plane.unwrap_or_else(default_plane);

        let point = Vector3::new(3.0, 4.0, 7.0);
        let projected = plane.project(point);

        assert_relative_eq!(projected.x, 3.0, epsilon = 1e-10);
        assert_relative_eq!(projected.y, 4.0, epsilon = 1e-10);
        assert_relative_eq!(projected.z, 0.0, epsilon = 1e-10);
    }

    #[test]
    fn is_inlier() {
        let plane = Plane::new(Vector3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0));
        let plane = plane.unwrap_or_else(default_plane);

        let close = Vector3::new(0.0, 0.0, 0.05);
        let far = Vector3::new(0.0, 0.0, 0.2);

        assert!(plane.is_inlier(close, 0.1));
        assert!(!plane.is_inlier(far, 0.1));
    }

    #[test]
    fn zero_normal_no_plane() {
        let plane = Plane::new(Vector3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 0.0));
        assert!(plane.is_none());
    }
}
