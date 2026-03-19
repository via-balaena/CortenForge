//! Sphere defined by center and radius.
//!
//! Replaces `cf_spatial::Sphere`.

use nalgebra::{Point3, Vector3};

use crate::Aabb;
use crate::bounded::Bounded;

/// A sphere defined by center and radius.
///
/// Radius is clamped to be non-negative on construction.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Sphere {
    /// Center of the sphere.
    pub center: Point3<f64>,
    /// Radius (always >= 0).
    pub radius: f64,
}

impl Sphere {
    /// Creates a new sphere. Radius is clamped to 0 if negative.
    #[must_use]
    pub const fn new(center: Point3<f64>, radius: f64) -> Self {
        // const-compatible: can't use f64::max() in const context on all targets
        let r = if radius > 0.0 { radius } else { 0.0 };
        Self { center, radius: r }
    }

    /// Returns `true` if the point is inside or on the surface.
    #[must_use]
    pub fn contains(&self, point: &Point3<f64>) -> bool {
        (point - self.center).norm_squared() <= self.radius * self.radius
    }

    /// Returns the signed distance from the point to the surface.
    ///
    /// Negative inside, zero on surface, positive outside.
    #[must_use]
    pub fn signed_distance(&self, point: &Point3<f64>) -> f64 {
        (point - self.center).norm() - self.radius
    }

    /// Returns the diameter.
    #[must_use]
    pub fn diameter(&self) -> f64 {
        self.radius * 2.0
    }

    /// Returns the volume.
    #[must_use]
    pub fn volume(&self) -> f64 {
        (4.0 / 3.0) * std::f64::consts::PI * self.radius * self.radius * self.radius
    }

    /// Returns the surface area.
    #[must_use]
    pub fn surface_area(&self) -> f64 {
        4.0 * std::f64::consts::PI * self.radius * self.radius
    }

    /// Returns `true` if this sphere intersects another sphere.
    #[must_use]
    pub fn intersects(&self, other: &Self) -> bool {
        let dist_sq = (other.center - self.center).norm_squared();
        let radius_sum = self.radius + other.radius;
        dist_sq <= radius_sum * radius_sum
    }

    /// Returns `true` if this sphere intersects an AABB.
    ///
    /// Uses the closest point on the AABB to the sphere center.
    #[must_use]
    pub fn intersects_aabb(&self, aabb: &Aabb) -> bool {
        let closest = Point3::new(
            self.center.x.clamp(aabb.min.x, aabb.max.x),
            self.center.y.clamp(aabb.min.y, aabb.max.y),
            self.center.z.clamp(aabb.min.z, aabb.max.z),
        );
        self.contains(&closest)
    }
}

impl Default for Sphere {
    /// Unit sphere at the origin.
    fn default() -> Self {
        Self {
            center: Point3::origin(),
            radius: 1.0,
        }
    }
}

impl Bounded for Sphere {
    fn aabb(&self) -> Aabb {
        let r = Vector3::new(self.radius, self.radius, self.radius);
        Aabb::new(self.center - r, self.center + r)
    }
}
