//! Ray and ray-hit result types.
//!
//! [`Ray`] replaces `cf_spatial::Ray` and sim-core's internal ray representation.
//! [`RayHit`] replaces `sim_core::RaycastHit`.
//!
//! Note: `cf_spatial::RaycastHit` (which includes `VoxelCoord`) is a different
//! type serving a different purpose — it stays in cf-spatial.

use nalgebra::{Point3, Vector3};

/// A ray defined by origin and unit direction.
///
/// Direction should be unit-length. Use [`Ray::new`] when you already have a
/// unit vector, or [`Ray::new_normalize`] to normalize an arbitrary direction.
#[derive(Debug, Clone, Copy, PartialEq)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct Ray {
    /// Origin of the ray.
    pub origin: Point3<f64>,
    /// Direction of the ray (should be unit-length).
    pub direction: Vector3<f64>,
}

impl Ray {
    /// Creates a ray from origin and direction.
    ///
    /// Caller is responsible for ensuring direction is unit-length. Use
    /// [`new_normalize`](Self::new_normalize) if the direction may not be normalized.
    #[must_use]
    pub fn new(origin: Point3<f64>, direction: Vector3<f64>) -> Self {
        debug_assert!(
            (direction.norm_squared() - 1.0).abs() < 1e-6,
            "Ray direction must be unit-length (got norm² = {})",
            direction.norm_squared()
        );
        Self { origin, direction }
    }

    /// Creates a ray, normalizing the direction vector.
    ///
    /// Returns `None` if the direction is zero (or too small to normalize).
    #[must_use]
    pub fn new_normalize(origin: Point3<f64>, direction: Vector3<f64>) -> Option<Self> {
        let norm = direction.norm();
        if norm < f64::EPSILON {
            return None;
        }
        Some(Self {
            origin,
            direction: direction / norm,
        })
    }

    /// Returns the point along the ray at parameter `t`.
    ///
    /// `ray.point_at(0.0)` is the origin. Negative `t` goes behind the origin.
    #[must_use]
    pub fn point_at(&self, t: f64) -> Point3<f64> {
        self.origin + self.direction * t
    }
}

/// Result of a ray intersection query against a geometric shape.
///
/// Replaces `sim_core::RaycastHit`. Contains the distance from the ray origin,
/// the hit point in world coordinates, and the surface normal at the hit point.
#[derive(Debug, Clone, Copy)]
#[cfg_attr(feature = "serde", derive(serde::Serialize, serde::Deserialize))]
pub struct RayHit {
    /// Distance from the ray origin to the hit point.
    pub distance: f64,
    /// Hit point in world coordinates.
    pub point: Point3<f64>,
    /// Outward surface normal at the hit point.
    pub normal: Vector3<f64>,
}

impl RayHit {
    /// Creates a new ray hit result.
    #[must_use]
    pub const fn new(distance: f64, point: Point3<f64>, normal: Vector3<f64>) -> Self {
        Self {
            distance,
            point,
            normal,
        }
    }
}
