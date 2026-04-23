//! `Geometric` trait for shapes that support closest-point, containment, and ray queries.
//!
//! This trait is implemented by the convex `Support*` wrapper types in
//! [`support_map`](crate::support_map). For the [`Shape`](crate::Shape) enum,
//! use the free functions [`closest_point`](crate::closest_point),
//! [`ray_cast`](crate::ray_cast) instead — they dispatch exhaustively on all
//! 10 variants.

use nalgebra::Point3;

use crate::{Bounded, Ray, RayHit, Shape};

/// Types that support closest-point, containment, and ray intersection queries.
///
/// All methods operate in **local space** — the shape is centered at the origin
/// with identity orientation.
///
/// # Implementors
///
/// - [`SupportSphere`](crate::SupportSphere), [`SupportBox`](crate::SupportBox),
///   [`SupportCapsule`](crate::SupportCapsule), [`SupportCylinder`](crate::SupportCylinder),
///   [`SupportEllipsoid`](crate::SupportEllipsoid) — analytic implementations
/// - [`ConvexHull`](crate::ConvexHull) — GJK-based closest point, face-by-face ray test
pub trait Geometric: Bounded {
    /// Closest point on the shape's surface to `point`.
    fn closest_point(&self, point: &Point3<f64>) -> Point3<f64>;

    /// Whether the shape contains `point` (on surface counts as inside).
    fn contains(&self, point: &Point3<f64>) -> bool;

    /// Ray intersection test. Returns the closest hit within `max_distance`.
    fn ray_intersect(&self, ray: &Ray, max_distance: f64) -> Option<RayHit>;
}

// =============================================================================
// Implementations — delegate to existing free functions via Shape construction
// =============================================================================

impl Geometric for crate::SupportSphere {
    fn closest_point(&self, point: &Point3<f64>) -> Point3<f64> {
        crate::closest_point(&Shape::sphere(self.radius), point)
    }

    fn contains(&self, point: &Point3<f64>) -> bool {
        point.coords.norm_squared() <= self.radius * self.radius
    }

    fn ray_intersect(&self, ray: &Ray, max_distance: f64) -> Option<RayHit> {
        crate::ray_cast(ray, &Shape::sphere(self.radius), max_distance)
    }
}

impl Geometric for crate::SupportBox {
    fn closest_point(&self, point: &Point3<f64>) -> Point3<f64> {
        crate::closest_point(&Shape::box_shape(self.half_extents), point)
    }

    fn contains(&self, point: &Point3<f64>) -> bool {
        point.x.abs() <= self.half_extents.x
            && point.y.abs() <= self.half_extents.y
            && point.z.abs() <= self.half_extents.z
    }

    fn ray_intersect(&self, ray: &Ray, max_distance: f64) -> Option<RayHit> {
        crate::ray_cast(ray, &Shape::box_shape(self.half_extents), max_distance)
    }
}

impl Geometric for crate::SupportCapsule {
    fn closest_point(&self, point: &Point3<f64>) -> Point3<f64> {
        crate::closest_point(&Shape::capsule(self.half_length, self.radius), point)
    }

    // Floating-point form chosen for numerical stability, not micro-optimization.
    #[allow(clippy::suboptimal_flops)]
    fn contains(&self, point: &Point3<f64>) -> bool {
        // Closest point on axis segment, then check distance to that
        let axis_z = point.z.clamp(-self.half_length, self.half_length);
        let dx = point.x;
        let dy = point.y;
        let dz = point.z - axis_z;
        dx * dx + dy * dy + dz * dz <= self.radius * self.radius
    }

    fn ray_intersect(&self, ray: &Ray, max_distance: f64) -> Option<RayHit> {
        crate::ray_cast(
            ray,
            &Shape::capsule(self.half_length, self.radius),
            max_distance,
        )
    }
}

impl Geometric for crate::SupportCylinder {
    fn closest_point(&self, point: &Point3<f64>) -> Point3<f64> {
        crate::closest_point(&Shape::cylinder(self.half_length, self.radius), point)
    }

    // Floating-point form chosen for numerical stability, not micro-optimization.
    #[allow(clippy::suboptimal_flops)]
    fn contains(&self, point: &Point3<f64>) -> bool {
        point.z.abs() <= self.half_length
            && (point.x * point.x + point.y * point.y) <= self.radius * self.radius
    }

    fn ray_intersect(&self, ray: &Ray, max_distance: f64) -> Option<RayHit> {
        crate::ray_cast(
            ray,
            &Shape::cylinder(self.half_length, self.radius),
            max_distance,
        )
    }
}

impl Geometric for crate::SupportEllipsoid {
    fn closest_point(&self, point: &Point3<f64>) -> Point3<f64> {
        crate::closest_point(&Shape::ellipsoid(self.radii), point)
    }

    // Floating-point form chosen for numerical stability, not micro-optimization.
    #[allow(clippy::suboptimal_flops)]
    fn contains(&self, point: &Point3<f64>) -> bool {
        let rx = self.radii.x.max(1e-10);
        let ry = self.radii.y.max(1e-10);
        let rz = self.radii.z.max(1e-10);
        let sx = point.x / rx;
        let sy = point.y / ry;
        let sz = point.z / rz;
        sx * sx + sy * sy + sz * sz <= 1.0
    }

    fn ray_intersect(&self, ray: &Ray, max_distance: f64) -> Option<RayHit> {
        crate::ray_cast(ray, &Shape::ellipsoid(self.radii), max_distance)
    }
}

impl Geometric for crate::ConvexHull {
    fn closest_point(&self, point: &Point3<f64>) -> Point3<f64> {
        crate::closest_point(&Shape::convex_mesh(self.clone()), point)
    }

    fn contains(&self, point: &Point3<f64>) -> bool {
        // Point is inside the convex hull iff it is on the interior side of
        // every face (dot product with outward normal is non-positive).
        if self.faces.is_empty() {
            return false;
        }
        for (face, normal) in self.faces.iter().zip(self.normals.iter()) {
            let v0 = self.vertices[face[0] as usize];
            let to_point = point - v0;
            if to_point.dot(normal) > 1e-10 {
                return false;
            }
        }
        true
    }

    fn ray_intersect(&self, ray: &Ray, max_distance: f64) -> Option<RayHit> {
        crate::ray_cast(ray, &Shape::convex_mesh(self.clone()), max_distance)
    }
}

// =============================================================================
// Tests
// =============================================================================

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::float_cmp)]
mod tests {
    use super::*;
    use crate::support_map::*;
    use approx::assert_relative_eq;
    use nalgebra::Vector3;

    // ── Sphere ───────────────────────────────────────────────────────

    #[test]
    fn sphere_closest_point_outside() {
        let s = SupportSphere { radius: 1.0 };
        let cp = s.closest_point(&Point3::new(3.0, 0.0, 0.0));
        assert_relative_eq!(cp, Point3::new(1.0, 0.0, 0.0), epsilon = 1e-10);
    }

    #[test]
    fn sphere_contains() {
        let s = SupportSphere { radius: 1.0 };
        assert!(s.contains(&Point3::new(0.5, 0.0, 0.0)));
        assert!(s.contains(&Point3::new(1.0, 0.0, 0.0))); // on surface
        assert!(!s.contains(&Point3::new(1.5, 0.0, 0.0)));
    }

    #[test]
    fn sphere_ray_hit() {
        let s = SupportSphere { radius: 1.0 };
        let ray = Ray::new(Point3::new(-5.0, 0.0, 0.0), Vector3::x());
        let hit = s.ray_intersect(&ray, 100.0);
        assert!(hit.is_some());
        assert_relative_eq!(hit.unwrap().distance, 4.0, epsilon = 1e-6);
    }

    #[test]
    fn sphere_ray_miss() {
        let s = SupportSphere { radius: 1.0 };
        let ray = Ray::new(Point3::new(-5.0, 5.0, 0.0), Vector3::x());
        assert!(s.ray_intersect(&ray, 100.0).is_none());
    }

    // ── Box ──────────────────────────────────────────────────────────

    #[test]
    fn box_contains() {
        let b = SupportBox {
            half_extents: Vector3::new(1.0, 2.0, 3.0),
        };
        assert!(b.contains(&Point3::new(0.5, 1.0, 2.0)));
        assert!(b.contains(&Point3::new(1.0, 2.0, 3.0))); // corner
        assert!(!b.contains(&Point3::new(1.5, 0.0, 0.0)));
    }

    #[test]
    fn box_ray_hit() {
        let b = SupportBox {
            half_extents: Vector3::new(1.0, 1.0, 1.0),
        };
        let ray = Ray::new(Point3::new(-5.0, 0.0, 0.0), Vector3::x());
        let hit = b.ray_intersect(&ray, 100.0);
        assert!(hit.is_some());
        assert_relative_eq!(hit.unwrap().distance, 4.0, epsilon = 1e-6);
    }

    // ── Capsule ──────────────────────────────────────────────────────

    #[test]
    fn capsule_contains() {
        let c = SupportCapsule {
            half_length: 2.0,
            radius: 0.5,
        };
        assert!(c.contains(&Point3::origin()));
        assert!(c.contains(&Point3::new(0.0, 0.0, 2.4))); // near top cap
        assert!(!c.contains(&Point3::new(0.0, 0.0, 2.6))); // beyond cap
        assert!(!c.contains(&Point3::new(1.0, 0.0, 0.0))); // outside barrel
    }

    // ── Cylinder ─────────────────────────────────────────────────────

    #[test]
    fn cylinder_contains() {
        let c = SupportCylinder {
            half_length: 1.0,
            radius: 0.5,
        };
        assert!(c.contains(&Point3::origin()));
        assert!(c.contains(&Point3::new(0.4, 0.0, 0.9)));
        assert!(!c.contains(&Point3::new(0.0, 0.0, 1.5))); // beyond cap
        assert!(!c.contains(&Point3::new(1.0, 0.0, 0.0))); // outside barrel
    }

    // ── Ellipsoid ────────────────────────────────────────────────────

    #[test]
    fn ellipsoid_contains() {
        let e = SupportEllipsoid {
            radii: Vector3::new(2.0, 1.0, 0.5),
        };
        assert!(e.contains(&Point3::origin()));
        assert!(e.contains(&Point3::new(1.0, 0.0, 0.0)));
        assert!(!e.contains(&Point3::new(2.5, 0.0, 0.0)));
        assert!(!e.contains(&Point3::new(0.0, 1.5, 0.0)));
    }

    // ── ConvexHull ───────────────────────────────────────────────────

    #[test]
    fn convex_hull_contains() {
        let vertices = vec![
            Point3::new(-1.0, -1.0, -1.0),
            Point3::new(1.0, -1.0, -1.0),
            Point3::new(1.0, 1.0, -1.0),
            Point3::new(-1.0, 1.0, -1.0),
            Point3::new(-1.0, -1.0, 1.0),
            Point3::new(1.0, -1.0, 1.0),
            Point3::new(1.0, 1.0, 1.0),
            Point3::new(-1.0, 1.0, 1.0),
        ];
        let hull = crate::convex_hull(&vertices, None).unwrap();

        assert!(hull.contains(&Point3::origin()));
        assert!(hull.contains(&Point3::new(0.5, 0.5, 0.5)));
        assert!(!hull.contains(&Point3::new(1.5, 0.0, 0.0)));
    }

    #[test]
    fn convex_hull_closest_point() {
        let vertices = vec![
            Point3::new(-1.0, -1.0, -1.0),
            Point3::new(1.0, -1.0, -1.0),
            Point3::new(1.0, 1.0, -1.0),
            Point3::new(-1.0, 1.0, -1.0),
            Point3::new(-1.0, -1.0, 1.0),
            Point3::new(1.0, -1.0, 1.0),
            Point3::new(1.0, 1.0, 1.0),
            Point3::new(-1.0, 1.0, 1.0),
        ];
        let hull = crate::convex_hull(&vertices, None).unwrap();

        let cp = hull.closest_point(&Point3::new(3.0, 0.0, 0.0));
        assert_relative_eq!(cp.x, 1.0, epsilon = 0.1);
    }
}
