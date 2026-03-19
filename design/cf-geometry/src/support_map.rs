//! Support map trait for convex shapes.
//!
//! The support function is the core building block for GJK and EPA algorithms.
//! Given a direction, it returns the point on the shape's surface that is
//! furthest in that direction.
//!
//! Only convex shapes have a natural support function. Non-convex shapes
//! (`HeightField`, `TriangleMesh`, `SdfGrid`) do not implement this trait.
//!
//! All implementations operate in **local space** — the shape is centered at
//! the origin with identity orientation. The consuming layer (e.g. sim-core)
//! handles world-space pose transforms by wrapping `(Shape, Pose)` into a
//! type that implements `SupportMap`.

use nalgebra::{Point3, Vector3};

use crate::Bounded;

/// Numerical tolerance for support function edge cases.
const EPSILON: f64 = 1e-8;

/// Convex shapes that can compute a support point in a given direction.
///
/// The support point is the point on the shape's surface (or boundary) that
/// has the maximum dot product with the given direction vector.
///
/// # Local space
///
/// All implementations assume the shape is centered at the origin with
/// identity orientation. For world-space queries, wrap with a posed adapter
/// in the consuming layer.
///
/// # Implementors
///
/// - `Sphere`, `Box`, `Capsule`, `Cylinder`, `Ellipsoid` — analytic formulas
/// - [`ConvexHull`](crate::ConvexHull) — hill-climbing or exhaustive search on vertex set
pub trait SupportMap: Bounded {
    /// Return the point on the shape's surface furthest in `direction`.
    ///
    /// `direction` need not be unit-length — implementations handle
    /// normalization internally where needed.
    fn support(&self, direction: &Vector3<f64>) -> Point3<f64>;
}

// =============================================================================
// Wrapper types for Shape variants (local-space support)
// =============================================================================
// Rather than implementing SupportMap on Shape directly (which includes
// non-convex variants), we provide standalone support functions that the
// GJK/EPA algorithms can use through the trait.

/// Sphere support: center + normalized(dir) * radius.
#[must_use]
pub fn support_sphere(radius: f64, direction: &Vector3<f64>) -> Point3<f64> {
    let dir_norm = direction.norm();
    if dir_norm < EPSILON {
        Point3::new(radius, 0.0, 0.0)
    } else {
        Point3::from(direction * (radius / dir_norm))
    }
}

/// Box support: `sign(dir_i) * half_extent_i` per axis.
///
/// Uses `>= 0.0` comparison instead of `signum()` to handle IEEE 754 negative
/// zero correctly: `-0.0.signum() == -1.0` but `-0.0 >= 0.0` is true.
/// When GJK negates the direction for shape B, zero components become `-0.0`,
/// and inconsistent sign choices produce incorrect Minkowski difference points.
#[must_use]
pub fn support_box(half_extents: &Vector3<f64>, direction: &Vector3<f64>) -> Point3<f64> {
    Point3::new(
        if direction.x >= 0.0 {
            half_extents.x
        } else {
            -half_extents.x
        },
        if direction.y >= 0.0 {
            half_extents.y
        } else {
            -half_extents.y
        },
        if direction.z >= 0.0 {
            half_extents.z
        } else {
            -half_extents.z
        },
    )
}

/// Capsule support: choose endpoint hemisphere + sphere offset.
///
/// Capsule axis is Z. Half-length is the distance from center to hemisphere center.
#[must_use]
pub fn support_capsule(half_length: f64, radius: f64, direction: &Vector3<f64>) -> Point3<f64> {
    // Choose which endpoint is further in the direction
    let center_z = if direction.z >= 0.0 {
        half_length
    } else {
        -half_length
    };

    // Add sphere support from that endpoint
    let dir_norm = direction.norm();
    let sphere_offset = if dir_norm > EPSILON {
        direction * (radius / dir_norm)
    } else {
        Vector3::new(radius, 0.0, 0.0)
    };

    Point3::new(sphere_offset.x, sphere_offset.y, center_z + sphere_offset.z)
}

/// Cylinder support: choose cap + point on circular edge.
///
/// Cylinder axis is Z. Half-length is distance from center to cap.
#[must_use]
pub fn support_cylinder(half_length: f64, radius: f64, direction: &Vector3<f64>) -> Point3<f64> {
    // Choose cap based on axial component
    let local_z = if direction.z >= 0.0 {
        half_length
    } else {
        -half_length
    };

    // Choose point on circular edge based on radial component
    let radial = Vector3::new(direction.x, direction.y, 0.0);
    let radial_norm = radial.norm();

    let (local_x, local_y) = if radial_norm > EPSILON {
        (
            direction.x * radius / radial_norm,
            direction.y * radius / radial_norm,
        )
    } else {
        // Direction is purely axial — any point on cap edge
        (radius, 0.0)
    };

    Point3::new(local_x, local_y, local_z)
}

/// Ellipsoid support: radii² * dir / |radii * dir|.
#[must_use]
pub fn support_ellipsoid(radii: &Vector3<f64>, direction: &Vector3<f64>) -> Point3<f64> {
    let rx = radii.x.max(EPSILON);
    let ry = radii.y.max(EPSILON);
    let rz = radii.z.max(EPSILON);

    let scaled_dir = Vector3::new(direction.x * rx, direction.y * ry, direction.z * rz);

    let scaled_norm = scaled_dir.norm();
    if scaled_norm < EPSILON {
        return Point3::new(rx, 0.0, 0.0);
    }

    Point3::new(
        rx * scaled_dir.x / scaled_norm,
        ry * scaled_dir.y / scaled_norm,
        rz * scaled_dir.z / scaled_norm,
    )
}

// =============================================================================
// SupportMap implementations for shape component types
// =============================================================================

/// A sphere centered at the origin with given radius.
///
/// Implements [`SupportMap`] for use with GJK/EPA.
pub struct SupportSphere {
    /// Sphere radius.
    pub radius: f64,
}

impl Bounded for SupportSphere {
    fn aabb(&self) -> crate::Aabb {
        crate::Aabb::from_center(
            Point3::origin(),
            Vector3::new(self.radius, self.radius, self.radius),
        )
    }
}

impl SupportMap for SupportSphere {
    fn support(&self, direction: &Vector3<f64>) -> Point3<f64> {
        support_sphere(self.radius, direction)
    }
}

/// A box centered at the origin with given half-extents.
pub struct SupportBox {
    /// Half-extents along each axis.
    pub half_extents: Vector3<f64>,
}

impl Bounded for SupportBox {
    fn aabb(&self) -> crate::Aabb {
        crate::Aabb::from_center(Point3::origin(), self.half_extents)
    }
}

impl SupportMap for SupportBox {
    fn support(&self, direction: &Vector3<f64>) -> Point3<f64> {
        support_box(&self.half_extents, direction)
    }
}

/// A capsule centered at the origin, aligned along Z.
pub struct SupportCapsule {
    /// Distance from center to hemisphere center.
    pub half_length: f64,
    /// Radius of the hemispheres and barrel.
    pub radius: f64,
}

impl Bounded for SupportCapsule {
    fn aabb(&self) -> crate::Aabb {
        crate::Aabb::from_center(
            Point3::origin(),
            Vector3::new(self.radius, self.radius, self.half_length + self.radius),
        )
    }
}

impl SupportMap for SupportCapsule {
    fn support(&self, direction: &Vector3<f64>) -> Point3<f64> {
        support_capsule(self.half_length, self.radius, direction)
    }
}

/// A cylinder centered at the origin, aligned along Z.
pub struct SupportCylinder {
    /// Distance from center to cap.
    pub half_length: f64,
    /// Radius of the circular cross-section.
    pub radius: f64,
}

impl Bounded for SupportCylinder {
    fn aabb(&self) -> crate::Aabb {
        crate::Aabb::from_center(
            Point3::origin(),
            Vector3::new(self.radius, self.radius, self.half_length),
        )
    }
}

impl SupportMap for SupportCylinder {
    fn support(&self, direction: &Vector3<f64>) -> Point3<f64> {
        support_cylinder(self.half_length, self.radius, direction)
    }
}

/// An ellipsoid centered at the origin with given radii.
pub struct SupportEllipsoid {
    /// Radii along each axis.
    pub radii: Vector3<f64>,
}

impl Bounded for SupportEllipsoid {
    fn aabb(&self) -> crate::Aabb {
        crate::Aabb::from_center(Point3::origin(), self.radii)
    }
}

impl SupportMap for SupportEllipsoid {
    fn support(&self, direction: &Vector3<f64>) -> Point3<f64> {
        support_ellipsoid(&self.radii, direction)
    }
}

// ConvexHull already has support() — implement the trait for it.
impl SupportMap for crate::ConvexHull {
    fn support(&self, direction: &Vector3<f64>) -> Point3<f64> {
        // Delegate to ConvexHull's existing support method
        self.support(direction)
    }
}

// =============================================================================
// Tests
// =============================================================================

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::float_cmp)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    #[test]
    fn sphere_support_axis_aligned() {
        let s = SupportSphere { radius: 2.0 };
        let p = s.support(&Vector3::x());
        assert_relative_eq!(p, Point3::new(2.0, 0.0, 0.0), epsilon = 1e-10);

        let p = s.support(&Vector3::y());
        assert_relative_eq!(p, Point3::new(0.0, 2.0, 0.0), epsilon = 1e-10);

        let p = s.support(&(-Vector3::z()));
        assert_relative_eq!(p, Point3::new(0.0, 0.0, -2.0), epsilon = 1e-10);
    }

    #[test]
    fn sphere_support_diagonal() {
        let s = SupportSphere { radius: 1.0 };
        let dir = Vector3::new(1.0, 1.0, 1.0);
        let p = s.support(&dir);
        let expected_r = 1.0 / 3.0_f64.sqrt();
        assert_relative_eq!(p.x, expected_r, epsilon = 1e-10);
        assert_relative_eq!(p.y, expected_r, epsilon = 1e-10);
        assert_relative_eq!(p.z, expected_r, epsilon = 1e-10);
    }

    #[test]
    fn box_support() {
        let b = SupportBox {
            half_extents: Vector3::new(1.0, 2.0, 3.0),
        };
        let p = b.support(&Vector3::new(1.0, 1.0, 1.0));
        assert_relative_eq!(p, Point3::new(1.0, 2.0, 3.0), epsilon = 1e-10);

        let p = b.support(&Vector3::new(-1.0, 1.0, -1.0));
        assert_relative_eq!(p, Point3::new(-1.0, 2.0, -3.0), epsilon = 1e-10);
    }

    #[test]
    fn capsule_support_along_axis() {
        let c = SupportCapsule {
            half_length: 2.0,
            radius: 0.5,
        };
        let p = c.support(&Vector3::z());
        assert_relative_eq!(p, Point3::new(0.0, 0.0, 2.5), epsilon = 1e-10);

        let p = c.support(&(-Vector3::z()));
        assert_relative_eq!(p, Point3::new(0.0, 0.0, -2.5), epsilon = 1e-10);
    }

    #[test]
    fn capsule_support_radial() {
        let c = SupportCapsule {
            half_length: 2.0,
            radius: 0.5,
        };
        let p = c.support(&Vector3::x());
        assert_relative_eq!(p.x, 0.5, epsilon = 1e-10);
        assert_relative_eq!(p.y, 0.0, epsilon = 1e-10);
        // z should be at +half_length (x >= 0 resolves to z >= 0 in capsule, but
        // actually the z component of direction is 0, so it picks +half_length)
        assert_relative_eq!(p.z, 2.0, epsilon = 1e-10);
    }

    #[test]
    fn cylinder_support_top_edge() {
        let c = SupportCylinder {
            half_length: 1.0,
            radius: 0.5,
        };
        let dir = Vector3::new(1.0, 0.0, 1.0);
        let p = c.support(&dir);
        assert_relative_eq!(p.x, 0.5, epsilon = 1e-10);
        assert_relative_eq!(p.y, 0.0, epsilon = 1e-10);
        assert_relative_eq!(p.z, 1.0, epsilon = 1e-10);
    }

    #[test]
    fn cylinder_support_purely_axial() {
        let c = SupportCylinder {
            half_length: 1.0,
            radius: 0.5,
        };
        let p = c.support(&Vector3::z());
        assert_relative_eq!(p.z, 1.0, epsilon = 1e-10);
        // x should be radius (default edge point)
        assert_relative_eq!(p.x, 0.5, epsilon = 1e-10);
    }

    #[test]
    fn ellipsoid_support_axes() {
        let e = SupportEllipsoid {
            radii: Vector3::new(2.0, 1.0, 0.5),
        };

        let p = e.support(&Vector3::x());
        assert_relative_eq!(p, Point3::new(2.0, 0.0, 0.0), epsilon = 1e-10);

        let p = e.support(&Vector3::y());
        assert_relative_eq!(p, Point3::new(0.0, 1.0, 0.0), epsilon = 1e-10);

        let p = e.support(&Vector3::z());
        assert_relative_eq!(p, Point3::new(0.0, 0.0, 0.5), epsilon = 1e-10);
    }

    #[test]
    fn ellipsoid_unit_sphere_equals_sphere() {
        let e = SupportEllipsoid {
            radii: Vector3::new(1.0, 1.0, 1.0),
        };
        let s = SupportSphere { radius: 1.0 };

        let dir = Vector3::new(1.0, 2.0, 3.0);
        let pe = e.support(&dir);
        let ps = s.support(&dir);
        assert_relative_eq!(pe, ps, epsilon = 1e-10);
    }

    #[test]
    fn convex_hull_support_trait() {
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

        let p: Point3<f64> = SupportMap::support(&hull, &Vector3::new(1.0, 1.0, 1.0));
        assert_relative_eq!(p, Point3::new(1.0, 1.0, 1.0), epsilon = 1e-10);
    }
}
