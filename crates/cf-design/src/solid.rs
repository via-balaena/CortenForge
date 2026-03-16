//! Opaque design primitive wrapping a field expression tree.
//!
//! `Solid` is the public API for constructing and querying implicit surface
//! fields. Consumers never see `FieldNode` — they interact entirely through
//! `Solid` methods.
//!
//! The internal representation can change without breaking downstream code
//! (expression tree today, B-Rep in the future — see `CF_DESIGN_SPEC` §8).

use cf_geometry::Aabb;
use nalgebra::{Point3, Vector3};

use crate::field_node::FieldNode;

/// Opaque solid defined by an implicit surface field.
///
/// Construct with primitive factory methods (`sphere`, `cuboid`, etc.) and
/// compose with boolean operations (`union`, `subtract`, etc.) and transforms
/// (`translate`, `rotate`, etc.) — added in later sessions.
///
/// Convention: the field is negative inside, positive outside, zero on surface.
#[derive(Debug, Clone)]
pub struct Solid {
    pub(crate) node: FieldNode,
}

impl Solid {
    // ── Primitive constructors ────────────────────────────────────────

    /// Sphere centered at origin with the given radius.
    ///
    /// Exact SDF.
    ///
    /// # Panics
    ///
    /// Panics if `radius` is not positive and finite.
    #[must_use]
    pub fn sphere(radius: f64) -> Self {
        assert!(
            radius > 0.0 && radius.is_finite(),
            "sphere radius must be positive and finite, got {radius}"
        );
        Self {
            node: FieldNode::Sphere { radius },
        }
    }

    /// Axis-aligned box centered at origin with the given half-extents.
    ///
    /// Exact SDF.
    ///
    /// # Panics
    ///
    /// Panics if any half-extent is not positive and finite.
    #[must_use]
    pub fn cuboid(half_extents: Vector3<f64>) -> Self {
        assert!(
            half_extents.iter().all(|&v| v > 0.0 && v.is_finite()),
            "cuboid half_extents must be positive and finite, got {half_extents:?}"
        );
        Self {
            node: FieldNode::Cuboid { half_extents },
        }
    }

    /// Z-aligned cylinder centered at origin.
    ///
    /// Exact SDF. The cylinder extends from `z = -half_height` to
    /// `z = half_height` with the given radius.
    ///
    /// # Panics
    ///
    /// Panics if `radius` or `half_height` is not positive and finite.
    #[must_use]
    pub fn cylinder(radius: f64, half_height: f64) -> Self {
        assert!(
            radius > 0.0 && radius.is_finite(),
            "cylinder radius must be positive and finite, got {radius}"
        );
        assert!(
            half_height > 0.0 && half_height.is_finite(),
            "cylinder half_height must be positive and finite, got {half_height}"
        );
        Self {
            node: FieldNode::Cylinder {
                radius,
                half_height,
            },
        }
    }

    /// Z-aligned capsule (cylinder with hemispherical caps) centered at origin.
    ///
    /// Exact SDF. The cylindrical segment extends from `z = -half_height` to
    /// `z = half_height`. Total height is `2 * (half_height + radius)`.
    ///
    /// # Panics
    ///
    /// Panics if `radius` is not positive and finite, or `half_height` is
    /// negative or non-finite.
    #[must_use]
    pub fn capsule(radius: f64, half_height: f64) -> Self {
        assert!(
            radius > 0.0 && radius.is_finite(),
            "capsule radius must be positive and finite, got {radius}"
        );
        assert!(
            half_height >= 0.0 && half_height.is_finite(),
            "capsule half_height must be non-negative and finite, got {half_height}"
        );
        Self {
            node: FieldNode::Capsule {
                radius,
                half_height,
            },
        }
    }

    /// Ellipsoid centered at origin with the given axis radii.
    ///
    /// **Not an exact SDF** — the field magnitude is approximate, but the
    /// zero-isosurface is correct. Safe for meshing; `shell()` and `round()`
    /// will produce non-uniform results.
    ///
    /// # Panics
    ///
    /// Panics if any radius is not positive and finite.
    #[must_use]
    pub fn ellipsoid(radii: Vector3<f64>) -> Self {
        assert!(
            radii.iter().all(|&v| v > 0.0 && v.is_finite()),
            "ellipsoid radii must be positive and finite, got {radii:?}"
        );
        Self {
            node: FieldNode::Ellipsoid { radii },
        }
    }

    /// Torus in the XY plane centered at origin.
    ///
    /// Exact SDF. `major` is the distance from the center to the tube center.
    /// `minor` is the tube radius.
    ///
    /// # Panics
    ///
    /// Panics if `major` or `minor` is not positive and finite.
    #[must_use]
    pub fn torus(major: f64, minor: f64) -> Self {
        assert!(
            major > 0.0 && major.is_finite(),
            "torus major radius must be positive and finite, got {major}"
        );
        assert!(
            minor > 0.0 && minor.is_finite(),
            "torus minor radius must be positive and finite, got {minor}"
        );
        Self {
            node: FieldNode::Torus { major, minor },
        }
    }

    /// Cone with apex at origin, expanding downward along -Z.
    ///
    /// Exact SDF. The base is at `z = -height` with the given `radius`.
    ///
    /// # Panics
    ///
    /// Panics if `radius` or `height` is not positive and finite.
    #[must_use]
    pub fn cone(radius: f64, height: f64) -> Self {
        assert!(
            radius > 0.0 && radius.is_finite(),
            "cone radius must be positive and finite, got {radius}"
        );
        assert!(
            height > 0.0 && height.is_finite(),
            "cone height must be positive and finite, got {height}"
        );
        Self {
            node: FieldNode::Cone { radius, height },
        }
    }

    /// Half-space defined by a plane. Points on the `normal` side are outside
    /// (positive); points on the opposite side are inside (negative).
    ///
    /// Exact SDF when `normal` is unit length.
    ///
    /// # Panics
    ///
    /// Panics if `normal` is zero-length or non-finite, or `offset` is
    /// non-finite.
    #[must_use]
    pub fn plane(normal: Vector3<f64>, offset: f64) -> Self {
        let len = normal.norm();
        assert!(
            len > 1e-12 && normal.iter().all(|v| v.is_finite()),
            "plane normal must be non-zero and finite, got {normal:?}"
        );
        assert!(
            offset.is_finite(),
            "plane offset must be finite, got {offset}"
        );
        // Normalize the normal to ensure exact SDF property.
        let unit_normal = normal / len;
        let scaled_offset = offset / len;
        Self {
            node: FieldNode::Plane {
                normal: unit_normal,
                offset: scaled_offset,
            },
        }
    }

    // ── Queries ──────────────────────────────────────────────────────

    /// Evaluate the field at a point.
    ///
    /// Returns the signed distance (exact or approximate depending on the
    /// primitives involved). Negative = inside, positive = outside, zero =
    /// on surface.
    #[must_use]
    pub fn evaluate(&self, point: &Point3<f64>) -> f64 {
        self.node.evaluate(point)
    }

    /// Compute conservative (min, max) bounds of the field over an `Aabb`.
    ///
    /// The returned interval `(lo, hi)` satisfies:
    /// `lo <= self.evaluate(p) <= hi` for all `p` in the box.
    ///
    /// Used for octree pruning during meshing: if `lo > 0`, the box is fully
    /// outside; if `hi < 0`, fully inside.
    #[must_use]
    pub fn evaluate_interval(&self, aabb: &Aabb) -> (f64, f64) {
        self.node.evaluate_interval(aabb)
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    // ── Constructor validation ───────────────────────────────────────

    #[test]
    #[should_panic(expected = "positive and finite")]
    fn sphere_rejects_zero_radius() {
        drop(Solid::sphere(0.0));
    }

    #[test]
    #[should_panic(expected = "positive and finite")]
    fn sphere_rejects_negative_radius() {
        drop(Solid::sphere(-1.0));
    }

    #[test]
    #[should_panic(expected = "positive and finite")]
    fn sphere_rejects_nan() {
        drop(Solid::sphere(f64::NAN));
    }

    #[test]
    #[should_panic(expected = "positive and finite")]
    fn sphere_rejects_infinity() {
        drop(Solid::sphere(f64::INFINITY));
    }

    #[test]
    #[should_panic(expected = "positive and finite")]
    fn cuboid_rejects_zero_extent() {
        drop(Solid::cuboid(Vector3::new(1.0, 0.0, 1.0)));
    }

    #[test]
    #[should_panic(expected = "positive and finite")]
    fn cylinder_rejects_bad_radius() {
        drop(Solid::cylinder(-1.0, 2.0));
    }

    #[test]
    #[should_panic(expected = "positive and finite")]
    fn capsule_rejects_bad_radius() {
        drop(Solid::capsule(0.0, 2.0));
    }

    #[test]
    #[should_panic(expected = "positive and finite")]
    fn ellipsoid_rejects_bad_radii() {
        drop(Solid::ellipsoid(Vector3::new(1.0, -1.0, 1.0)));
    }

    #[test]
    #[should_panic(expected = "positive and finite")]
    fn torus_rejects_bad_major() {
        drop(Solid::torus(0.0, 1.0));
    }

    #[test]
    #[should_panic(expected = "positive and finite")]
    fn cone_rejects_bad_radius() {
        drop(Solid::cone(0.0, 2.0));
    }

    #[test]
    #[should_panic(expected = "non-zero and finite")]
    fn plane_rejects_zero_normal() {
        drop(Solid::plane(Vector3::zeros(), 0.0));
    }

    // ── Evaluation through Solid ─────────────────────────────────────

    #[test]
    fn solid_sphere_evaluate() {
        let s = Solid::sphere(2.0);
        assert!((s.evaluate(&Point3::origin()) - (-2.0)).abs() < 1e-10);
        assert!((s.evaluate(&Point3::new(2.0, 0.0, 0.0))).abs() < 1e-10);
        assert!(s.evaluate(&Point3::new(5.0, 0.0, 0.0)) > 0.0);
    }

    #[test]
    fn solid_sphere_interval() {
        let s = Solid::sphere(2.0);
        let aabb = Aabb::new(Point3::new(-0.5, -0.5, -0.5), Point3::new(0.5, 0.5, 0.5));
        let (lo, hi) = s.evaluate_interval(&aabb);
        // Interval should be fully negative (inside sphere)
        assert!(hi < 0.0, "Expected negative interval for box inside sphere");
        assert!(lo < hi);
    }

    #[test]
    fn solid_cuboid_evaluate() {
        let c = Solid::cuboid(Vector3::new(1.0, 2.0, 3.0));
        assert!(c.evaluate(&Point3::origin()) < 0.0);
        assert!((c.evaluate(&Point3::new(1.0, 0.0, 0.0))).abs() < 1e-10);
    }

    #[test]
    fn solid_plane_normalizes() {
        // Non-unit normal should still produce correct SDF
        let p = Solid::plane(Vector3::new(0.0, 0.0, 2.0), 6.0);
        // Normalized: normal=(0,0,1), offset=3
        assert!((p.evaluate(&Point3::new(0.0, 0.0, 3.0))).abs() < 1e-10);
        assert!((p.evaluate(&Point3::new(0.0, 0.0, 5.0)) - 2.0).abs() < 1e-10);
    }

    #[test]
    fn capsule_zero_half_height_is_sphere() {
        let cap = Solid::capsule(2.0, 0.0);
        let sph = Solid::sphere(2.0);
        let test_points = [
            Point3::origin(),
            Point3::new(2.0, 0.0, 0.0),
            Point3::new(0.0, 0.0, 2.0),
            Point3::new(5.0, 0.0, 0.0),
            Point3::new(1.0, 1.0, 1.0),
        ];
        for p in &test_points {
            assert!(
                (cap.evaluate(p) - sph.evaluate(p)).abs() < 1e-10,
                "Capsule(r=2, h=0) should match Sphere(r=2) at {p:?}"
            );
        }
    }
}
