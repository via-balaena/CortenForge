//! Opaque design primitive wrapping a field expression tree.
//!
//! `Solid` is the public API for constructing and querying implicit surface
//! fields. Consumers never see `FieldNode` — they interact entirely through
//! `Solid` methods.
//!
//! The internal representation can change without breaking downstream code
//! (expression tree today, B-Rep in the future — see `CF_DESIGN_SPEC` §8).

use cf_geometry::Aabb;
use nalgebra::{Point3, UnitQuaternion, Vector3};

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

    // ── Boolean operations ────────────────────────────────────────────

    /// Union of two solids: material where either solid exists.
    ///
    /// `min(self, other)`. Preserves SDF lower-bound property.
    #[must_use]
    pub fn union(self, other: Self) -> Self {
        Self {
            node: FieldNode::Union(Box::new(self.node), Box::new(other.node)),
        }
    }

    /// Subtract `other` from `self`: material where `self` exists but `other`
    /// does not.
    ///
    /// `max(self, -other)`.
    #[must_use]
    pub fn subtract(self, other: Self) -> Self {
        Self {
            node: FieldNode::Subtract(Box::new(self.node), Box::new(other.node)),
        }
    }

    /// Intersection of two solids: material where both solids exist.
    ///
    /// `max(self, other)`. Preserves SDF lower-bound property.
    #[must_use]
    pub fn intersect(self, other: Self) -> Self {
        Self {
            node: FieldNode::Intersect(Box::new(self.node), Box::new(other.node)),
        }
    }

    /// Smooth union — blends two solids with blend radius `k`.
    ///
    /// `k = 0` approaches sharp union. Larger `k` produces a wider organic
    /// fillet. The blend region adds material (field value decreases).
    ///
    /// # Panics
    ///
    /// Panics if `k` is not positive and finite.
    #[must_use]
    pub fn smooth_union(self, other: Self, k: f64) -> Self {
        assert!(
            k > 0.0 && k.is_finite(),
            "smooth_union blend radius k must be positive and finite, got {k}"
        );
        Self {
            node: FieldNode::SmoothUnion(Box::new(self.node), Box::new(other.node), k),
        }
    }

    /// Smooth subtraction — smoothly removes `other` from `self` with blend
    /// radius `k`.
    ///
    /// # Panics
    ///
    /// Panics if `k` is not positive and finite.
    #[must_use]
    pub fn smooth_subtract(self, other: Self, k: f64) -> Self {
        assert!(
            k > 0.0 && k.is_finite(),
            "smooth_subtract blend radius k must be positive and finite, got {k}"
        );
        Self {
            node: FieldNode::SmoothSubtract(Box::new(self.node), Box::new(other.node), k),
        }
    }

    /// Smooth intersection — smoothly intersects two solids with blend
    /// radius `k`. Removes material in the blend region.
    ///
    /// # Panics
    ///
    /// Panics if `k` is not positive and finite.
    #[must_use]
    pub fn smooth_intersect(self, other: Self, k: f64) -> Self {
        assert!(
            k > 0.0 && k.is_finite(),
            "smooth_intersect blend radius k must be positive and finite, got {k}"
        );
        Self {
            node: FieldNode::SmoothIntersect(Box::new(self.node), Box::new(other.node), k),
        }
    }

    /// Symmetric n-ary smooth union — blends multiple solids with blend
    /// radius `k`. Order-independent (unlike chaining binary `smooth_union`).
    ///
    /// Uses log-sum-exp internally for symmetric blending.
    ///
    /// # Panics
    ///
    /// Panics if `solids` is empty or `k` is not positive and finite.
    #[must_use]
    pub fn smooth_union_all(solids: Vec<Self>, k: f64) -> Self {
        assert!(
            !solids.is_empty(),
            "smooth_union_all requires at least one solid"
        );
        assert!(
            k > 0.0 && k.is_finite(),
            "smooth_union_all blend radius k must be positive and finite, got {k}"
        );
        let nodes: Vec<FieldNode> = solids.into_iter().map(|s| s.node).collect();
        Self {
            node: FieldNode::SmoothUnionAll(nodes, k),
        }
    }

    // ── Transforms ───────────────────────────────────────────────────

    /// Translate (move) the solid by the given offset.
    ///
    /// Preserves SDF property.
    ///
    /// # Panics
    ///
    /// Panics if any component of `offset` is non-finite.
    #[must_use]
    pub fn translate(self, offset: Vector3<f64>) -> Self {
        assert!(
            offset.iter().all(|v| v.is_finite()),
            "translate offset must be finite, got {offset:?}"
        );
        Self {
            node: FieldNode::Translate(Box::new(self.node), offset),
        }
    }

    /// Rotate the solid by the given unit quaternion.
    ///
    /// Preserves SDF property.
    #[must_use]
    pub fn rotate(self, rotation: UnitQuaternion<f64>) -> Self {
        Self {
            node: FieldNode::Rotate(Box::new(self.node), rotation),
        }
    }

    /// Uniformly scale the solid by the given factor.
    ///
    /// Preserves SDF property. Factor must be positive.
    ///
    /// # Panics
    ///
    /// Panics if `factor` is not positive and finite.
    #[must_use]
    pub fn scale_uniform(self, factor: f64) -> Self {
        assert!(
            factor > 0.0 && factor.is_finite(),
            "scale_uniform factor must be positive and finite, got {factor}"
        );
        Self {
            node: FieldNode::ScaleUniform(Box::new(self.node), factor),
        }
    }

    /// Mirror the solid across a plane through the origin with the given
    /// normal.
    ///
    /// The geometry on the positive side of the plane is reflected to the
    /// negative side.
    ///
    /// # Panics
    ///
    /// Panics if `normal` is zero-length or non-finite.
    #[must_use]
    pub fn mirror(self, normal: Vector3<f64>) -> Self {
        let len = normal.norm();
        assert!(
            len > 1e-12 && normal.iter().all(|v| v.is_finite()),
            "mirror normal must be non-zero and finite, got {normal:?}"
        );
        let unit_normal = normal / len;
        Self {
            node: FieldNode::Mirror(Box::new(self.node), unit_normal),
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
    use std::f64::consts::PI;

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

    // ── Boolean validation ──────────────────────────────────────────

    #[test]
    #[should_panic(expected = "positive and finite")]
    fn smooth_union_rejects_zero_k() {
        let a = Solid::sphere(1.0);
        let b = Solid::sphere(1.0);
        drop(a.smooth_union(b, 0.0));
    }

    #[test]
    #[should_panic(expected = "positive and finite")]
    fn smooth_subtract_rejects_negative_k() {
        let a = Solid::sphere(1.0);
        let b = Solid::sphere(1.0);
        drop(a.smooth_subtract(b, -1.0));
    }

    #[test]
    #[should_panic(expected = "positive and finite")]
    fn smooth_intersect_rejects_nan_k() {
        let a = Solid::sphere(1.0);
        let b = Solid::sphere(1.0);
        drop(a.smooth_intersect(b, f64::NAN));
    }

    #[test]
    #[should_panic(expected = "at least one solid")]
    fn smooth_union_all_rejects_empty() {
        drop(Solid::smooth_union_all(vec![], 1.0));
    }

    // ── Transform validation ────────────────────────────────────────

    #[test]
    #[should_panic(expected = "positive and finite")]
    fn scale_uniform_rejects_zero() {
        drop(Solid::sphere(1.0).scale_uniform(0.0));
    }

    #[test]
    #[should_panic(expected = "positive and finite")]
    fn scale_uniform_rejects_negative() {
        drop(Solid::sphere(1.0).scale_uniform(-1.0));
    }

    #[test]
    #[should_panic(expected = "non-zero and finite")]
    fn mirror_rejects_zero_normal() {
        drop(Solid::sphere(1.0).mirror(Vector3::zeros()));
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

    // ── Boolean builder methods ──────────────────────────────────────

    #[test]
    fn solid_union_method() {
        let a = Solid::sphere(2.0);
        let b = Solid::sphere(3.0);
        let u = a.union(b);
        assert!((u.evaluate(&Point3::origin()) - (-3.0)).abs() < 1e-10);
    }

    #[test]
    fn solid_subtract_method() {
        let big = Solid::sphere(5.0);
        let small = Solid::sphere(2.0);
        let sub = big.subtract(small);
        // Origin: max(-5, 2) = 2
        assert!(sub.evaluate(&Point3::origin()) > 0.0);
        // Shell region: max(3-5, -(3-2)) = max(-2, -1) = -1
        assert!(sub.evaluate(&Point3::new(3.0, 0.0, 0.0)) < 0.0);
    }

    #[test]
    fn solid_intersect_method() {
        let a = Solid::sphere(5.0);
        let b = Solid::sphere(2.0);
        let inter = a.intersect(b);
        assert!((inter.evaluate(&Point3::origin()) - (-2.0)).abs() < 1e-10);
    }

    #[test]
    fn solid_smooth_union_method() {
        let a = Solid::sphere(2.0);
        let b = Solid::sphere(2.0).translate(Vector3::new(3.0, 0.0, 0.0));
        let k = 1.0;
        let su = a.smooth_union(b, k);
        // In the blend region, smooth union adds material
        let p = Point3::new(1.5, 0.0, 0.0);
        // Should be more negative (more inside) than the sharper of the two
        assert!(su.evaluate(&p) < 1.0);
    }

    #[test]
    fn solid_smooth_union_all_method() {
        let solids = vec![
            Solid::sphere(2.0),
            Solid::sphere(2.0).translate(Vector3::new(3.0, 0.0, 0.0)),
            Solid::sphere(2.0).translate(Vector3::new(0.0, 3.0, 0.0)),
        ];
        let sua = Solid::smooth_union_all(solids, 1.0);
        // Should be inside near each sphere center
        assert!(sua.evaluate(&Point3::origin()) < 0.0);
        assert!(sua.evaluate(&Point3::new(3.0, 0.0, 0.0)) < 0.0);
        assert!(sua.evaluate(&Point3::new(0.0, 3.0, 0.0)) < 0.0);
    }

    // ── Transform builder methods ────────────────────────────────────

    #[test]
    fn solid_translate_method() {
        let s = Solid::sphere(1.0).translate(Vector3::new(5.0, 0.0, 0.0));
        assert!((s.evaluate(&Point3::new(5.0, 0.0, 0.0)) - (-1.0)).abs() < 1e-10);
        assert!((s.evaluate(&Point3::origin()) - 4.0).abs() < 1e-10);
    }

    #[test]
    fn solid_rotate_method() {
        let c = Solid::cuboid(Vector3::new(1.0, 2.0, 1.0));
        let rot = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), PI / 2.0);
        let r = c.rotate(rot);
        // After 90° Z rotation: x-extent becomes 2, y-extent becomes 1
        assert!(r.evaluate(&Point3::new(1.5, 0.0, 0.0)) < 0.0);
        assert!(r.evaluate(&Point3::new(0.0, 1.5, 0.0)) > 0.0);
    }

    #[test]
    fn solid_scale_uniform_method() {
        let s = Solid::sphere(1.0).scale_uniform(3.0);
        assert!((s.evaluate(&Point3::origin()) - (-3.0)).abs() < 1e-10);
        assert!((s.evaluate(&Point3::new(3.0, 0.0, 0.0))).abs() < 1e-10);
    }

    #[test]
    fn solid_mirror_method() {
        let s = Solid::sphere(1.0).translate(Vector3::new(3.0, 0.0, 0.0));
        let m = s.mirror(Vector3::x());
        // Mirrored: sphere at both (3,0,0) and (-3,0,0)
        assert!(m.evaluate(&Point3::new(3.0, 0.0, 0.0)) < 0.0);
        assert!(m.evaluate(&Point3::new(-3.0, 0.0, 0.0)) < 0.0);
    }

    #[test]
    fn solid_mirror_normalizes() {
        // Non-unit normal should still work correctly
        let s = Solid::sphere(1.0).translate(Vector3::new(3.0, 0.0, 0.0));
        let m = s.mirror(Vector3::new(5.0, 0.0, 0.0)); // unnormalized
        assert!(m.evaluate(&Point3::new(-3.0, 0.0, 0.0)) < 0.0);
    }

    // ── Method chaining ──────────────────────────────────────────────

    #[test]
    fn solid_method_chaining() {
        // Build a hollowed, translated, scaled sphere.
        // Evaluation chain: scale(translate(subtract(sphere(5), sphere(3)), (10,0,0)), 2)
        // scale_uniform(2) evaluates f(p/2)*2, so effective center is (20,0,0),
        // effective outer radius = 10, effective inner radius = 6.
        let part = Solid::sphere(5.0)
            .subtract(Solid::sphere(3.0))
            .translate(Vector3::new(10.0, 0.0, 0.0))
            .scale_uniform(2.0);
        // Center at (20, 0, 0) — inside the hole (inner radius 6)
        assert!(part.evaluate(&Point3::new(20.0, 0.0, 0.0)) > 0.0);
        // On outer surface at x = 30 (center 20 + outer radius 10)
        assert!((part.evaluate(&Point3::new(30.0, 0.0, 0.0))).abs() < 1e-6);
        // Inside the shell at x = 28 (8 from center, between inner 6 and outer 10)
        assert!(part.evaluate(&Point3::new(28.0, 0.0, 0.0)) < 0.0);
    }

    #[test]
    fn solid_interval_for_union() {
        let u = Solid::sphere(2.0).union(Solid::sphere(3.0));
        let aabb = Aabb::new(Point3::new(-0.5, -0.5, -0.5), Point3::new(0.5, 0.5, 0.5));
        let (lo, hi) = u.evaluate_interval(&aabb);
        // Both spheres fully contain the box, interval should be negative
        assert!(hi < 0.0);
        assert!(lo < hi);
    }

    #[test]
    fn solid_interval_for_translate() {
        let s = Solid::sphere(2.0).translate(Vector3::new(5.0, 0.0, 0.0));
        // Box at origin: fully outside the translated sphere
        let aabb = Aabb::new(Point3::new(-0.5, -0.5, -0.5), Point3::new(0.5, 0.5, 0.5));
        let (lo, _hi) = s.evaluate_interval(&aabb);
        assert!(
            lo > 0.0,
            "Box at origin should be fully outside sphere at x=5"
        );
    }

    #[test]
    fn solid_interval_for_scale() {
        let s = Solid::sphere(1.0).scale_uniform(5.0);
        // Small box at origin: fully inside the scaled sphere
        let aabb = Aabb::new(Point3::new(-0.5, -0.5, -0.5), Point3::new(0.5, 0.5, 0.5));
        let (_lo, hi) = s.evaluate_interval(&aabb);
        assert!(hi < 0.0, "Box at origin should be fully inside sphere(r=5)");
    }
}
