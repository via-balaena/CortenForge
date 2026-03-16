//! Point evaluation of field nodes.
//!
//! Each primitive computes its signed distance (or approximate signed distance)
//! at a query point. Convention: negative inside, positive outside, zero on
//! surface.

use nalgebra::Point3;

use crate::field_node::FieldNode;

impl FieldNode {
    /// Evaluate the scalar field at a point.
    ///
    /// Returns the signed distance (exact or approximate depending on the
    /// primitive). Negative = inside, positive = outside, zero = on surface.
    pub(crate) fn evaluate(&self, p: &Point3<f64>) -> f64 {
        match self {
            // Primitives
            Self::Sphere { radius } => eval_sphere(*radius, p),
            Self::Cuboid { half_extents } => eval_cuboid(half_extents, p),
            Self::Cylinder {
                radius,
                half_height,
            } => eval_cylinder(*radius, *half_height, p),
            Self::Capsule {
                radius,
                half_height,
            } => eval_capsule(*radius, *half_height, p),
            Self::Ellipsoid { radii } => eval_ellipsoid(radii, p),
            Self::Torus { major, minor } => eval_torus(*major, *minor, p),
            Self::Cone { radius, height } => eval_cone(*radius, *height, p),
            Self::Plane { normal, offset } => eval_plane(normal, *offset, p),

            // Booleans
            Self::Union(a, b) => a.evaluate(p).min(b.evaluate(p)),
            Self::Subtract(a, b) => a.evaluate(p).max(-b.evaluate(p)),
            Self::Intersect(a, b) => a.evaluate(p).max(b.evaluate(p)),
            Self::SmoothUnion(a, b, k) => eval_smooth_union(a.evaluate(p), b.evaluate(p), *k),
            Self::SmoothSubtract(a, b, k) => {
                // smooth_subtract(a, b, k) = -smooth_union(-a, b, k)
                -eval_smooth_union(-a.evaluate(p), b.evaluate(p), *k)
            }
            Self::SmoothIntersect(a, b, k) => {
                // smooth_intersect(a, b, k) = -smooth_union(-a, -b, k)
                -eval_smooth_union(-a.evaluate(p), -b.evaluate(p), *k)
            }
            Self::SmoothUnionAll(children, k) => {
                let values: Vec<f64> = children.iter().map(|c| c.evaluate(p)).collect();
                eval_smooth_union_all(&values, *k)
            }

            // Transforms
            Self::Translate(child, offset) => {
                let p_local = Point3::new(p.x - offset.x, p.y - offset.y, p.z - offset.z);
                child.evaluate(&p_local)
            }
            Self::Rotate(child, q) => {
                let p_local = q.inverse_transform_point(p);
                child.evaluate(&p_local)
            }
            Self::ScaleUniform(child, s) => {
                let inv_s = 1.0 / *s;
                let p_local = Point3::new(p.x * inv_s, p.y * inv_s, p.z * inv_s);
                child.evaluate(&p_local) * *s
            }
            Self::Mirror(child, normal) => {
                let d = p.coords.dot(normal).min(0.0);
                let two_d = 2.0 * d;
                let p_mirrored = Point3::new(
                    two_d.mul_add(-normal.x, p.x),
                    two_d.mul_add(-normal.y, p.y),
                    two_d.mul_add(-normal.z, p.z),
                );
                child.evaluate(&p_mirrored)
            }

            // Domain operations
            Self::Shell(child, thickness) => child.evaluate(p).abs() - thickness,
            Self::Round(child, radius) => child.evaluate(p) - radius,
            Self::Offset(child, distance) => child.evaluate(p) - distance,
            Self::Elongate(child, half) => {
                let q = Point3::new(
                    p.x - p.x.clamp(-half.x, half.x),
                    p.y - p.y.clamp(-half.y, half.y),
                    p.z - p.z.clamp(-half.z, half.z),
                );
                child.evaluate(&q)
            }

            // User function
            Self::UserFn { eval, .. } => (eval.0)(*p),
        }
    }
}

// ── Smooth boolean helpers ──────────────────────────────────────────────

/// Polynomial smooth minimum (IQ). Blend radius `k`.
///
/// `h = clamp(0.5 + 0.5*(b-a)/k, 0, 1)`
/// `result = mix(b, a, h) - k*h*(1-h)`
///
/// Always ≤ min(a, b). Maximum correction is k/4 at h=0.5.
fn eval_smooth_union(a: f64, b: f64, k: f64) -> f64 {
    let h = (0.5 + 0.5 * (b - a) / k).clamp(0.0, 1.0);
    let one_minus_h = 1.0 - h;
    let mix = a.mul_add(h, b * one_minus_h);
    (k * h).mul_add(-one_minus_h, mix)
}

/// Symmetric n-ary smooth union via log-sum-exp.
///
/// `f = m - k * ln(Σ exp(-(xᵢ - m) / k))`
/// where `m = min(xᵢ)` for numerical stability.
///
/// Order-independent. Approaches `min()` as k→0.
fn eval_smooth_union_all(values: &[f64], k: f64) -> f64 {
    if values.is_empty() {
        return f64::INFINITY;
    }
    if values.len() == 1 {
        return values[0];
    }
    // Log-sum-exp with shift for numerical stability
    let m = values.iter().copied().fold(f64::INFINITY, f64::min);
    let sum: f64 = values.iter().map(|&v| (-(v - m) / k).exp()).sum();
    k.mul_add(-sum.ln(), m)
}

// ── Primitive SDF implementations ────────────────────────────────────────

/// Sphere: `|p| - r`. Exact SDF.
fn eval_sphere(radius: f64, p: &Point3<f64>) -> f64 {
    p.coords.norm() - radius
}

/// Axis-aligned box: exact signed distance to box with given half-extents.
fn eval_cuboid(half: &nalgebra::Vector3<f64>, p: &Point3<f64>) -> f64 {
    let q = nalgebra::Vector3::new(p.x.abs() - half.x, p.y.abs() - half.y, p.z.abs() - half.z);
    let outside = nalgebra::Vector3::new(q.x.max(0.0), q.y.max(0.0), q.z.max(0.0)).norm();
    let inside = q.x.max(q.y.max(q.z)).min(0.0);
    outside + inside
}

/// Z-aligned cylinder: exact signed distance.
fn eval_cylinder(radius: f64, half_height: f64, p: &Point3<f64>) -> f64 {
    let d_radial = p.x.hypot(p.y) - radius;
    let d_axial = p.z.abs() - half_height;
    let outside = d_radial.max(0.0).hypot(d_axial.max(0.0));
    let inside = d_radial.max(d_axial).min(0.0);
    outside + inside
}

/// Z-aligned capsule: exact signed distance.
/// The capsule is a cylinder of half-height `half_height` capped with
/// hemispheres of radius `radius`.
fn eval_capsule(radius: f64, half_height: f64, p: &Point3<f64>) -> f64 {
    let clamped_z = p.z.clamp(-half_height, half_height);
    let closest_axis = Point3::new(0.0, 0.0, clamped_z);
    nalgebra::distance(p, &closest_axis) - radius
}

/// Ellipsoid: approximate signed distance via scaled-sphere method.
///
/// This is NOT an exact SDF — the magnitude is only approximately correct,
/// but the sign and zero-isosurface are correct. The approximation uses
/// Inigo Quilez's method: evaluate on the unit sphere in scaled space,
/// then correct by the local scaling factor.
fn eval_ellipsoid(radii: &nalgebra::Vector3<f64>, p: &Point3<f64>) -> f64 {
    // Scale point into unit-sphere space
    let scaled = nalgebra::Vector3::new(p.x / radii.x, p.y / radii.y, p.z / radii.z);
    let scaled_norm = scaled.norm();

    if scaled_norm < 1.0e-12 {
        // At the center — distance is the smallest radius
        return -radii.x.min(radii.y.min(radii.z));
    }

    // Gradient of the scaling for distance correction
    let grad = nalgebra::Vector3::new(
        p.x / (radii.x * radii.x),
        p.y / (radii.y * radii.y),
        p.z / (radii.z * radii.z),
    );
    let grad_norm = grad.norm();

    if grad_norm < 1.0e-12 {
        return scaled_norm - 1.0;
    }

    // Approximate signed distance
    scaled_norm * (scaled_norm - 1.0) / grad_norm
}

/// Torus in the XY plane: exact SDF.
/// `major` = distance from center to tube center.
/// `minor` = tube radius.
fn eval_torus(major: f64, minor: f64, p: &Point3<f64>) -> f64 {
    let q_xy = p.x.hypot(p.y) - major;
    q_xy.hypot(p.z) - minor
}

/// Cone with apex at origin, expanding along -Z with base at `z = -height`.
/// Exact SDF.
fn eval_cone(radius: f64, height: f64, p: &Point3<f64>) -> f64 {
    // Work in the 2D (r, z) plane where r = distance from Z axis.
    let r = p.x.hypot(p.y);

    // Cone axis: apex at z=0, base at z=-height with radius `radius`.
    // The cone surface in (r, z) is the line from (0, 0) to (radius, -height).
    let cone_len = radius.hypot(height);
    // Outward normal of the cone surface in (r, z) space: (height, radius) / cone_len
    let nr = height / cone_len;
    let nz = radius / cone_len;

    // Project onto the cone surface
    let dot_surface = r.mul_add(nr, p.z * nz);

    if p.z > 0.0 {
        // Above apex — distance to apex point
        r.hypot(p.z)
    } else if p.z < -height {
        // Below base — distance to base disk
        let d_radial = (r - radius).max(0.0);
        let d_base = -(p.z + height);
        d_radial.hypot(d_base)
    } else {
        // Between apex and base height
        let t = -p.z / height;
        let cone_r_at_z = radius * t;

        if r <= cone_r_at_z {
            // Inside the cone — negative distance
            let d_surf = -dot_surface;
            let d_base_inner = p.z + height;
            -(d_surf.min(d_base_inner))
        } else {
            // Outside the cone surface between apex and base
            dot_surface
        }
    }
}

/// Half-space: `dot(normal, p) - offset`. Exact SDF when normal is unit length.
fn eval_plane(normal: &nalgebra::Vector3<f64>, offset: f64, p: &Point3<f64>) -> f64 {
    normal.dot(&p.coords) - offset
}

#[cfg(test)]
mod tests {
    use super::*;
    use approx::assert_abs_diff_eq;
    use nalgebra::{UnitQuaternion, Vector3};
    use std::f64::consts::PI;

    const EPS: f64 = 1e-10;

    // ── Sphere ───────────────────────────────────────────────────────

    #[test]
    fn sphere_origin_is_negative_radius() {
        let s = FieldNode::Sphere { radius: 3.0 };
        assert_abs_diff_eq!(s.evaluate(&Point3::origin()), -3.0, epsilon = EPS);
    }

    #[test]
    fn sphere_on_surface_is_zero() {
        let s = FieldNode::Sphere { radius: 2.0 };
        assert_abs_diff_eq!(s.evaluate(&Point3::new(2.0, 0.0, 0.0)), 0.0, epsilon = EPS);
        assert_abs_diff_eq!(s.evaluate(&Point3::new(0.0, 2.0, 0.0)), 0.0, epsilon = EPS);
        assert_abs_diff_eq!(s.evaluate(&Point3::new(0.0, 0.0, 2.0)), 0.0, epsilon = EPS);
    }

    #[test]
    fn sphere_outside_is_positive() {
        let s = FieldNode::Sphere { radius: 1.0 };
        assert!(s.evaluate(&Point3::new(5.0, 0.0, 0.0)) > 0.0);
    }

    // ── Cuboid ───────────────────────────────────────────────────────

    #[test]
    fn cuboid_origin_is_negative() {
        let c = FieldNode::Cuboid {
            half_extents: nalgebra::Vector3::new(1.0, 2.0, 3.0),
        };
        assert_abs_diff_eq!(c.evaluate(&Point3::origin()), -1.0, epsilon = EPS);
    }

    #[test]
    fn cuboid_on_face_is_zero() {
        let c = FieldNode::Cuboid {
            half_extents: nalgebra::Vector3::new(1.0, 2.0, 3.0),
        };
        assert_abs_diff_eq!(c.evaluate(&Point3::new(1.0, 0.0, 0.0)), 0.0, epsilon = EPS);
        assert_abs_diff_eq!(c.evaluate(&Point3::new(0.0, 2.0, 0.0)), 0.0, epsilon = EPS);
    }

    #[test]
    fn cuboid_corner_distance() {
        let c = FieldNode::Cuboid {
            half_extents: nalgebra::Vector3::new(1.0, 1.0, 1.0),
        };
        // Point at (2, 2, 2) — distance to corner (1,1,1) = sqrt(3)
        let d = c.evaluate(&Point3::new(2.0, 2.0, 2.0));
        assert_abs_diff_eq!(d, 3.0_f64.sqrt(), epsilon = EPS);
    }

    // ── Cylinder ─────────────────────────────────────────────────────

    #[test]
    fn cylinder_origin_is_negative() {
        let c = FieldNode::Cylinder {
            radius: 2.0,
            half_height: 3.0,
        };
        assert_abs_diff_eq!(c.evaluate(&Point3::origin()), -2.0, epsilon = EPS);
    }

    #[test]
    fn cylinder_on_barrel_is_zero() {
        let c = FieldNode::Cylinder {
            radius: 2.0,
            half_height: 3.0,
        };
        assert_abs_diff_eq!(c.evaluate(&Point3::new(2.0, 0.0, 0.0)), 0.0, epsilon = EPS);
    }

    #[test]
    fn cylinder_on_cap_is_zero() {
        let c = FieldNode::Cylinder {
            radius: 2.0,
            half_height: 3.0,
        };
        assert_abs_diff_eq!(c.evaluate(&Point3::new(0.0, 0.0, 3.0)), 0.0, epsilon = EPS);
    }

    // ── Capsule ──────────────────────────────────────────────────────

    #[test]
    fn capsule_origin_is_negative_radius() {
        let c = FieldNode::Capsule {
            radius: 1.0,
            half_height: 2.0,
        };
        assert_abs_diff_eq!(c.evaluate(&Point3::origin()), -1.0, epsilon = EPS);
    }

    #[test]
    fn capsule_on_barrel_is_zero() {
        let c = FieldNode::Capsule {
            radius: 1.0,
            half_height: 2.0,
        };
        assert_abs_diff_eq!(c.evaluate(&Point3::new(1.0, 0.0, 0.0)), 0.0, epsilon = EPS);
    }

    #[test]
    fn capsule_on_cap_is_zero() {
        let c = FieldNode::Capsule {
            radius: 1.0,
            half_height: 2.0,
        };
        // Top of capsule: z = half_height + radius = 3.0
        assert_abs_diff_eq!(c.evaluate(&Point3::new(0.0, 0.0, 3.0)), 0.0, epsilon = EPS);
    }

    // ── Ellipsoid ────────────────────────────────────────────────────

    #[test]
    fn ellipsoid_origin_is_negative() {
        let e = FieldNode::Ellipsoid {
            radii: nalgebra::Vector3::new(2.0, 3.0, 4.0),
        };
        assert!(e.evaluate(&Point3::origin()) < 0.0);
    }

    #[test]
    fn ellipsoid_on_surface_is_near_zero() {
        let e = FieldNode::Ellipsoid {
            radii: nalgebra::Vector3::new(2.0, 3.0, 4.0),
        };
        // On-axis points are on the surface
        assert_abs_diff_eq!(e.evaluate(&Point3::new(2.0, 0.0, 0.0)), 0.0, epsilon = 1e-6);
        assert_abs_diff_eq!(e.evaluate(&Point3::new(0.0, 3.0, 0.0)), 0.0, epsilon = 1e-6);
        assert_abs_diff_eq!(e.evaluate(&Point3::new(0.0, 0.0, 4.0)), 0.0, epsilon = 1e-6);
    }

    #[test]
    fn ellipsoid_sphere_equivalence() {
        // When all radii are equal, should match sphere SDF
        let e = FieldNode::Ellipsoid {
            radii: nalgebra::Vector3::new(3.0, 3.0, 3.0),
        };
        let s = FieldNode::Sphere { radius: 3.0 };
        let test_points = [
            Point3::origin(),
            Point3::new(3.0, 0.0, 0.0),
            Point3::new(5.0, 0.0, 0.0),
            Point3::new(1.0, 1.0, 1.0),
        ];
        for p in &test_points {
            assert_abs_diff_eq!(e.evaluate(p), s.evaluate(p), epsilon = 1e-6);
        }
    }

    // ── Torus ────────────────────────────────────────────────────────

    #[test]
    fn torus_center_of_tube_is_zero() {
        let t = FieldNode::Torus {
            major: 5.0,
            minor: 1.0,
        };
        // Point on the ring at (5, 0, 0) should be on tube center -> d = -minor
        assert_abs_diff_eq!(t.evaluate(&Point3::new(5.0, 0.0, 0.0)), -1.0, epsilon = EPS);
    }

    #[test]
    fn torus_on_outer_surface_is_zero() {
        let t = FieldNode::Torus {
            major: 5.0,
            minor: 1.0,
        };
        assert_abs_diff_eq!(t.evaluate(&Point3::new(6.0, 0.0, 0.0)), 0.0, epsilon = EPS);
    }

    #[test]
    fn torus_on_inner_surface_is_zero() {
        let t = FieldNode::Torus {
            major: 5.0,
            minor: 1.0,
        };
        assert_abs_diff_eq!(t.evaluate(&Point3::new(4.0, 0.0, 0.0)), 0.0, epsilon = EPS);
    }

    #[test]
    fn torus_origin_is_positive() {
        let t = FieldNode::Torus {
            major: 5.0,
            minor: 1.0,
        };
        // Origin: q_xy = sqrt(0) - 5 = -5, dist = sqrt(25 + 0) - 1 = 4
        assert_abs_diff_eq!(t.evaluate(&Point3::origin()), 4.0, epsilon = EPS);
    }

    // ── Cone ─────────────────────────────────────────────────────────

    #[test]
    fn cone_apex_is_zero() {
        let c = FieldNode::Cone {
            radius: 2.0,
            height: 4.0,
        };
        assert_abs_diff_eq!(c.evaluate(&Point3::origin()), 0.0, epsilon = EPS);
    }

    #[test]
    fn cone_base_center_is_negative() {
        let c = FieldNode::Cone {
            radius: 2.0,
            height: 4.0,
        };
        // Base center at (0, 0, -4) — should be inside (negative)
        assert!(c.evaluate(&Point3::new(0.0, 0.0, -4.0)) <= 0.0);
    }

    #[test]
    fn cone_outside_is_positive() {
        let c = FieldNode::Cone {
            radius: 2.0,
            height: 4.0,
        };
        assert!(c.evaluate(&Point3::new(5.0, 0.0, -2.0)) > 0.0);
    }

    #[test]
    fn cone_above_apex_is_positive() {
        let c = FieldNode::Cone {
            radius: 2.0,
            height: 4.0,
        };
        assert!(c.evaluate(&Point3::new(0.0, 0.0, 1.0)) > 0.0);
    }

    // ── Plane ────────────────────────────────────────────────────────

    #[test]
    fn plane_on_surface_is_zero() {
        let pl = FieldNode::Plane {
            normal: nalgebra::Vector3::z(),
            offset: 0.0,
        };
        assert_abs_diff_eq!(pl.evaluate(&Point3::new(5.0, 3.0, 0.0)), 0.0, epsilon = EPS);
    }

    #[test]
    fn plane_above_is_positive() {
        let pl = FieldNode::Plane {
            normal: nalgebra::Vector3::z(),
            offset: 0.0,
        };
        assert!(pl.evaluate(&Point3::new(0.0, 0.0, 1.0)) > 0.0);
    }

    #[test]
    fn plane_below_is_negative() {
        let pl = FieldNode::Plane {
            normal: nalgebra::Vector3::z(),
            offset: 0.0,
        };
        assert!(pl.evaluate(&Point3::new(0.0, 0.0, -1.0)) < 0.0);
    }

    #[test]
    fn plane_with_offset() {
        let pl = FieldNode::Plane {
            normal: nalgebra::Vector3::z(),
            offset: 3.0,
        };
        assert_abs_diff_eq!(pl.evaluate(&Point3::new(0.0, 0.0, 3.0)), 0.0, epsilon = EPS);
        assert_abs_diff_eq!(pl.evaluate(&Point3::new(0.0, 0.0, 5.0)), 2.0, epsilon = EPS);
    }

    // ── Union ───────────────────────────────────────────────────────

    #[test]
    fn union_picks_smaller_value() {
        let a = FieldNode::Sphere { radius: 2.0 };
        let b = FieldNode::Sphere { radius: 3.0 };
        let u = FieldNode::Union(Box::new(a), Box::new(b));
        // At origin: min(-2, -3) = -3
        assert_abs_diff_eq!(u.evaluate(&Point3::origin()), -3.0, epsilon = EPS);
    }

    #[test]
    fn union_is_commutative() {
        let a = FieldNode::Sphere { radius: 2.0 };
        let b = FieldNode::Sphere { radius: 3.0 };
        let u1 = FieldNode::Union(Box::new(a.clone()), Box::new(b.clone()));
        let u2 = FieldNode::Union(Box::new(b), Box::new(a));
        let p = Point3::new(2.5, 0.0, 0.0);
        assert_abs_diff_eq!(u1.evaluate(&p), u2.evaluate(&p), epsilon = EPS);
    }

    #[test]
    fn union_inside_either_is_inside() {
        // Two non-overlapping spheres
        let a = FieldNode::Translate(
            Box::new(FieldNode::Sphere { radius: 1.0 }),
            Vector3::new(-3.0, 0.0, 0.0),
        );
        let b = FieldNode::Translate(
            Box::new(FieldNode::Sphere { radius: 1.0 }),
            Vector3::new(3.0, 0.0, 0.0),
        );
        let u = FieldNode::Union(Box::new(a), Box::new(b));
        assert!(u.evaluate(&Point3::new(-3.0, 0.0, 0.0)) < 0.0); // inside a
        assert!(u.evaluate(&Point3::new(3.0, 0.0, 0.0)) < 0.0); // inside b
        assert!(u.evaluate(&Point3::new(0.0, 0.0, 0.0)) > 0.0); // outside both
    }

    // ── Subtract ────────────────────────────────────────────────────

    #[test]
    fn subtract_removes_second() {
        // Big sphere minus small sphere at origin
        let big = FieldNode::Sphere { radius: 5.0 };
        let small = FieldNode::Sphere { radius: 2.0 };
        let sub = FieldNode::Subtract(Box::new(big), Box::new(small));
        // Origin: inside big (-5) but inside small (-2), so max(-5, 2) = 2 > 0 (outside)
        assert!(sub.evaluate(&Point3::origin()) > 0.0);
        // On surface of small sphere (just outside): inside big, outside small
        assert!(sub.evaluate(&Point3::new(2.5, 0.0, 0.0)) < 0.0);
    }

    #[test]
    fn subtract_is_not_commutative() {
        let a = FieldNode::Sphere { radius: 5.0 };
        let b = FieldNode::Sphere { radius: 2.0 };
        let sub_ab = FieldNode::Subtract(Box::new(a.clone()), Box::new(b.clone()));
        let sub_ba = FieldNode::Subtract(Box::new(b), Box::new(a));
        let p = Point3::new(3.0, 0.0, 0.0);
        // a-b at (3,0,0): max(-2, 2) = 2... wait let me recalculate.
        // a = sphere(5): eval at (3,0,0) = 3-5 = -2
        // b = sphere(2): eval at (3,0,0) = 3-2 = 1
        // a-b: max(-2, -1) = -1 (inside)
        // b-a: max(1, 2) = 2 (outside)
        assert!(sub_ab.evaluate(&p) < 0.0);
        assert!(sub_ba.evaluate(&p) > 0.0);
    }

    // ── Intersect ───────────────────────────────────────────────────

    #[test]
    fn intersect_requires_both_inside() {
        let a = FieldNode::Sphere { radius: 5.0 };
        let b = FieldNode::Sphere { radius: 2.0 };
        let inter = FieldNode::Intersect(Box::new(a), Box::new(b));
        // At origin: max(-5, -2) = -2 (inside both)
        assert_abs_diff_eq!(inter.evaluate(&Point3::origin()), -2.0, epsilon = EPS);
        // At (3,0,0): max(-2, 1) = 1 (outside small sphere)
        assert!(inter.evaluate(&Point3::new(3.0, 0.0, 0.0)) > 0.0);
    }

    #[test]
    fn intersect_is_commutative() {
        let a = FieldNode::Sphere { radius: 5.0 };
        let b = FieldNode::Sphere { radius: 2.0 };
        let i1 = FieldNode::Intersect(Box::new(a.clone()), Box::new(b.clone()));
        let i2 = FieldNode::Intersect(Box::new(b), Box::new(a));
        let p = Point3::new(1.0, 1.0, 0.0);
        assert_abs_diff_eq!(i1.evaluate(&p), i2.evaluate(&p), epsilon = EPS);
    }

    // ── Smooth Union ────────────────────────────────────────────────

    #[test]
    fn smooth_union_adds_material_in_blend_region() {
        let a = FieldNode::Sphere { radius: 2.0 };
        let b = FieldNode::Translate(
            Box::new(FieldNode::Sphere { radius: 2.0 }),
            Vector3::new(3.0, 0.0, 0.0),
        );
        let k = 1.0;
        let su = FieldNode::SmoothUnion(Box::new(a.clone()), Box::new(b.clone()), k);
        let sharp = FieldNode::Union(Box::new(a), Box::new(b));
        // In the blend region between the two spheres, smooth union should
        // produce a smaller (more negative or less positive) value than sharp
        let p = Point3::new(1.5, 0.0, 0.0);
        assert!(su.evaluate(&p) <= sharp.evaluate(&p) + EPS);
    }

    #[test]
    fn smooth_union_far_from_blend_matches_sharp() {
        let a = FieldNode::Sphere { radius: 2.0 };
        let b = FieldNode::Translate(
            Box::new(FieldNode::Sphere { radius: 2.0 }),
            Vector3::new(10.0, 0.0, 0.0),
        );
        let su = FieldNode::SmoothUnion(Box::new(a.clone()), Box::new(b.clone()), 0.5);
        let sharp = FieldNode::Union(Box::new(a), Box::new(b));
        // Far from blend region: should be nearly identical
        let p = Point3::origin();
        assert_abs_diff_eq!(su.evaluate(&p), sharp.evaluate(&p), epsilon = 1e-6);
    }

    #[test]
    fn smooth_union_max_correction_is_k_over_4() {
        // When a ≈ b, correction is at most k/4
        let a = FieldNode::Sphere { radius: 3.0 };
        let b = FieldNode::Sphere { radius: 3.0 }; // identical
        let k = 2.0;
        let su = FieldNode::SmoothUnion(Box::new(a.clone()), Box::new(b.clone()), k);
        let sharp = FieldNode::Union(Box::new(a), Box::new(b));
        let p = Point3::origin();
        // sharp = -3, smooth should be -3 - k/4 = -3.5
        let diff = sharp.evaluate(&p) - su.evaluate(&p);
        assert!(diff >= -EPS, "smooth union should not exceed sharp union");
        assert!(diff <= k / 4.0 + EPS, "correction should be at most k/4");
    }

    // ── Smooth Subtract ─────────────────────────────────────────────

    #[test]
    fn smooth_subtract_removes_with_blend() {
        let big = FieldNode::Sphere { radius: 5.0 };
        let small = FieldNode::Sphere { radius: 2.0 };
        let ss = FieldNode::SmoothSubtract(Box::new(big), Box::new(small), 1.0);
        // Origin: big is deep inside (-5), small is deep inside (-2).
        // Sharp subtract: max(-5, 2) = 2. Smooth should be close.
        assert!(ss.evaluate(&Point3::origin()) > 0.0);
        // Far from subtraction zone: should still be inside
        assert!(ss.evaluate(&Point3::new(4.0, 0.0, 0.0)) < 0.0);
    }

    #[test]
    fn smooth_subtract_degenerates_to_sharp_when_far() {
        let big = FieldNode::Sphere { radius: 5.0 };
        let small = FieldNode::Translate(
            Box::new(FieldNode::Sphere { radius: 1.0 }),
            Vector3::new(0.0, 0.0, 0.0),
        );
        let k = 0.1;
        let ss = FieldNode::SmoothSubtract(Box::new(big.clone()), Box::new(small.clone()), k);
        let sharp = FieldNode::Subtract(Box::new(big), Box::new(small));
        // Far from blend region
        let p = Point3::new(4.0, 0.0, 0.0);
        assert_abs_diff_eq!(ss.evaluate(&p), sharp.evaluate(&p), epsilon = 1e-3);
    }

    // ── Smooth Intersect ────────────────────────────────────────────

    #[test]
    fn smooth_intersect_removes_material_in_blend() {
        let a = FieldNode::Sphere { radius: 3.0 };
        let b = FieldNode::Sphere { radius: 3.0 };
        let k = 2.0;
        let si = FieldNode::SmoothIntersect(Box::new(a.clone()), Box::new(b.clone()), k);
        let sharp = FieldNode::Intersect(Box::new(a), Box::new(b));
        // Smooth intersect removes material: field value should be ≥ sharp
        let p = Point3::origin();
        assert!(si.evaluate(&p) >= sharp.evaluate(&p) - EPS);
    }

    #[test]
    fn smooth_intersect_matches_sharp_when_far() {
        let a = FieldNode::Sphere { radius: 5.0 };
        let b = FieldNode::Cuboid {
            half_extents: Vector3::new(2.0, 2.0, 2.0),
        };
        let k = 0.1;
        let si = FieldNode::SmoothIntersect(Box::new(a.clone()), Box::new(b.clone()), k);
        let sharp = FieldNode::Intersect(Box::new(a), Box::new(b));
        // Deep inside intersection
        let p = Point3::origin();
        assert_abs_diff_eq!(si.evaluate(&p), sharp.evaluate(&p), epsilon = 0.1);
    }

    // ── Smooth Union All ────────────────────────────────────────────

    #[test]
    fn smooth_union_all_single_element() {
        let a = FieldNode::Sphere { radius: 2.0 };
        let sua = FieldNode::SmoothUnionAll(vec![a.clone()], 1.0);
        let p = Point3::origin();
        assert_abs_diff_eq!(sua.evaluate(&p), a.evaluate(&p), epsilon = EPS);
    }

    #[test]
    fn smooth_union_all_is_order_independent() {
        let sa = FieldNode::Sphere { radius: 2.0 };
        let sb = FieldNode::Translate(
            Box::new(FieldNode::Sphere { radius: 2.0 }),
            Vector3::new(3.0, 0.0, 0.0),
        );
        let sc = FieldNode::Translate(
            Box::new(FieldNode::Sphere { radius: 2.0 }),
            Vector3::new(0.0, 3.0, 0.0),
        );
        let blend = 1.0;
        let abc = FieldNode::SmoothUnionAll(vec![sa.clone(), sb.clone(), sc.clone()], blend);
        let bca = FieldNode::SmoothUnionAll(vec![sb.clone(), sc.clone(), sa.clone()], blend);
        let cab = FieldNode::SmoothUnionAll(vec![sc, sa, sb], blend);
        let pt = Point3::new(1.0, 1.0, 0.0);
        assert_abs_diff_eq!(abc.evaluate(&pt), bca.evaluate(&pt), epsilon = EPS);
        assert_abs_diff_eq!(abc.evaluate(&pt), cab.evaluate(&pt), epsilon = EPS);
    }

    #[test]
    fn smooth_union_all_adds_material() {
        let a = FieldNode::Sphere { radius: 2.0 };
        let b = FieldNode::Translate(
            Box::new(FieldNode::Sphere { radius: 2.0 }),
            Vector3::new(3.0, 0.0, 0.0),
        );
        let k = 1.0;
        let sua = FieldNode::SmoothUnionAll(vec![a.clone(), b.clone()], k);
        let sharp = FieldNode::Union(Box::new(a), Box::new(b));
        let p = Point3::new(1.5, 0.0, 0.0);
        assert!(sua.evaluate(&p) <= sharp.evaluate(&p) + EPS);
    }

    // ── Translate ───────────────────────────────────────────────────

    #[test]
    fn translate_moves_shape() {
        let s = FieldNode::Sphere { radius: 1.0 };
        let t = FieldNode::Translate(Box::new(s), Vector3::new(5.0, 0.0, 0.0));
        // Center of translated sphere is at (5, 0, 0)
        assert_abs_diff_eq!(t.evaluate(&Point3::new(5.0, 0.0, 0.0)), -1.0, epsilon = EPS);
        // Origin should be 4 units from surface
        assert_abs_diff_eq!(t.evaluate(&Point3::origin()), 4.0, epsilon = EPS);
    }

    #[test]
    fn translate_preserves_sdf() {
        let s = FieldNode::Sphere { radius: 2.0 };
        let t = FieldNode::Translate(Box::new(s), Vector3::new(3.0, 4.0, 0.0));
        // Distance from origin to center (3,4,0) is 5. SDF = 5 - 2 = 3.
        assert_abs_diff_eq!(t.evaluate(&Point3::origin()), 3.0, epsilon = EPS);
    }

    // ── Rotate ──────────────────────────────────────────────────────

    #[test]
    fn rotate_90_degrees_around_z() {
        // Cuboid at origin, 1x2x1 half-extents. Rotate 90° around Z.
        let c = FieldNode::Cuboid {
            half_extents: Vector3::new(1.0, 2.0, 1.0),
        };
        let rot = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), PI / 2.0);
        let r = FieldNode::Rotate(Box::new(c), rot);
        // After 90° rotation: half_extents become (2, 1, 1) in world space.
        // Point (1.5, 0, 0): was outside x-extent (1.0), now inside rotated x-extent (2.0)
        assert!(r.evaluate(&Point3::new(1.5, 0.0, 0.0)) < 0.0);
        // Point (0, 1.5, 0): was inside y-extent (2.0), now outside rotated y-extent (1.0)
        assert!(r.evaluate(&Point3::new(0.0, 1.5, 0.0)) > 0.0);
    }

    #[test]
    fn rotate_preserves_sphere() {
        // Sphere is rotationally symmetric
        let s = FieldNode::Sphere { radius: 3.0 };
        let rot = UnitQuaternion::from_axis_angle(&Vector3::y_axis(), 1.234);
        let r = FieldNode::Rotate(Box::new(s.clone()), rot);
        let test_points = [
            Point3::origin(),
            Point3::new(3.0, 0.0, 0.0),
            Point3::new(1.0, 1.0, 1.0),
            Point3::new(5.0, 0.0, 0.0),
        ];
        for p in &test_points {
            assert_abs_diff_eq!(r.evaluate(p), s.evaluate(p), epsilon = EPS);
        }
    }

    // ── Scale Uniform ───────────────────────────────────────────────

    #[test]
    fn scale_doubles_size() {
        let s = FieldNode::Sphere { radius: 1.0 };
        let scaled = FieldNode::ScaleUniform(Box::new(s), 2.0);
        // Effective radius is 2.0
        assert_abs_diff_eq!(scaled.evaluate(&Point3::origin()), -2.0, epsilon = EPS);
        assert_abs_diff_eq!(
            scaled.evaluate(&Point3::new(2.0, 0.0, 0.0)),
            0.0,
            epsilon = EPS
        );
        assert_abs_diff_eq!(
            scaled.evaluate(&Point3::new(3.0, 0.0, 0.0)),
            1.0,
            epsilon = EPS
        );
    }

    #[test]
    fn scale_preserves_sdf_property() {
        let c = FieldNode::Cuboid {
            half_extents: Vector3::new(1.0, 1.0, 1.0),
        };
        let scaled = FieldNode::ScaleUniform(Box::new(c), 3.0);
        // Effective half-extents are (3, 3, 3)
        assert_abs_diff_eq!(scaled.evaluate(&Point3::origin()), -3.0, epsilon = EPS);
        assert_abs_diff_eq!(
            scaled.evaluate(&Point3::new(3.0, 0.0, 0.0)),
            0.0,
            epsilon = EPS
        );
    }

    // ── Mirror ──────────────────────────────────────────────────────

    #[test]
    fn mirror_x_reflects_across_yz_plane() {
        // Sphere at (3, 0, 0), mirrored across YZ plane
        let s = FieldNode::Translate(
            Box::new(FieldNode::Sphere { radius: 1.0 }),
            Vector3::new(3.0, 0.0, 0.0),
        );
        let m = FieldNode::Mirror(Box::new(s), Vector3::new(1.0, 0.0, 0.0));
        // Point at (3, 0, 0): positive side, should be inside sphere center
        assert_abs_diff_eq!(m.evaluate(&Point3::new(3.0, 0.0, 0.0)), -1.0, epsilon = EPS);
        // Point at (-3, 0, 0): reflected to (3, 0, 0), should also be inside
        assert_abs_diff_eq!(
            m.evaluate(&Point3::new(-3.0, 0.0, 0.0)),
            -1.0,
            epsilon = EPS
        );
        // Origin: reflected to origin, which is 2 units from sphere center
        assert_abs_diff_eq!(m.evaluate(&Point3::origin()), 2.0, epsilon = EPS);
    }

    #[test]
    fn mirror_positive_side_unchanged() {
        let s = FieldNode::Translate(
            Box::new(FieldNode::Sphere { radius: 1.0 }),
            Vector3::new(3.0, 0.0, 0.0),
        );
        let m = FieldNode::Mirror(Box::new(s.clone()), Vector3::new(1.0, 0.0, 0.0));
        // Points on the positive x side should evaluate identically to the original
        let positive_points = [
            Point3::new(3.0, 0.0, 0.0),
            Point3::new(4.0, 0.0, 0.0),
            Point3::new(2.0, 1.0, 0.0),
        ];
        for p in &positive_points {
            assert_abs_diff_eq!(m.evaluate(p), s.evaluate(p), epsilon = EPS);
        }
    }

    // ── Composed trees ──────────────────────────────────────────────

    #[test]
    fn union_of_translated_spheres() {
        let a = FieldNode::Translate(
            Box::new(FieldNode::Sphere { radius: 1.0 }),
            Vector3::new(-2.0, 0.0, 0.0),
        );
        let b = FieldNode::Translate(
            Box::new(FieldNode::Sphere { radius: 1.0 }),
            Vector3::new(2.0, 0.0, 0.0),
        );
        let u = FieldNode::Union(Box::new(a), Box::new(b));
        assert!(u.evaluate(&Point3::new(-2.0, 0.0, 0.0)) < 0.0);
        assert!(u.evaluate(&Point3::new(2.0, 0.0, 0.0)) < 0.0);
        assert!(u.evaluate(&Point3::origin()) > 0.0);
    }

    #[test]
    fn subtract_then_translate() {
        let big = FieldNode::Sphere { radius: 5.0 };
        let hole = FieldNode::Sphere { radius: 2.0 };
        let hollowed = FieldNode::Subtract(Box::new(big), Box::new(hole));
        let moved = FieldNode::Translate(Box::new(hollowed), Vector3::new(10.0, 0.0, 0.0));
        // Center of moved hollow sphere is at (10, 0, 0) — hole is there
        assert!(moved.evaluate(&Point3::new(10.0, 0.0, 0.0)) > 0.0);
        // Shell region
        assert!(moved.evaluate(&Point3::new(13.0, 0.0, 0.0)) < 0.0);
    }

    #[test]
    fn rotate_then_scale() {
        let c = FieldNode::Cuboid {
            half_extents: Vector3::new(1.0, 2.0, 1.0),
        };
        let rot = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), PI / 2.0);
        let rotated = FieldNode::Rotate(Box::new(c), rot);
        let scaled = FieldNode::ScaleUniform(Box::new(rotated), 2.0);
        // Original: 1x2x1. After 90° Z rotation: 2x1x1. After 2x scale: 4x2x2.
        assert!(scaled.evaluate(&Point3::new(3.5, 0.0, 0.0)) < 0.0);
        assert!(scaled.evaluate(&Point3::new(4.5, 0.0, 0.0)) > 0.0);
        assert!(scaled.evaluate(&Point3::new(0.0, 1.5, 0.0)) < 0.0);
        assert!(scaled.evaluate(&Point3::new(0.0, 2.5, 0.0)) > 0.0);
    }

    // ── Shell ──────────────────────────────────────────────────────────

    #[test]
    fn shell_sphere_wall_thickness() {
        // Shell a sphere of radius 5 with thickness 1.
        // Field = |dist| - thickness = ||p|-5| - 1.
        // At origin: ||0|-5|-1 = |5|-1 = 4 (inside the wall? No, 4 > 0 = outside wall)
        // Wait: origin is deep inside the sphere: dist = -5.
        // |dist| = 5, shell = 5 - 1 = 4. Positive = outside the shell wall. Correct.
        let s = FieldNode::Shell(Box::new(FieldNode::Sphere { radius: 5.0 }), 1.0);
        assert!(
            s.evaluate(&Point3::origin()) > 0.0,
            "origin should be outside shell"
        );

        // On the original surface (r=5): |0| - 1 = -1 (inside wall)
        assert_abs_diff_eq!(s.evaluate(&Point3::new(5.0, 0.0, 0.0)), -1.0, epsilon = EPS);

        // At r=4 (inner surface): |(-1)| - 1 = 0
        assert_abs_diff_eq!(s.evaluate(&Point3::new(4.0, 0.0, 0.0)), 0.0, epsilon = EPS);

        // At r=6 (outer surface): |1| - 1 = 0
        assert_abs_diff_eq!(s.evaluate(&Point3::new(6.0, 0.0, 0.0)), 0.0, epsilon = EPS);

        // At r=4.5 (inside wall, inner side): |(-0.5)| - 1 = -0.5
        assert_abs_diff_eq!(s.evaluate(&Point3::new(4.5, 0.0, 0.0)), -0.5, epsilon = EPS);

        // At r=5.5 (inside wall, outer side): |0.5| - 1 = -0.5
        assert_abs_diff_eq!(s.evaluate(&Point3::new(5.5, 0.0, 0.0)), -0.5, epsilon = EPS);
    }

    // ── Round ──────────────────────────────────────────────────────────

    #[test]
    fn round_cuboid_shifts_surface() {
        // Round a unit cube by 0.2 — surface moves inward by 0.2.
        let c = FieldNode::Cuboid {
            half_extents: Vector3::new(1.0, 1.0, 1.0),
        };
        let r = FieldNode::Round(Box::new(c), 0.2);

        // Origin: cuboid SDF = -1.0, rounded = -1.0 - 0.2 = -1.2
        assert_abs_diff_eq!(r.evaluate(&Point3::origin()), -1.2, epsilon = EPS);

        // On the original face (1,0,0): cuboid SDF = 0, rounded = -0.2 (now inside)
        assert_abs_diff_eq!(r.evaluate(&Point3::new(1.0, 0.0, 0.0)), -0.2, epsilon = EPS);

        // At (1.2, 0, 0): cuboid SDF = 0.2, rounded = 0 (new surface)
        assert_abs_diff_eq!(r.evaluate(&Point3::new(1.2, 0.0, 0.0)), 0.0, epsilon = EPS);
    }

    // ── Offset ─────────────────────────────────────────────────────────

    #[test]
    fn offset_grow_sphere() {
        // Offset sphere(3) by +1 → effective radius 4.
        let s = FieldNode::Offset(Box::new(FieldNode::Sphere { radius: 3.0 }), 1.0);
        assert_abs_diff_eq!(s.evaluate(&Point3::origin()), -4.0, epsilon = EPS);
        assert_abs_diff_eq!(s.evaluate(&Point3::new(4.0, 0.0, 0.0)), 0.0, epsilon = EPS);
    }

    #[test]
    fn offset_shrink_sphere() {
        // Offset sphere(3) by -1 → effective radius 2.
        let s = FieldNode::Offset(Box::new(FieldNode::Sphere { radius: 3.0 }), -1.0);
        assert_abs_diff_eq!(s.evaluate(&Point3::origin()), -2.0, epsilon = EPS);
        assert_abs_diff_eq!(s.evaluate(&Point3::new(2.0, 0.0, 0.0)), 0.0, epsilon = EPS);
    }

    // ── Elongate ──────────────────────────────────────────────────────

    #[test]
    fn elongate_sphere_becomes_capsule_like() {
        // Elongate a sphere(1) by (2, 0, 0) → stretches along X.
        // At (0,0,0): q = (0,0,0) - clamp(0, -2, 2) = 0, sphere(0) = -1
        let s = FieldNode::Elongate(
            Box::new(FieldNode::Sphere { radius: 1.0 }),
            Vector3::new(2.0, 0.0, 0.0),
        );
        assert_abs_diff_eq!(s.evaluate(&Point3::origin()), -1.0, epsilon = EPS);

        // At (2,0,0): q = (2-2,0,0) = (0,0,0), sphere(0) = -1
        assert_abs_diff_eq!(s.evaluate(&Point3::new(2.0, 0.0, 0.0)), -1.0, epsilon = EPS);

        // At (3,0,0): q = (3-2,0,0) = (1,0,0), sphere = |1|-1 = 0 (surface)
        assert_abs_diff_eq!(s.evaluate(&Point3::new(3.0, 0.0, 0.0)), 0.0, epsilon = EPS);

        // At (-3,0,0): q = (-3-(-2),0,0) = (-1,0,0), sphere = 0 (surface)
        assert_abs_diff_eq!(s.evaluate(&Point3::new(-3.0, 0.0, 0.0)), 0.0, epsilon = EPS);

        // Y direction not elongated: at (0,1,0) → surface
        assert_abs_diff_eq!(s.evaluate(&Point3::new(0.0, 1.0, 0.0)), 0.0, epsilon = EPS);

        // At (0,2,0): q = (0,2,0) → sphere = 2-1 = 1 (outside)
        assert_abs_diff_eq!(s.evaluate(&Point3::new(0.0, 2.0, 0.0)), 1.0, epsilon = EPS);
    }

    #[test]
    fn elongate_preserves_sdf() {
        // Elongated sphere: surface at x=±(h+r), y/z=±r
        let s = FieldNode::Elongate(
            Box::new(FieldNode::Sphere { radius: 2.0 }),
            Vector3::new(3.0, 0.0, 0.0),
        );
        // Surface at x = ±5 (elongation 3 + radius 2)
        assert_abs_diff_eq!(s.evaluate(&Point3::new(5.0, 0.0, 0.0)), 0.0, epsilon = EPS);
        assert_abs_diff_eq!(s.evaluate(&Point3::new(-5.0, 0.0, 0.0)), 0.0, epsilon = EPS);
    }

    // ── UserFn ─────────────────────────────────────────────────────────

    #[test]
    fn user_fn_evaluates_closure() {
        use crate::field_node::UserEvalFn;
        use cf_geometry::Aabb;
        use std::sync::Arc;

        // Custom sphere of radius 4 via UserFn
        let node = FieldNode::UserFn {
            eval: UserEvalFn(Arc::new(|p: Point3<f64>| p.coords.norm() - 4.0)),
            interval: None,
            bounds: Aabb::new(Point3::new(-5.0, -5.0, -5.0), Point3::new(5.0, 5.0, 5.0)),
        };

        assert_abs_diff_eq!(node.evaluate(&Point3::origin()), -4.0, epsilon = EPS);
        assert_abs_diff_eq!(
            node.evaluate(&Point3::new(4.0, 0.0, 0.0)),
            0.0,
            epsilon = EPS
        );
        assert!(node.evaluate(&Point3::new(6.0, 0.0, 0.0)) > 0.0);
    }
}
