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
        }
    }
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
}
