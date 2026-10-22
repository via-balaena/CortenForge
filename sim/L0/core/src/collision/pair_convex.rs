//! Analytical pairwise collision for convex primitives (sphere, capsule, box).

use super::mj_primitive::{
    Frame, raw_capsule_capsule, raw_sphere_box, raw_sphere_capsule, sphere_sphere,
};
use crate::types::{Contact, GeomType, Model};
use nalgebra::{Matrix3, Vector3};

/// Sphere–sphere collision: MuJoCo's `mjc_SphereSphere`
/// ([`mj_primitive`](super::mj_primitive)). The frames are read where the
/// centres coincide: the normal is then the cross of the two z axes.
// The full per-geom pair context, so dispatch from the broadphase passes one uniform arg list.
#[allow(clippy::too_many_arguments)]
pub fn collide_sphere_sphere(
    model: &Model,
    geom1: usize,
    geom2: usize,
    pos1: Vector3<f64>,
    mat1: Matrix3<f64>,
    pos2: Vector3<f64>,
    mat2: Matrix3<f64>,
    size1: Vector3<f64>,
    size2: Vector3<f64>,
    margin: f64,
) -> Vec<Contact> {
    let sphere1 = Frame {
        pos: &pos1,
        mat: &mat1,
    };
    let sphere2 = Frame {
        pos: &pos2,
        mat: &mat2,
    };
    sphere_sphere(margin, sphere1, size1.x, sphere2, size2.x)
        .map(|c| c.contact(model, geom1, geom2, margin))
        .into_iter()
        .collect()
}

/// Capsule–capsule collision: MuJoCo's `mjraw_CapsuleCapsule`
/// ([`mj_primitive`](super::mj_primitive)), up to two contacts where the
/// axes are parallel (`|det|` below mjMINVAL), one otherwise.
// The full per-geom pair context, so dispatch from the broadphase passes one uniform arg list.
#[allow(clippy::too_many_arguments)]
pub fn collide_capsule_capsule(
    model: &Model,
    geom1: usize,
    geom2: usize,
    pos1: Vector3<f64>,
    mat1: Matrix3<f64>,
    pos2: Vector3<f64>,
    mat2: Matrix3<f64>,
    size1: Vector3<f64>,
    size2: Vector3<f64>,
    margin: f64,
) -> Vec<Contact> {
    let capsule1 = Frame {
        pos: &pos1,
        mat: &mat1,
    };
    let capsule2 = Frame {
        pos: &pos2,
        mat: &mat2,
    };
    raw_capsule_capsule(margin, capsule1, &size1, capsule2, &size2)
        .iter()
        .map(|c| c.contact(model, geom1, geom2, margin))
        .collect()
}

/// Sphere–capsule collision: MuJoCo's `mjraw_SphereCapsule`
/// ([`mj_primitive`](super::mj_primitive)), the sphere first.
// The full per-geom pair context, so dispatch from the broadphase passes one uniform arg list.
#[allow(clippy::too_many_arguments)]
pub fn collide_sphere_capsule(
    model: &Model,
    geom1: usize,
    geom2: usize,
    type1: GeomType,
    pos1: Vector3<f64>,
    mat1: Matrix3<f64>,
    pos2: Vector3<f64>,
    mat2: Matrix3<f64>,
    size1: Vector3<f64>,
    size2: Vector3<f64>,
    margin: f64,
) -> Vec<Contact> {
    let frame1 = Frame {
        pos: &pos1,
        mat: &mat1,
    };
    let frame2 = Frame {
        pos: &pos2,
        mat: &mat2,
    };
    let ((sphere_geom, sphere, radius), (capsule_geom, capsule, size)) =
        if type1 == GeomType::Sphere {
            ((geom1, frame1, size1.x), (geom2, frame2, size2))
        } else {
            ((geom2, frame2, size2.x), (geom1, frame1, size1))
        };
    raw_sphere_capsule(margin, sphere, radius, capsule, &size)
        .map(|c| c.contact(model, sphere_geom, capsule_geom, margin))
        .into_iter()
        .collect()
}

/// Sphere–box collision: MuJoCo's `mjraw_SphereBox`
/// ([`mj_primitive`](super::mj_primitive)), the sphere first.
// The full per-geom pair context, so dispatch from the broadphase passes one uniform arg list.
#[allow(clippy::too_many_arguments)]
pub fn collide_sphere_box(
    model: &Model,
    geom1: usize,
    geom2: usize,
    type1: GeomType,
    pos1: Vector3<f64>,
    mat1: Matrix3<f64>,
    pos2: Vector3<f64>,
    mat2: Matrix3<f64>,
    size1: Vector3<f64>,
    size2: Vector3<f64>,
    margin: f64,
) -> Vec<Contact> {
    let frame1 = Frame {
        pos: &pos1,
        mat: &mat1,
    };
    let frame2 = Frame {
        pos: &pos2,
        mat: &mat2,
    };
    let ((sphere_geom, sphere, radius), (box_geom, box_frame, size)) = if type1 == GeomType::Sphere
    {
        ((geom1, frame1, size1.x), (geom2, frame2, size2))
    } else {
        ((geom2, frame2, size2.x), (geom1, frame1, size1))
    };
    raw_sphere_box(margin, &sphere.pos(), radius, box_frame, &size)
        .map(|c| c.contact(model, sphere_geom, box_geom, margin))
        .into_iter()
        .collect()
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used, clippy::float_cmp)]
mod tests {
    use super::{
        collide_capsule_capsule, collide_sphere_box, collide_sphere_capsule, collide_sphere_sphere,
    };
    use crate::test_fixtures::collision::collision_geoms as make_model;
    use crate::types::GeomType;
    use approx::assert_relative_eq;
    use nalgebra::{Matrix3, UnitQuaternion, Vector3};

    const I: fn() -> Matrix3<f64> = Matrix3::identity;

    /// Rotation mapping local +Z onto world +X (about Y by 90°).
    fn z_to_x() -> Matrix3<f64> {
        UnitQuaternion::from_axis_angle(&Vector3::y_axis(), std::f64::consts::FRAC_PI_2)
            .to_rotation_matrix()
            .into_inner()
    }

    // ── sphere ↔ sphere ───────────────────────────────────────────────

    #[test]
    fn sphere_sphere_penetration() {
        // r1=0.3 @origin, r2=0.2 @0.4x → centers 0.4 apart, sum 0.5, pen 0.1.
        let mut m = make_model(2);
        m.geom_size[0] = Vector3::new(0.3, 0.0, 0.0);
        m.geom_size[1] = Vector3::new(0.2, 0.0, 0.0);
        let c = collide_sphere_sphere(
            &m,
            0,
            1,
            Vector3::zeros(),
            Matrix3::identity(),
            Vector3::new(0.4, 0.0, 0.0),
            Matrix3::identity(),
            m.geom_size[0],
            m.geom_size[1],
            0.0,
        );
        assert_eq!(c.len(), 1);
        assert_relative_eq!(c[0].depth, 0.1, epsilon = 1e-9);
        assert_relative_eq!(c[0].normal, Vector3::x(), epsilon = 1e-9);
    }

    #[test]
    fn sphere_sphere_separated_returns_empty() {
        let mut m = make_model(2);
        m.geom_size[0] = Vector3::new(0.3, 0.0, 0.0);
        m.geom_size[1] = Vector3::new(0.2, 0.0, 0.0);
        let c = collide_sphere_sphere(
            &m,
            0,
            1,
            Vector3::zeros(),
            Matrix3::identity(),
            Vector3::new(1.0, 0.0, 0.0),
            Matrix3::identity(),
            m.geom_size[0],
            m.geom_size[1],
            0.0,
        );
        assert!(c.is_empty());
    }

    /// Coincident centres, both frames upright: the normal is the cross of
    /// the two z axes, which is 0, so MuJoCo's `mju_normalize3` makes it +x
    /// (`mjraw_SphereSphere`); the depth is the radii's sum.
    #[test]
    fn sphere_sphere_coincident_takes_the_x_normal() {
        let mut m = make_model(2);
        m.geom_size[0] = Vector3::new(0.3, 0.0, 0.0);
        m.geom_size[1] = Vector3::new(0.2, 0.0, 0.0);
        let c = collide_sphere_sphere(
            &m,
            0,
            1,
            Vector3::zeros(),
            Matrix3::identity(),
            Vector3::zeros(),
            Matrix3::identity(),
            m.geom_size[0],
            m.geom_size[1],
            0.0,
        );
        assert_eq!(c.len(), 1);
        assert_relative_eq!(c[0].depth, 0.5, epsilon = 1e-9);
        assert_relative_eq!(c[0].normal, Vector3::x(), epsilon = 1e-9);
    }

    // ── capsule ↔ capsule ─────────────────────────────────────────────

    #[test]
    fn capsule_capsule_parallel_two_contacts() {
        // Both axis +Z, r=0.2, offset 0.3 in X → 0.3 gap between axes, sum 0.4,
        // pen 0.1. Parallel path returns a contact per cap-1 endpoint.
        let mut m = make_model(2);
        m.geom_type[0] = GeomType::Capsule;
        m.geom_size[0] = Vector3::new(0.2, 0.3, 0.0);
        m.geom_type[1] = GeomType::Capsule;
        m.geom_size[1] = Vector3::new(0.2, 0.3, 0.0);
        let c = collide_capsule_capsule(
            &m,
            0,
            1,
            Vector3::zeros(),
            I(),
            Vector3::new(0.3, 0.0, 0.0),
            I(),
            m.geom_size[0],
            m.geom_size[1],
            0.0,
        );
        assert_eq!(c.len(), 2);
        for contact in &c {
            assert_relative_eq!(contact.depth, 0.1, epsilon = 1e-9);
            assert_relative_eq!(contact.normal, Vector3::x(), epsilon = 1e-9);
        }
    }

    #[test]
    fn capsule_capsule_perpendicular_one_contact() {
        // cap1 axis +Z @origin; cap2 axis +X @ z=0.5. Closest features: cap1 top
        // (0,0,0.3) and cap2 (0,0,0.5); gap 0.2, sum 0.4, pen 0.2, normal +Z.
        let mut m = make_model(2);
        m.geom_type[0] = GeomType::Capsule;
        m.geom_size[0] = Vector3::new(0.2, 0.3, 0.0);
        m.geom_type[1] = GeomType::Capsule;
        m.geom_size[1] = Vector3::new(0.2, 0.3, 0.0);
        let c = collide_capsule_capsule(
            &m,
            0,
            1,
            Vector3::zeros(),
            I(),
            Vector3::new(0.0, 0.0, 0.5),
            z_to_x(),
            m.geom_size[0],
            m.geom_size[1],
            0.0,
        );
        assert_eq!(c.len(), 1);
        assert_relative_eq!(c[0].depth, 0.2, epsilon = 1e-9);
        assert_relative_eq!(c[0].normal, Vector3::z(), epsilon = 1e-9);
    }

    #[test]
    fn capsule_capsule_separated_returns_empty() {
        let mut m = make_model(2);
        m.geom_type[0] = GeomType::Capsule;
        m.geom_size[0] = Vector3::new(0.2, 0.3, 0.0);
        m.geom_type[1] = GeomType::Capsule;
        m.geom_size[1] = Vector3::new(0.2, 0.3, 0.0);
        let c = collide_capsule_capsule(
            &m,
            0,
            1,
            Vector3::zeros(),
            I(),
            Vector3::new(1.0, 0.0, 0.0),
            I(),
            m.geom_size[0],
            m.geom_size[1],
            0.0,
        );
        assert!(c.is_empty());
    }

    // ── sphere ↔ capsule ──────────────────────────────────────────────

    #[test]
    fn sphere_capsule_penetration() {
        // Sphere r=0.2 @0.3x; capsule r=0.15 hl=0.3 axis +Z @origin. Closest
        // capsule-segment point is the origin; gap 0.3, sum 0.35, pen 0.05.
        // Sphere is g1 → normal points sphere→capsule = -X.
        let mut m = make_model(2);
        m.geom_type[0] = GeomType::Sphere;
        m.geom_size[0] = Vector3::new(0.2, 0.0, 0.0);
        m.geom_type[1] = GeomType::Capsule;
        m.geom_size[1] = Vector3::new(0.15, 0.3, 0.0);
        let c = collide_sphere_capsule(
            &m,
            0,
            1,
            GeomType::Sphere,
            Vector3::new(0.3, 0.0, 0.0),
            I(),
            Vector3::zeros(),
            I(),
            m.geom_size[0],
            m.geom_size[1],
            0.0,
        );
        assert_eq!(c.len(), 1);
        assert_relative_eq!(c[0].depth, 0.05, epsilon = 1e-9);
        assert_relative_eq!(c[0].normal, -Vector3::x(), epsilon = 1e-9);
    }

    #[test]
    fn sphere_capsule_separated_returns_empty() {
        let mut m = make_model(2);
        m.geom_type[0] = GeomType::Sphere;
        m.geom_size[0] = Vector3::new(0.2, 0.0, 0.0);
        m.geom_type[1] = GeomType::Capsule;
        m.geom_size[1] = Vector3::new(0.15, 0.3, 0.0);
        let c = collide_sphere_capsule(
            &m,
            0,
            1,
            GeomType::Sphere,
            Vector3::new(1.0, 0.0, 0.0),
            I(),
            Vector3::zeros(),
            I(),
            m.geom_size[0],
            m.geom_size[1],
            0.0,
        );
        assert!(c.is_empty());
    }

    // ── sphere ↔ box ──────────────────────────────────────────────────

    #[test]
    fn sphere_box_face_penetration() {
        // Sphere r=0.2 @0.6x; box half 0.5 @origin. Closest face point (0.5,0,0);
        // gap 0.1, pen 0.1. Sphere g1 → normal sphere→box = -X.
        let mut m = make_model(2);
        m.geom_type[0] = GeomType::Sphere;
        m.geom_size[0] = Vector3::new(0.2, 0.0, 0.0);
        m.geom_type[1] = GeomType::Box;
        m.geom_size[1] = Vector3::new(0.5, 0.5, 0.5);
        let c = collide_sphere_box(
            &m,
            0,
            1,
            GeomType::Sphere,
            Vector3::new(0.6, 0.0, 0.0),
            I(),
            Vector3::zeros(),
            I(),
            m.geom_size[0],
            m.geom_size[1],
            0.0,
        );
        assert_eq!(c.len(), 1);
        assert_relative_eq!(c[0].depth, 0.1, epsilon = 1e-9);
        assert_relative_eq!(c[0].normal, -Vector3::x(), epsilon = 1e-9);
    }

    #[test]
    fn sphere_box_center_inside_deepest_axis() {
        // Sphere center inside box: dist 0, so the "deepest axis" branch runs.
        // Center @ (0.1, 0, 0) in a half-0.5 box → nearest face is +X, pen
        // = radius + (0.5 - 0.1) = 0.2 + 0.4 = 0.6, normal along X.
        let mut m = make_model(2);
        m.geom_type[0] = GeomType::Sphere;
        m.geom_size[0] = Vector3::new(0.2, 0.0, 0.0);
        m.geom_type[1] = GeomType::Box;
        m.geom_size[1] = Vector3::new(0.5, 0.5, 0.5);
        let c = collide_sphere_box(
            &m,
            0,
            1,
            GeomType::Sphere,
            Vector3::new(0.1, 0.0, 0.0),
            I(),
            Vector3::zeros(),
            I(),
            m.geom_size[0],
            m.geom_size[1],
            0.0,
        );
        assert_eq!(c.len(), 1);
        assert!(c[0].depth > 0.0);
        assert_relative_eq!(c[0].normal.x.abs(), 1.0, epsilon = 1e-9);
    }

    #[test]
    fn sphere_box_reversed_order_geom_dispatch() {
        // Box is geom 0 / type1, sphere geom 1: exercises the dispatcher's
        // `else` branch. The contact lists the sphere first, as MuJoCo's
        // `mjraw_SphereBox`, its normal from the sphere toward the box = -X.
        let mut m = make_model(2);
        m.geom_type[0] = GeomType::Box;
        m.geom_size[0] = Vector3::new(0.5, 0.5, 0.5);
        m.geom_type[1] = GeomType::Sphere;
        m.geom_size[1] = Vector3::new(0.2, 0.0, 0.0);
        let c = collide_sphere_box(
            &m,
            0,
            1,
            GeomType::Box,
            Vector3::zeros(),
            I(),
            Vector3::new(0.6, 0.0, 0.0),
            I(),
            m.geom_size[0],
            m.geom_size[1],
            0.0,
        );
        assert_eq!(c.len(), 1);
        assert_eq!(c[0].geom, [Some(1), Some(0)]);
        assert_relative_eq!(c[0].depth, 0.1, epsilon = 1e-9);
        assert_relative_eq!(c[0].normal, -Vector3::x(), epsilon = 1e-9);
    }

    #[test]
    fn sphere_box_separated_returns_empty() {
        let mut m = make_model(2);
        m.geom_type[0] = GeomType::Sphere;
        m.geom_size[0] = Vector3::new(0.2, 0.0, 0.0);
        m.geom_type[1] = GeomType::Box;
        m.geom_size[1] = Vector3::new(0.5, 0.5, 0.5);
        let c = collide_sphere_box(
            &m,
            0,
            1,
            GeomType::Sphere,
            Vector3::new(1.0, 0.0, 0.0),
            I(),
            Vector3::zeros(),
            I(),
            m.geom_size[0],
            m.geom_size[1],
            0.0,
        );
        assert!(c.is_empty());
    }
}
