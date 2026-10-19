//! Analytical pairwise collision for cylinders, and box–box and capsule–box
//! through MuJoCo's ported functions (`mj_primitive`).

use super::mj_primitive::{Frame, box_box_filter, raw_box_box, raw_capsule_box, sphere_cylinder};
use super::narrow::{CAP_COLLISION_THRESHOLD, GEOM_EPSILON, make_contact_from_geoms};
use crate::forward::closest_points_segments;
use crate::types::{Contact, GeomType, Model};
use nalgebra::{Matrix3, Vector3};

/// Cylinder–sphere collision: MuJoCo's `mjc_SphereCylinder`
/// ([`mj_primitive`](super::mj_primitive)), the sphere first.
// The full per-geom pair context, so dispatch from the broadphase passes one uniform arg list.
#[allow(clippy::too_many_arguments)]
pub fn collide_cylinder_sphere(
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
    let ((sphere_geom, sphere, radius), (cylinder_geom, cylinder, size)) =
        if type1 == GeomType::Sphere {
            ((geom1, frame1, size1.x), (geom2, frame2, size2))
        } else {
            ((geom2, frame2, size2.x), (geom1, frame1, size1))
        };
    sphere_cylinder(margin, sphere, radius, cylinder, &size)
        .map(|c| c.contact(model, sphere_geom, cylinder_geom, margin))
        .into_iter()
        .collect()
}

/// Cylinder-capsule collision detection.
///
/// Computes collision between a cylinder and a capsule by finding the closest
/// points between the cylinder axis segment and the capsule axis segment,
/// then checking if the cylinder surface intersects the capsule's swept sphere.
///
/// # Limitations
///
/// This algorithm treats the cylinder's curved surface correctly but does not
/// handle collisions with the flat caps. For cap collisions (capsule directly
/// above/below cylinder), returns empty `Vec` to fall through to GJK/EPA.
///
/// Both shapes have their axis along local Z.
// Cylinder-sphere pair dispatcher takes the full per-geom context (poses, sizes, friction, margin).
#[allow(clippy::too_many_arguments)]
pub fn collide_cylinder_capsule(
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
    // Identify cylinder and capsule
    let (cyl_geom, cyl_pos, cyl_mat, cyl_size, cap_geom, cap_pos, cap_mat, cap_size) =
        if type1 == GeomType::Cylinder {
            (geom1, pos1, mat1, size1, geom2, pos2, mat2, size2)
        } else {
            (geom2, pos2, mat2, size2, geom1, pos1, mat1, size1)
        };

    let cyl_radius = cyl_size.x;
    let cyl_half_height = cyl_size.y;
    let cap_radius = cap_size.x;
    let cap_half_len = cap_size.y;

    let cyl_axis = cyl_mat.column(2).into_owned();
    let cap_axis = cap_mat.column(2).into_owned();

    // Cylinder axis segment endpoints
    let cyl_a = cyl_pos - cyl_axis * cyl_half_height;
    let cyl_b = cyl_pos + cyl_axis * cyl_half_height;

    // Capsule axis segment endpoints
    let cap_a = cap_pos - cap_axis * cap_half_len;
    let cap_b = cap_pos + cap_axis * cap_half_len;

    // Find closest points between the two axis segments
    let (cyl_closest, cap_closest) = closest_points_segments(cyl_a, cyl_b, cap_a, cap_b);

    // Vector from cylinder axis point to capsule axis point
    let diff = cap_closest - cyl_closest;
    let dist = diff.norm();

    if dist < GEOM_EPSILON {
        // Axes intersect or nearly intersect - degenerate case where analytical
        // solution is unreliable. Return None to fall through to GJK/EPA.
        return vec![];
    }

    let normal = diff / dist;

    // Check if this is a cap collision scenario:
    // If cyl_closest is at an endpoint AND normal points mostly along the axis,
    // we're hitting the flat cap, not the curved surface. Fall back to GJK/EPA.
    let cyl_closest_on_cap =
        (cyl_closest - cyl_a).norm() < GEOM_EPSILON || (cyl_closest - cyl_b).norm() < GEOM_EPSILON;
    let normal_along_axis = normal.dot(&cyl_axis).abs();
    if cyl_closest_on_cap && normal_along_axis > CAP_COLLISION_THRESHOLD {
        // Cap collision - this algorithm doesn't handle flat caps correctly
        return vec![];
    }

    // Closest point on cylinder surface (in direction of capsule)
    let cyl_surface = cyl_closest + normal * cyl_radius;

    // Distance from cylinder surface to capsule axis
    let surface_to_cap_dist = (cap_closest - cyl_surface).dot(&normal);
    let penetration = cap_radius - surface_to_cap_dist;

    if penetration <= -margin {
        return vec![];
    }

    // Contact position is between the two surfaces
    let contact_pos = cyl_surface + normal * (penetration * 0.5);

    // Order geoms consistently (lower index first)
    let (g1, g2, final_normal) = if cyl_geom < cap_geom {
        (cyl_geom, cap_geom, normal)
    } else {
        (cap_geom, cyl_geom, -normal)
    };

    vec![make_contact_from_geoms(
        model,
        contact_pos,
        final_normal,
        penetration,
        g1,
        g2,
        margin,
    )]
}

/// Capsule–box collision: MuJoCo's `mjraw_CapsuleBox`
/// ([`mj_primitive`](super::mj_primitive)), the capsule first, up to two
/// contacts.
// The full per-geom pair context, so dispatch from the broadphase passes one uniform arg list.
#[allow(clippy::too_many_arguments)]
pub fn collide_capsule_box(
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
    let ((capsule_geom, capsule, capsule_size), (box_geom, box_frame, box_size)) =
        if type1 == GeomType::Capsule {
            ((geom1, frame1, size1), (geom2, frame2, size2))
        } else {
            ((geom2, frame2, size2), (geom1, frame1, size1))
        };
    raw_capsule_box(margin, capsule, &capsule_size, box_frame, &box_size)
        .iter()
        .map(|c| c.contact(model, capsule_geom, box_geom, margin))
        .collect()
}

/// Box–box collision: MuJoCo's `mjc_BoxBox` and its driver's box–box filter
/// ([`mj_primitive`](super::mj_primitive)), up to MuJoCo's `mjMAXCONPAIR`
/// (50) contacts, the first box's geom first.
// The full per-geom pair context, so dispatch from the broadphase passes one uniform arg list.
#[allow(clippy::too_many_arguments)]
pub fn collide_box_box(
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
    let box1 = Frame {
        pos: &pos1,
        mat: &mat1,
    };
    let box2 = Frame {
        pos: &pos2,
        mat: &mat2,
    };
    let raw = raw_box_box(margin, box1, &size1, box2, &size2);
    box_box_filter(raw, margin, box1, &size1, box2, &size2)
        .iter()
        .map(|c| c.contact(model, geom1, geom2, margin))
        .collect()
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used, clippy::float_cmp)]
mod tests {
    use super::{
        collide_box_box, collide_capsule_box, collide_cylinder_capsule, collide_cylinder_sphere,
    };
    use crate::test_fixtures::collision::collision_geoms as make_model;
    use crate::types::GeomType;
    use approx::assert_relative_eq;
    use nalgebra::{Matrix3, UnitQuaternion, Vector3};

    const I: fn() -> Matrix3<f64> = Matrix3::identity;

    // ── cylinder ↔ sphere ─────────────────────────────────────────────

    #[test]
    fn cyl_sphere_side_penetration() {
        // Cylinder (r=0.3, hh=0.5) axis +Z at origin; sphere (r=0.2) beside it
        // at x=0.4. Closest surface point x=0.3, gap 0.1, penetration 0.2-0.1=0.1.
        let mut m = make_model(2);
        m.geom_type[0] = GeomType::Cylinder;
        m.geom_size[0] = Vector3::new(0.3, 0.5, 0.0);
        m.geom_type[1] = GeomType::Sphere;
        m.geom_size[1] = Vector3::new(0.2, 0.0, 0.0);

        let c = collide_cylinder_sphere(
            &m,
            0,
            1,
            GeomType::Cylinder,
            Vector3::zeros(),
            I(),
            Vector3::new(0.4, 0.0, 0.0),
            I(),
            m.geom_size[0],
            m.geom_size[1],
            0.0,
        );
        assert_eq!(c.len(), 1);
        assert_relative_eq!(c[0].depth, 0.1, epsilon = 1e-9);
        // The sphere is listed first, as MuJoCo's `mjc_SphereCylinder`, the
        // normal from it toward the cylinder (-X).
        assert_eq!((c[0].geom1, c[0].geom2), (1, 0));
        assert_relative_eq!(c[0].normal, -Vector3::x(), epsilon = 1e-9);
    }

    #[test]
    fn cyl_sphere_reversed_order_geom_dispatch() {
        // Sphere is geom 0 / type1, cylinder geom 1: the dispatcher's `if`
        // branch, the sphere first as in MuJoCo's type order. Same geometry as
        // the side-penetration case; the normal points from the sphere toward
        // the cylinder = -X.
        let mut m = make_model(2);
        m.geom_type[0] = GeomType::Sphere;
        m.geom_size[0] = Vector3::new(0.2, 0.0, 0.0);
        m.geom_type[1] = GeomType::Cylinder;
        m.geom_size[1] = Vector3::new(0.3, 0.5, 0.0);
        let c = collide_cylinder_sphere(
            &m,
            0,
            1,
            GeomType::Sphere,
            Vector3::new(0.4, 0.0, 0.0),
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
    fn cyl_sphere_separated_returns_empty() {
        let mut m = make_model(2);
        m.geom_type[0] = GeomType::Cylinder;
        m.geom_size[0] = Vector3::new(0.3, 0.5, 0.0);
        m.geom_size[1] = Vector3::new(0.2, 0.0, 0.0);
        let c = collide_cylinder_sphere(
            &m,
            0,
            1,
            GeomType::Cylinder,
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

    #[test]
    fn cyl_sphere_within_margin_emits_contact() {
        // Not overlapping (surface gap 0.05) but inside a 0.1 margin band, so a
        // contact with negative depth (a gap) is still emitted.
        let mut m = make_model(2);
        m.geom_type[0] = GeomType::Cylinder;
        m.geom_size[0] = Vector3::new(0.3, 0.5, 0.0);
        m.geom_size[1] = Vector3::new(0.2, 0.0, 0.0);
        let c = collide_cylinder_sphere(
            &m,
            0,
            1,
            GeomType::Cylinder,
            Vector3::zeros(),
            I(),
            Vector3::new(0.55, 0.0, 0.0),
            I(),
            m.geom_size[0],
            m.geom_size[1],
            0.1,
        );
        assert_eq!(c.len(), 1);
        assert_relative_eq!(c[0].depth, -0.05, epsilon = 1e-9);
    }

    #[test]
    fn cyl_sphere_cap_penetration() {
        // Sphere directly above the top cap: cap face at z=0.5, sphere center
        // z=0.6, gap 0.1, penetration 0.2-0.1=0.1; the sphere listed first,
        // the normal from it down into the cap, -Z.
        let mut m = make_model(2);
        m.geom_type[0] = GeomType::Cylinder;
        m.geom_size[0] = Vector3::new(0.3, 0.5, 0.0);
        m.geom_size[1] = Vector3::new(0.2, 0.0, 0.0);
        let c = collide_cylinder_sphere(
            &m,
            0,
            1,
            GeomType::Cylinder,
            Vector3::zeros(),
            I(),
            Vector3::new(0.0, 0.0, 0.6),
            I(),
            m.geom_size[0],
            m.geom_size[1],
            0.0,
        );
        assert_eq!(c.len(), 1);
        assert_eq!((c[0].geom1, c[0].geom2), (1, 0));
        assert_relative_eq!(c[0].depth, 0.1, epsilon = 1e-9);
        assert_relative_eq!(c[0].normal, -Vector3::z(), epsilon = 1e-9);
    }

    #[test]
    fn cyl_sphere_edge_contact() {
        // Sphere off the rim corner (r=0.3, hh=0.5) at (0.4, 0, 0.6): beyond both
        // the radius and the cap, so the closest feature is the rim edge point
        // (0.3, 0, 0.5). Distance = sqrt(0.1^2 + 0.1^2) ≈ 0.1414 < 0.2 → contact.
        let mut m = make_model(2);
        m.geom_type[0] = GeomType::Cylinder;
        m.geom_size[0] = Vector3::new(0.3, 0.5, 0.0);
        m.geom_size[1] = Vector3::new(0.2, 0.0, 0.0);
        let c = collide_cylinder_sphere(
            &m,
            0,
            1,
            GeomType::Cylinder,
            Vector3::zeros(),
            I(),
            Vector3::new(0.4, 0.0, 0.6),
            I(),
            m.geom_size[0],
            m.geom_size[1],
            0.0,
        );
        assert_eq!(c.len(), 1);
        let expected = 0.2 - 0.1_f64.hypot(0.1);
        assert_relative_eq!(c[0].depth, expected, epsilon = 1e-9);
        assert_relative_eq!(c[0].normal.norm(), 1.0, epsilon = 1e-9);
    }

    // ── cylinder ↔ capsule ────────────────────────────────────────────

    #[test]
    fn cyl_capsule_parallel_side_penetration() {
        // Both axes +Z, side-by-side. Cylinder r=0.3 at origin, capsule r=0.2
        // hl=0.4 at x=0.4. Axes 0.4 apart; cyl surface at x=0.3; capsule surface
        // reaches x=0.4-0.2=0.2 → overlap 0.1.
        let mut m = make_model(2);
        m.geom_type[0] = GeomType::Cylinder;
        m.geom_size[0] = Vector3::new(0.3, 0.5, 0.0);
        m.geom_type[1] = GeomType::Capsule;
        m.geom_size[1] = Vector3::new(0.2, 0.4, 0.0);
        let c = collide_cylinder_capsule(
            &m,
            0,
            1,
            GeomType::Cylinder,
            Vector3::zeros(),
            I(),
            Vector3::new(0.4, 0.0, 0.0),
            I(),
            m.geom_size[0],
            m.geom_size[1],
            0.0,
        );
        assert_eq!(c.len(), 1);
        assert_relative_eq!(c[0].depth, 0.1, epsilon = 1e-9);
        assert_relative_eq!(c[0].normal, Vector3::x(), epsilon = 1e-9);
    }

    #[test]
    fn cyl_capsule_separated_returns_empty() {
        let mut m = make_model(2);
        m.geom_type[0] = GeomType::Cylinder;
        m.geom_size[0] = Vector3::new(0.3, 0.5, 0.0);
        m.geom_type[1] = GeomType::Capsule;
        m.geom_size[1] = Vector3::new(0.2, 0.4, 0.0);
        let c = collide_cylinder_capsule(
            &m,
            0,
            1,
            GeomType::Cylinder,
            Vector3::zeros(),
            I(),
            Vector3::new(1.2, 0.0, 0.0),
            I(),
            m.geom_size[0],
            m.geom_size[1],
            0.0,
        );
        assert!(c.is_empty());
    }

    // ── capsule ↔ box ─────────────────────────────────────────────────

    #[test]
    fn capsule_box_face_penetration() {
        // Box half-extents 0.5 at origin; capsule r=0.2 hl=0.3 axis +Z at x=0.6.
        // Capsule axis 0.6 from center; +X face at 0.5; axis-to-face gap 0.1;
        // penetration 0.2-0.1=0.1 along +X.
        let mut m = make_model(2);
        m.geom_type[0] = GeomType::Capsule;
        m.geom_size[0] = Vector3::new(0.2, 0.3, 0.0);
        m.geom_type[1] = GeomType::Box;
        m.geom_size[1] = Vector3::new(0.5, 0.5, 0.5);
        let c = collide_capsule_box(
            &m,
            0,
            1,
            GeomType::Capsule,
            Vector3::new(0.6, 0.0, 0.0),
            I(),
            Vector3::zeros(),
            I(),
            m.geom_size[0],
            m.geom_size[1],
            0.0,
        );
        assert!(!c.is_empty());
        assert_relative_eq!(c[0].depth, 0.1, epsilon = 1e-6);
        // Contact normal is aligned with the box's ±X face axis.
        assert_relative_eq!(c[0].normal.x.abs(), 1.0, epsilon = 1e-6);
    }

    #[test]
    fn capsule_box_separated_returns_empty() {
        let mut m = make_model(2);
        m.geom_type[0] = GeomType::Capsule;
        m.geom_size[0] = Vector3::new(0.2, 0.3, 0.0);
        m.geom_type[1] = GeomType::Box;
        m.geom_size[1] = Vector3::new(0.5, 0.5, 0.5);
        let c = collide_capsule_box(
            &m,
            0,
            1,
            GeomType::Capsule,
            Vector3::new(1.5, 0.0, 0.0),
            I(),
            Vector3::zeros(),
            I(),
            m.geom_size[0],
            m.geom_size[1],
            0.0,
        );
        assert!(c.is_empty());
    }

    // ── box ↔ box ─────────────────────────────────────────────────────

    #[test]
    fn box_box_face_overlap() {
        // Two unit boxes (half 0.5). Centers 0.9 apart on X; combined reach 1.0;
        // penetration 0.1 along X.
        let mut m = make_model(2);
        m.geom_type[0] = GeomType::Box;
        m.geom_size[0] = Vector3::new(0.5, 0.5, 0.5);
        m.geom_type[1] = GeomType::Box;
        m.geom_size[1] = Vector3::new(0.5, 0.5, 0.5);
        let c = collide_box_box(
            &m,
            0,
            1,
            Vector3::zeros(),
            I(),
            Vector3::new(0.9, 0.0, 0.0),
            I(),
            m.geom_size[0],
            m.geom_size[1],
            0.0,
        );
        assert!(!c.is_empty());
        for contact in &c {
            assert_relative_eq!(contact.depth, 0.1, epsilon = 1e-6);
            assert_relative_eq!(contact.normal.x.abs(), 1.0, epsilon = 1e-6);
        }
    }

    #[test]
    fn box_box_separated_returns_empty() {
        let mut m = make_model(2);
        m.geom_type[0] = GeomType::Box;
        m.geom_size[0] = Vector3::new(0.5, 0.5, 0.5);
        m.geom_type[1] = GeomType::Box;
        m.geom_size[1] = Vector3::new(0.5, 0.5, 0.5);
        let c = collide_box_box(
            &m,
            0,
            1,
            Vector3::zeros(),
            I(),
            Vector3::new(1.5, 0.0, 0.0),
            I(),
            m.geom_size[0],
            m.geom_size[1],
            0.0,
        );
        assert!(c.is_empty());
    }

    #[test]
    fn box_box_rotated_corner_contact() {
        // Box 1 axis-aligned (half 0.5); box 2 rotated 45° about Z, its corner
        // poking the +X face. Sanity: a contact exists with a unit normal and
        // positive penetration.
        let mut m = make_model(2);
        m.geom_type[0] = GeomType::Box;
        m.geom_size[0] = Vector3::new(0.5, 0.5, 0.5);
        m.geom_type[1] = GeomType::Box;
        m.geom_size[1] = Vector3::new(0.5, 0.5, 0.5);
        let rot = UnitQuaternion::from_axis_angle(&Vector3::z_axis(), std::f64::consts::FRAC_PI_4)
            .to_rotation_matrix()
            .into_inner();
        // 45°-rotated box has corner reach 0.5*sqrt(2) ≈ 0.707 along X; place its
        // center at 1.15 so the corner (x ≈ 0.443) overlaps box 1's +X face (0.5).
        let c = collide_box_box(
            &m,
            0,
            1,
            Vector3::zeros(),
            I(),
            Vector3::new(1.15, 0.0, 0.0),
            rot,
            m.geom_size[0],
            m.geom_size[1],
            0.0,
        );
        assert!(!c.is_empty());
        assert_relative_eq!(c[0].normal.norm(), 1.0, epsilon = 1e-6);
        assert!(c.iter().all(|x| x.depth > 0.0));
    }
}
