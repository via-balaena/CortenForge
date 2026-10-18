//! Analytical pairwise collision for cylinders and box-box SAT.

use super::mj_primitive::{Frame, raw_capsule_box, sphere_cylinder};
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

/// Box-box collision detection using SAT + Sutherland-Hodgman face clipping.
///
/// Returns up to ~8 contacts, matching MuJoCo's `_boxbox` algorithm:
/// - **Face-face**: Clip incident face polygon against reference face edges,
///   producing up to ~8 contact points with individual depths.
/// - **Edge-edge**: Closest points on the two contacting edge segments → 1 contact.
///
/// Tests 15 potential separating axes: 3 face normals from each box
/// and 9 edge-edge cross products.
// Cylinder-box pair dispatcher takes the full per-geom context; inlined as a single function so the close-form geometry math reads top-to-bottom.
#[allow(clippy::too_many_arguments, clippy::too_many_lines)]
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
    let half1 = size1;
    let half2 = size2;

    let axes1: [Vector3<f64>; 3] = [
        mat1.column(0).into_owned(),
        mat1.column(1).into_owned(),
        mat1.column(2).into_owned(),
    ];
    let axes2: [Vector3<f64>; 3] = [
        mat2.column(0).into_owned(),
        mat2.column(1).into_owned(),
        mat2.column(2).into_owned(),
    ];

    let center_diff = pos2 - pos1;

    // ── SAT Phase: find minimum penetration axis ──

    let mut min_pen = f64::MAX;
    let mut best_axis = Vector3::x();
    let mut best_type = SatAxisType::Face1(0);

    // Test face normals of box 1 (3 axes)
    for i in 0..3 {
        let axis = axes1[i];
        let pen = test_sat_axis(&axis, &center_diff, &axes1, &half1, &axes2, &half2);
        if pen <= -margin {
            return vec![];
        }
        if pen < min_pen {
            min_pen = pen;
            best_axis = axis;
            best_type = SatAxisType::Face1(i);
        }
    }

    // Test face normals of box 2 (3 axes)
    for i in 0..3 {
        let axis = axes2[i];
        let pen = test_sat_axis(&axis, &center_diff, &axes1, &half1, &axes2, &half2);
        if pen <= -margin {
            return vec![];
        }
        if pen < min_pen {
            min_pen = pen;
            best_axis = axis;
            best_type = SatAxisType::Face2(i);
        }
    }

    // Test edge-edge cross products (9 axes)
    for i in 0..3 {
        for j in 0..3 {
            let cross = axes1[i].cross(&axes2[j]);
            let len = cross.norm();
            if len < GEOM_EPSILON {
                continue;
            }
            let axis = cross / len;

            let pen = test_sat_axis(&axis, &center_diff, &axes1, &half1, &axes2, &half2);
            if pen <= -margin {
                return vec![];
            }
            // Edge-edge only wins if significantly better than face (0.95 bias)
            if pen < min_pen * 0.95 {
                min_pen = pen;
                best_axis = axis;
                best_type = SatAxisType::Edge(i, j);
            }
        }
    }

    // Ensure normal points from box1 to box2
    if best_axis.dot(&center_diff) < 0.0 {
        best_axis = -best_axis;
    }

    // ── Contact Generation ──

    match best_type {
        SatAxisType::Face1(face_idx) | SatAxisType::Face2(face_idx) => {
            let is_face1 = matches!(best_type, SatAxisType::Face1(_));

            // Reference box owns the separating face; incident box is the other.
            let (ref_pos, ref_axes, ref_half, inc_pos, inc_axes, inc_half) = if is_face1 {
                (pos1, &axes1, &half1, pos2, &axes2, &half2)
            } else {
                (pos2, &axes2, &half2, pos1, &axes1, &half1)
            };

            // Reference face outward normal (from reference box toward the other).
            let ref_normal = if is_face1 { best_axis } else { -best_axis };
            let ref_half_arr = [ref_half.x, ref_half.y, ref_half.z];
            let ref_face_center = ref_pos + ref_normal * ref_half_arr[face_idx];

            // Two axes spanning the reference face
            let ea = (face_idx + 1) % 3;
            let eb = (face_idx + 2) % 3;
            let ref_edge_half = [ref_half_arr[ea], ref_half_arr[eb]];
            let ref_edge_dirs = [ref_axes[ea], ref_axes[eb]];

            // Incident face: most anti-parallel to ref_normal
            let inc_half_arr = [inc_half.x, inc_half.y, inc_half.z];
            let mut best_inc_dot = 0.0_f64;
            let mut inc_face_idx = 0;
            let mut inc_face_sign = 1.0_f64;
            for (k, inc_axis) in inc_axes.iter().enumerate() {
                let d = ref_normal.dot(inc_axis);
                if d.abs() > best_inc_dot.abs() {
                    best_inc_dot = d;
                    inc_face_idx = k;
                    inc_face_sign = if d < 0.0 { 1.0 } else { -1.0 };
                }
            }

            // Build incident face polygon (4 vertices)
            let inc_face_normal = inc_axes[inc_face_idx] * inc_face_sign;
            let inc_face_center = inc_pos + inc_face_normal * inc_half_arr[inc_face_idx];
            let ia = (inc_face_idx + 1) % 3;
            let ib = (inc_face_idx + 2) % 3;
            let ha = inc_half_arr[ia];
            let hb = inc_half_arr[ib];
            let da = inc_axes[ia];
            let db = inc_axes[ib];

            let mut polygon = vec![
                inc_face_center + da * ha + db * hb,
                inc_face_center - da * ha + db * hb,
                inc_face_center - da * ha - db * hb,
                inc_face_center + da * ha - db * hb,
            ];

            // Clip polygon against 4 reference face edge planes
            for &(dir, half) in &[
                (ref_edge_dirs[0], ref_edge_half[0]),
                (-ref_edge_dirs[0], ref_edge_half[0]),
                (ref_edge_dirs[1], ref_edge_half[1]),
                (-ref_edge_dirs[1], ref_edge_half[1]),
            ] {
                let limit = dir.dot(&ref_face_center) + half;
                polygon = clip_polygon_by_plane(&polygon, &dir, limit);
                if polygon.is_empty() {
                    return vec![];
                }
            }

            // Compute per-vertex depth along ref_normal
            let ref_plane_d = ref_normal.dot(&ref_face_center);
            let mut contacts = Vec::with_capacity(polygon.len());

            for pt in &polygon {
                let signed_dist = ref_normal.dot(pt) - ref_plane_d;
                let depth = -signed_dist;

                if depth > -margin {
                    let contact_pos = pt - ref_normal * (signed_dist / 2.0);
                    contacts.push(make_contact_from_geoms(
                        model,
                        contact_pos,
                        best_axis,
                        depth,
                        geom1,
                        geom2,
                        margin,
                    ));
                }
            }

            dedup_contacts(&mut contacts);
            // NOTE: MuJoCo also applies mju_outsideBox (1% tolerance) and caps
            // at mjMAXCONPAIR=50. Neither is needed here: S-H clipping already
            // constrains output to the reference face bounds (max 8 vertices),
            // and no box-box configuration can exceed the cap.
            contacts
        }

        SatAxisType::Edge(edge1_idx, edge2_idx) => {
            // Edge-edge: closest points on the two contacting edge segments
            let half1_arr = [half1.x, half1.y, half1.z];
            let half2_arr = [half2.x, half2.y, half2.z];

            // Edge of box1 along axes1[edge1_idx]. The other two axes pick
            // which of the 4 parallel edges via support in -best_axis direction.
            let mut edge1_center = pos1;
            for k in 0..3 {
                if k != edge1_idx {
                    let sign = if best_axis.dot(&axes1[k]) > 0.0 {
                        -1.0
                    } else {
                        1.0
                    };
                    edge1_center += axes1[k] * (sign * half1_arr[k]);
                }
            }
            let e1_half = half1_arr[edge1_idx];
            let e1_start = edge1_center - axes1[edge1_idx] * e1_half;
            let e1_end = edge1_center + axes1[edge1_idx] * e1_half;

            // Edge of box2 along axes2[edge2_idx]
            let mut edge2_center = pos2;
            for k in 0..3 {
                if k != edge2_idx {
                    let sign = if best_axis.dot(&axes2[k]) < 0.0 {
                        -1.0
                    } else {
                        1.0
                    };
                    edge2_center += axes2[k] * (sign * half2_arr[k]);
                }
            }
            let e2_half = half2_arr[edge2_idx];
            let e2_start = edge2_center - axes2[edge2_idx] * e2_half;
            let e2_end = edge2_center + axes2[edge2_idx] * e2_half;

            let (cp1, cp2) = closest_points_segments(e1_start, e1_end, e2_start, e2_end);
            let contact_pos = (cp1 + cp2) * 0.5;

            vec![make_contact_from_geoms(
                model,
                contact_pos,
                best_axis,
                min_pen,
                geom1,
                geom2,
                margin,
            )]
        }
    }
}

/// Classification of the best SAT separating axis.
#[derive(Clone, Copy)]
enum SatAxisType {
    /// Face normal of box 1, axis index 0–2.
    Face1(usize),
    /// Face normal of box 2, axis index 0–2.
    Face2(usize),
    /// Edge cross-product: edge `i` of box 1 × edge `j` of box 2.
    Edge(usize, usize),
}

/// Sutherland-Hodgman: clip polygon against half-plane `dot(normal, p) <= limit`.
fn clip_polygon_by_plane(
    polygon: &[Vector3<f64>],
    normal: &Vector3<f64>,
    limit: f64,
) -> Vec<Vector3<f64>> {
    if polygon.is_empty() {
        return vec![];
    }

    let mut output = Vec::with_capacity(polygon.len() + 1);
    let n = polygon.len();

    for i in 0..n {
        let current = polygon[i];
        let next = polygon[(i + 1) % n];

        let d_current = normal.dot(&current) - limit;
        let d_next = normal.dot(&next) - limit;

        if d_current <= 0.0 {
            output.push(current);
        }

        // Edge crosses the plane → emit intersection
        if (d_current <= 0.0) != (d_next <= 0.0) {
            let t = d_current / (d_current - d_next);
            output.push(current + (next - current) * t);
        }
    }

    output
}

/// Remove duplicate contacts within 1e-6 distance.
fn dedup_contacts(contacts: &mut Vec<Contact>) {
    const DEDUP_DIST_SQ: f64 = 1e-12; // (1e-6)^2

    let mut i = 0;
    while i < contacts.len() {
        let mut j = i + 1;
        while j < contacts.len() {
            let diff = contacts[i].pos - contacts[j].pos;
            if diff.norm_squared() < DEDUP_DIST_SQ {
                contacts.swap_remove(j);
            } else {
                j += 1;
            }
        }
        i += 1;
    }
}

/// Test a single SAT axis and return penetration depth (negative = separated).
#[inline]
fn test_sat_axis(
    axis: &Vector3<f64>,
    center_diff: &Vector3<f64>,
    axes1: &[Vector3<f64>; 3],
    half1: &Vector3<f64>,
    axes2: &[Vector3<f64>; 3],
    half2: &Vector3<f64>,
) -> f64 {
    let r1 = (half1.x * axis.dot(&axes1[0]).abs())
        + (half1.y * axis.dot(&axes1[1]).abs())
        + (half1.z * axis.dot(&axes1[2]).abs());

    let r2 = (half2.x * axis.dot(&axes2[0]).abs())
        + (half2.y * axis.dot(&axes2[1]).abs())
        + (half2.z * axis.dot(&axes2[2]).abs());

    let dist = axis.dot(center_diff).abs();

    r1 + r2 - dist
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
        // Sphere is geom 0 / type1, cylinder geom 1: exercises the dispatcher's
        // `else` branch. Same geometry as the side-penetration case; because the
        // cylinder is the higher index the emitted normal is flipped to point
        // from geom 0 (sphere) toward geom 1 (cylinder) = -X.
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
