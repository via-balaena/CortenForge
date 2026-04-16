//! Analytical pairwise collision for cylinders and box-box SAT.

use super::narrow::{CAP_COLLISION_THRESHOLD, GEOM_EPSILON, make_contact_from_geoms};
use crate::forward::{closest_points_segments, closest_points_segments_parametric};
use crate::types::{Contact, GeomType, Model};
use nalgebra::{Matrix3, Vector3};

/// Cylinder-sphere collision detection.
///
/// Handles three collision cases:
/// - Side collision: sphere beside cylinder body
/// - Cap collision: sphere above/below cylinder, within radius
/// - Edge collision: sphere near rim of cylinder cap
///
/// Cylinder axis is local Z.
// Cylinder-cylinder pair dispatcher takes the full per-geom context (poses, sizes, friction, margin).
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
    // Determine which is cylinder and which is sphere
    // Note: sphere doesn't use its rotation matrix, but we need mat2 for the cylinder case
    let (cyl_geom, cyl_pos, cyl_mat, cyl_size, sph_geom, sph_pos, sph_radius) =
        if type1 == GeomType::Cylinder {
            (geom1, pos1, mat1, size1, geom2, pos2, size2.x)
        } else {
            (geom2, pos2, mat2, size2, geom1, pos1, size1.x)
        };

    let cyl_radius = cyl_size.x;
    let cyl_half_height = cyl_size.y;
    let cyl_axis = cyl_mat.column(2).into_owned();

    // Vector from cylinder center to sphere center
    let d = sph_pos - cyl_pos;

    // Project onto cylinder axis
    let axis_dist = d.dot(&cyl_axis);

    // Clamp to cylinder height
    let clamped_axis = axis_dist.clamp(-cyl_half_height, cyl_half_height);

    // Closest point on cylinder axis to sphere center
    let axis_point = cyl_pos + cyl_axis * clamped_axis;

    // Radial vector from axis to sphere
    let radial = sph_pos - axis_point;
    let radial_dist = radial.norm();

    // Determine closest point on cylinder surface and contact normal
    let (closest_on_cyl, normal) = if axis_dist.abs() <= cyl_half_height {
        // Sphere is beside the cylinder (side collision)
        if radial_dist < GEOM_EPSILON {
            // Sphere center on axis - degenerate case, pick arbitrary radial direction
            let arb = if cyl_axis.x.abs() < 0.9 {
                Vector3::x()
            } else {
                Vector3::y()
            };
            let n = cyl_axis.cross(&arb).normalize();
            (axis_point + n * cyl_radius, n)
        } else {
            let n = radial / radial_dist;
            (axis_point + n * cyl_radius, n)
        }
    } else {
        // Sphere is above/below cylinder (cap or edge collision)
        let cap_center = cyl_pos + cyl_axis * clamped_axis;

        // Compute radial direction in the plane perpendicular to cylinder axis.
        // d - (d · axis) * axis gives the perpendicular component.
        let perp = d - cyl_axis * axis_dist;
        let perp_dist = perp.norm();

        if perp_dist <= cyl_radius {
            // Cap collision - sphere projects onto cap face
            let n = if axis_dist > 0.0 { cyl_axis } else { -cyl_axis };
            (cap_center + perp, n)
        } else {
            // Edge collision - sphere near rim of cap
            let radial_n = perp / perp_dist;
            let edge_point = cap_center + radial_n * cyl_radius;
            let to_sphere = sph_pos - edge_point;
            let to_sphere_dist = to_sphere.norm();
            let n = if to_sphere_dist > GEOM_EPSILON {
                to_sphere / to_sphere_dist
            } else {
                // Degenerate: sphere center exactly on edge
                radial_n
            };
            (edge_point, n)
        }
    };

    // Compute penetration depth
    let dist = (sph_pos - closest_on_cyl).norm();
    let penetration = sph_radius - dist;

    if penetration <= -margin {
        return vec![];
    }

    // Contact position is on the surface between the two shapes
    let contact_pos = closest_on_cyl + normal * (penetration * 0.5);

    // Order geoms consistently (lower index first)
    let (g1, g2, final_normal) = if cyl_geom < sph_geom {
        (cyl_geom, sph_geom, normal)
    } else {
        (sph_geom, cyl_geom, -normal)
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

/// Feature type for capsule-box collision Phase 1/2 classification.
enum CapsuleBoxFeature {
    Face {
        face_axis: usize,
        is_endpoint_a: bool,
    },
    Edge {
        clamp_capsule: u8,
    },
}

/// Capsule-box collision detection using MuJoCo's 4-phase algorithm.
///
/// Phase 1: face test, Phase 2: 12-edge test, Phase 3: second contact search,
/// Phase 4: sphere-box delegation. Returns up to 2 contacts.
// Cylinder-capsule pair dispatcher takes the full per-geom context (poses, sizes, friction, margin).
#[allow(clippy::too_many_arguments)]
#[allow(clippy::unreachable)] // match on CapsuleBoxFeature::Face subtypes; Edge variant handled in else branch
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
    // Determine which is capsule and which is box
    let (
        capsule_geom,
        box_geom,
        capsule_pos,
        capsule_mat,
        box_pos,
        box_mat,
        capsule_size,
        box_half,
    ) = if type1 == GeomType::Capsule {
        (geom1, geom2, pos1, mat1, pos2, mat2, size1, size2)
    } else {
        (geom2, geom1, pos2, mat2, pos1, mat1, size2, size1)
    };

    let capsule_radius = capsule_size.x;
    let capsule_half_len = capsule_size.y;
    let capsule_axis = capsule_mat.column(2).into_owned();

    let cap_a = capsule_pos - capsule_axis * capsule_half_len;
    let cap_b = capsule_pos + capsule_axis * capsule_half_len;

    let box_mat_t = box_mat.transpose();

    let (g1, g2) = if capsule_geom < box_geom {
        (capsule_geom, box_geom)
    } else {
        (box_geom, capsule_geom)
    };
    let normal_sign = if capsule_geom < box_geom { 1.0 } else { -1.0 };

    // Test a capsule point as a sphere against the box.
    // Returns Some((contact_pos, normal, penetration)) if penetrating.
    let sphere_box_test =
        |sphere_center: Vector3<f64>| -> Option<(Vector3<f64>, Vector3<f64>, f64)> {
            let local_pt = box_mat_t * (sphere_center - box_pos);
            let clamped = Vector3::new(
                local_pt.x.clamp(-box_half.x, box_half.x),
                local_pt.y.clamp(-box_half.y, box_half.y),
                local_pt.z.clamp(-box_half.z, box_half.z),
            );
            let box_pt = box_pos + box_mat * clamped;
            let diff = sphere_center - box_pt;
            let dist = diff.norm();
            let penetration = capsule_radius - dist;

            if penetration <= -margin {
                return None;
            }

            let normal = if dist > GEOM_EPSILON {
                diff / dist
            } else {
                // Sphere center inside box: find nearest face
                let mut min_face_dist = f64::MAX;
                let mut normal_local = Vector3::x();
                for i in 0..3 {
                    let pen_pos = box_half[i] - local_pt[i];
                    let pen_neg = box_half[i] + local_pt[i];
                    if pen_pos < min_face_dist {
                        min_face_dist = pen_pos;
                        normal_local = Vector3::zeros();
                        normal_local[i] = 1.0;
                    }
                    if pen_neg < min_face_dist {
                        min_face_dist = pen_neg;
                        normal_local = Vector3::zeros();
                        normal_local[i] = -1.0;
                    }
                }
                box_mat * normal_local
            };

            let contact_pos = box_pt + normal * (penetration * 0.5);
            Some((contact_pos, normal, penetration))
        };

    // ── Phase 1: Face test ──
    // Transform endpoints to box-local frame. Count clamped axes per endpoint.
    let local_a = box_mat_t * (cap_a - box_pos);
    let local_b = box_mat_t * (cap_b - box_pos);

    let mut best_face_dist = f64::MAX;
    let mut face_feature: Option<CapsuleBoxFeature> = None;

    for (local_ep, is_a) in [(&local_a, true), (&local_b, false)] {
        let mut clamped_count = 0usize;
        let mut clamped_axis = 0usize;
        for i in 0..3 {
            if local_ep[i].abs() > box_half[i] {
                clamped_count += 1;
                clamped_axis = i;
            }
        }
        if clamped_count <= 1 {
            // On a face (or inside box if 0 clamped)
            let clamped = Vector3::new(
                local_ep.x.clamp(-box_half.x, box_half.x),
                local_ep.y.clamp(-box_half.y, box_half.y),
                local_ep.z.clamp(-box_half.z, box_half.z),
            );
            let dist = (local_ep - clamped).norm();
            if dist < best_face_dist {
                best_face_dist = dist;
                let axis = if clamped_count == 1 {
                    clamped_axis
                } else {
                    // Inside box: pick nearest face
                    let mut min_d = f64::MAX;
                    let mut best_ax = 0;
                    for i in 0..3 {
                        let d = (box_half[i] - local_ep[i].abs()).abs();
                        if d < min_d {
                            min_d = d;
                            best_ax = i;
                        }
                    }
                    best_ax
                };
                face_feature = Some(CapsuleBoxFeature::Face {
                    face_axis: axis,
                    is_endpoint_a: is_a,
                });
            }
        }
    }

    // ── Phase 2: Edge test (12 box edges) ──
    let mut best_edge_dist = f64::MAX;
    let mut best_edge_s = 0.0_f64;
    let mut best_edge_clamp_s = 0_u8;

    for a in 0..3 {
        let b = (a + 1) % 3;
        let c = (a + 2) % 3;
        for &sb in &[-1.0_f64, 1.0] {
            for &sc in &[-1.0_f64, 1.0] {
                let mut edge_start = Vector3::zeros();
                let mut edge_end = Vector3::zeros();
                edge_start[a] = -box_half[a];
                edge_end[a] = box_half[a];
                edge_start[b] = sb * box_half[b];
                edge_end[b] = sb * box_half[b];
                edge_start[c] = sc * box_half[c];
                edge_end[c] = sc * box_half[c];

                let (s, t, clamp_s, _clamp_t) =
                    closest_points_segments_parametric(local_a, local_b, edge_start, edge_end);

                let cap_pt = local_a + (local_b - local_a) * s;
                let edge_pt = edge_start + (edge_end - edge_start) * t;
                let dist = (cap_pt - edge_pt).norm();

                if dist < best_edge_dist {
                    best_edge_dist = dist;
                    best_edge_s = s;
                    best_edge_clamp_s = clamp_s;
                }
            }
        }
    }

    // ── Select primary feature and t1 ──
    let (feature, t1) = if let Some(feat) =
        face_feature.filter(|_| best_face_dist <= best_edge_dist + GEOM_EPSILON)
    {
        let t1 = match &feat {
            CapsuleBoxFeature::Face {
                is_endpoint_a: true,
                ..
            } => 0.0,
            CapsuleBoxFeature::Face {
                is_endpoint_a: false,
                ..
            } => 1.0,
            _ => unreachable!(),
        };
        (feat, t1)
    } else {
        (
            CapsuleBoxFeature::Edge {
                clamp_capsule: best_edge_clamp_s,
            },
            best_edge_s,
        )
    };

    // ── Phase 3: Second contact search ──
    let cap_dir = cap_b - cap_a;

    let t2 = match &feature {
        // 3A: Edge, capsule endpoint clamped → second contact at opposite endpoint
        CapsuleBoxFeature::Edge { clamp_capsule } if *clamp_capsule != 1 => {
            if *clamp_capsule == 0 {
                Some(1.0)
            } else {
                Some(0.0)
            }
        }
        // 3B: Edge, capsule interior (T/X crossing) → test both endpoints, pick closer
        CapsuleBoxFeature::Edge { clamp_capsule: _ } => {
            let clamped_a = Vector3::new(
                local_a.x.clamp(-box_half.x, box_half.x),
                local_a.y.clamp(-box_half.y, box_half.y),
                local_a.z.clamp(-box_half.z, box_half.z),
            );
            let clamped_b = Vector3::new(
                local_b.x.clamp(-box_half.x, box_half.x),
                local_b.y.clamp(-box_half.y, box_half.y),
                local_b.z.clamp(-box_half.z, box_half.z),
            );
            let dist_a = (local_a - clamped_a).norm();
            let dist_b = (local_b - clamped_b).norm();
            Some(if dist_a < dist_b { 0.0 } else { 1.0 })
        }
        // 3C: Face closest → walk toward other endpoint, clamp to face boundary
        CapsuleBoxFeature::Face {
            face_axis,
            is_endpoint_a,
        } => {
            let (t_on_face, t_other) = if *is_endpoint_a {
                (0.0, 1.0)
            } else {
                (1.0, 0.0)
            };
            let on_face_local = if *is_endpoint_a { &local_a } else { &local_b };
            let other_local = if *is_endpoint_a { &local_b } else { &local_a };

            let mut min_t_cross = 1.0_f64; // walk fraction from on_face to other
            for i in 0..3 {
                if i == *face_axis {
                    continue;
                }
                let delta = other_local[i] - on_face_local[i];
                if delta.abs() < GEOM_EPSILON {
                    continue;
                }
                if other_local[i] > box_half[i] {
                    let tc = (box_half[i] - on_face_local[i]) / delta;
                    if tc > 0.0 && tc < min_t_cross {
                        min_t_cross = tc;
                    }
                } else if other_local[i] < -box_half[i] {
                    let tc = (-box_half[i] - on_face_local[i]) / delta;
                    if tc > 0.0 && tc < min_t_cross {
                        min_t_cross = tc;
                    }
                }
            }
            // Convert walk fraction to capsule parameter space
            let t2_val = t_on_face + min_t_cross * (t_other - t_on_face);
            Some(t2_val)
        }
    };

    // Filter t2
    let t2 = t2.filter(|&t2_val| (t2_val - t1).abs() > GEOM_EPSILON);

    // ── Phase 4: Sphere-box delegation ──
    let dedup_dist_sq = 1e-6_f64;
    let mut contacts = Vec::with_capacity(2);
    let mut contact_positions: Vec<Vector3<f64>> = Vec::with_capacity(2);

    let mut try_add = |t: f64| {
        let center = cap_a + cap_dir * t;
        if let Some((cpos, normal, pen)) = sphere_box_test(center) {
            let too_close = contact_positions
                .iter()
                .any(|p| (p - cpos).norm_squared() < dedup_dist_sq);
            if !too_close {
                contact_positions.push(cpos);
                contacts.push(make_contact_from_geoms(
                    model,
                    cpos,
                    normal * normal_sign,
                    pen,
                    g1,
                    g2,
                    margin,
                ));
            }
        }
    };

    try_add(t1);
    if let Some(t2_val) = t2 {
        try_add(t2_val);
    }

    contacts
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
