//! Analytical pairwise collision for cylinders and box-box SAT.

use super::narrow::{CAP_COLLISION_THRESHOLD, GEOM_EPSILON, make_contact_from_geoms};
use crate::mujoco_pipeline::closest_point_segment; // monolith: removed in Phase 8a
use crate::mujoco_pipeline::closest_points_segments; // monolith: removed in Phase 8a
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
) -> Option<Contact> {
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
        return None;
    }

    // Contact position is on the surface between the two shapes
    let contact_pos = closest_on_cyl + normal * (penetration * 0.5);

    // Order geoms consistently (lower index first)
    let (g1, g2, final_normal) = if cyl_geom < sph_geom {
        (cyl_geom, sph_geom, normal)
    } else {
        (sph_geom, cyl_geom, -normal)
    };

    Some(make_contact_from_geoms(
        model,
        contact_pos,
        final_normal,
        penetration,
        g1,
        g2,
        margin,
    ))
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
/// above/below cylinder), returns `None` to fall through to GJK/EPA.
///
/// Both shapes have their axis along local Z.
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
) -> Option<Contact> {
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
        return None;
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
        return None;
    }

    // Closest point on cylinder surface (in direction of capsule)
    let cyl_surface = cyl_closest + normal * cyl_radius;

    // Distance from cylinder surface to capsule axis
    let surface_to_cap_dist = (cap_closest - cyl_surface).dot(&normal);
    let penetration = cap_radius - surface_to_cap_dist;

    if penetration <= -margin {
        return None;
    }

    // Contact position is between the two surfaces
    let contact_pos = cyl_surface + normal * (penetration * 0.5);

    // Order geoms consistently (lower index first)
    let (g1, g2, final_normal) = if cyl_geom < cap_geom {
        (cyl_geom, cap_geom, normal)
    } else {
        (cap_geom, cyl_geom, -normal)
    };

    Some(make_contact_from_geoms(
        model,
        contact_pos,
        final_normal,
        penetration,
        g1,
        g2,
        margin,
    ))
}

/// Capsule-box collision detection.
///
/// Tests both capsule endpoints and the closest point on the capsule axis
/// to find the minimum distance configuration.
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
) -> Option<Contact> {
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

    // Capsule endpoints
    let cap_a = capsule_pos - capsule_axis * capsule_half_len;
    let cap_b = capsule_pos + capsule_axis * capsule_half_len;

    // Find closest point on capsule axis to box
    // We sample multiple points along the capsule and find the minimum distance
    let mut min_dist = f64::MAX;
    let mut best_capsule_point = capsule_pos;
    let mut best_box_point = box_pos;

    // Transform box to local coordinates once
    let box_mat_t = box_mat.transpose();

    // Sample along capsule axis (including endpoints)
    let samples = [0.0, 0.25, 0.5, 0.75, 1.0];
    for &t in &samples {
        let capsule_point = cap_a + (cap_b - cap_a) * t;
        let local_point = box_mat_t * (capsule_point - box_pos);

        // Closest point on box to this capsule point
        let closest_local = Vector3::new(
            local_point.x.clamp(-box_half.x, box_half.x),
            local_point.y.clamp(-box_half.y, box_half.y),
            local_point.z.clamp(-box_half.z, box_half.z),
        );
        let closest_world = box_pos + box_mat * closest_local;

        let dist = (capsule_point - closest_world).norm();
        if dist < min_dist {
            min_dist = dist;
            best_capsule_point = capsule_point;
            best_box_point = closest_world;
        }
    }

    // Refine by finding closest point on capsule axis to best box point
    let closest_on_capsule = closest_point_segment(cap_a, cap_b, best_box_point);
    let local_closest = box_mat_t * (closest_on_capsule - box_pos);
    let box_closest_local = Vector3::new(
        local_closest.x.clamp(-box_half.x, box_half.x),
        local_closest.y.clamp(-box_half.y, box_half.y),
        local_closest.z.clamp(-box_half.z, box_half.z),
    );
    let box_closest_world = box_pos + box_mat * box_closest_local;
    let final_dist = (closest_on_capsule - box_closest_world).norm();

    if final_dist < min_dist {
        min_dist = final_dist;
        best_capsule_point = closest_on_capsule;
        best_box_point = box_closest_world;
    }

    let penetration = capsule_radius - min_dist;

    if penetration > -margin {
        let diff = best_capsule_point - best_box_point;
        let normal = if min_dist > GEOM_EPSILON {
            diff / min_dist
        } else {
            // Edge case: capsule axis passes through box
            // Find deepest penetration direction
            let local_cap = box_mat_t * (best_capsule_point - box_pos);
            let mut min_pen = f64::MAX;
            let mut normal_local = Vector3::x();
            for i in 0..3 {
                let pen_pos = box_half[i] - local_cap[i];
                let pen_neg = box_half[i] + local_cap[i];
                if pen_pos < min_pen {
                    min_pen = pen_pos;
                    normal_local = Vector3::zeros();
                    normal_local[i] = 1.0;
                }
                if pen_neg < min_pen {
                    min_pen = pen_neg;
                    normal_local = Vector3::zeros();
                    normal_local[i] = -1.0;
                }
            }
            box_mat * normal_local
        };

        let contact_pos = best_box_point + normal * (penetration * 0.5);

        let (g1, g2) = if capsule_geom < box_geom {
            (capsule_geom, box_geom)
        } else {
            (box_geom, capsule_geom)
        };

        Some(make_contact_from_geoms(
            model,
            contact_pos,
            if capsule_geom < box_geom {
                normal
            } else {
                -normal
            },
            penetration,
            g1,
            g2,
            margin,
        ))
    } else {
        None
    }
}

/// Box-box collision detection using Separating Axis Theorem (SAT).
///
/// Tests 15 axes: 3 face normals of box A, 3 face normals of box B,
/// and 9 edge-edge cross products.
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
) -> Option<Contact> {
    let half1 = size1;
    let half2 = size2;

    // Get box axes
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

    // Track minimum penetration
    let mut min_pen = f64::MAX;
    let mut best_axis = Vector3::x();
    let mut best_axis_is_face = true;

    // Test face normals of box 1 (3 axes)
    for i in 0..3 {
        let axis = axes1[i];
        let pen = test_sat_axis(&axis, &center_diff, &axes1, &half1, &axes2, &half2);
        if pen <= -margin {
            return None; // Separating axis found (beyond margin zone)
        }
        if pen < min_pen {
            min_pen = pen;
            best_axis = axis;
            best_axis_is_face = true;
        }
    }

    // Test face normals of box 2 (3 axes)
    for i in 0..3 {
        let axis = axes2[i];
        let pen = test_sat_axis(&axis, &center_diff, &axes1, &half1, &axes2, &half2);
        if pen <= -margin {
            return None;
        }
        if pen < min_pen {
            min_pen = pen;
            best_axis = axis;
            best_axis_is_face = true;
        }
    }

    // Test edge-edge cross products (9 axes)
    for i in 0..3 {
        for j in 0..3 {
            let axis = axes1[i].cross(&axes2[j]);
            let len = axis.norm();
            if len < GEOM_EPSILON {
                continue; // Parallel edges
            }
            let axis = axis / len;

            let pen = test_sat_axis(&axis, &center_diff, &axes1, &half1, &axes2, &half2);
            if pen <= -margin {
                return None;
            }
            // Edge-edge contacts have a bias - they're less stable
            // Only use if significantly better than face contact
            if pen < min_pen * 0.95 {
                min_pen = pen;
                best_axis = axis;
                best_axis_is_face = false;
            }
        }
    }

    // Ensure normal points from box1 to box2
    if best_axis.dot(&center_diff) < 0.0 {
        best_axis = -best_axis;
    }

    // Find contact point
    // For face contacts: find vertex of one box most in the other's direction
    // For edge contacts: find closest points on the two edges
    let contact_pos = if best_axis_is_face {
        // Find support point on box2 in direction of -normal
        let support_local = Vector3::new(
            if best_axis.dot(&axes2[0]) < 0.0 {
                half2.x
            } else {
                -half2.x
            },
            if best_axis.dot(&axes2[1]) < 0.0 {
                half2.y
            } else {
                -half2.y
            },
            if best_axis.dot(&axes2[2]) < 0.0 {
                half2.z
            } else {
                -half2.z
            },
        );
        pos2 + mat2 * support_local - best_axis * (min_pen * 0.5)
    } else {
        // For edge-edge, use midpoint between closest points on edges
        // Approximate: use center point shifted by penetration
        pos1 + center_diff * 0.5
    };

    Some(make_contact_from_geoms(
        model,
        contact_pos,
        best_axis,
        min_pen,
        geom1,
        geom2,
        margin,
    ))
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
    // Project box extents onto axis
    let r1 = (half1.x * axis.dot(&axes1[0]).abs())
        + (half1.y * axis.dot(&axes1[1]).abs())
        + (half1.z * axis.dot(&axes1[2]).abs());

    let r2 = (half2.x * axis.dot(&axes2[0]).abs())
        + (half2.y * axis.dot(&axes2[1]).abs())
        + (half2.z * axis.dot(&axes2[2]).abs());

    // Distance between centers projected onto axis
    let dist = axis.dot(center_diff).abs();

    // Penetration = sum of radii - distance
    r1 + r2 - dist
}
