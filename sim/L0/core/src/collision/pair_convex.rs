//! Analytical pairwise collision for convex primitives (sphere, capsule, box).

use super::narrow::{GEOM_EPSILON, make_contact_from_geoms};
use crate::forward::{closest_point_segment, closest_points_segments};
use crate::types::{Contact, GeomType, Model};
use nalgebra::{Matrix3, Vector3};

/// Sphere-sphere collision detection.
///
/// This is a simple analytical calculation that's more robust than GJK/EPA
/// for the sphere-sphere case.
#[allow(clippy::too_many_arguments)]
pub fn collide_sphere_sphere(
    model: &Model,
    geom1: usize,
    geom2: usize,
    pos1: Vector3<f64>,
    pos2: Vector3<f64>,
    size1: Vector3<f64>,
    size2: Vector3<f64>,
    margin: f64,
) -> Option<Contact> {
    let radius1 = size1.x;
    let radius2 = size2.x;

    let diff = pos2 - pos1;
    let dist = diff.norm();

    // Check for penetration (or within margin zone)
    let sum_radii = radius1 + radius2;
    let penetration = sum_radii - dist;

    if penetration > -margin {
        // Normal points from sphere1 to sphere2.
        // For coincident/nearly-coincident centers (degenerate case), pick +Z.
        let normal = if dist > GEOM_EPSILON {
            diff / dist
        } else {
            Vector3::z()
        };

        // Contact point is on the surface of sphere1 toward sphere2
        // (midpoint of penetration region)
        let contact_pos = pos1 + normal * (radius1 - penetration * 0.5);

        Some(make_contact_from_geoms(
            model,
            contact_pos,
            normal,
            penetration,
            geom1,
            geom2,
            margin,
        ))
    } else {
        None
    }
}

/// Capsule-capsule collision detection.
///
/// Capsules are represented as line segments with radius. The collision
/// is computed by finding the closest points between the two line segments,
/// then checking if the distance is less than the sum of radii.
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
) -> Option<Contact> {
    // Capsule parameters: size.x = radius, size.y = half_length
    let radius1 = size1.x;
    let half_len1 = size1.y;
    let radius2 = size2.x;
    let half_len2 = size2.y;

    // Get capsule axes (Z-axis of their rotation matrices)
    let axis1 = mat1.column(2).into_owned();
    let axis2 = mat2.column(2).into_owned();

    // Endpoints of capsule line segments
    let p1a = pos1 - axis1 * half_len1;
    let p1b = pos1 + axis1 * half_len1;
    let p2a = pos2 - axis2 * half_len2;
    let p2b = pos2 + axis2 * half_len2;

    // Find closest points between the two line segments
    let (closest1, closest2) = closest_points_segments(p1a, p1b, p2a, p2b);

    // Check distance (or within margin zone)
    let diff = closest2 - closest1;
    let dist = diff.norm();
    let sum_radii = radius1 + radius2;
    let penetration = sum_radii - dist;

    if penetration > -margin {
        // Normal points from capsule1 toward capsule2.
        // For degenerate case (segments intersect), pick +Z.
        let normal = if dist > GEOM_EPSILON {
            diff / dist
        } else {
            Vector3::z()
        };
        let contact_pos = closest1 + normal * (radius1 - penetration * 0.5);

        Some(make_contact_from_geoms(
            model,
            contact_pos,
            normal,
            penetration,
            geom1,
            geom2,
            margin,
        ))
    } else {
        None
    }
}

/// Sphere-capsule collision detection.
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
) -> Option<Contact> {
    // Determine which is sphere and which is capsule
    let (
        sphere_geom,
        capsule_geom,
        sphere_pos,
        capsule_pos,
        capsule_mat,
        sphere_radius,
        capsule_size,
    ) = if type1 == GeomType::Sphere {
        (geom1, geom2, pos1, pos2, mat2, size1.x, size2)
    } else {
        (geom2, geom1, pos2, pos1, mat1, size2.x, size1)
    };

    let capsule_radius = capsule_size.x;
    let capsule_half_len = capsule_size.y;
    let capsule_axis = capsule_mat.column(2).into_owned();

    // Capsule endpoints
    let cap_a = capsule_pos - capsule_axis * capsule_half_len;
    let cap_b = capsule_pos + capsule_axis * capsule_half_len;

    // Find closest point on capsule line segment to sphere center
    let closest_on_capsule = closest_point_segment(cap_a, cap_b, sphere_pos);

    // Check distance
    let diff = sphere_pos - closest_on_capsule;
    let dist = diff.norm();
    let sum_radii = sphere_radius + capsule_radius;
    let penetration = sum_radii - dist;

    if penetration > -margin {
        // Normal points from capsule toward sphere.
        // For degenerate case (sphere center on capsule axis), pick +Z.
        let normal = if dist > GEOM_EPSILON {
            diff / dist
        } else {
            Vector3::z()
        };
        let contact_pos = closest_on_capsule + normal * (capsule_radius - penetration * 0.5);

        // Ensure geom1 < geom2 for consistency
        let (g1, g2) = if sphere_geom < capsule_geom {
            (sphere_geom, capsule_geom)
        } else {
            (capsule_geom, sphere_geom)
        };

        // Normal convention: points from g1 toward g2.
        // `normal` points from capsule toward sphere.
        // If capsule is g1 (capsule_geom < sphere_geom), normal already points g1→g2.
        // If sphere is g1 (sphere_geom < capsule_geom), we need -normal to point g1→g2.
        let final_normal = if capsule_geom < sphere_geom {
            normal
        } else {
            -normal
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
    } else {
        None
    }
}

/// Sphere-box collision detection.
///
/// Uses the closest point on box surface to sphere center algorithm.
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
) -> Option<Contact> {
    // Determine which is sphere and which is box
    let (sphere_geom, box_geom, sphere_pos, box_pos, box_mat, sphere_radius, box_half) =
        if type1 == GeomType::Sphere {
            (geom1, geom2, pos1, pos2, mat2, size1.x, size2)
        } else {
            (geom2, geom1, pos2, pos1, mat1, size2.x, size1)
        };

    // Transform sphere center to box local coordinates
    let local_center = box_mat.transpose() * (sphere_pos - box_pos);

    // Find closest point on box to sphere center (clamp to box bounds)
    let closest_local = Vector3::new(
        local_center.x.clamp(-box_half.x, box_half.x),
        local_center.y.clamp(-box_half.y, box_half.y),
        local_center.z.clamp(-box_half.z, box_half.z),
    );

    // Transform back to world space
    let closest_world = box_pos + box_mat * closest_local;

    // Compute distance and penetration
    let diff = sphere_pos - closest_world;
    let dist = diff.norm();
    let penetration = sphere_radius - dist;

    if penetration > -margin {
        // Compute normal (from box toward sphere)
        let normal = if dist > GEOM_EPSILON {
            diff / dist
        } else {
            // Sphere center is inside box - find deepest penetration axis
            let mut min_pen = f64::MAX;
            let mut normal_local = Vector3::x();
            for i in 0..3 {
                let pen_pos = box_half[i] - local_center[i];
                let pen_neg = box_half[i] + local_center[i];
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

        let contact_pos = closest_world + normal * (penetration * 0.5);

        let (g1, g2) = if sphere_geom < box_geom {
            (sphere_geom, box_geom)
        } else {
            (box_geom, sphere_geom)
        };

        // Normal convention: points from geom1 (g1) toward geom2 (g2).
        // `normal` is computed as sphere_pos - closest_world, i.e., from box toward sphere.
        // If box is g1 (box_geom < sphere_geom), normal already points g1→g2.
        // If sphere is g1 (sphere_geom < box_geom), we need -normal to point g1→g2.
        let final_normal = if box_geom < sphere_geom {
            normal
        } else {
            -normal
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
    } else {
        None
    }
}
