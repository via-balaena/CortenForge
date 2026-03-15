//! Geom distance computation for distance/normal/fromto sensors.
//!
//! Implements `geom_distance()` — the CortenForge equivalent of MuJoCo's
//! `mj_geomDistance()` in `engine_support.c`. Returns the signed distance
//! and nearest surface points between two geoms.

use crate::collision::narrow::geom_to_shape;
use crate::gjk_epa::{gjk_distance, gjk_epa_contact};
use crate::types::{Data, GeomType, Model};
use nalgebra::{Point3, UnitQuaternion, Vector3};
use sim_types::Pose;

/// Compute signed distance and nearest surface points between two geoms.
///
/// Returns `(signed_distance, fromto)` where:
/// - `signed_distance` > 0: separation gap (non-overlapping)
/// - `signed_distance` < 0: penetration depth (overlapping)
/// - `fromto = [from_x, from_y, from_z, to_x, to_y, to_z]`:
///   from = nearest surface point on geom1, to = nearest surface point on geom2
///
/// When `signed_distance >= cutoff` (and cutoff > 0), returns `(cutoff, [0; 6])` —
/// geoms beyond cutoff are not computed.
///
/// MuJoCo ref: `mj_geomDistance()` in `engine_support.c`.
#[allow(clippy::similar_names)]
pub fn geom_distance(
    model: &Model,
    data: &Data,
    geom1: usize,
    geom2: usize,
    cutoff: f64,
) -> (f64, [f64; 6]) {
    let zero_fromto = [0.0f64; 6];

    // 1. Read world-frame poses from Data
    let pos1 = data.geom_xpos[geom1];
    let pos2 = data.geom_xpos[geom2];
    let mat1 = data.geom_xmat[geom1];
    let mat2 = data.geom_xmat[geom2];

    // 2. Bounding sphere early-out
    let center_dist = (pos2 - pos1).norm();
    let rbound1 = model.geom_rbound[geom1];
    let rbound2 = model.geom_rbound[geom2];
    let min_possible = center_dist - rbound1 - rbound2;
    if cutoff > 0.0 && min_possible > cutoff {
        return (cutoff, zero_fromto);
    }

    // 3. Read shape parameters
    let gtype1 = model.geom_type[geom1];
    let gtype2 = model.geom_type[geom2];
    let size1 = model.geom_size[geom1];
    let size2 = model.geom_size[geom2];

    // 4. Analytic sphere-sphere fast path
    if gtype1 == GeomType::Sphere && gtype2 == GeomType::Sphere {
        return sphere_sphere_distance(pos1, size1.x, pos2, size2.x, cutoff);
    }

    // 5. Convert to Shape (convex primitives only)
    let Some(shape1) = geom_to_shape(gtype1, size1) else {
        // Non-convex or plane — deferred (DT-122)
        return (cutoff, zero_fromto);
    };
    let Some(shape2) = geom_to_shape(gtype2, size2) else {
        return (cutoff, zero_fromto);
    };

    // 6. Build Pose from Data pose arrays
    let rot1 =
        UnitQuaternion::from_rotation_matrix(&nalgebra::Rotation3::from_matrix_unchecked(mat1));
    let rot2 =
        UnitQuaternion::from_rotation_matrix(&nalgebra::Rotation3::from_matrix_unchecked(mat2));
    let pose1 = Pose::from_position_rotation(Point3::from(pos1), rot1);
    let pose2 = Pose::from_position_rotation(Point3::from(pos2), rot2);

    // 7. GJK/EPA query
    if let Some(contact) = gjk_epa_contact(
        &shape1,
        &pose1,
        &shape2,
        &pose2,
        model.ccd_iterations,
        model.ccd_tolerance,
    ) {
        // Overlapping — signed distance is negative penetration
        let dist = -contact.penetration;
        // Surface points: from = geom1 surface, to = geom2 surface
        // EPA contact.point is on shape A's surface (approximately).
        // Decompose using contact normal and penetration depth:
        //   from = contact.point + normal * penetration/2 (on geom1 surface)
        //   to = contact.point - normal * penetration/2 (on geom2 surface)
        let half_pen = contact.penetration * 0.5;
        let from_pt = contact.point + contact.normal * half_pen;
        let to_pt = contact.point - contact.normal * half_pen;
        let fromto = [from_pt.x, from_pt.y, from_pt.z, to_pt.x, to_pt.y, to_pt.z];
        if cutoff > 0.0 && dist >= cutoff {
            return (cutoff, zero_fromto);
        }
        (dist, fromto)
    } else {
        // Non-overlapping — use GJK distance to get closest points
        if let Some(dist_result) = gjk_distance(
            &shape1,
            &pose1,
            &shape2,
            &pose2,
            model.ccd_iterations,
            model.ccd_tolerance,
        ) {
            let fromto = [
                dist_result.witness_a.x,
                dist_result.witness_a.y,
                dist_result.witness_a.z,
                dist_result.witness_b.x,
                dist_result.witness_b.y,
                dist_result.witness_b.z,
            ];
            if cutoff > 0.0 && dist_result.distance >= cutoff {
                return (cutoff, zero_fromto);
            }
            (dist_result.distance, fromto)
        } else {
            // Touching — distance = 0
            (0.0, zero_fromto)
        }
    }
}

/// Analytic sphere-sphere distance computation.
///
/// Exact and fast — no need for GJK/EPA.
fn sphere_sphere_distance(
    pos1: Vector3<f64>,
    r1: f64,
    pos2: Vector3<f64>,
    r2: f64,
    cutoff: f64,
) -> (f64, [f64; 6]) {
    let zero_fromto = [0.0f64; 6];
    let diff = pos2 - pos1;
    let center_dist = diff.norm();
    let dist = center_dist - r1 - r2;

    // Direction from geom1 center to geom2 center
    let dir = if center_dist > 1e-14 {
        diff / center_dist
    } else {
        Vector3::x() // arbitrary for coincident centers
    };

    // Surface points: from = g1 surface toward g2, to = g2 surface toward g1
    let from_pt = pos1 + r1 * dir;
    let to_pt = pos2 - r2 * dir;
    let fromto = [from_pt.x, from_pt.y, from_pt.z, to_pt.x, to_pt.y, to_pt.z];

    // Cutoff cap (matches mj_geomDistance return-value semantics)
    if cutoff > 0.0 && dist >= cutoff {
        return (cutoff, zero_fromto);
    }
    (dist, fromto)
}
