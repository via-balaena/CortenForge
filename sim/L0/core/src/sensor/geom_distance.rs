//! Geom distance computation for distance/normal/fromto sensors.
//!
//! Implements `geom_distance()` — the CortenForge equivalent of MuJoCo's
//! `mj_geomDistance()` in `engine_support.c`. Returns the signed distance
//! and nearest surface points between two geoms.

use crate::collision::narrow::geom_to_collision_shape;
use crate::gjk_epa::{gjk_epa_contact, gjk_query};
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

    // 5. Convert to CollisionShape (convex primitives only)
    let Some(shape1) = geom_to_collision_shape(gtype1, size1) else {
        // Non-convex or plane — deferred (DT-122)
        return (cutoff, zero_fromto);
    };
    let Some(shape2) = geom_to_collision_shape(gtype2, size2) else {
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
    if let Some(contact) = gjk_epa_contact(&shape1, &pose1, &shape2, &pose2) {
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
        // Non-overlapping — use GJK simplex to get closest points
        let result = gjk_query(&shape1, &pose1, &shape2, &pose2);
        let (closest_a, closest_b) = closest_points_from_simplex(&result.simplex);
        let diff = closest_a - closest_b;
        let dist = diff.norm();
        let fromto = [
            closest_a.x,
            closest_a.y,
            closest_a.z,
            closest_b.x,
            closest_b.y,
            closest_b.z,
        ];
        if cutoff > 0.0 && dist >= cutoff {
            return (cutoff, zero_fromto);
        }
        (dist, fromto)
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

/// Extract closest points on shapes A and B from the GJK simplex.
///
/// When GJK terminates without finding intersection, the simplex contains
/// 1-3 points that form the closest feature on the Minkowski difference to
/// the origin. Each simplex vertex stores the original support points from
/// both shapes. We find the closest point on the simplex to the origin
/// (using barycentric coordinates), then reconstruct the individual closest
/// points on A and B using those same barycentric weights.
#[allow(clippy::many_single_char_names)]
fn closest_points_from_simplex(simplex: &crate::gjk_epa::Simplex) -> (Point3<f64>, Point3<f64>) {
    let pts = simplex.points();
    match pts.len() {
        0 => (Point3::origin(), Point3::origin()),
        1 => {
            // Single point — closest is the support points themselves
            (pts[0].support_a, pts[0].support_b)
        }
        2 => {
            // Line segment — project origin onto segment, clamp to [0, 1]
            let a = pts[0].point.coords;
            let b = pts[1].point.coords;
            let ab = b - a;
            let ab_sq = ab.dot(&ab);
            if ab_sq < 1e-14 {
                return (pts[0].support_a, pts[0].support_b);
            }
            let t = (-a.dot(&ab) / ab_sq).clamp(0.0, 1.0);
            let closest_a = pts[0].support_a.coords * (1.0 - t) + pts[1].support_a.coords * t;
            let closest_b = pts[0].support_b.coords * (1.0 - t) + pts[1].support_b.coords * t;
            (Point3::from(closest_a), Point3::from(closest_b))
        }
        _ => {
            // Triangle (3+ points) — project origin onto triangle, use barycentric coords
            let a = pts[0].point.coords;
            let b = pts[1].point.coords;
            let c = pts[2].point.coords;
            let (u, v, w) = barycentric_closest_point_triangle(a, b, c);
            let closest_a = pts[0].support_a.coords * u
                + pts[1].support_a.coords * v
                + pts[2].support_a.coords * w;
            let closest_b = pts[0].support_b.coords * u
                + pts[1].support_b.coords * v
                + pts[2].support_b.coords * w;
            (Point3::from(closest_a), Point3::from(closest_b))
        }
    }
}

/// Compute barycentric coordinates of the closest point on triangle ABC to the origin.
///
/// Returns (u, v, w) such that closest_point = u*A + v*B + w*C and u + v + w = 1.
/// Handles all Voronoi regions (vertex, edge, face).
#[allow(clippy::many_single_char_names)]
fn barycentric_closest_point_triangle(
    a: Vector3<f64>,
    b: Vector3<f64>,
    c: Vector3<f64>,
) -> (f64, f64, f64) {
    let ab = b - a;
    let ac = c - a;
    let ao = -a; // origin - a

    let d1 = ab.dot(&ao);
    let d2 = ac.dot(&ao);
    // Vertex A region
    if d1 <= 0.0 && d2 <= 0.0 {
        return (1.0, 0.0, 0.0);
    }

    let bo = -b;
    let d3 = ab.dot(&bo);
    let d4 = ac.dot(&bo);
    // Vertex B region
    if d3 >= 0.0 && d4 <= d3 {
        return (0.0, 1.0, 0.0);
    }

    // Edge AB region
    let vc = d1 * d4 - d3 * d2;
    if vc <= 0.0 && d1 >= 0.0 && d3 <= 0.0 {
        let v = d1 / (d1 - d3);
        return (1.0 - v, v, 0.0);
    }

    let co = -c;
    let d5 = ab.dot(&co);
    let d6 = ac.dot(&co);
    // Vertex C region
    if d6 >= 0.0 && d5 <= d6 {
        return (0.0, 0.0, 1.0);
    }

    // Edge AC region
    let vb = d5 * d2 - d1 * d6;
    if vb <= 0.0 && d2 >= 0.0 && d6 <= 0.0 {
        let w = d2 / (d2 - d6);
        return (1.0 - w, 0.0, w);
    }

    // Edge BC region
    let va = d3 * d6 - d5 * d4;
    if va <= 0.0 && (d4 - d3) >= 0.0 && (d5 - d6) >= 0.0 {
        let w = (d4 - d3) / ((d4 - d3) + (d5 - d6));
        return (0.0, 1.0 - w, w);
    }

    // Inside face region
    let denom = 1.0 / (va + vb + vc);
    let v = vb * denom;
    let w = vc * denom;
    (1.0 - v - w, v, w)
}
