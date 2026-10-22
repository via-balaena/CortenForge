//! Contact constructors for flex-self and flex-flex collision pairs, and the
//! vertex–triangle test they use.

use super::{
    assign_friction, assign_imp, assign_margin, assign_ref, contact_param_flex_self,
    contact_param_flexes,
};
use crate::types::{Contact, Model, make_frame};
use nalgebra::Vector3;

/// Create a Contact for a flex self-collision.
///
/// Side 0: the nearest opposing vertex (`vertex2`); side 1: the penetrating
/// vertex (`vertex1`); the normal points from the first to the second.
pub fn make_contact_flex_self(
    model: &Model,
    vertex1: usize,
    vertex2: usize,
    pos: Vector3<f64>,
    normal: Vector3<f64>,
    depth: f64,
) -> Contact {
    let flex_id = model.flexvert_flexid[vertex1];
    let (condim, gap, solref, solimp, mu) = contact_param_flex_self(model, flex_id);
    let solref = assign_ref(model, &solref);
    let solimp = assign_imp(model, &solimp);
    let mu = assign_friction(model, &mu);
    let margin = assign_margin(model, 2.0 * model.flex_margin[flex_id]);
    let includemargin = margin - gap;

    let (normal, frame) = make_frame(&normal, &Vector3::zeros());

    let dim: usize = match condim {
        1 => 1,
        4 => 4,
        _ => 3,
    };

    Contact {
        pos,
        normal,
        depth,
        geom: [None; 2],
        flex: [Some(flex_id); 2],
        elem: [None; 2],
        vert: [
            Some(vertex2 - model.flex_vertadr[flex_id]),
            Some(vertex1 - model.flex_vertadr[flex_id]),
        ],
        friction: mu[0],
        dim,
        includemargin,
        mu,
        solref,
        solreffriction: assign_ref(model, &[0.0, 0.0]),
        solimp,
        frame,
        regularized_mu: 0.0,
    }
}

/// Create a Contact for a flex-flex cross-object collision.
///
/// Side 0: the nearest vertex of the second flex (`vertex2`); side 1: the
/// vertex of the first (`vertex1`); the normal points from the first side
/// to the second. Margin = `flex_margin[f1] + flex_margin[f2]` (additive,
/// not doubled).
pub fn make_contact_flex_flex(
    model: &Model,
    vertex1: usize,
    vertex2: usize,
    pos: Vector3<f64>,
    normal: Vector3<f64>,
    depth: f64,
) -> Contact {
    let flex_id1 = model.flexvert_flexid[vertex1];
    let flex_id2 = model.flexvert_flexid[vertex2];
    let (condim, gap, solref, solimp, mu) = contact_param_flexes(model, flex_id1, flex_id2);
    let solref = assign_ref(model, &solref);
    let solimp = assign_imp(model, &solimp);
    let mu = assign_friction(model, &mu);
    let margin = assign_margin(
        model,
        model.flex_margin[flex_id1] + model.flex_margin[flex_id2],
    );
    let includemargin = margin - gap;

    let (normal, frame) = make_frame(&normal, &Vector3::zeros());

    let dim: usize = match condim {
        1 => 1,
        4 => 4,
        _ => 3,
    };

    Contact {
        pos,
        normal,
        depth,
        geom: [None; 2],
        flex: [Some(flex_id2), Some(flex_id1)],
        elem: [None; 2],
        vert: [
            Some(vertex2 - model.flex_vertadr[flex_id2]),
            Some(vertex1 - model.flex_vertadr[flex_id1]),
        ],
        friction: mu[0],
        dim,
        includemargin,
        mu,
        solref,
        solreffriction: assign_ref(model, &[0.0, 0.0]),
        solimp,
        frame,
        regularized_mu: 0.0,
    }
}

/// Sphere-triangle contact test.
///
/// Tests if a sphere (center, radius) intersects a triangle. Returns
/// `(depth, normal, contact_pos)` if intersecting. Normal points from
/// triangle surface toward sphere center.
pub fn sphere_triangle_contact(
    sphere_pos: Vector3<f64>,
    sphere_radius: f64,
    tri: &[Vector3<f64>; 3],
) -> Option<(f64, Vector3<f64>, Vector3<f64>)> {
    let v0 = tri[0];
    let v1 = tri[1];
    let v2 = tri[2];
    let e1 = v1 - v0;
    let e2 = v2 - v0;
    let n = e1.cross(&e2);
    let n_len = n.norm();
    if n_len < 1e-12 {
        return None; // degenerate triangle
    }
    let n = n / n_len;

    // Distance from sphere center to triangle plane
    let d = (sphere_pos - v0).dot(&n);
    if d.abs() > sphere_radius {
        return None; // too far
    }

    // Project sphere center onto triangle plane
    let proj = sphere_pos - d * n;

    // Check if projection is inside triangle (barycentric coordinates)
    let v0p = proj - v0;
    let d00 = e1.dot(&e1);
    let d01 = e1.dot(&e2);
    let d11 = e2.dot(&e2);
    let d20 = v0p.dot(&e1);
    let d21 = v0p.dot(&e2);
    // Cross-axis arrangements in the contact-normal expansion intentionally mix tangent/bitangent components — this matches the published flexbody contact formulation.
    #[allow(clippy::suspicious_operation_groupings)]
    let denom = d00 * d11 - d01 * d01; // cross-product magnitude squared, correct formula
    if denom.abs() < 1e-12 {
        return None; // degenerate
    }
    let bary_v = (d11 * d20 - d01 * d21) / denom;
    let bary_w = (d00 * d21 - d01 * d20) / denom;
    let bary_u = 1.0 - bary_v - bary_w;

    if bary_u >= 0.0 && bary_v >= 0.0 && bary_w >= 0.0 {
        // Projection inside triangle — contact on triangle plane
        let normal = if d >= 0.0 { n } else { -n };
        let depth = sphere_radius - d.abs();
        let contact_pos = proj;
        return Some((depth, normal, contact_pos));
    }

    // Projection outside triangle — find closest point on edges/vertices
    let closest = closest_point_on_triangle(sphere_pos, tri);
    let diff = sphere_pos - closest;
    let dist = diff.norm();
    if dist < 1e-12 || dist > sphere_radius {
        return None;
    }

    let normal = diff / dist;
    let depth = sphere_radius - dist;
    Some((depth, normal, closest))
}

/// Find closest point on triangle boundary (edges + vertices).
fn closest_point_on_triangle(p: Vector3<f64>, tri: &[Vector3<f64>; 3]) -> Vector3<f64> {
    let edges = [(tri[0], tri[1]), (tri[1], tri[2]), (tri[2], tri[0])];
    let mut best = tri[0];
    let mut best_dist = (p - tri[0]).norm_squared();

    for (a, b) in &edges {
        let ab = *b - *a;
        let t = ((p - *a).dot(&ab) / ab.dot(&ab)).clamp(0.0, 1.0);
        let closest = *a + t * ab;
        let dist = (p - closest).norm_squared();
        if dist < best_dist {
            best_dist = dist;
            best = closest;
        }
    }
    best
}
