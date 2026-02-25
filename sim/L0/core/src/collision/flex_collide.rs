//! Flex-rigid collision helpers — narrowphase for flex vertex spheres against rigid geoms.

use super::{assign_friction, assign_imp, assign_margin, assign_ref, contact_param_flex_rigid};
use crate::forward::closest_point_segment;
use crate::types::{Contact, GeomType, Model, compute_tangent_frame};
use nalgebra::{Matrix3, Vector2, Vector3};

/// Narrowphase collision between a sphere (flex vertex) and a rigid geom.
///
/// Returns `(depth, normal, contact_pos)` if penetrating, `None` otherwise.
/// The normal points from the rigid geom surface toward the vertex.
pub fn narrowphase_sphere_geom(
    sphere_pos: Vector3<f64>,
    sphere_radius: f64,
    geom_idx: usize,
    model: &Model,
    geom_pos: Vector3<f64>,
    geom_mat: Matrix3<f64>,
) -> Option<(f64, Vector3<f64>, Vector3<f64>)> {
    let v = sphere_pos;

    let (d_surface, normal) = match model.geom_type[geom_idx] {
        GeomType::Plane => {
            let plane_normal = geom_mat.column(2).into_owned();
            let d = (v - geom_pos).dot(&plane_normal);
            (d, plane_normal)
        }
        GeomType::Sphere => {
            let radius = model.geom_size[geom_idx].x;
            let diff = v - geom_pos;
            let dist = diff.norm();
            if dist < 1e-10 {
                (-radius, Vector3::new(0.0, 0.0, 1.0))
            } else {
                (dist - radius, diff / dist)
            }
        }
        GeomType::Box => {
            let half = model.geom_size[geom_idx];
            let v_local = geom_mat.transpose() * (v - geom_pos);
            let clamped = Vector3::new(
                v_local.x.clamp(-half.x, half.x),
                v_local.y.clamp(-half.y, half.y),
                v_local.z.clamp(-half.z, half.z),
            );
            let diff_local = v_local - clamped;
            let dist_outside = diff_local.norm();
            if dist_outside > 1e-10 {
                let normal_world = geom_mat * (diff_local / dist_outside);
                (dist_outside, normal_world)
            } else {
                let dx = half.x - v_local.x.abs();
                let dy = half.y - v_local.y.abs();
                let dz = half.z - v_local.z.abs();
                let min_depth = dx.min(dy).min(dz);
                let normal_local = if (dx - min_depth).abs() < 1e-10 {
                    Vector3::new(v_local.x.signum(), 0.0, 0.0)
                } else if (dy - min_depth).abs() < 1e-10 {
                    Vector3::new(0.0, v_local.y.signum(), 0.0)
                } else {
                    Vector3::new(0.0, 0.0, v_local.z.signum())
                };
                (-min_depth, geom_mat * normal_local)
            }
        }
        GeomType::Capsule => {
            let radius = model.geom_size[geom_idx].x;
            let half_len = model.geom_size[geom_idx].y;
            let axis = geom_mat.column(2).into_owned();
            let a = geom_pos - axis * half_len;
            let b = geom_pos + axis * half_len;
            let closest = closest_point_segment(a, b, v);
            let diff = v - closest;
            let dist = diff.norm();
            if dist < 1e-10 {
                (-radius, Vector3::new(0.0, 0.0, 1.0))
            } else {
                (dist - radius, diff / dist)
            }
        }
        GeomType::Cylinder => {
            let radius = model.geom_size[geom_idx].x;
            let half_len = model.geom_size[geom_idx].y;
            let v_local = geom_mat.transpose() * (v - geom_pos);
            let radial_dist = Vector2::new(v_local.x, v_local.y).norm();
            let on_barrel = v_local.z.abs() <= half_len;
            let in_radial = radial_dist <= radius;

            if on_barrel && in_radial {
                let d_radial = radius - radial_dist;
                let d_axial = half_len - v_local.z.abs();
                if d_radial < d_axial {
                    let nl = if radial_dist > 1e-10 {
                        Vector3::new(v_local.x / radial_dist, v_local.y / radial_dist, 0.0)
                    } else {
                        Vector3::new(1.0, 0.0, 0.0)
                    };
                    (-d_radial, geom_mat * nl)
                } else {
                    let nl = Vector3::new(0.0, 0.0, v_local.z.signum());
                    (-d_axial, geom_mat * nl)
                }
            } else if on_barrel {
                let nl = if radial_dist > 1e-10 {
                    Vector3::new(v_local.x / radial_dist, v_local.y / radial_dist, 0.0)
                } else {
                    Vector3::new(1.0, 0.0, 0.0)
                };
                (radial_dist - radius, geom_mat * nl)
            } else if in_radial {
                let nl = Vector3::new(0.0, 0.0, v_local.z.signum());
                (v_local.z.abs() - half_len, geom_mat * nl)
            } else {
                let z_clamped = v_local.z.clamp(-half_len, half_len);
                let closest_local = Vector3::new(
                    v_local.x * radius / radial_dist,
                    v_local.y * radius / radial_dist,
                    z_clamped,
                );
                let diff_local = Vector3::new(v_local.x, v_local.y, v_local.z) - closest_local;
                let dist = diff_local.norm();
                if dist < 1e-10 {
                    return None;
                }
                (dist, geom_mat * (diff_local / dist))
            }
        }
        GeomType::Ellipsoid => {
            let radii = model.geom_size[geom_idx];
            let v_local = geom_mat.transpose() * (v - geom_pos);
            let v_scaled = Vector3::new(
                v_local.x / radii.x,
                v_local.y / radii.y,
                v_local.z / radii.z,
            );
            let dist_scaled = v_scaled.norm();
            if dist_scaled < 1e-10 {
                return None;
            }
            let d_approx = (dist_scaled - 1.0) * radii.x.min(radii.y).min(radii.z);
            let grad_local = Vector3::new(
                v_local.x / (radii.x * radii.x),
                v_local.y / (radii.y * radii.y),
                v_local.z / (radii.z * radii.z),
            );
            let grad_len = grad_local.norm();
            if grad_len < 1e-10 {
                return None;
            }
            (d_approx, geom_mat * (grad_local / grad_len))
        }
        // Mesh/Hfield/Sdf: not yet supported for flex-vertex collision
        _ => return None,
    };

    let depth = sphere_radius - d_surface;
    if depth <= 0.0 {
        return None;
    }

    let contact_pos = v - normal * d_surface;
    Some((depth, normal, contact_pos))
}

/// Create a Contact for a flex-rigid collision.
///
/// Uses the unified `contact_param_flex_rigid()` function for parameter
/// combination, mirroring `make_contact_from_geoms()` for rigid-rigid contacts.
pub fn make_contact_flex_rigid(
    model: &Model,
    vertex_idx: usize,
    geom_idx: usize,
    pos: Vector3<f64>,
    normal: Vector3<f64>,
    depth: f64,
) -> Contact {
    let flex_id = model.flexvert_flexid[vertex_idx];
    let (condim, gap, solref, solimp, mu) = contact_param_flex_rigid(model, flex_id, geom_idx);
    // S10: apply global override to solver params when ENABLE_OVERRIDE is active.
    let solref = assign_ref(model, &solref);
    let solimp = assign_imp(model, &solimp);
    let mu = assign_friction(model, &mu);
    // Effective margin = flex_margin + geom_margin, overridden by assign_margin.
    let margin = assign_margin(
        model,
        model.flex_margin[flex_id] + model.geom_margin[geom_idx],
    );
    let includemargin = margin - gap;

    let (t1, t2) = compute_tangent_frame(&normal);

    let dim: usize = match condim {
        1 => 1,
        4 => 4,
        _ => 3, // condim 3 and any other value default to 3D friction cone
    };

    Contact {
        pos,
        normal,
        depth,
        // For flex contacts, geom1 = geom2 = the rigid geom index.
        // The vertex index is stored in flex_vertex.
        // This ensures model.geom_body[contact.geom1] is always valid.
        geom1: geom_idx,
        geom2: geom_idx,
        friction: mu[0],
        dim,
        includemargin,
        mu,
        solref,
        solreffriction: [0.0, 0.0],
        solimp,
        frame: (t1, t2).into(),
        flex_vertex: Some(vertex_idx),
    }
}
