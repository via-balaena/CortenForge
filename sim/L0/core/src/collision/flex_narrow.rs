//! Sphere-geom narrowphase for flex vertex collisions, and contact constructors
//! for flex-rigid, flex-self, and flex-flex collision pairs.

use super::{
    assign_friction, assign_imp, assign_margin, assign_ref, contact_param_flex_flex,
    contact_param_flex_rigid, contact_param_flex_self,
};
use crate::forward::closest_point_segment;
use crate::gjk_epa::gjk_epa_contact;
use crate::heightfield::heightfield_sphere_contact;
use crate::mesh::mesh_sphere_contact;
use crate::sdf::sdf_sphere_contact;
use crate::types::{Contact, DISABLE_MIDPHASE, GeomType, Model, compute_tangent_frame, disabled};
use cf_geometry::Shape;
use nalgebra::{Matrix3, Point3, UnitQuaternion, Vector2, Vector3};
use sim_types::Pose;

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
        GeomType::Mesh => {
            let mesh_id = model.geom_mesh[geom_idx]?;
            let mesh = &model.mesh_data[mesh_id];
            let geom_quat = UnitQuaternion::from_matrix(&geom_mat);
            let mesh_pose = Pose::from_position_rotation(Point3::from(geom_pos), geom_quat);

            // Primary path: convex hull GJK (MuJoCo-conformant)
            if let Some(hull) = mesh.convex_hull() {
                let hull_shape = Shape::convex_mesh(hull.clone());
                let sphere_shape = Shape::Sphere {
                    radius: sphere_radius,
                };
                let sphere_pose =
                    Pose::from_position_rotation(Point3::from(v), UnitQuaternion::identity());

                if let Some(gjk) = gjk_epa_contact(
                    &sphere_shape,
                    &sphere_pose,
                    &hull_shape,
                    &mesh_pose,
                    model.ccd_iterations,
                    model.ccd_tolerance,
                ) {
                    // GJK/EPA returns normal from shape_a toward shape_b
                    // (see hfield.rs:170 comment). Here A=sphere, B=hull, so
                    // the raw normal points from vertex toward mesh. Negate to
                    // get outward-from-mesh-toward-vertex convention.
                    return Some((gjk.penetration, -gjk.normal, gjk.point.coords));
                }
                return None;
            }

            // Fallback: per-triangle BVH (meshes without hull)
            let use_bvh = !disabled(model, DISABLE_MIDPHASE);
            if let Some(mc) =
                mesh_sphere_contact(mesh, &mesh_pose, Point3::from(v), sphere_radius, use_bvh)
            {
                return Some((mc.penetration, mc.normal, mc.point.coords));
            }
            return None;
        }
        GeomType::Hfield => {
            let hfield_id = model.geom_hfield[geom_idx]?;
            let hfield = &model.hfield_data[hfield_id];
            let hf_size = &model.hfield_size[hfield_id];

            // Centering offset: convert center-origin to corner-origin
            let hf_offset = geom_mat * Vector3::new(-hf_size[0], -hf_size[1], 0.0);
            let geom_quat = UnitQuaternion::from_matrix(&geom_mat);
            let hf_pose =
                Pose::from_position_rotation(Point3::from(geom_pos + hf_offset), geom_quat);

            if let Some(hc) =
                heightfield_sphere_contact(hfield, &hf_pose, Point3::from(v), sphere_radius)
            {
                return Some((hc.penetration, hc.normal, hc.point.coords));
            }
            return None;
        }
        GeomType::Sdf => {
            let sdf_id = model.geom_shape[geom_idx]?;
            let sdf = model.shape_data[sdf_id].sdf_grid();
            let geom_quat = UnitQuaternion::from_matrix(&geom_mat);
            let sdf_pose = Pose::from_position_rotation(Point3::from(geom_pos), geom_quat);

            if let Some(sc) = sdf_sphere_contact(sdf, &sdf_pose, Point3::from(v), sphere_radius) {
                return Some((sc.penetration, sc.normal, sc.point.coords));
            }
            return None;
        } // All GeomType variants handled — exhaustive match.
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
        solreffriction: assign_ref(model, &[0.0, 0.0]),
        solimp,
        frame: (t1, t2).into(),
        // Safety: vertex_idx comes from iterating 0..model.nflexvert in
        // mj_collision_flex, so it is always in-bounds. Rust's bounds-checked
        // indexing would panic loudly if a malformed model violated this.
        // No additional validation needed (Phase 8 DT-25 audit decision).
        flex_vertex: Some(vertex_idx),
        flex_vertex2: None,
    }
}

/// Create a Contact for a flex self-collision.
///
/// Side 1: penetrating vertex (`flex_vertex`).
/// Side 2: nearest opposing vertex (`flex_vertex2`).
/// No rigid geom — `geom1`/`geom2` set to `usize::MAX` (sentinel).
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

    let (t1, t2) = compute_tangent_frame(&normal);

    let dim: usize = match condim {
        1 => 1,
        4 => 4,
        _ => 3,
    };

    Contact {
        pos,
        normal,
        depth,
        geom1: usize::MAX, // sentinel — no rigid geom
        geom2: usize::MAX,
        friction: mu[0],
        dim,
        includemargin,
        mu,
        solref,
        solreffriction: assign_ref(model, &[0.0, 0.0]),
        solimp,
        frame: (t1, t2).into(),
        flex_vertex: Some(vertex1),
        flex_vertex2: Some(vertex2),
    }
}

/// Create a Contact for a flex-flex cross-object collision.
///
/// Side 1: nearest vertex from flex1 (`flex_vertex`).
/// Side 2: nearest vertex from flex2 (`flex_vertex2`).
/// No rigid geom — `geom1`/`geom2` set to `usize::MAX` (sentinel).
/// Margin = flex_margin[f1] + flex_margin[f2] (additive, not doubled).
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
    let (condim, gap, solref, solimp, mu) = contact_param_flex_flex(model, flex_id1, flex_id2);
    let solref = assign_ref(model, &solref);
    let solimp = assign_imp(model, &solimp);
    let mu = assign_friction(model, &mu);
    let margin = assign_margin(
        model,
        model.flex_margin[flex_id1] + model.flex_margin[flex_id2],
    );
    let includemargin = margin - gap;

    let (t1, t2) = compute_tangent_frame(&normal);

    let dim: usize = match condim {
        1 => 1,
        4 => 4,
        _ => 3,
    };

    Contact {
        pos,
        normal,
        depth,
        geom1: usize::MAX, // sentinel — no rigid geom
        geom2: usize::MAX,
        friction: mu[0],
        dim,
        includemargin,
        mu,
        solref,
        solreffriction: assign_ref(model, &[0.0, 0.0]),
        solimp,
        frame: (t1, t2).into(),
        flex_vertex: Some(vertex1),
        flex_vertex2: Some(vertex2),
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
