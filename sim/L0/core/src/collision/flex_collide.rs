//! Flex collision helpers — narrowphase for flex vertex spheres against rigid geoms,
//! and flex self-collision (internal + non-adjacent element pairs).

use super::{
    assign_friction, assign_imp, assign_margin, assign_ref, contact_param_flex_rigid,
    contact_param_flex_self,
};
use crate::collision_shape::{Aabb, CollisionShape};
use crate::forward::closest_point_segment;
use crate::gjk_epa::gjk_epa_contact;
use crate::heightfield::heightfield_sphere_contact;
use crate::mesh::mesh_sphere_contact;
use crate::mid_phase::{Bvh, BvhPrimitive};
use crate::sdf::sdf_sphere_contact;
use crate::types::{
    Contact, DISABLE_MIDPHASE, Data, GeomType, Model, compute_tangent_frame, disabled,
};
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
                let hull_shape = CollisionShape::convex_mesh_from_hull(hull);
                let sphere_shape = CollisionShape::Sphere {
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
            let sdf_id = model.geom_sdf[geom_idx]?;
            let sdf = &model.sdf_data[sdf_id];
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

// =============================================================================
// S6: Internal collision (adjacent elements)
// =============================================================================

/// Internal collision: contacts between adjacent elements within a flex.
///
/// For each adjacent element pair, tests non-shared vertices of each element
/// against faces of the other element. Generates vertex-face contacts where
/// the vertex sphere penetrates the opposing face within margin.
pub fn mj_collide_flex_internal(model: &Model, data: &mut Data, f: usize) {
    let elem_start = model.flex_elemadr[f];
    let elem_count = model.flex_elemnum[f];
    let dim = model.flex_dim[f];

    if elem_count < 2 {
        return;
    }

    // Iterate unique adjacent pairs
    for local_e in 0..elem_count {
        let e1 = elem_start + local_e;
        let adr = model.flex_elem_adj_adr[e1];
        let num = model.flex_elem_adj_num[e1];
        for &e2 in &model.flex_elem_adj[adr..adr + num] {
            // Only process each pair once (e1 < e2)
            if e2 <= e1 {
                continue;
            }
            // Both elements must belong to the same flex
            if model.flexelem_flexid[e2] != f {
                continue;
            }

            // Get vertex sets for both elements
            let verts1 = elem_vertices(model, e1);
            let verts2 = elem_vertices(model, e2);

            // Test non-shared vertices of e1 against faces of e2
            for &v in &verts1 {
                if !verts2.contains(&v) {
                    test_vertex_against_element_faces(model, data, v, &verts2, dim, f);
                }
            }
            // Test non-shared vertices of e2 against faces of e1
            for &v in &verts2 {
                if !verts1.contains(&v) {
                    test_vertex_against_element_faces(model, data, v, &verts1, dim, f);
                }
            }
        }
    }
}

/// Get vertex indices for an element.
fn elem_vertices(model: &Model, elem: usize) -> Vec<usize> {
    let adr = model.flexelem_dataadr[elem];
    let num = model.flexelem_datanum[elem];
    model.flexelem_data[adr..adr + num].to_vec()
}

/// Test a vertex sphere against the faces of an element.
///
/// For dim=2 (triangle): one face (the triangle itself).
/// For dim=3 (tetrahedron): four triangular faces.
/// For dim=1 (edge): skip (no faces for vertex-face test).
fn test_vertex_against_element_faces(
    model: &Model,
    data: &mut Data,
    vertex: usize,
    elem_verts: &[usize],
    dim: usize,
    flex_id: usize,
) {
    let vpos = data.flexvert_xpos[vertex];
    let radius = model.flexvert_radius[vertex];
    let margin = 2.0 * model.flex_margin[flex_id];
    let gap = 2.0 * model.flex_gap[flex_id];
    let includemargin = margin - gap;

    match dim {
        2 => {
            // Triangle face: elem_verts = [v0, v1, v2]
            if elem_verts.len() < 3 {
                return;
            }
            let tri = [
                data.flexvert_xpos[elem_verts[0]],
                data.flexvert_xpos[elem_verts[1]],
                data.flexvert_xpos[elem_verts[2]],
            ];
            if let Some((depth, normal, contact_pos)) = sphere_triangle_contact(vpos, radius, &tri)
            {
                if depth > -includemargin {
                    let nearest = nearest_vertex(vpos, elem_verts, data);
                    let contact =
                        make_contact_flex_self(model, vertex, nearest, contact_pos, normal, depth);
                    data.contacts.push(contact);
                    data.ncon += 1;
                }
            }
        }
        3 => {
            // Tetrahedron: 4 triangular faces
            if elem_verts.len() < 4 {
                return;
            }
            let faces = [
                [elem_verts[0], elem_verts[1], elem_verts[2]],
                [elem_verts[0], elem_verts[1], elem_verts[3]],
                [elem_verts[0], elem_verts[2], elem_verts[3]],
                [elem_verts[1], elem_verts[2], elem_verts[3]],
            ];
            for face in &faces {
                let tri = [
                    data.flexvert_xpos[face[0]],
                    data.flexvert_xpos[face[1]],
                    data.flexvert_xpos[face[2]],
                ];
                if let Some((depth, normal, contact_pos)) =
                    sphere_triangle_contact(vpos, radius, &tri)
                {
                    if depth > -includemargin {
                        let nearest = nearest_vertex(vpos, face, data);
                        let contact = make_contact_flex_self(
                            model,
                            vertex,
                            nearest,
                            contact_pos,
                            normal,
                            depth,
                        );
                        data.contacts.push(contact);
                        data.ncon += 1;
                    }
                }
            }
        }
        _ => {} // dim=1: no faces for vertex-face test
    }
}

/// Find the nearest vertex (by Euclidean distance) from a set.
///
/// # Panics
/// Panics if `candidates` is empty (caller guarantees non-empty).
#[allow(clippy::expect_used)]
fn nearest_vertex(pos: Vector3<f64>, candidates: &[usize], data: &Data) -> usize {
    candidates
        .iter()
        .copied()
        .min_by(|&a, &b| {
            let da = (data.flexvert_xpos[a] - pos).norm_squared();
            let db = (data.flexvert_xpos[b] - pos).norm_squared();
            da.partial_cmp(&db).unwrap_or(std::cmp::Ordering::Equal)
        })
        .expect("candidates must not be empty")
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

// =============================================================================
// S7: Self-collision narrowphase (NARROW brute-force)
// =============================================================================

/// Self-collision (NARROW): brute-force all non-adjacent element pairs.
///
/// O(n²) element pair iteration. For each non-adjacent pair, run element-
/// element narrowphase.
pub fn mj_collide_flex_self_narrow(model: &Model, data: &mut Data, f: usize) {
    let elem_start = model.flex_elemadr[f];
    let elem_count = model.flex_elemnum[f];
    let dim = model.flex_dim[f];

    if elem_count < 2 {
        return;
    }

    for i in 0..elem_count {
        let e1 = elem_start + i;
        for j in (i + 1)..elem_count {
            let e2 = elem_start + j;

            // Skip adjacent elements (handled by internal collision)
            if elements_adjacent(model, e1, e2) {
                continue;
            }

            collide_element_pair(model, data, e1, e2, dim, f);
        }
    }
}

/// Check if two elements are adjacent (share at least one vertex).
/// Uses binary search on the precomputed sorted adjacency list.
#[inline]
fn elements_adjacent(model: &Model, e1: usize, e2: usize) -> bool {
    if e1 >= model.flex_elem_adj_adr.len() {
        return false;
    }
    let adr = model.flex_elem_adj_adr[e1];
    let num = model.flex_elem_adj_num[e1];
    if num == 0 {
        return false;
    }
    model.flex_elem_adj[adr..adr + num]
        .binary_search(&e2)
        .is_ok()
}

/// Element-element narrowphase for non-adjacent elements.
///
/// Dispatches based on dim. This is the narrowphase primitive reused by Spec D.
pub fn collide_element_pair(
    model: &Model,
    data: &mut Data,
    e1: usize,
    e2: usize,
    dim: usize,
    flex_id: usize,
) {
    let verts1 = elem_vertices(model, e1);
    let verts2 = elem_vertices(model, e2);

    match dim {
        2 => collide_triangles(model, data, &verts1, &verts2, flex_id),
        3 => collide_tetrahedra(model, data, &verts1, &verts2, flex_id),
        1 => collide_edges(model, data, &verts1, &verts2, flex_id),
        _ => {}
    }
}

/// Triangle-triangle narrowphase for dim=2 self-collision.
///
/// Uses the existing `triangle_triangle_intersection()` (SAT-based) from
/// mesh.rs. Generates at most one contact per triangle pair.
fn collide_triangles(
    model: &Model,
    data: &mut Data,
    verts1: &[usize],
    verts2: &[usize],
    flex_id: usize,
) {
    use crate::mesh::triangle_triangle_intersection;

    let tri_a = [
        Point3::from(data.flexvert_xpos[verts1[0]]),
        Point3::from(data.flexvert_xpos[verts1[1]]),
        Point3::from(data.flexvert_xpos[verts1[2]]),
    ];
    let tri_b = [
        Point3::from(data.flexvert_xpos[verts2[0]]),
        Point3::from(data.flexvert_xpos[verts2[1]]),
        Point3::from(data.flexvert_xpos[verts2[2]]),
    ];

    let margin = 2.0 * model.flex_margin[flex_id];
    let gap = 2.0 * model.flex_gap[flex_id];
    let includemargin = margin - gap;

    if let Some(contact_result) = triangle_triangle_intersection(&tri_a, &tri_b) {
        if contact_result.depth > -includemargin {
            let nearest1 = nearest_vertex(contact_result.point.coords, verts1, data);
            let nearest2 = nearest_vertex(contact_result.point.coords, verts2, data);

            let contact = make_contact_flex_self(
                model,
                nearest1,
                nearest2,
                contact_result.point.coords,
                contact_result.normal,
                contact_result.depth,
            );
            data.contacts.push(contact);
            data.ncon += 1;
        }
    }
}

/// Tetrahedron-tetrahedron narrowphase for dim=3 self-collision.
///
/// Tests each vertex of tet A against faces of tet B (and vice versa).
/// Edge-edge tests deferred to DT-151.
fn collide_tetrahedra(
    model: &Model,
    data: &mut Data,
    verts1: &[usize],
    verts2: &[usize],
    flex_id: usize,
) {
    // Vertex-face: each vertex of tet1 against 4 faces of tet2
    for &v in verts1 {
        test_vertex_against_element_faces(model, data, v, verts2, 3, flex_id);
    }
    // Vertex-face: each vertex of tet2 against 4 faces of tet1
    for &v in verts2 {
        test_vertex_against_element_faces(model, data, v, verts1, 3, flex_id);
    }
}

/// Edge-edge proximity for dim=1 self-collision.
///
/// Tests two cable segments (each = 2 vertices) for proximity.
fn collide_edges(
    model: &Model,
    data: &mut Data,
    verts1: &[usize],
    verts2: &[usize],
    flex_id: usize,
) {
    let p0 = data.flexvert_xpos[verts1[0]];
    let p1 = data.flexvert_xpos[verts1[1]];
    let q0 = data.flexvert_xpos[verts2[0]];
    let q1 = data.flexvert_xpos[verts2[1]];

    let r1 = model.flexvert_radius[verts1[0]].max(model.flexvert_radius[verts1[1]]);
    let r2 = model.flexvert_radius[verts2[0]].max(model.flexvert_radius[verts2[1]]);
    let combined_radius = r1 + r2;
    let margin = 2.0 * model.flex_margin[flex_id];
    let gap = 2.0 * model.flex_gap[flex_id];
    let includemargin = margin - gap;

    let (closest1, closest2) = closest_points_segments(p0, p1, q0, q1);
    let diff = closest1 - closest2;
    let dist = diff.norm();

    if dist < 1e-12 {
        return; // degenerate
    }
    let depth = combined_radius - dist;
    if depth <= -includemargin {
        return;
    }

    let normal = diff / dist;
    let contact_pos = (closest1 + closest2) * 0.5;

    let v1 = if (closest1 - p0).norm_squared() < (closest1 - p1).norm_squared() {
        verts1[0]
    } else {
        verts1[1]
    };
    let v2 = if (closest2 - q0).norm_squared() < (closest2 - q1).norm_squared() {
        verts2[0]
    } else {
        verts2[1]
    };

    let contact = make_contact_flex_self(model, v1, v2, contact_pos, normal, depth);
    data.contacts.push(contact);
    data.ncon += 1;
}

/// Closest points between two line segments.
///
/// Uses the standard segment-segment closest-point algorithm with
/// degenerate-case handling. Variable names follow the canonical
/// formulation (Ericson, "Real-Time Collision Detection").
#[allow(
    clippy::many_single_char_names,
    clippy::suspicious_operation_groupings,
    clippy::similar_names
)]
fn closest_points_segments(
    seg_a0: Vector3<f64>,
    seg_a1: Vector3<f64>,
    seg_b0: Vector3<f64>,
    seg_b1: Vector3<f64>,
) -> (Vector3<f64>, Vector3<f64>) {
    let dir_a = seg_a1 - seg_a0;
    let dir_b = seg_b1 - seg_b0;
    let rel = seg_a0 - seg_b0;
    let len_a_sq = dir_a.dot(&dir_a);
    let len_b_sq = dir_b.dot(&dir_b);
    let fb = dir_b.dot(&rel);

    if len_a_sq < 1e-12 && len_b_sq < 1e-12 {
        return (seg_a0, seg_b0); // both degenerate to points
    }

    let (param_s, param_t) = if len_a_sq < 1e-12 {
        (0.0, (fb / len_b_sq).clamp(0.0, 1.0))
    } else {
        let fa = dir_a.dot(&rel);
        if len_b_sq < 1e-12 {
            ((-fa / len_a_sq).clamp(0.0, 1.0), 0.0)
        } else {
            let dot_ab = dir_a.dot(&dir_b);
            let denom = len_a_sq * len_b_sq - dot_ab * dot_ab;
            let param_s = if denom.abs() > 1e-12 {
                ((dot_ab * fb - fa * len_b_sq) / denom).clamp(0.0, 1.0)
            } else {
                0.0
            };
            let param_t = ((dot_ab * param_s + fb) / len_b_sq).clamp(0.0, 1.0);
            // Recompute s for clamped t
            let param_s = if len_a_sq.abs() > 1e-12 {
                ((dot_ab * param_t - fa) / len_a_sq).clamp(0.0, 1.0)
            } else {
                param_s
            };
            (param_s, param_t)
        }
    };

    (seg_a0 + param_s * dir_a, seg_b0 + param_t * dir_b)
}

// =============================================================================
// S8: Midphase acceleration (BVH + SAP + AUTO)
// =============================================================================

/// Self-collision with BVH midphase.
///
/// Builds a per-element AABB tree each step (vertex positions change),
/// queries for overlapping element pairs, filters adjacent, runs narrowphase.
pub fn mj_collide_flex_self_bvh(model: &Model, data: &mut Data, f: usize) {
    let elem_start = model.flex_elemadr[f];
    let elem_count = model.flex_elemnum[f];
    let dim = model.flex_dim[f];

    if elem_count < 2 {
        return;
    }

    // Build per-element AABBs
    let primitives: Vec<BvhPrimitive> = (0..elem_count)
        .map(|i| {
            let e = elem_start + i;
            let verts = elem_vertices(model, e);
            let aabb = element_aabb(&verts, data);
            BvhPrimitive {
                aabb,
                index: e,
                data: 0,
            }
        })
        .collect();

    let bvh = Bvh::build(primitives);

    // Query all overlapping pairs
    for i in 0..elem_count {
        let e1 = elem_start + i;
        let verts1 = elem_vertices(model, e1);
        let aabb1 = element_aabb(&verts1, data);

        for candidate_idx in bvh.query(&aabb1) {
            let e2 = elem_start + candidate_idx;

            // Skip self-overlap and duplicate pairs
            if e2 <= e1 {
                continue;
            }

            // Skip adjacent elements
            if elements_adjacent(model, e1, e2) {
                continue;
            }

            collide_element_pair(model, data, e1, e2, dim, f);
        }
    }
}

/// Compute AABB for an element from its vertex positions.
fn element_aabb(verts: &[usize], data: &Data) -> Aabb {
    let first = data.flexvert_xpos[verts[0]];
    let mut min_v = first;
    let mut max_v = first;

    for &v in &verts[1..] {
        let p = data.flexvert_xpos[v];
        min_v = Vector3::new(min_v.x.min(p.x), min_v.y.min(p.y), min_v.z.min(p.z));
        max_v = Vector3::new(max_v.x.max(p.x), max_v.y.max(p.y), max_v.z.max(p.z));
    }

    Aabb {
        min: Point3::from(min_v),
        max: Point3::from(max_v),
    }
}

/// Self-collision with SAP (sweep-and-prune) midphase.
///
/// Computes element AABBs, sorts along axis of maximum variance, sweeps
/// for overlapping projections. Filters adjacent, runs narrowphase.
pub fn mj_collide_flex_self_sap(model: &Model, data: &mut Data, f: usize) {
    let elem_start = model.flex_elemadr[f];
    let elem_count = model.flex_elemnum[f];
    let dim = model.flex_dim[f];

    if elem_count < 2 {
        return;
    }

    // Compute element AABBs
    let mut aabbs: Vec<(usize, Aabb)> = (0..elem_count)
        .map(|i| {
            let e = elem_start + i;
            let verts = elem_vertices(model, e);
            let aabb = element_aabb(&verts, data);
            (e, aabb)
        })
        .collect();

    // Find axis of maximum variance
    let axis = axis_of_max_variance(&aabbs);

    // Sort by AABB min along chosen axis
    aabbs.sort_by(|a, b| {
        let a_min = match axis {
            0 => a.1.min.x,
            1 => a.1.min.y,
            _ => a.1.min.z,
        };
        let b_min = match axis {
            0 => b.1.min.x,
            1 => b.1.min.y,
            _ => b.1.min.z,
        };
        a_min
            .partial_cmp(&b_min)
            .unwrap_or(std::cmp::Ordering::Equal)
    });

    // Sweep: for each element, check subsequent elements until no overlap
    for i in 0..aabbs.len() {
        let (elem_i, ref box_i) = aabbs[i];
        let max_i = match axis {
            0 => box_i.max.x,
            1 => box_i.max.y,
            _ => box_i.max.z,
        };

        for (elem_j, box_j) in &aabbs[(i + 1)..] {
            let min_j = match axis {
                0 => box_j.min.x,
                1 => box_j.min.y,
                _ => box_j.min.z,
            };

            // No more overlaps along sweep axis
            if min_j > max_i {
                break;
            }

            // Check full 3D AABB overlap
            if !aabb_overlap(box_i, box_j) {
                continue;
            }

            // Skip adjacent elements
            let (ea, eb) = if elem_i < *elem_j {
                (elem_i, *elem_j)
            } else {
                (*elem_j, elem_i)
            };
            if elements_adjacent(model, ea, eb) {
                continue;
            }

            collide_element_pair(model, data, ea, eb, dim, f);
        }
    }
}

/// Find axis (0=x, 1=y, 2=z) of maximum AABB centroid variance.
#[allow(clippy::cast_precision_loss)]
fn axis_of_max_variance(aabbs: &[(usize, Aabb)]) -> usize {
    let n = aabbs.len() as f64;
    if n < 2.0 {
        return 0;
    }

    let mut mean = [0.0_f64; 3];
    for (_, aabb) in aabbs {
        let c = (aabb.min.coords + aabb.max.coords) * 0.5;
        mean[0] += c.x;
        mean[1] += c.y;
        mean[2] += c.z;
    }
    mean[0] /= n;
    mean[1] /= n;
    mean[2] /= n;

    let mut var = [0.0_f64; 3];
    for (_, aabb) in aabbs {
        let c = (aabb.min.coords + aabb.max.coords) * 0.5;
        var[0] += (c.x - mean[0]).powi(2);
        var[1] += (c.y - mean[1]).powi(2);
        var[2] += (c.z - mean[2]).powi(2);
    }

    if var[0] >= var[1] && var[0] >= var[2] {
        0
    } else if var[1] >= var[2] {
        1
    } else {
        2
    }
}

/// Test 3D AABB overlap.
fn aabb_overlap(a: &Aabb, b: &Aabb) -> bool {
    a.min.x <= b.max.x
        && a.max.x >= b.min.x
        && a.min.y <= b.max.y
        && a.max.y >= b.min.y
        && a.min.z <= b.max.z
        && a.max.z >= b.min.z
}

// =============================================================================
// Spec E tests — flex-vs-complex-geom narrowphase
// =============================================================================

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used, clippy::cast_precision_loss)]
mod spec_e_tests {
    use super::*;
    use crate::heightfield::HeightFieldData;
    use crate::mesh::TriangleMeshData;
    use crate::sdf::SdfCollisionData;
    use crate::types::Model;
    use nalgebra::{Matrix3, Point3, UnitQuaternion};
    use std::sync::Arc;

    /// Build a minimal Model with one geom at the given type and identity pose.
    fn make_single_geom_model(geom_type: GeomType) -> Model {
        let mut model = Model::empty();
        model.ngeom = 1;
        model.geom_type = vec![geom_type];
        model.geom_body = vec![0];
        model.geom_pos = vec![Vector3::zeros()];
        model.geom_quat = vec![UnitQuaternion::identity()];
        model.geom_size = vec![Vector3::new(0.5, 0.5, 0.5)];
        model.geom_mesh = vec![None];
        model.geom_hfield = vec![None];
        model.geom_sdf = vec![None];
        model.geom_contype = vec![1];
        model.geom_conaffinity = vec![1];
        model.geom_friction = vec![Vector3::new(1.0, 0.005, 0.0001)];
        model.geom_condim = vec![3];
        model.geom_solref = vec![[0.02, 1.0]];
        model.geom_solimp = vec![[0.9, 0.95, 0.001, 0.5, 2.0]];
        model.geom_priority = vec![0];
        model.geom_solmix = vec![1.0];
        model.geom_margin = vec![0.0];
        model.geom_gap = vec![0.0];
        model.geom_group = vec![0];
        model.geom_rgba = vec![[1.0; 4]];
        model.geom_fluid = vec![[0.0; 12]];
        model.geom_name = vec![None];
        model.geom_rbound = vec![1.0];
        model
    }

    /// Unit cube mesh (half-size 0.5) with convex hull.
    fn cube_mesh_with_hull() -> Arc<TriangleMeshData> {
        let h = 0.5;
        let vertices = vec![
            Point3::new(-h, -h, -h),
            Point3::new(-h, -h, h),
            Point3::new(h, -h, h),
            Point3::new(h, -h, -h),
            Point3::new(-h, h, -h),
            Point3::new(-h, h, h),
            Point3::new(h, h, h),
            Point3::new(h, h, -h),
        ];
        let indices = vec![
            0, 1, 2, 0, 2, 3, // -Y face
            4, 6, 5, 4, 7, 6, // +Y face
            0, 4, 5, 0, 5, 1, // -X face
            3, 2, 6, 3, 6, 7, // +X face
            0, 3, 7, 0, 7, 4, // -Z face
            1, 5, 6, 1, 6, 2, // +Z face
        ];
        let mut mesh = TriangleMeshData::new(vertices, indices);
        mesh.compute_convex_hull(None);
        Arc::new(mesh)
    }

    /// Single-triangle mesh (no convex hull).
    fn single_triangle_mesh() -> Arc<TriangleMeshData> {
        let vertices = vec![
            Point3::new(-1.0, -1.0, 0.0),
            Point3::new(1.0, -1.0, 0.0),
            Point3::new(0.0, 1.0, 0.0),
        ];
        let indices = vec![0, 1, 2];
        Arc::new(TriangleMeshData::new(vertices, indices))
    }

    // --- T1: Flex vertex sphere vs mesh with convex hull (AC1) ---

    #[test]
    fn t1_flex_vs_mesh_with_hull() {
        let mut model = make_single_geom_model(GeomType::Mesh);
        let mesh = cube_mesh_with_hull();
        assert!(mesh.convex_hull().is_some());
        model.mesh_data = vec![mesh];
        model.geom_mesh = vec![Some(0)];

        // Vertex at z=0.45, radius=0.1. Sphere bottom at 0.35, hull top at 0.5.
        let result = narrowphase_sphere_geom(
            Vector3::new(0.0, 0.0, 0.45),
            0.1,
            0,
            &model,
            Vector3::zeros(),
            Matrix3::identity(),
        );

        let (depth, normal, _contact_pos) = result.expect("Should generate contact");
        assert!(
            depth > 0.10 && depth < 0.20,
            "depth={depth}, expected ~0.15"
        );
        assert!(normal.z > 0.5, "normal.z={}, expected >0.5", normal.z);
    }

    // --- T2: Flex vertex sphere vs heightfield (AC2) ---

    #[test]
    fn t2_flex_vs_hfield() {
        let mut model = make_single_geom_model(GeomType::Hfield);

        // 5×5 flat heightfield at height=0.5, half-extent 2.0
        let n = 5;
        let hf_size: [f64; 4] = [2.0, 2.0, 0.5, 0.0];
        let cell_size = 2.0 * hf_size[0] / (n - 1) as f64;
        let heights = vec![0.5_f64; n * n];
        let hfield = HeightFieldData::new(heights, n, n, cell_size);
        model.hfield_data = vec![Arc::new(hfield)];
        model.hfield_size = vec![hf_size];
        model.geom_hfield = vec![Some(0)];

        // Vertex at z=0.45, radius=0.1. Sphere bottom at 0.35, terrain at 0.5.
        let result = narrowphase_sphere_geom(
            Vector3::new(0.0, 0.0, 0.45),
            0.1,
            0,
            &model,
            Vector3::zeros(),
            Matrix3::identity(),
        );

        let (depth, normal, _contact_pos) = result.expect("Should generate contact");
        assert!(
            depth > 0.05 && depth < 0.25,
            "depth={depth}, expected ~0.15"
        );
        assert!(normal.z > 0.9, "normal.z={}, expected ~1.0", normal.z);
    }

    // --- T3: Flex vertex sphere vs SDF (AC3) ---

    #[test]
    fn t3_flex_vs_sdf() {
        let mut model = make_single_geom_model(GeomType::Sdf);

        // Sphere SDF, radius 1.0, centered at origin, 32 resolution
        let sdf = SdfCollisionData::sphere(Point3::origin(), 1.0, 32, 1.0);
        model.sdf_data = vec![Arc::new(sdf)];
        model.geom_sdf = vec![Some(0)];

        // Vertex at z=0.95, radius=0.1. Distance to SDF surface = 0.05.
        // Penetration = 0.1 - 0.05 = 0.05.
        let result = narrowphase_sphere_geom(
            Vector3::new(0.0, 0.0, 0.95),
            0.1,
            0,
            &model,
            Vector3::zeros(),
            Matrix3::identity(),
        );

        let (depth, normal, _contact_pos) = result.expect("Should generate contact");
        assert!(
            depth > 0.01 && depth < 0.15,
            "depth={depth}, expected ~0.05"
        );
        assert!(normal.z > 0.5, "normal.z={}, expected >0.5", normal.z);
    }

    // --- T4: Missing geom data guards (AC4, AC5, AC6) ---

    #[test]
    fn t4_missing_data_no_panic() {
        // Mesh with no data
        let model_mesh = make_single_geom_model(GeomType::Mesh);
        assert!(model_mesh.geom_mesh[0].is_none());
        let result = narrowphase_sphere_geom(
            Vector3::new(0.0, 0.0, 0.0),
            0.1,
            0,
            &model_mesh,
            Vector3::zeros(),
            Matrix3::identity(),
        );
        assert!(result.is_none(), "Mesh with no data should return None");

        // Hfield with no data
        let model_hfield = make_single_geom_model(GeomType::Hfield);
        assert!(model_hfield.geom_hfield[0].is_none());
        let result = narrowphase_sphere_geom(
            Vector3::new(0.0, 0.0, 0.0),
            0.1,
            0,
            &model_hfield,
            Vector3::zeros(),
            Matrix3::identity(),
        );
        assert!(result.is_none(), "Hfield with no data should return None");

        // SDF with no data
        let model_sdf = make_single_geom_model(GeomType::Sdf);
        assert!(model_sdf.geom_sdf[0].is_none());
        let result = narrowphase_sphere_geom(
            Vector3::new(0.0, 0.0, 0.0),
            0.1,
            0,
            &model_sdf,
            Vector3::zeros(),
            Matrix3::identity(),
        );
        assert!(result.is_none(), "SDF with no data should return None");
    }

    // --- T5: Vertex above surface — no contact (AC7) ---

    #[test]
    fn t5_above_surface_no_contact() {
        // Mesh: vertex well above
        let mut model = make_single_geom_model(GeomType::Mesh);
        model.mesh_data = vec![cube_mesh_with_hull()];
        model.geom_mesh = vec![Some(0)];
        let result = narrowphase_sphere_geom(
            Vector3::new(0.0, 0.0, 10.0),
            0.1,
            0,
            &model,
            Vector3::zeros(),
            Matrix3::identity(),
        );
        assert!(result.is_none(), "Vertex above mesh should return None");

        // Hfield: vertex well above
        let mut model = make_single_geom_model(GeomType::Hfield);
        let n = 5;
        let hf_size: [f64; 4] = [2.0, 2.0, 0.5, 0.0];
        let cell_size = 2.0 * hf_size[0] / (n - 1) as f64;
        let heights = vec![0.5_f64; n * n];
        let hfield = HeightFieldData::new(heights, n, n, cell_size);
        model.hfield_data = vec![Arc::new(hfield)];
        model.hfield_size = vec![hf_size];
        model.geom_hfield = vec![Some(0)];
        let result = narrowphase_sphere_geom(
            Vector3::new(0.0, 0.0, 10.0),
            0.1,
            0,
            &model,
            Vector3::zeros(),
            Matrix3::identity(),
        );
        assert!(result.is_none(), "Vertex above hfield should return None");

        // SDF: vertex well above
        let mut model = make_single_geom_model(GeomType::Sdf);
        let sdf = SdfCollisionData::sphere(Point3::origin(), 1.0, 32, 1.0);
        model.sdf_data = vec![Arc::new(sdf)];
        model.geom_sdf = vec![Some(0)];
        let result = narrowphase_sphere_geom(
            Vector3::new(0.0, 0.0, 10.0),
            0.1,
            0,
            &model,
            Vector3::zeros(),
            Matrix3::identity(),
        );
        assert!(result.is_none(), "Vertex above SDF should return None");
    }

    // --- T6: Per-triangle BVH fallback (AC10) ---

    #[test]
    fn t6_flex_vs_mesh_per_triangle_fallback() {
        let mut model = make_single_geom_model(GeomType::Mesh);
        let mesh = single_triangle_mesh();
        assert!(
            mesh.convex_hull().is_none(),
            "No hull on single-triangle mesh"
        );
        model.mesh_data = vec![mesh];
        model.geom_mesh = vec![Some(0)];

        // Triangle in Z=0 plane. Vertex at z=-0.05, radius=0.1.
        // Sphere top at z=0.05, triangle at z=0.0 → penetrating.
        let result = narrowphase_sphere_geom(
            Vector3::new(0.0, 0.0, -0.05),
            0.1,
            0,
            &model,
            Vector3::zeros(),
            Matrix3::identity(),
        );

        let (depth, _normal, _contact_pos) = result.expect("BVH fallback should generate contact");
        assert!(depth > 0.0, "depth={depth}, expected >0");
    }

    // --- TS1: Zero-radius vertex vs mesh ---

    #[test]
    fn ts1_zero_radius_vs_mesh() {
        let mut model = make_single_geom_model(GeomType::Mesh);
        model.mesh_data = vec![cube_mesh_with_hull()];
        model.geom_mesh = vec![Some(0)];

        // Zero-radius sphere inside cube → should not panic
        let result = narrowphase_sphere_geom(
            Vector3::new(0.0, 0.0, 0.0),
            0.0,
            0,
            &model,
            Vector3::zeros(),
            Matrix3::identity(),
        );
        // Zero radius means no penetration extent — result may be None or Some
        // depending on GJK/EPA; the key requirement is no panic.
        let _ = result;
    }

    // --- TS2: Hfield vertex on surface boundary ---

    #[test]
    fn ts2_hfield_vertex_on_surface() {
        let mut model = make_single_geom_model(GeomType::Hfield);
        let n = 5;
        let hf_size: [f64; 4] = [2.0, 2.0, 0.5, 0.0];
        let cell_size = 2.0 * hf_size[0] / (n - 1) as f64;
        let heights = vec![0.5_f64; n * n];
        let hfield = HeightFieldData::new(heights, n, n, cell_size);
        model.hfield_data = vec![Arc::new(hfield)];
        model.hfield_size = vec![hf_size];
        model.geom_hfield = vec![Some(0)];

        // Vertex at z=0.55, radius=0.05 → sphere bottom at 0.50 = terrain height
        // Boundary: penetration ≈ 0. Should not panic regardless of result.
        let result = narrowphase_sphere_geom(
            Vector3::new(0.0, 0.0, 0.55),
            0.05,
            0,
            &model,
            Vector3::zeros(),
            Matrix3::identity(),
        );
        let _ = result; // No panic is the key requirement
    }

    // --- TS3: SDF vertex outside bounding box ---

    #[test]
    fn ts3_sdf_vertex_outside_bbox() {
        let mut model = make_single_geom_model(GeomType::Sdf);
        let sdf = SdfCollisionData::sphere(Point3::origin(), 1.0, 32, 1.0);
        model.sdf_data = vec![Arc::new(sdf)];
        model.geom_sdf = vec![Some(0)];

        // Vertex far outside SDF bounding box
        let result = narrowphase_sphere_geom(
            Vector3::new(100.0, 100.0, 100.0),
            0.1,
            0,
            &model,
            Vector3::zeros(),
            Matrix3::identity(),
        );
        assert!(
            result.is_none(),
            "Vertex outside SDF bbox should return None"
        );
    }

    // --- TS4: Single-cell (2×2) hfield ---

    #[test]
    fn ts4_single_cell_hfield() {
        let mut model = make_single_geom_model(GeomType::Hfield);
        let n = 2; // Minimum valid hfield: 2×2 grid = 1 cell
        let hf_size: [f64; 4] = [1.0, 1.0, 0.5, 0.0];
        let cell_size = 2.0 * hf_size[0] / (n - 1) as f64;
        let heights = vec![0.5_f64; n * n];
        let hfield = HeightFieldData::new(heights, n, n, cell_size);
        model.hfield_data = vec![Arc::new(hfield)];
        model.hfield_size = vec![hf_size];
        model.geom_hfield = vec![Some(0)];

        // Vertex penetrating
        let result = narrowphase_sphere_geom(
            Vector3::new(0.0, 0.0, 0.45),
            0.1,
            0,
            &model,
            Vector3::zeros(),
            Matrix3::identity(),
        );

        let (depth, normal, _contact_pos) = result.expect("Should generate contact on 2×2 hfield");
        assert!(depth > 0.0, "depth={depth}");
        assert!(normal.z > 0.5, "normal.z={}", normal.z);
    }
}

// =============================================================================
// Spec C tests — flex self-collision dispatch
// =============================================================================

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::expect_used, clippy::cast_precision_loss)]
mod spec_c_tests {
    use super::*;
    use crate::types::enums::FlexSelfCollide;

    /// Compute element adjacency for test use (duplicates builder logic).
    fn compute_adjacency_for_test(
        flexelem_data: &[usize],
        flexelem_dataadr: &[usize],
        flexelem_datanum: &[usize],
        nflexelem: usize,
        nflexvert: usize,
    ) -> (Vec<usize>, Vec<usize>, Vec<usize>) {
        let mut vert_to_elems: Vec<Vec<usize>> = vec![vec![]; nflexvert];
        for e in 0..nflexelem {
            let adr = flexelem_dataadr[e];
            let num = flexelem_datanum[e];
            for i in 0..num {
                let v = flexelem_data[adr + i];
                if v < nflexvert {
                    vert_to_elems[v].push(e);
                }
            }
        }
        let mut adj_data = Vec::new();
        let mut adj_adr = Vec::with_capacity(nflexelem);
        let mut adj_num = Vec::with_capacity(nflexelem);
        for e in 0..nflexelem {
            let adr = flexelem_dataadr[e];
            let num = flexelem_datanum[e];
            let mut neighbors = Vec::new();
            for i in 0..num {
                let v = flexelem_data[adr + i];
                if v < nflexvert {
                    for &neighbor in &vert_to_elems[v] {
                        if neighbor != e {
                            neighbors.push(neighbor);
                        }
                    }
                }
            }
            neighbors.sort_unstable();
            neighbors.dedup();
            adj_adr.push(adj_data.len());
            adj_num.push(neighbors.len());
            adj_data.extend(neighbors);
        }
        (adj_data, adj_adr, adj_num)
    }

    // T1: FlexSelfCollide enum parsing → AC1, AC2
    #[test]
    fn test_flex_self_collide_enum_variants() {
        assert_eq!(FlexSelfCollide::None as u8, 0);
        assert_eq!(FlexSelfCollide::Narrow as u8, 1);
        assert_eq!(FlexSelfCollide::Bvh as u8, 2);
        assert_eq!(FlexSelfCollide::Sap as u8, 3);
        assert_eq!(FlexSelfCollide::Auto as u8, 4);
        assert_eq!(FlexSelfCollide::default(), FlexSelfCollide::Auto);
    }

    // T14: sphere-triangle primitive → AC7
    #[test]
    fn test_sphere_triangle_contact_basic() {
        // Sphere at (0, 0, 0.05) radius 0.1, triangle at z=0
        let tri = [
            Vector3::new(-1.0, -1.0, 0.0),
            Vector3::new(1.0, -1.0, 0.0),
            Vector3::new(0.0, 1.0, 0.0),
        ];
        let result = sphere_triangle_contact(Vector3::new(0.0, 0.0, 0.05), 0.1, &tri);
        assert!(result.is_some());
        let (depth, normal, contact_pos) = result.unwrap();
        assert!((depth - 0.05).abs() < 1e-10, "depth={depth}");
        assert!((normal.z - 1.0).abs() < 1e-10, "normal={normal:?}");
        assert!(contact_pos.z.abs() < 1e-10, "contact_pos={contact_pos:?}");
    }

    #[test]
    fn test_sphere_triangle_contact_no_intersection() {
        let tri = [
            Vector3::new(-1.0, -1.0, 0.0),
            Vector3::new(1.0, -1.0, 0.0),
            Vector3::new(0.0, 1.0, 0.0),
        ];
        let result = sphere_triangle_contact(Vector3::new(0.0, 0.0, 0.5), 0.1, &tri);
        assert!(result.is_none());
    }

    #[test]
    fn test_sphere_triangle_contact_edge_case() {
        // Sphere near triangle edge
        let tri = [
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(1.0, 0.0, 0.0),
            Vector3::new(0.0, 1.0, 0.0),
        ];
        let result = sphere_triangle_contact(Vector3::new(0.6, 0.6, 0.0), 0.2, &tri);
        assert!(result.is_some());
    }

    // T12: Element adjacency → AC15
    #[test]
    fn test_element_adjacency_two_triangles() {
        let flexelem_data = vec![0, 1, 2, 1, 2, 3];
        let flexelem_dataadr = vec![0, 3];
        let flexelem_datanum = vec![3, 3];

        let (adj, adj_adr, adj_num) =
            compute_adjacency_for_test(&flexelem_data, &flexelem_dataadr, &flexelem_datanum, 2, 4);

        assert_eq!(adj_num[0], 1);
        assert_eq!(adj[adj_adr[0]], 1);
        assert_eq!(adj_num[1], 1);
        assert_eq!(adj[adj_adr[1]], 0);
    }

    #[test]
    fn test_element_adjacency_four_triangles() {
        // T0=[0,1,2] T1=[0,1,3] T2=[0,2,4] T3=[1,3,5]
        // T0 adj T1,T2; T1 adj T0,T3; T2 adj T0; T3 adj T1
        let flexelem_data = vec![0, 1, 2, 0, 1, 3, 0, 2, 4, 1, 3, 5];
        let flexelem_dataadr = vec![0, 3, 6, 9];
        let flexelem_datanum = vec![3, 3, 3, 3];

        let (adj, adj_adr, adj_num) =
            compute_adjacency_for_test(&flexelem_data, &flexelem_dataadr, &flexelem_datanum, 4, 6);

        // T0=[0,1,2] shares vertex 0,1 with T1, vertex 0,2 with T2, vertex 1 with T3
        assert_eq!(adj_num[0], 3);
        let t0_n: Vec<usize> = adj[adj_adr[0]..adj_adr[0] + adj_num[0]].to_vec();
        assert!(t0_n.contains(&1));
        assert!(t0_n.contains(&2));
        assert!(t0_n.contains(&3));

        // T3=[1,3,5] shares vertex 1 with T0, vertex 1,3 with T1
        assert_eq!(adj_num[3], 2);
        let t3_n: Vec<usize> = adj[adj_adr[3]..adj_adr[3] + adj_num[3]].to_vec();
        assert!(t3_n.contains(&0));
        assert!(t3_n.contains(&1));
    }

    // Test elements_adjacent helper
    #[test]
    fn test_elements_adjacent_lookup() {
        let mut model = Model::empty();
        // 2 elements: [0,1,2] and [1,2,3] — adjacent
        model.flexelem_data = vec![0, 1, 2, 1, 2, 3];
        model.flexelem_dataadr = vec![0, 3];
        model.flexelem_datanum = vec![3, 3];
        let (adj, adj_adr, adj_num) = compute_adjacency_for_test(
            &model.flexelem_data,
            &model.flexelem_dataadr,
            &model.flexelem_datanum,
            2,
            4,
        );
        model.flex_elem_adj = adj;
        model.flex_elem_adj_adr = adj_adr;
        model.flex_elem_adj_num = adj_num;

        assert!(elements_adjacent(&model, 0, 1));
        assert!(elements_adjacent(&model, 1, 0));
    }

    // Test closest_points_segments
    #[test]
    fn test_closest_points_segments_parallel() {
        let (p1, p2) = closest_points_segments(
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(1.0, 0.0, 0.0),
            Vector3::new(0.0, 1.0, 0.0),
            Vector3::new(1.0, 1.0, 0.0),
        );
        // Closest points should be at matching x, separated by y=1
        assert!((p1 - p2).norm() < 1.0 + 1e-10);
    }

    #[test]
    fn test_closest_points_segments_crossing() {
        let (p1, p2) = closest_points_segments(
            Vector3::new(0.0, 0.0, 0.0),
            Vector3::new(1.0, 0.0, 0.0),
            Vector3::new(0.5, -0.5, 1.0),
            Vector3::new(0.5, 0.5, 1.0),
        );
        assert!((p1.x - 0.5).abs() < 1e-10);
        assert!((p2.x - 0.5).abs() < 1e-10);
        assert!((p1.z - 0.0).abs() < 1e-10);
        assert!((p2.z - 1.0).abs() < 1e-10);
    }
}
