//! Mesh collision dispatch — routes mesh-geom pairs to the standalone mesh library.

use super::narrow::{GEOM_EPSILON, make_contact_from_geoms, multiccd_contacts};
use crate::gjk_epa::{gjk_distance, gjk_epa_contact};
use crate::mesh::{
    MeshContact, TriangleMeshData, mesh_box_contact, mesh_capsule_contact,
    mesh_mesh_deepest_contact, mesh_sphere_contact,
};
use crate::types::{
    Contact, DISABLE_MIDPHASE, ENABLE_MULTICCD, GeomType, Model, disabled, enabled,
};
use cf_geometry::Shape;
use nalgebra::{Matrix3, Point3, UnitQuaternion, Vector3};
use sim_types::Pose;

/// GJK/EPA collision between two convex shapes with MULTICCD and margin support.
///
/// Arguments must be in geom-order: shape1/pose1 correspond to geom1,
/// shape2/pose2 correspond to geom2. This ensures the GJK normal convention
/// ("from B toward A" = from geom2 toward geom1) is correct without
/// manual negation.
///
/// Returns `Contact`(s) directly — no `MeshContact` intermediate.
#[allow(clippy::too_many_arguments)]
fn gjk_epa_shape_pair(
    model: &Model,
    shape1: &Shape,
    pose1: &Pose,
    shape2: &Shape,
    pose2: &Pose,
    geom1: usize,
    geom2: usize,
    margin: f64,
) -> Vec<Contact> {
    if let Some(gjk) = gjk_epa_contact(
        shape1,
        pose1,
        shape2,
        pose2,
        model.ccd_iterations,
        model.ccd_tolerance,
    ) {
        // MULTICCD: generate multiple contacts for flat surfaces
        if enabled(model, ENABLE_MULTICCD) {
            let multi = multiccd_contacts(
                shape1,
                pose1,
                shape2,
                pose2,
                &gjk,
                model.ccd_iterations,
                model.ccd_tolerance,
            );
            return multi
                .into_iter()
                .map(|c| {
                    make_contact_from_geoms(
                        model,
                        c.point.coords,
                        c.normal,
                        c.penetration,
                        geom1,
                        geom2,
                        margin,
                    )
                })
                .collect();
        }

        return vec![make_contact_from_geoms(
            model,
            gjk.point.coords,
            gjk.normal,
            gjk.penetration,
            geom1,
            geom2,
            margin,
        )];
    } else if margin > 0.0 {
        // Margin-zone contact: shapes don't overlap but are within margin distance
        if let Some(dist) = gjk_distance(
            shape1,
            pose1,
            shape2,
            pose2,
            model.ccd_iterations,
            model.ccd_tolerance,
        ) {
            if dist.distance < margin {
                let depth = -dist.distance;
                let diff = dist.witness_b - dist.witness_a;
                let diff_norm = diff.norm();
                let normal = if diff_norm > GEOM_EPSILON {
                    diff / diff_norm
                } else {
                    Vector3::z()
                };
                let midpoint = nalgebra::center(&dist.witness_a, &dist.witness_b);
                return vec![make_contact_from_geoms(
                    model,
                    midpoint.coords,
                    normal,
                    depth,
                    geom1,
                    geom2,
                    margin,
                )];
            }
        }
    }

    vec![]
}

/// Collision detection involving at least one mesh geometry.
///
/// Dispatches to specialized mesh-primitive or mesh-mesh implementations.
/// For mesh-primitive collisions, approximations are used for some geometry types:
/// - Cylinder: approximated as capsule (conservative, may report false positives)
/// - Ellipsoid: approximated as sphere with max radius (conservative)
#[allow(clippy::too_many_arguments)]
#[allow(clippy::similar_names)] // pos1/pose1, pos2/pose2 are intentionally related
pub fn collide_with_mesh(
    model: &Model,
    geom1: usize,
    geom2: usize,
    pos1: Vector3<f64>,
    mat1: Matrix3<f64>,
    pos2: Vector3<f64>,
    mat2: Matrix3<f64>,
    margin: f64, // DT-175: thread margin into mesh collision helpers
) -> Vec<Contact> {
    let type1 = model.geom_type[geom1];
    let type2 = model.geom_type[geom2];
    let size1 = model.geom_size[geom1];
    let size2 = model.geom_size[geom2];

    // Decide once whether to use BVH midphase (§41 DISABLE_MIDPHASE)
    let use_bvh = !disabled(model, DISABLE_MIDPHASE);

    // Build poses (expensive quaternion conversion, but needed for mesh collision)
    let quat1 = UnitQuaternion::from_matrix(&mat1);
    let quat2 = UnitQuaternion::from_matrix(&mat2);
    let pose1 = Pose::from_position_rotation(Point3::from(pos1), quat1);
    let pose2 = Pose::from_position_rotation(Point3::from(pos2), quat2);

    let mesh_contact: Option<MeshContact> = match (type1, type2) {
        // Mesh-Mesh
        (GeomType::Mesh, GeomType::Mesh) => {
            let Some(mesh1_id) = model.geom_mesh[geom1] else {
                return vec![];
            };
            let Some(mesh2_id) = model.geom_mesh[geom2] else {
                return vec![];
            };
            let mesh1 = &model.mesh_data[mesh1_id];
            let mesh2 = &model.mesh_data[mesh2_id];

            // If both meshes have convex hulls, use GJK/EPA on hulls
            // (MuJoCo-conformant path). Early return: hull path produces
            // Contact directly, bypassing the MeshContact→Contact conversion.
            if let (Some(hull1), Some(hull2)) = (mesh1.convex_hull(), mesh2.convex_hull()) {
                let shape1 = Shape::convex_mesh(hull1.clone());
                let shape2 = Shape::convex_mesh(hull2.clone());
                return gjk_epa_shape_pair(
                    model, &shape1, &pose1, &shape2, &pose2, geom1, geom2, margin,
                );
            }

            // Fallback to per-triangle BVH if either mesh lacks hull.
            mesh_mesh_deepest_contact(mesh1, &pose1, mesh2, &pose2, use_bvh)
        }

        // Mesh (geom1) vs Primitive (geom2)
        (GeomType::Mesh, prim_type) => {
            let Some(mesh_id) = model.geom_mesh[geom1] else {
                return vec![];
            };
            let mesh = &model.mesh_data[mesh_id];

            match prim_type {
                GeomType::Sphere => {
                    mesh_sphere_contact(mesh, &pose1, pose2.position, size2.x, use_bvh)
                }
                // Capsule and Cylinder both use capsule collision (cylinder approximated as capsule)
                GeomType::Capsule | GeomType::Cylinder => {
                    let half_len = size2.y;
                    let axis = pose2.rotation * Vector3::z();
                    let start = pose2.position - axis * half_len;
                    let end = pose2.position + axis * half_len;
                    mesh_capsule_contact(mesh, &pose1, start, end, size2.x, use_bvh)
                }
                GeomType::Box => mesh_box_contact(mesh, &pose1, &pose2, &size2, use_bvh),
                GeomType::Ellipsoid => {
                    // Approximate as sphere with max radius (conservative)
                    let max_r = size2.x.max(size2.y).max(size2.z);
                    mesh_sphere_contact(mesh, &pose1, pose2.position, max_r, use_bvh)
                }
                GeomType::Plane => {
                    // Plane normal is local Z-axis — multi-contact (up to 3)
                    let plane_normal = mat2.column(2).into_owned();
                    let plane_d = plane_normal.dot(&pos2);
                    let mesh_contacts =
                        collide_mesh_plane(mesh, &pose1, plane_normal, plane_d, margin);
                    return mesh_contacts
                        .into_iter()
                        .map(|mc| {
                            make_contact_from_geoms(
                                model,
                                mc.point.coords,
                                mc.normal,
                                mc.penetration,
                                geom1,
                                geom2,
                                margin,
                            )
                        })
                        .collect();
                }
                GeomType::Mesh => unreachable!("handled in Mesh-Mesh case above"),
                // Hfield pairs routed before mesh dispatch (narrow.rs S1 ordering fix)
                GeomType::Hfield => return vec![],
                GeomType::Sdf => unreachable!("handled by collide_with_sdf"),
            }
        }

        // Primitive (geom1) vs Mesh (geom2) - swap and negate normal
        (prim_type, GeomType::Mesh) => {
            let Some(mesh_id) = model.geom_mesh[geom2] else {
                return vec![];
            };
            let mesh = &model.mesh_data[mesh_id];

            let contact = match prim_type {
                GeomType::Sphere => {
                    mesh_sphere_contact(mesh, &pose2, pose1.position, size1.x, use_bvh)
                }
                // Capsule and Cylinder both use capsule collision (cylinder approximated as capsule)
                GeomType::Capsule | GeomType::Cylinder => {
                    let half_len = size1.y;
                    let axis = pose1.rotation * Vector3::z();
                    let start = pose1.position - axis * half_len;
                    let end = pose1.position + axis * half_len;
                    mesh_capsule_contact(mesh, &pose2, start, end, size1.x, use_bvh)
                }
                GeomType::Box => mesh_box_contact(mesh, &pose2, &pose1, &size1, use_bvh),
                GeomType::Ellipsoid => {
                    let max_r = size1.x.max(size1.y).max(size1.z);
                    mesh_sphere_contact(mesh, &pose2, pose1.position, max_r, use_bvh)
                }
                GeomType::Plane => {
                    // Multi-contact (up to 3) — negate normals for swapped order
                    let plane_normal = mat1.column(2).into_owned();
                    let plane_d = plane_normal.dot(&pos1);
                    let mesh_contacts =
                        collide_mesh_plane(mesh, &pose2, plane_normal, plane_d, margin);
                    return mesh_contacts
                        .into_iter()
                        .map(|mut mc| {
                            mc.normal = -mc.normal;
                            make_contact_from_geoms(
                                model,
                                mc.point.coords,
                                mc.normal,
                                mc.penetration,
                                geom1,
                                geom2,
                                margin,
                            )
                        })
                        .collect();
                }
                GeomType::Mesh => unreachable!("handled in Mesh-Mesh case above"),
                // Hfield pairs routed before mesh dispatch (narrow.rs S1 ordering fix)
                GeomType::Hfield => return vec![],
                GeomType::Sdf => unreachable!("handled by collide_with_sdf"),
            };

            // Negate normal since we swapped the order (mesh was geom2, but contact
            // normal points from mesh outward - we need it pointing from geom1 to geom2)
            contact.map(|mut c| {
                c.normal = -c.normal;
                c
            })
        }

        _ => unreachable!("collide_with_mesh called but neither geom is a mesh"),
    };

    // Convert MeshContact to Contact
    mesh_contact
        .map(|mc| {
            make_contact_from_geoms(
                model,
                mc.point.coords,
                mc.normal,
                mc.penetration,
                geom1,
                geom2,
                margin,
            )
        })
        .into_iter()
        .collect()
}

/// Mesh vs infinite plane collision — up to 3 contacts.
///
/// Matches MuJoCo's `mjc_PlaneConvex` mesh path (`engine_collision_convex.c`).
/// Uses Path B (brute-force vertex scan, no adjacency graph).
///
/// Algorithm:
/// 1. Scan all vertices, find the deepest penetrating one → primary contact.
/// 2. Continue scanning for up to 2 more vertices that:
///    - Have `depth > -margin` (within margin of plane)
///    - Are at least `0.3 * rbound` away from the primary contact (prevents clustering)
/// 3. Return 1–3 contacts with individual depths.
///
/// Returns normal pointing FROM mesh outward (i.e. `-plane_normal`), consistent with
/// all other mesh collision functions.
///
/// # Parameters
/// - `margin`: collision margin — vertices with `depth > -margin` are candidates
pub fn collide_mesh_plane(
    mesh: &TriangleMeshData,
    mesh_pose: &Pose,
    plane_normal: Vector3<f64>,
    plane_d: f64,
    margin: f64,
) -> Vec<MeshContact> {
    let vertices = mesh.vertices();
    if vertices.is_empty() {
        return vec![];
    }

    // Compute bounding radius from AABB half-diagonal (conservative >= true rbound)
    let (aabb_min, aabb_max) = mesh.aabb();
    let rbound = (aabb_max - aabb_min).norm() / 2.0;
    let min_separation = 0.3 * rbound; // MuJoCo's tolplanemesh
    let min_sep_sq = min_separation * min_separation;

    // Pass 1: find all penetrating vertices with their depths, and identify the deepest
    let mut best_depth = f64::NEG_INFINITY;
    let mut best_idx = 0;

    // Collect (world_point, depth, vertex_index) for all penetrating vertices
    let mut candidates: Vec<(Point3<f64>, f64, usize)> = Vec::new();

    for (i, vertex) in vertices.iter().enumerate() {
        let world_v = mesh_pose.transform_point(vertex);
        let signed_dist = plane_normal.dot(&world_v.coords) - plane_d;
        let depth = -signed_dist;

        if depth > -margin {
            if depth > best_depth {
                best_depth = depth;
                best_idx = candidates.len();
            }
            candidates.push((world_v, depth, i));
        }
    }

    if candidates.is_empty() {
        return vec![];
    }

    // Primary contact: deepest vertex
    let (primary_pt, primary_depth, primary_vi) = candidates[best_idx];
    let normal_out = -plane_normal;

    let mut contacts = Vec::with_capacity(3);
    contacts.push(MeshContact {
        point: primary_pt,
        normal: normal_out,
        penetration: primary_depth,
        triangle_index: primary_vi,
    });

    // Pass 2: find up to 2 additional contacts, sorted by depth (deepest first),
    // that are far enough from the primary and from each other.
    // Sort candidates by depth descending (skip primary).
    let mut extras: Vec<(Point3<f64>, f64, usize)> = candidates
        .iter()
        .enumerate()
        .filter(|&(i, _)| i != best_idx)
        .map(|(_, c)| *c)
        .collect();
    extras.sort_by(|a, b| b.1.partial_cmp(&a.1).unwrap_or(std::cmp::Ordering::Equal));

    for &(pt, depth, vi) in &extras {
        if contacts.len() >= 3 {
            break;
        }
        // Proximity filter: reject if too close to any existing contact
        let too_close = contacts
            .iter()
            .any(|c| (c.point - pt).norm_squared() < min_sep_sq);
        if !too_close {
            contacts.push(MeshContact {
                point: pt,
                normal: normal_out,
                penetration: depth,
                triangle_index: vi,
            });
        }
    }

    contacts
}
