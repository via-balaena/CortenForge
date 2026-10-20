//! Mesh–plane collision: the mesh pairs MuJoCo's table does not give
//! `mjc_Convex` (the others collide through [`mj_ccd`](super::mj_ccd)).

use super::narrow::make_contact_from_geoms;
use crate::mesh::{MeshContact, TriangleMeshData};
use crate::types::{Contact, GeomType, Model};
use nalgebra::{Matrix3, Point3, UnitQuaternion, Vector3};
use sim_types::Pose;

/// A mesh against a plane, either first: up to three contacts
/// ([`collide_mesh_plane`]), each normal from `geom1` toward `geom2`.
// Pair dispatcher takes the full per-geom collision context (poses, output sink); bundling would just move the argument list one level up.
#[allow(clippy::too_many_arguments)]
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
    // The mesh, its pose, the plane's normal and offset, and whether the
    // normal (from the mesh outward) turns round for the listed order.
    let (mesh_geom, mesh_pos, mesh_mat, plane_pos, plane_mat, flip) =
        match (model.geom_type[geom1], model.geom_type[geom2]) {
            (GeomType::Mesh, GeomType::Plane) => (geom1, pos1, mat1, pos2, mat2, false),
            (GeomType::Plane, GeomType::Mesh) => (geom2, pos2, mat2, pos1, mat1, true),
            _ => return vec![],
        };
    let Some(mesh_id) = model.geom_mesh[mesh_geom] else {
        return vec![];
    };
    let mesh: &TriangleMeshData = &model.mesh_data[mesh_id];
    let mesh_pose = Pose::from_position_rotation(
        Point3::from(mesh_pos),
        UnitQuaternion::from_matrix(&mesh_mat),
    );
    let plane_normal = plane_mat.column(2).into_owned();
    let plane_d = plane_normal.dot(&plane_pos);
    collide_mesh_plane(mesh, &mesh_pose, plane_normal, plane_d, margin)
        .into_iter()
        .map(|mc| {
            let normal = if flip { -mc.normal } else { mc.normal };
            make_contact_from_geoms(
                model,
                mc.point.coords,
                normal,
                mc.penetration,
                geom1,
                geom2,
                margin,
            )
        })
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
///    - Are at least `0.3 * rbound` away from the primary contact (prevents clustering),
///      `rbound` the half-diagonal of the mesh's box. MuJoCo's `addplanemesh` takes
///      `geom_rbound` (`engine_collision_convex.c:1018`, `:1103`, `:1129`), the norm
///      of each axis's larger bound of the mesh's box in its mesh frame, at the
///      centre of mass along the principal axes (`user_objects.cc:3443-3449`): the
///      half-diagonal where the authored box is that box. sim-mjcf keeps a mesh in
///      its authored frame (book Rigid-loading L35), where `geom_rbound` is about
///      the authored origin (ledger L176)
/// 3. Return 1–3 contacts with individual depths.
///
/// Returns normal pointing FROM mesh outward (i.e. `-plane_normal`).
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
