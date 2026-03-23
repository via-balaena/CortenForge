//! SDF collision dispatch — routes geom-SDF pairs to the standalone SDF library.

use super::narrow::make_contact_from_geoms;
use crate::sdf::{
    SdfContact, compute_shape_contact, compute_shape_plane_contact, sdf_box_contact,
    sdf_capsule_contact, sdf_cylinder_contact, sdf_ellipsoid_contact, sdf_heightfield_contact,
    sdf_sphere_contact, sdf_triangle_mesh_contact,
};
use crate::types::{Contact, GeomType, Model};
use nalgebra::{Matrix3, Point3, UnitQuaternion, Vector3};
use sim_types::Pose;

/// Cap SDF-SDF contacts per pair with greedy spatial selection.
///
/// Grid-based tracing generates contacts for every near-surface cell.
/// Instead of just keeping the deepest (which cluster spatially), this
/// sorts by depth then greedily skips contacts too close to already-kept
/// ones. This ensures the capped set has good spatial coverage for
/// constraining all DOF.
fn cap_sdf_contacts(contacts: &mut Vec<SdfContact>, max: usize) {
    if contacts.len() <= max {
        return;
    }

    // Sort by penetration depth (deepest first)
    contacts.sort_by(|a, b| {
        b.penetration
            .partial_cmp(&a.penetration)
            .unwrap_or(std::cmp::Ordering::Equal)
    });

    // Estimate spacing from contact extent to hit the cap
    let extent_sq = contacts
        .iter()
        .flat_map(|a| {
            contacts
                .iter()
                .map(move |b| (a.point - b.point).norm_squared())
        })
        .fold(0.0_f64, f64::max);
    let extent = extent_sq.sqrt();
    #[allow(clippy::cast_precision_loss)]
    let min_dist = extent / (max as f64).sqrt();
    let min_dist_sq = min_dist * min_dist;

    let mut kept: Vec<SdfContact> = Vec::with_capacity(max);
    for c in contacts.drain(..) {
        let too_close = kept
            .iter()
            .any(|k| (k.point - c.point).norm_squared() < min_dist_sq);
        if !too_close {
            kept.push(c);
            if kept.len() >= max {
                break;
            }
        }
    }
    *contacts = kept;
}

/// Collision detection involving at least one SDF geometry.
///
/// Dispatches to the appropriate `sdf_*_contact()` function from `sdf.rs`.
/// Handles all `GeomType` pairings including Mesh, Hfield, Plane, and SDF↔SDF.
#[allow(clippy::too_many_arguments)]
pub fn collide_with_sdf(
    model: &Model,
    geom1: usize,
    geom2: usize,
    pos1: Vector3<f64>,
    mat1: Matrix3<f64>,
    pos2: Vector3<f64>,
    mat2: Matrix3<f64>,
    margin: f64,
) -> Vec<Contact> {
    // Identify which geom is the SDF, which is the other
    let (sdf_geom, other_geom, sdf_pos, sdf_mat, other_pos, other_mat) =
        if model.geom_type[geom1] == GeomType::Sdf {
            (geom1, geom2, pos1, mat1, pos2, mat2)
        } else {
            (geom2, geom1, pos2, mat2, pos1, mat1)
        };

    let Some(sdf_id) = model.geom_shape[sdf_geom] else {
        return vec![];
    };
    let sdf = model.shape_data[sdf_id].sdf_grid();

    // Build SDF pose (no centering offset needed — SdfGrid uses an
    // arbitrary origin stored internally, unlike HeightFieldData which
    // uses corner-origin requiring a centering offset)
    let sdf_quat = UnitQuaternion::from_matrix(&sdf_mat);
    let sdf_pose = Pose::from_position_rotation(Point3::from(sdf_pos), sdf_quat);

    // Build other geom's parameters
    let other_quat = UnitQuaternion::from_matrix(&other_mat);
    let other_pose = Pose::from_position_rotation(Point3::from(other_pos), other_quat);
    let other_size = model.geom_size[other_geom];

    // SDF option parameters from model (§57).
    let initpoints = model.sdf_initpoints;

    // Normal direction conventions:
    //
    // - Most sdf_*_contact: normal points outward from SDF surface
    // - sdf_plane_contact: normal is the plane normal (from plane toward SDF)
    // - sdf_sdf_contact: normal from whichever SDF the contact is closer to
    //
    // Pipeline convention: normal must point from geom1 toward geom2.
    //
    // Standard case: SDF outward normal points away from the SDF. When SDF is
    // geom1, this is toward geom2 (correct). When SDF is geom2 (swapped), negate.
    //
    // Plane exception: sdf_plane_contact returns the plane normal (from plane
    // toward SDF). When SDF is geom1 and plane is geom2, this points geom2→geom1
    // (wrong) — negate. When SDF is geom2 and plane is geom1, this points
    // geom1→geom2 (correct) — keep. So for plane, swap logic is inverted.
    let swapped = sdf_geom != geom1;
    let is_plane = model.geom_type[other_geom] == GeomType::Plane;
    let negate = swapped ^ is_plane;

    // Helper: convert SdfContact → pipeline Contact with normal fixup.
    let convert = |c: &crate::sdf::SdfContact| -> Contact {
        let normal = if negate { -c.normal } else { c.normal };
        make_contact_from_geoms(
            model,
            c.point.coords,
            normal,
            c.penetration,
            geom1,
            geom2,
            margin,
        )
    };

    // SDF-Plane: dispatch through PhysicsShape trait for analytical depth
    // (or fallback to grid-based multi-contact plane tracing).
    if model.geom_type[other_geom] == GeomType::Plane {
        let plane_normal = other_mat.column(2).into_owned();
        let sdf_contacts = compute_shape_plane_contact(
            &*model.shape_data[sdf_id],
            &sdf_pose,
            &other_pos,
            &plane_normal,
            margin,
        );
        return sdf_contacts.iter().map(&convert).collect();
    }
    // SDF-SDF: dispatch through PhysicsShape trait. Analytical single-contact
    // for convex pairs, grid-based multi-contact for concave.
    // When GPU collision is available, non-convex pairs use GPU grid tracing.
    if model.geom_type[other_geom] == GeomType::Sdf {
        let Some(other_sdf_id) = model.geom_shape[other_geom] else {
            return vec![];
        };
        // Use the pair's combined margin directly. The old cell_size*0.5 floor
        // filled tight clearances (like bore/shaft gaps) with contacts, locking
        // joints. The grid handles contacts fine at any margin.
        let contact_margin = margin;

        let shape_a = &*model.shape_data[sdf_id];
        let shape_b = &*model.shape_data[other_sdf_id];

        let mut sdf_contacts = if let Some(gpu) = &model.gpu_collider {
            // GPU available: check if both shapes are convex
            let delta = other_pose.position - sdf_pose.position;
            let dist = delta.norm();
            let dir = if dist > 1e-10 {
                delta / dist
            } else {
                Vector3::z()
            };
            let both_convex = shape_a
                .effective_radius(&(sdf_pose.rotation.inverse() * dir))
                .is_some()
                && shape_b
                    .effective_radius(&(other_pose.rotation.inverse() * (-dir)))
                    .is_some();

            if both_convex {
                // Analytical convex: CPU (cheap, exact 1 contact)
                compute_shape_contact(shape_a, &sdf_pose, shape_b, &other_pose, contact_margin)
            } else {
                // Grid-based multi-contact: GPU
                gpu.sdf_sdf_contacts(sdf_id, &sdf_pose, other_sdf_id, &other_pose, contact_margin)
            }
        } else {
            // No GPU: existing CPU path
            compute_shape_contact(shape_a, &sdf_pose, shape_b, &other_pose, contact_margin)
        };

        // Cap SDF-SDF contacts per pair. Grid-based multi-contact tracing
        // generates contacts for every near-surface cell, producing hundreds
        // for concave geometry. The constraint solver is O(nefc²) per PGS
        // iteration, so capping at 50 keeps the solver tractable while still
        // providing enough contacts to constrain all DOF.
        cap_sdf_contacts(&mut sdf_contacts, model.sdf_maxcontact);

        return sdf_contacts.iter().map(&convert).collect();
    }

    // All other types: single-contact (returns Option<SdfContact>)
    let contact = match model.geom_type[other_geom] {
        GeomType::Sphere => sdf_sphere_contact(sdf, &sdf_pose, other_pose.position, other_size.x),
        GeomType::Capsule => {
            let axis = other_pose.rotation * Vector3::z();
            let start = other_pose.position - axis * other_size.y;
            let end = other_pose.position + axis * other_size.y;
            sdf_capsule_contact(sdf, &sdf_pose, start, end, other_size.x, initpoints)
        }
        GeomType::Box => sdf_box_contact(sdf, &sdf_pose, &other_pose, &other_size),
        GeomType::Cylinder => sdf_cylinder_contact(
            sdf,
            &sdf_pose,
            &other_pose,
            other_size.y,
            other_size.x,
            initpoints,
        ),
        GeomType::Ellipsoid => {
            sdf_ellipsoid_contact(sdf, &sdf_pose, &other_pose, &other_size, initpoints)
        }
        GeomType::Mesh => {
            let Some(mesh_id) = model.geom_mesh[other_geom] else {
                return vec![];
            };
            let mesh_data = &model.mesh_data[mesh_id];
            sdf_triangle_mesh_contact(sdf, &sdf_pose, mesh_data, &other_pose)
        }
        GeomType::Hfield => {
            let Some(hfield_id) = model.geom_hfield[other_geom] else {
                return vec![];
            };
            let hfield = &model.hfield_data[hfield_id];
            let hf_size = &model.hfield_size[hfield_id];
            let hf_offset = other_mat * Vector3::new(-hf_size[0], -hf_size[1], 0.0);
            let hf_pose =
                Pose::from_position_rotation(Point3::from(other_pos + hf_offset), other_quat);
            sdf_heightfield_contact(sdf, &sdf_pose, hfield, &hf_pose)
        }
        GeomType::Sdf | GeomType::Plane => unreachable!(), // handled above
    };

    contact.map(|c| convert(&c)).into_iter().collect()
}
