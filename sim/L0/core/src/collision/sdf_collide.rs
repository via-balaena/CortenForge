//! SDF collision dispatch — routes geom-SDF pairs to the standalone SDF library.

use super::narrow::make_contact_from_geoms;
use crate::sdf::{
    compute_shape_contact, compute_shape_plane_contact, sdf_box_contact, sdf_capsule_contact,
    sdf_cylinder_contact, sdf_ellipsoid_contact, sdf_heightfield_contact, sdf_sphere_contact,
    sdf_triangle_mesh_contact,
};
use crate::types::{Contact, GeomType, Model};
use nalgebra::{Matrix3, Point3, UnitQuaternion, Vector3};
use sim_types::Pose;

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
    if model.geom_type[other_geom] == GeomType::Sdf {
        let Some(other_sdf_id) = model.geom_shape[other_geom] else {
            return vec![];
        };
        // Floor margin at half a cell to tolerate grid discretization.
        let cell_size_min = sdf
            .cell_size()
            .min(model.shape_data[other_sdf_id].sdf_grid().cell_size());
        let contact_margin = margin.max(cell_size_min * 0.5);
        let sdf_contacts = compute_shape_contact(
            &*model.shape_data[sdf_id],
            &sdf_pose,
            &*model.shape_data[other_sdf_id],
            &other_pose,
            contact_margin,
        );

        // Analytical convex-convex contacts (both shapes return effective_radius)
        // are inherently unstable: the support surface is convex, so any lateral
        // perturbation causes the upper body to slide off. Pyramidal friction
        // facets on such contacts produce force asymmetry proportional to the
        // lateral offset (through the angular Jacobian's lever-arm dependence),
        // seeding exponential lateral drift at ~26/s even with a stabilized
        // contact normal. Friction serves no physical restoring purpose here.
        //
        // Override to condim=1 (frictionless) to eliminate the tangential force
        // channel entirely. Combined with the stabilized normal from
        // compute_shape_contact(), this ensures zero net lateral force.
        //
        // See sim/docs/PYRAMIDAL_FRICTION_INSTABILITY.md for the full analysis.
        let both_convex = model.shape_data[sdf_id]
            .effective_radius(&Vector3::z())
            .is_some()
            && model.shape_data[other_sdf_id]
                .effective_radius(&Vector3::z())
                .is_some();

        return sdf_contacts
            .iter()
            .map(|c| {
                let mut contact = convert(c);
                if both_convex {
                    contact.dim = 1;
                }
                contact
            })
            .collect();
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
