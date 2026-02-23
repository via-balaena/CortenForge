//! SDF collision dispatch — routes geom-SDF pairs to the standalone SDF library.

use super::narrow::make_contact_from_geoms;
use crate::sdf::{
    sdf_box_contact, sdf_capsule_contact, sdf_cylinder_contact, sdf_ellipsoid_contact,
    sdf_heightfield_contact, sdf_plane_contact, sdf_sdf_contact, sdf_sphere_contact,
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
    margin: f64, // TODO: thread margin into SDF helpers
) -> Option<Contact> {
    // Identify which geom is the SDF, which is the other
    let (sdf_geom, other_geom, sdf_pos, sdf_mat, other_pos, other_mat) =
        if model.geom_type[geom1] == GeomType::Sdf {
            (geom1, geom2, pos1, mat1, pos2, mat2)
        } else {
            (geom2, geom1, pos2, mat2, pos1, mat1)
        };

    let sdf_id = model.geom_sdf[sdf_geom]?;
    let sdf = &model.sdf_data[sdf_id];

    // Build SDF pose (no centering offset needed — SdfCollisionData uses
    // an arbitrary origin stored internally, unlike HeightFieldData which
    // uses corner-origin requiring a centering offset)
    let sdf_quat = UnitQuaternion::from_matrix(&sdf_mat);
    let sdf_pose = Pose::from_position_rotation(Point3::from(sdf_pos), sdf_quat);

    // Build other geom's parameters
    let other_quat = UnitQuaternion::from_matrix(&other_mat);
    let other_pose = Pose::from_position_rotation(Point3::from(other_pos), other_quat);
    let other_size = model.geom_size[other_geom];

    // Dispatch on the other geom's type
    let contact = match model.geom_type[other_geom] {
        GeomType::Sphere => sdf_sphere_contact(sdf, &sdf_pose, other_pose.position, other_size.x),
        GeomType::Capsule => {
            let axis = other_pose.rotation * Vector3::z();
            let start = other_pose.position - axis * other_size.y;
            let end = other_pose.position + axis * other_size.y;
            sdf_capsule_contact(sdf, &sdf_pose, start, end, other_size.x)
        }
        GeomType::Box => sdf_box_contact(sdf, &sdf_pose, &other_pose, &other_size),
        GeomType::Cylinder => {
            // sdf_cylinder_contact takes (pose, half_height, radius)
            // geom_size for Cylinder: x = radius, y = half_length
            sdf_cylinder_contact(sdf, &sdf_pose, &other_pose, other_size.y, other_size.x)
        }
        GeomType::Ellipsoid => sdf_ellipsoid_contact(sdf, &sdf_pose, &other_pose, &other_size),
        GeomType::Mesh => {
            let mesh_id = model.geom_mesh[other_geom]?;
            let mesh_data = &model.mesh_data[mesh_id];
            sdf_triangle_mesh_contact(sdf, &sdf_pose, mesh_data, &other_pose)
        }
        GeomType::Hfield => {
            let hfield_id = model.geom_hfield[other_geom]?;
            let hfield = &model.hfield_data[hfield_id];
            let hf_size = &model.hfield_size[hfield_id];
            // Apply centering offset (same as collide_with_hfield)
            let hf_offset = other_mat * Vector3::new(-hf_size[0], -hf_size[1], 0.0);
            let hf_pose =
                Pose::from_position_rotation(Point3::from(other_pos + hf_offset), other_quat);
            sdf_heightfield_contact(sdf, &sdf_pose, hfield, &hf_pose)
        }
        GeomType::Plane => {
            // Plane normal = Z-axis of plane's frame; offset = normal · position
            let plane_normal = other_mat.column(2).into_owned();
            let plane_offset = plane_normal.dot(&other_pos);
            sdf_plane_contact(sdf, &sdf_pose, &plane_normal, plane_offset)
        }
        GeomType::Sdf => {
            let other_sdf_id = model.geom_sdf[other_geom]?;
            let other_sdf = &model.sdf_data[other_sdf_id];
            sdf_sdf_contact(sdf, &sdf_pose, other_sdf, &other_pose)
        }
    };

    // Convert SdfContact → pipeline Contact.
    //
    // Normal direction conventions:
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

    contact.map(|c| {
        // Negate when exactly one of swapped/is_plane is true (XOR).
        // See comments above for the full derivation.
        let normal = if swapped ^ is_plane {
            -c.normal
        } else {
            c.normal
        };
        make_contact_from_geoms(
            model,
            c.point.coords,
            normal,
            c.penetration,
            geom1,
            geom2,
            margin,
        )
    })
}
