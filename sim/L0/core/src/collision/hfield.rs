//! Height field collision dispatch — routes geom-hfield pairs to the standalone heightfield library.

use super::narrow::make_contact_from_geoms;
use crate::heightfield::{
    heightfield_box_contact, heightfield_capsule_contact, heightfield_sphere_contact,
};
use crate::types::{Contact, GeomType, Model};
use nalgebra::{Matrix3, Point3, UnitQuaternion, Vector3};
use sim_types::Pose;

/// Height field collision: dispatch to the appropriate contact function
/// based on the other geom's type, then convert to pipeline Contact.
#[allow(clippy::too_many_arguments)]
pub fn collide_with_hfield(
    model: &Model,
    geom1: usize,
    geom2: usize,
    pos1: Vector3<f64>,
    mat1: Matrix3<f64>,
    pos2: Vector3<f64>,
    mat2: Matrix3<f64>,
    margin: f64, // TODO: thread margin into heightfield helpers
) -> Option<Contact> {
    // Identify which geom is the hfield, which is the other
    let (hf_geom, other_geom, hf_pos, hf_mat, other_pos, other_mat) =
        if model.geom_type[geom1] == GeomType::Hfield {
            (geom1, geom2, pos1, mat1, pos2, mat2)
        } else {
            (geom2, geom1, pos2, mat2, pos1, mat1)
        };

    let hfield_id = model.geom_hfield[hf_geom]?;
    let hfield = &model.hfield_data[hfield_id];
    let hf_size = &model.hfield_size[hfield_id];

    // Build hfield pose — apply centering offset so HeightFieldData's
    // corner-origin (0,0) maps to MuJoCo's center-origin (-size[0], -size[1])
    let quat = UnitQuaternion::from_matrix(&hf_mat);
    let offset = hf_mat * Vector3::new(-hf_size[0], -hf_size[1], 0.0);
    let hf_pose = Pose::from_position_rotation(Point3::from(hf_pos + offset), quat);

    // Build other geom's parameters
    let other_quat = UnitQuaternion::from_matrix(&other_mat);
    let other_pose = Pose::from_position_rotation(Point3::from(other_pos), other_quat);
    let other_size = model.geom_size[other_geom];

    // Dispatch on the other geom's type
    let hf_contact = match model.geom_type[other_geom] {
        GeomType::Sphere => {
            heightfield_sphere_contact(hfield, &hf_pose, other_pose.position, other_size.x)
        }
        GeomType::Capsule => {
            let axis = other_pose.rotation * Vector3::z();
            let start = other_pose.position - axis * other_size.y;
            let end = other_pose.position + axis * other_size.y;
            heightfield_capsule_contact(hfield, &hf_pose, start, end, other_size.x)
        }
        GeomType::Box => heightfield_box_contact(hfield, &hf_pose, &other_pose, &other_size),
        GeomType::Cylinder => {
            // Approximate cylinder as capsule (conservative, same as collide_with_mesh)
            let axis = other_pose.rotation * Vector3::z();
            let start = other_pose.position - axis * other_size.y;
            let end = other_pose.position + axis * other_size.y;
            heightfield_capsule_contact(hfield, &hf_pose, start, end, other_size.x)
        }
        GeomType::Ellipsoid => {
            // Approximate as sphere with max radius (conservative)
            let max_r = other_size.x.max(other_size.y).max(other_size.z);
            heightfield_sphere_contact(hfield, &hf_pose, other_pose.position, max_r)
        }
        // Hfield↔Hfield, Hfield↔Plane, Hfield↔Mesh: not supported
        _ => return None,
    };

    // Convert HeightFieldContact → pipeline Contact.
    // Normal direction: HeightFieldContact.normal always points "up from terrain".
    // Following the collide_with_mesh pattern, when the surface geometry (hfield)
    // is geom2 and the primitive is geom1, negate the normal so it points from
    // geom1 toward geom2.
    let swapped = hf_geom != geom1;
    hf_contact.map(|c| {
        let normal = if swapped { -c.normal } else { c.normal };
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
