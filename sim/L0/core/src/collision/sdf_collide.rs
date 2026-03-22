//! SDF collision dispatch — routes geom-SDF pairs to the standalone SDF library.

use super::narrow::make_contact_from_geoms;
use crate::sdf::{
    sdf_box_contact, sdf_capsule_contact, sdf_cylinder_contact, sdf_ellipsoid_contact,
    sdf_heightfield_contact, sdf_sdf_contact, sdf_sphere_contact, sdf_triangle_mesh_contact,
};
use crate::types::{Contact, GeomType, Model};
use nalgebra::{Matrix3, Point3, UnitQuaternion, Vector3};
use sim_types::Pose;

use super::super::sdf::SdfGrid;

/// Sphere-trace from the SDF origin along `local_dir` to find the surface
/// distance (effective radius along that axis).  Works for any convex SDF.
fn sdf_ray_radius(sdf: &SdfGrid, local_dir: Vector3<f64>) -> f64 {
    let dir = local_dir.try_normalize(1e-10).unwrap_or_else(Vector3::z);
    let mut t = 0.0;
    for _ in 0..200 {
        let point = Point3::from(dir * t);
        let Some(dist) = sdf.distance(point) else {
            return t;
        };
        if dist >= 0.0 {
            return t;
        }
        t += (-dist).max(sdf.cell_size() * 0.01);
    }
    t
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

    let Some(sdf_id) = model.geom_sdf[sdf_geom] else {
        return vec![];
    };
    let sdf = &model.sdf_data[sdf_id];

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

    // SDF-Plane: use analytical sphere-plane formula for depth (grid-based
    // surface tracing has discretization errors that destabilize stacking).
    // The effective radius is found by sphere-tracing the SDF along the
    // plane normal from the geom center.
    if model.geom_type[other_geom] == GeomType::Plane {
        let plane_normal = other_mat.column(2).into_owned();
        let plane_offset = plane_normal.dot(&other_pos);

        // Effective radius: ray-march from SDF center toward the plane
        let local_dir = sdf_mat.transpose() * (-plane_normal);
        let sdf_radius = sdf_ray_radius(sdf, local_dir);
        let dist_to_plane = plane_normal.dot(&sdf_pos) - plane_offset;
        let depth = sdf_radius - dist_to_plane;

        if depth > -margin {
            let contact_pos = sdf_pos - plane_normal * (sdf_radius - depth * 0.5);
            let normal = if negate { -plane_normal } else { plane_normal };
            return vec![make_contact_from_geoms(
                model,
                contact_pos,
                normal,
                depth,
                geom1,
                geom2,
                margin,
            )];
        }
        return vec![];
    }
    if model.geom_type[other_geom] == GeomType::Sdf {
        let Some(other_sdf_id) = model.geom_sdf[other_geom] else {
            return vec![];
        };
        let other_sdf = &model.sdf_data[other_sdf_id];
        // Use the broadphase margin for detection so SDF covers the full solver
        // margin zone.  Floor at half a cell to tolerate grid discretization.
        let cell_size_min = sdf.cell_size().min(other_sdf.cell_size());
        let contact_margin = margin.max(cell_size_min * 0.5);
        let mut sdf_contacts =
            sdf_sdf_contact(sdf, &sdf_pose, other_sdf, &other_pose, contact_margin);

        // Override depth with analytical overlap via SDF ray-marching.
        // Each SDF's effective radius is found by sphere-tracing from the
        // center along the separation axis (in local frame).
        let sep = (other_pos - sdf_pos)
            .try_normalize(1e-10)
            .unwrap_or_else(Vector3::z);
        let local_dir_a = sdf_mat.transpose() * sep;
        let local_dir_b = other_mat.transpose() * (-sep);
        let r_a = sdf_ray_radius(sdf, local_dir_a);
        let r_b = sdf_ray_radius(other_sdf, local_dir_b);
        let center_dist = (sdf_pos - other_pos).norm();
        let analytical_depth = (r_a + r_b - center_dist).max(0.0);
        for c in &mut sdf_contacts {
            c.penetration = analytical_depth;
        }

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
