//! Mesh collision dispatch — routes mesh-geom pairs to the standalone mesh library.

use super::narrow::make_contact_from_geoms;
use crate::mesh::{
    MeshContact, TriangleMeshData, mesh_box_contact, mesh_capsule_contact,
    mesh_mesh_deepest_contact, mesh_sphere_contact,
};
use crate::types::{Contact, DISABLE_MIDPHASE, GeomType, Model, disabled};
use nalgebra::{Matrix3, Point3, UnitQuaternion, Vector3};
use sim_types::Pose;

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
    margin: f64, // TODO: thread margin into mesh helpers
) -> Option<Contact> {
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
            let mesh1_id = model.geom_mesh[geom1]?;
            let mesh2_id = model.geom_mesh[geom2]?;
            let mesh1 = &model.mesh_data[mesh1_id];
            let mesh2 = &model.mesh_data[mesh2_id];

            mesh_mesh_deepest_contact(mesh1, &pose1, mesh2, &pose2, use_bvh)
        }

        // Mesh (geom1) vs Primitive (geom2)
        (GeomType::Mesh, prim_type) => {
            let mesh_id = model.geom_mesh[geom1]?;
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
                    // Plane normal is local Z-axis
                    let plane_normal = mat2.column(2).into_owned();
                    let plane_d = plane_normal.dot(&pos2);
                    collide_mesh_plane(mesh, &pose1, plane_normal, plane_d)
                }
                GeomType::Mesh => unreachable!("handled in Mesh-Mesh case above"),
                GeomType::Hfield => unreachable!("handled by collide_with_hfield"),
                GeomType::Sdf => unreachable!("handled by collide_with_sdf"),
            }
        }

        // Primitive (geom1) vs Mesh (geom2) - swap and negate normal
        (prim_type, GeomType::Mesh) => {
            let mesh_id = model.geom_mesh[geom2]?;
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
                    let plane_normal = mat1.column(2).into_owned();
                    let plane_d = plane_normal.dot(&pos1);
                    collide_mesh_plane(mesh, &pose2, plane_normal, plane_d)
                }
                GeomType::Mesh => unreachable!("handled in Mesh-Mesh case above"),
                GeomType::Hfield => unreachable!("handled by collide_with_hfield"),
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
    mesh_contact.map(|mc| {
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
}

/// Mesh vs infinite plane collision.
///
/// Tests all mesh vertices against the plane and returns the deepest penetrating vertex.
/// This is a simple but effective approach for mesh-plane collision:
/// - O(n) in number of vertices
/// - Handles any mesh topology
/// - Returns single deepest contact point
pub fn collide_mesh_plane(
    mesh: &TriangleMeshData,
    mesh_pose: &Pose,
    plane_normal: Vector3<f64>,
    plane_d: f64,
) -> Option<MeshContact> {
    let mut deepest: Option<MeshContact> = None;

    for (i, vertex) in mesh.vertices().iter().enumerate() {
        // Transform vertex to world space
        let world_v = mesh_pose.transform_point(vertex);

        // Signed distance from plane (positive = above plane, negative = below)
        let signed_dist = plane_normal.dot(&world_v.coords) - plane_d;
        let depth = -signed_dist;

        if depth > 0.0 {
            match &mut deepest {
                Some(d) if depth > d.penetration => {
                    d.point = world_v;
                    d.normal = plane_normal;
                    d.penetration = depth;
                    d.triangle_index = i; // Store vertex index (not triangle, but useful for debug)
                }
                None => {
                    deepest = Some(MeshContact {
                        point: world_v,
                        normal: plane_normal,
                        penetration: depth,
                        triangle_index: i,
                    });
                }
                _ => {}
            }
        }
    }

    deepest
}
