//! Primitive shape contact queries against SDF grids.
//!
//! Each function tests a specific geometric primitive (sphere, capsule, box, etc.)
//! against an SDF surface, returning contact information for the deepest penetration.

// Allow casting for grid indices - these are small values and bounds are checked
#![allow(
    clippy::cast_precision_loss,
    clippy::cast_sign_loss,
    clippy::cast_possible_truncation,
    clippy::cast_possible_wrap
)]

use nalgebra::{Point3, Vector3};
use sim_types::Pose;

use cf_geometry::Bounded;

use super::{SdfContact, SdfGrid};

/// Query an SDF for contact with a sphere.
///
/// Returns contact information if the sphere penetrates the SDF surface.
///
/// # Arguments
///
/// * `sdf` - The SDF collision data
/// * `sdf_pose` - The pose of the SDF in world space
/// * `sphere_center` - Center of the sphere in world space
/// * `sphere_radius` - Radius of the sphere
#[must_use]
pub fn sdf_sphere_contact(
    sdf: &SdfGrid,
    sdf_pose: &Pose,
    sphere_center: Point3<f64>,
    sphere_radius: f64,
) -> Option<SdfContact> {
    // Transform sphere center to SDF local space
    let local_center = sdf_pose.inverse_transform_point(&sphere_center);

    // Query SDF distance at sphere center
    let distance = sdf.distance(local_center)?;

    // Compute penetration: negative distance means inside surface
    // Contact occurs when distance < sphere_radius
    let penetration = sphere_radius - distance;

    if penetration <= 0.0 {
        return None;
    }

    // Get surface normal
    let local_normal = sdf.gradient(local_center)?;

    // Contact point is on the SDF surface, along the normal from sphere center
    let local_contact = local_center - local_normal * distance;

    // Transform back to world space
    let world_point = sdf_pose.transform_point(&local_contact);
    let world_normal = sdf_pose.rotation * local_normal;

    Some(SdfContact {
        point: world_point,
        normal: world_normal,
        penetration,
    })
}

/// Query an SDF for contact with a point.
///
/// Returns contact information if the point is inside the SDF surface.
///
/// # Arguments
///
/// * `sdf` - The SDF collision data
/// * `sdf_pose` - The pose of the SDF in world space
/// * `point` - The point to test in world space
#[must_use]
pub fn sdf_point_contact(sdf: &SdfGrid, sdf_pose: &Pose, point: Point3<f64>) -> Option<SdfContact> {
    // Transform point to SDF local space
    let local_point = sdf_pose.inverse_transform_point(&point);

    // Query SDF distance
    let distance = sdf.distance(local_point)?;

    // Penetration is negative of distance (positive when inside)
    let penetration = -distance;

    if penetration <= 0.0 {
        return None;
    }

    // Get surface normal
    let local_normal = sdf.gradient(local_point)?;

    // Contact point is on the surface, projected along normal
    let local_contact = local_point + local_normal * distance;

    // Transform back to world space
    let world_point = sdf_pose.transform_point(&local_contact);
    let world_normal = sdf_pose.rotation * local_normal;

    Some(SdfContact {
        point: world_point,
        normal: world_normal,
        penetration,
    })
}

/// Query an SDF for contact with a capsule.
///
/// Tests multiple points along the capsule axis for the deepest penetration.
/// The number of axis sample points is derived from `sdf_initpoints`:
/// `max(2, min(sdf_initpoints, axis_budget))` where axis_budget ensures
/// reasonable density. Box-corner-like fixed geometry does not apply here;
/// the axis is a continuous line that benefits from denser sampling.
///
/// # Arguments
///
/// * `sdf` - The SDF collision data
/// * `sdf_pose` - The pose of the SDF in world space
/// * `capsule_start` - Start point of capsule axis in world space
/// * `capsule_end` - End point of capsule axis in world space
/// * `capsule_radius` - Radius of the capsule
/// * `sdf_initpoints` - Number of initial sample points (from `mjOption.sdf_initpoints`)
#[must_use]
pub fn sdf_capsule_contact(
    sdf: &SdfGrid,
    sdf_pose: &Pose,
    capsule_start: Point3<f64>,
    capsule_end: Point3<f64>,
    capsule_radius: f64,
    sdf_initpoints: usize,
) -> Option<SdfContact> {
    // Transform capsule to SDF local space
    let local_start = sdf_pose.inverse_transform_point(&capsule_start);
    let local_end = sdf_pose.inverse_transform_point(&capsule_end);

    // Find the lowest distance point on the capsule axis
    let axis = local_end - local_start;
    let axis_len = axis.norm();

    if axis_len < 1e-10 {
        // Degenerate capsule - treat as sphere
        return sdf_sphere_contact(sdf, sdf_pose, capsule_start, capsule_radius);
    }

    // Sample points along the axis. Capsule only has a 1D axis, so we cap the
    // budget at a reasonable maximum (endpoints + intermediate samples).
    let n_axis = sdf_initpoints.clamp(2, 20);
    let mut test_points = Vec::with_capacity(n_axis);
    for i in 0..n_axis {
        let t = i as f64 / (n_axis - 1).max(1) as f64;
        test_points.push(local_start + axis * t);
    }

    let mut deepest_contact: Option<SdfContact> = None;
    let mut max_penetration = 0.0;

    for test_point in &test_points {
        if let Some(distance) = sdf.distance(*test_point) {
            let penetration = capsule_radius - distance;

            if penetration > max_penetration {
                max_penetration = penetration;

                if let Some(local_normal) = sdf.gradient(*test_point) {
                    let local_contact = *test_point - local_normal * distance;

                    let world_point = sdf_pose.transform_point(&local_contact);
                    let world_normal = sdf_pose.rotation * local_normal;

                    deepest_contact = Some(SdfContact {
                        point: world_point,
                        normal: world_normal,
                        penetration,
                    });
                }
            }
        }
    }

    deepest_contact
}

/// Query an SDF for contact with a box.
///
/// Tests all 8 corners of the box against the SDF.
///
/// # Arguments
///
/// * `sdf` - The SDF collision data
/// * `sdf_pose` - The pose of the SDF in world space
/// * `box_pose` - The pose of the box in world space
/// * `half_extents` - Half-extents of the box
#[must_use]
pub fn sdf_box_contact(
    sdf: &SdfGrid,
    sdf_pose: &Pose,
    box_pose: &Pose,
    half_extents: &Vector3<f64>,
) -> Option<SdfContact> {
    // Test all 8 corners of the box
    let corners = [
        Point3::new(-half_extents.x, -half_extents.y, -half_extents.z),
        Point3::new(half_extents.x, -half_extents.y, -half_extents.z),
        Point3::new(-half_extents.x, half_extents.y, -half_extents.z),
        Point3::new(half_extents.x, half_extents.y, -half_extents.z),
        Point3::new(-half_extents.x, -half_extents.y, half_extents.z),
        Point3::new(half_extents.x, -half_extents.y, half_extents.z),
        Point3::new(-half_extents.x, half_extents.y, half_extents.z),
        Point3::new(half_extents.x, half_extents.y, half_extents.z),
    ];

    let mut deepest_contact: Option<SdfContact> = None;
    let mut max_penetration = 0.0;

    for local_corner in &corners {
        let world_corner = box_pose.transform_point(local_corner);
        let sdf_local = sdf_pose.inverse_transform_point(&world_corner);

        if let Some(distance) = sdf.distance(sdf_local) {
            let penetration = -distance; // Positive when inside

            if penetration > max_penetration {
                max_penetration = penetration;

                if let Some(local_normal) = sdf.gradient(sdf_local) {
                    let local_contact = sdf_local + local_normal * distance;

                    let world_point = sdf_pose.transform_point(&local_contact);
                    let world_normal = sdf_pose.rotation * local_normal;

                    deepest_contact = Some(SdfContact {
                        point: world_point,
                        normal: world_normal,
                        penetration,
                    });
                }
            }
        }
    }

    deepest_contact
}

/// Query an SDF for contact with a cylinder.
///
/// Samples points on the cylinder surface: 2 cap centers plus circumference
/// points distributed across cap edges and middle ring. The number of
/// circumference points per ring is derived from `sdf_initpoints`:
/// `n_per_ring = max(4, (sdf_initpoints - 2) / 3)`, giving a total of
/// `2 + 3 * n_per_ring` sample points.
///
/// # Arguments
///
/// * `sdf` - The SDF collision data
/// * `sdf_pose` - The pose of the SDF in world space
/// * `cylinder_pose` - The pose of the cylinder in world space
/// * `half_height` - Half-height of the cylinder along its local Z-axis
/// * `radius` - Radius of the cylinder
/// * `sdf_initpoints` - Number of initial sample points (from `mjOption.sdf_initpoints`)
#[must_use]
pub fn sdf_cylinder_contact(
    sdf: &SdfGrid,
    sdf_pose: &Pose,
    cylinder_pose: &Pose,
    half_height: f64,
    radius: f64,
    sdf_initpoints: usize,
) -> Option<SdfContact> {
    use std::f64::consts::PI;

    // Derive circumference point count from sdf_initpoints.
    // Layout: 2 cap centers + n_per_ring top cap + n_per_ring bottom cap + n_per_ring middle.
    // Total = 2 + 3 * n_per_ring. With default 40: n_per_ring = 12, total = 38.
    let n_per_ring = (sdf_initpoints.saturating_sub(2) / 3).max(4) as u32;

    let mut sample_points = Vec::with_capacity(2 + 3 * n_per_ring as usize);

    // 2 cap centers
    sample_points.push(Point3::new(0.0, 0.0, half_height)); // Top cap center
    sample_points.push(Point3::new(0.0, 0.0, -half_height)); // Bottom cap center

    // n_per_ring points around each cap edge (2 * n_per_ring total)
    let angle_step = 2.0 * PI / f64::from(n_per_ring);
    for i in 0..n_per_ring {
        let angle = f64::from(i) * angle_step;
        let x = radius * angle.cos();
        let y = radius * angle.sin();
        sample_points.push(Point3::new(x, y, half_height)); // Top cap edge
        sample_points.push(Point3::new(x, y, -half_height)); // Bottom cap edge
    }

    // n_per_ring points around middle circumference
    for i in 0..n_per_ring {
        let angle = f64::from(i) * angle_step;
        let x = radius * angle.cos();
        let y = radius * angle.sin();
        sample_points.push(Point3::new(x, y, 0.0)); // Middle circumference
    }

    // Find the deepest penetrating point
    let mut deepest_contact: Option<SdfContact> = None;
    let mut max_penetration = 0.0;

    for local_point in &sample_points {
        let world_point = cylinder_pose.transform_point(local_point);
        let sdf_local = sdf_pose.inverse_transform_point(&world_point);

        if let Some(distance) = sdf.distance(sdf_local) {
            let penetration = -distance; // Positive when inside

            if penetration > max_penetration {
                max_penetration = penetration;

                if let Some(local_normal) = sdf.gradient(sdf_local) {
                    let local_contact = sdf_local + local_normal * distance;

                    let world_contact = sdf_pose.transform_point(&local_contact);
                    let world_normal = sdf_pose.rotation * local_normal;

                    deepest_contact = Some(SdfContact {
                        point: world_contact,
                        normal: world_normal,
                        penetration,
                    });
                }
            }
        }
    }

    deepest_contact
}

/// Query an SDF for contact with an ellipsoid.
///
/// Samples points on the ellipsoid surface using a latitude/longitude grid.
/// The number of sample points is controlled by `sdf_initpoints`: 2 poles
/// plus latitude rings with evenly distributed longitude points.
///
/// # Arguments
///
/// * `sdf` - The SDF collision data
/// * `sdf_pose` - The pose of the SDF in world space
/// * `ellipsoid_pose` - The pose of the ellipsoid in world space
/// * `radii` - Radii along each local axis (X, Y, Z)
/// * `sdf_initpoints` - Number of initial sample points (from `mjOption.sdf_initpoints`)
#[must_use]
pub fn sdf_ellipsoid_contact(
    sdf: &SdfGrid,
    sdf_pose: &Pose,
    ellipsoid_pose: &Pose,
    radii: &Vector3<f64>,
    sdf_initpoints: usize,
) -> Option<SdfContact> {
    use std::f64::consts::PI;

    let mut sample_points = Vec::with_capacity(sdf_initpoints.max(6));

    // 6 axis-aligned points (poles) — always included as geometric minimum
    sample_points.push(Point3::new(radii.x, 0.0, 0.0));
    sample_points.push(Point3::new(-radii.x, 0.0, 0.0));
    sample_points.push(Point3::new(0.0, radii.y, 0.0));
    sample_points.push(Point3::new(0.0, -radii.y, 0.0));
    sample_points.push(Point3::new(0.0, 0.0, radii.z));
    sample_points.push(Point3::new(0.0, 0.0, -radii.z));

    // Distribute remaining budget across latitude rings on the ellipsoid surface.
    let remaining = sdf_initpoints.saturating_sub(6);
    if remaining > 0 {
        // Choose n_rings latitude bands, each with n_per_ring longitude samples.
        let n_rings = ((remaining as f64).sqrt().ceil() as usize).max(1);
        let n_per_ring = (remaining / n_rings).max(1);
        for ring in 0..n_rings {
            // Latitude angle from +Z pole to -Z pole (exclude exact poles)
            let phi = PI * (ring + 1) as f64 / (n_rings + 1) as f64;
            let sin_phi = phi.sin();
            let cos_phi = phi.cos();
            for j in 0..n_per_ring {
                let theta = 2.0 * PI * j as f64 / n_per_ring as f64;
                sample_points.push(Point3::new(
                    radii.x * sin_phi * theta.cos(),
                    radii.y * sin_phi * theta.sin(),
                    radii.z * cos_phi,
                ));
            }
        }
    }

    // Find the deepest penetrating point
    let mut deepest_contact: Option<SdfContact> = None;
    let mut max_penetration = 0.0;

    for local_point in &sample_points {
        let world_point = ellipsoid_pose.transform_point(local_point);
        let sdf_local = sdf_pose.inverse_transform_point(&world_point);

        if let Some(distance) = sdf.distance(sdf_local) {
            let penetration = -distance; // Positive when inside

            if penetration > max_penetration {
                max_penetration = penetration;

                if let Some(local_normal) = sdf.gradient(sdf_local) {
                    let local_contact = sdf_local + local_normal * distance;

                    let world_contact = sdf_pose.transform_point(&local_contact);
                    let world_normal = sdf_pose.rotation * local_normal;

                    deepest_contact = Some(SdfContact {
                        point: world_contact,
                        normal: world_normal,
                        penetration,
                    });
                }
            }
        }
    }

    deepest_contact
}

/// Query an SDF for contact with a convex mesh.
///
/// Tests all mesh vertices against the SDF, returning the deepest penetration.
/// This is similar to box contact but with an arbitrary number of vertices.
///
/// # Arguments
///
/// * `sdf` - The SDF collision data
/// * `sdf_pose` - The pose of the SDF in world space
/// * `mesh_pose` - The pose of the convex mesh in world space
/// * `vertices` - Vertices of the convex mesh in local coordinates
#[must_use]
pub fn sdf_convex_mesh_contact(
    sdf: &SdfGrid,
    sdf_pose: &Pose,
    mesh_pose: &Pose,
    vertices: &[Point3<f64>],
) -> Option<SdfContact> {
    let mut deepest_contact: Option<SdfContact> = None;
    let mut max_penetration = 0.0;

    for local_vertex in vertices {
        // Transform: mesh local -> world -> SDF local
        let world_vertex = mesh_pose.transform_point(local_vertex);
        let sdf_local = sdf_pose.inverse_transform_point(&world_vertex);

        if let Some(distance) = sdf.distance(sdf_local) {
            let penetration = -distance; // Positive when inside

            if penetration > max_penetration {
                max_penetration = penetration;

                if let Some(local_normal) = sdf.gradient(sdf_local) {
                    let local_contact = sdf_local + local_normal * distance;

                    let world_contact = sdf_pose.transform_point(&local_contact);
                    let world_normal = sdf_pose.rotation * local_normal;

                    deepest_contact = Some(SdfContact {
                        point: world_contact,
                        normal: world_normal,
                        penetration,
                    });
                }
            }
        }
    }

    deepest_contact
}

/// Query an SDF for contact with a triangle mesh.
///
/// Samples all mesh vertices and optionally edge midpoints against the SDF,
/// returning the deepest penetration contact.
///
/// # Arguments
///
/// * `sdf` - The SDF collision data
/// * `sdf_pose` - The pose of the SDF in world space
/// * `mesh` - The triangle mesh data
/// * `mesh_pose` - The pose of the triangle mesh in world space
///
/// # Returns
///
/// Contact information if any mesh vertex penetrates the SDF surface, with:
/// - `point`: Contact point on the SDF surface (world space)
/// - `normal`: Surface normal at contact point (pointing outward from SDF)
/// - `penetration`: Depth of the deepest vertex penetration
#[must_use]
pub fn sdf_triangle_mesh_contact(
    sdf: &SdfGrid,
    sdf_pose: &Pose,
    mesh: &crate::mesh::TriangleMeshData,
    mesh_pose: &Pose,
) -> Option<SdfContact> {
    // Early out: Check if mesh AABB overlaps with SDF AABB
    let sdf_aabb = sdf.aabb();
    let (sdf_aabb_min, sdf_aabb_max) = (sdf_aabb.min, sdf_aabb.max);
    let (mesh_aabb_min, mesh_aabb_max) = mesh.aabb();

    // Transform SDF AABB corners to world space and compute world-space AABB
    let sdf_world_min = sdf_pose.transform_point(&sdf_aabb_min);
    let sdf_world_max = sdf_pose.transform_point(&sdf_aabb_max);
    let sdf_world_aabb_min = Point3::new(
        sdf_world_min.x.min(sdf_world_max.x),
        sdf_world_min.y.min(sdf_world_max.y),
        sdf_world_min.z.min(sdf_world_max.z),
    );
    let sdf_world_aabb_max = Point3::new(
        sdf_world_min.x.max(sdf_world_max.x),
        sdf_world_min.y.max(sdf_world_max.y),
        sdf_world_min.z.max(sdf_world_max.z),
    );

    // Transform mesh AABB corners to world space and compute world-space AABB
    let mesh_world_min = mesh_pose.transform_point(&mesh_aabb_min);
    let mesh_world_max = mesh_pose.transform_point(&mesh_aabb_max);
    let mesh_world_aabb_min = Point3::new(
        mesh_world_min.x.min(mesh_world_max.x),
        mesh_world_min.y.min(mesh_world_max.y),
        mesh_world_min.z.min(mesh_world_max.z),
    );
    let mesh_world_aabb_max = Point3::new(
        mesh_world_min.x.max(mesh_world_max.x),
        mesh_world_min.y.max(mesh_world_max.y),
        mesh_world_min.z.max(mesh_world_max.z),
    );

    // Check for AABB overlap
    if sdf_world_aabb_max.x < mesh_world_aabb_min.x
        || sdf_world_aabb_min.x > mesh_world_aabb_max.x
        || sdf_world_aabb_max.y < mesh_world_aabb_min.y
        || sdf_world_aabb_min.y > mesh_world_aabb_max.y
        || sdf_world_aabb_max.z < mesh_world_aabb_min.z
        || sdf_world_aabb_min.z > mesh_world_aabb_max.z
    {
        return None;
    }

    let mut deepest_contact: Option<SdfContact> = None;
    let mut max_penetration = 0.0;

    // Sample all mesh vertices against the SDF
    for local_vertex in mesh.vertices() {
        // Transform: mesh local -> world -> SDF local
        let world_vertex = mesh_pose.transform_point(local_vertex);
        let sdf_local = sdf_pose.inverse_transform_point(&world_vertex);

        if let Some(distance) = sdf.distance(sdf_local) {
            let penetration = -distance; // Positive when inside

            if penetration > max_penetration {
                max_penetration = penetration;

                if let Some(local_normal) = sdf.gradient(sdf_local) {
                    let local_contact = sdf_local + local_normal * distance;

                    let world_contact = sdf_pose.transform_point(&local_contact);
                    let world_normal = sdf_pose.rotation * local_normal;

                    deepest_contact = Some(SdfContact {
                        point: world_contact,
                        normal: world_normal,
                        penetration,
                    });
                }
            }
        }
    }

    // Also sample edge midpoints for better accuracy on large triangles
    for face in mesh.triangles() {
        let v0 = mesh.vertices()[face[0] as usize];
        let v1 = mesh.vertices()[face[1] as usize];
        let v2 = mesh.vertices()[face[2] as usize];

        // Edge midpoints
        let midpoints = [
            Point3::from((v0.coords + v1.coords) * 0.5),
            Point3::from((v1.coords + v2.coords) * 0.5),
            Point3::from((v2.coords + v0.coords) * 0.5),
        ];

        for local_midpoint in &midpoints {
            let world_midpoint = mesh_pose.transform_point(local_midpoint);
            let sdf_local = sdf_pose.inverse_transform_point(&world_midpoint);

            if let Some(distance) = sdf.distance(sdf_local) {
                let penetration = -distance;

                if penetration > max_penetration {
                    max_penetration = penetration;

                    if let Some(local_normal) = sdf.gradient(sdf_local) {
                        let local_contact = sdf_local + local_normal * distance;

                        let world_contact = sdf_pose.transform_point(&local_contact);
                        let world_normal = sdf_pose.rotation * local_normal;

                        deepest_contact = Some(SdfContact {
                            point: world_contact,
                            normal: world_normal,
                            penetration,
                        });
                    }
                }
            }
        }
    }

    deepest_contact
}

/// Query an SDF for contact with an infinite plane.
///
/// Samples SDF grid points that are near the surface (where |distance| is small)
/// and checks if they penetrate the plane. The plane equation is:
/// `normal · point = plane_offset` (in world space, accounting for plane body position).
///
/// # Arguments
///
/// * `sdf` - The SDF collision data
/// * `sdf_pose` - The pose of the SDF in world space
/// * `plane_normal` - Unit normal vector of the plane (in world space)
/// * `plane_offset` - Distance from world origin along the normal (includes plane body position)
///
/// # Returns
///
/// Contact information if the SDF surface intersects the plane, with:
/// - `point`: Contact point on the SDF surface (world space)
/// - `normal`: Plane normal (pointing from plane toward SDF)
/// - `penetration`: Distance each SDF surface point extends below the plane
///
/// Returns multiple contacts (up to `MAX_SDF_PLANE_CONTACTS`) to distribute
/// constraint forces across the contact patch, preventing single-point jitter.
#[must_use]
pub fn sdf_plane_contact(
    sdf: &SdfGrid,
    sdf_pose: &Pose,
    plane_normal: &Vector3<f64>,
    plane_offset: f64,
) -> Vec<SdfContact> {
    // Sample SDF grid points near the surface, project to the actual surface,
    // and collect all penetrating contacts. Deduplicating nearby contacts avoids
    // redundant forces from adjacent grid points that project to nearly the same
    // surface location.

    let mut contacts: Vec<SdfContact> = Vec::new();
    let surface_threshold = sdf.cell_size() * 2.0;

    for z in 0..sdf.depth() {
        for y in 0..sdf.height() {
            for x in 0..sdf.width() {
                let sdf_value = sdf.get(x, y, z).unwrap_or(f64::MAX);

                if sdf_value > surface_threshold {
                    continue;
                }

                let local_point = Point3::new(
                    sdf.origin().x + x as f64 * sdf.cell_size(),
                    sdf.origin().y + y as f64 * sdf.cell_size(),
                    sdf.origin().z + z as f64 * sdf.cell_size(),
                );

                let world_point = sdf_pose.transform_point(&local_point);
                let dist_to_plane = plane_normal.dot(&world_point.coords) - plane_offset;
                let penetration = -dist_to_plane;

                if penetration > 0.0 {
                    let surface_point = sdf.gradient(local_point).map_or(world_point, |grad| {
                        let surface_local = local_point - grad * sdf_value;
                        sdf_pose.transform_point(&surface_local)
                    });

                    let surface_dist = plane_normal.dot(&surface_point.coords) - plane_offset;
                    let surface_penetration = -surface_dist;

                    if surface_penetration > 0.0 {
                        contacts.push(SdfContact {
                            point: surface_point,
                            normal: *plane_normal,
                            penetration: surface_penetration,
                        });
                    }
                }
            }
        }
    }

    // Depth-sorted spatial dedup: deepest first, skip contacts too close to
    // already-kept ones. This ensures well-separated ground contacts for
    // rotational stability (instead of clustering at the deepest point).
    contacts.sort_by(|a, b| {
        b.penetration
            .partial_cmp(&a.penetration)
            .unwrap_or(std::cmp::Ordering::Equal)
    });
    let min_dist_sq = (sdf.cell_size() * 2.0) * (sdf.cell_size() * 2.0);
    let mut kept = Vec::with_capacity(MAX_SDF_PLANE_CONTACTS);
    for c in &contacts {
        let too_close = kept
            .iter()
            .any(|k: &&SdfContact| (k.point - c.point).norm_squared() < min_dist_sq);
        if !too_close {
            kept.push(c);
            if kept.len() >= MAX_SDF_PLANE_CONTACTS {
                break;
            }
        }
    }
    kept.into_iter().cloned().collect()
}

/// Maximum number of contacts returned by `sdf_plane_contact`.
/// Non-convex shapes (e.g., a bearing block) need multiple ground contacts
/// to resist rotation. 4 contacts provide a stable base. The sparse PGS
/// solver handles this efficiently.
const MAX_SDF_PLANE_CONTACTS: usize = 4;

/// Query an SDF for contact with a height field.
///
/// Samples height field grid points in the overlap region between the SDF AABB
/// and the height field AABB, finding the deepest penetration where the height
/// field surface intersects the SDF interior.
///
/// # Arguments
///
/// * `sdf` - The SDF collision data
/// * `sdf_pose` - The pose of the SDF in world space
/// * `heightfield` - The height field data
/// * `heightfield_pose` - The pose of the height field in world space
///
/// # Returns
///
/// Contact information if the height field surface penetrates the SDF, with:
/// - `point`: Contact point on the height field surface (world space)
/// - `normal`: Surface normal at contact point (pointing outward from SDF)
/// - `penetration`: Depth of the deepest height field point penetration
#[must_use]
pub fn sdf_heightfield_contact(
    sdf: &SdfGrid,
    sdf_pose: &Pose,
    heightfield: &crate::heightfield::HeightFieldData,
    heightfield_pose: &Pose,
) -> Option<SdfContact> {
    // Compute AABB overlap between SDF and height field in world space
    let sdf_aabb = sdf.aabb();
    let (sdf_aabb_min, sdf_aabb_max) = (sdf_aabb.min, sdf_aabb.max);
    let hf_aabb = heightfield.aabb();
    let (hf_aabb_min, hf_aabb_max) = (hf_aabb.min, hf_aabb.max);

    // Transform SDF AABB corners to world space
    let sdf_world_min = sdf_pose.transform_point(&sdf_aabb_min);
    let sdf_world_max = sdf_pose.transform_point(&sdf_aabb_max);

    // Handle rotation by computing actual world-space AABB
    let sdf_world_aabb_min = Point3::new(
        sdf_world_min.x.min(sdf_world_max.x),
        sdf_world_min.y.min(sdf_world_max.y),
        sdf_world_min.z.min(sdf_world_max.z),
    );
    let sdf_world_aabb_max = Point3::new(
        sdf_world_min.x.max(sdf_world_max.x),
        sdf_world_min.y.max(sdf_world_max.y),
        sdf_world_min.z.max(sdf_world_max.z),
    );

    // Transform height field AABB corners to world space
    let hf_world_min = heightfield_pose.transform_point(&hf_aabb_min);
    let hf_world_max = heightfield_pose.transform_point(&hf_aabb_max);

    // Handle rotation by computing actual world-space AABB
    let hf_world_aabb_min = Point3::new(
        hf_world_min.x.min(hf_world_max.x),
        hf_world_min.y.min(hf_world_max.y),
        hf_world_min.z.min(hf_world_max.z),
    );
    let hf_world_aabb_max = Point3::new(
        hf_world_min.x.max(hf_world_max.x),
        hf_world_min.y.max(hf_world_max.y),
        hf_world_min.z.max(hf_world_max.z),
    );

    // Check for AABB overlap
    if sdf_world_aabb_max.x < hf_world_aabb_min.x
        || sdf_world_aabb_min.x > hf_world_aabb_max.x
        || sdf_world_aabb_max.y < hf_world_aabb_min.y
        || sdf_world_aabb_min.y > hf_world_aabb_max.y
        || sdf_world_aabb_max.z < hf_world_aabb_min.z
        || sdf_world_aabb_min.z > hf_world_aabb_max.z
    {
        return None;
    }

    // Transform SDF world AABB to height field local space for cell iteration
    let sdf_in_hf_min = heightfield_pose.inverse_transform_point(&sdf_world_aabb_min);
    let sdf_in_hf_max = heightfield_pose.inverse_transform_point(&sdf_world_aabb_max);

    // Get the overlap region in height field local coordinates
    let overlap_min = Point3::new(
        sdf_in_hf_min.x.min(sdf_in_hf_max.x).max(0.0),
        sdf_in_hf_min.y.min(sdf_in_hf_max.y).max(0.0),
        sdf_in_hf_min.z.min(sdf_in_hf_max.z),
    );
    let overlap_max = Point3::new(
        sdf_in_hf_min
            .x
            .max(sdf_in_hf_max.x)
            .min(heightfield.extent_x()),
        sdf_in_hf_min
            .y
            .max(sdf_in_hf_max.y)
            .min(heightfield.extent_y()),
        sdf_in_hf_min.z.max(sdf_in_hf_max.z),
    );

    let mut deepest_contact: Option<SdfContact> = None;
    let mut max_penetration = 0.0;

    // Iterate over height field grid points in the overlap region
    for (cx, cy) in heightfield.cells_in_aabb(overlap_min, overlap_max) {
        // Check the four corners of each cell
        for (dx, dy) in &[(0, 0), (1, 0), (0, 1), (1, 1)] {
            let gx = cx + dx;
            let gy = cy + dy;

            // Get the height field vertex position in local space
            if let Some(local_hf_point) = heightfield.vertex_position(gx, gy) {
                // Transform to world space
                let world_point = heightfield_pose.transform_point(&local_hf_point);

                // Transform to SDF local space
                let sdf_local = sdf_pose.inverse_transform_point(&world_point);

                // Query SDF distance
                if let Some(distance) = sdf.distance(sdf_local) {
                    // Penetration is positive when inside the SDF (negative distance)
                    let penetration = -distance;

                    if penetration > max_penetration {
                        max_penetration = penetration;

                        // Get SDF normal at this point
                        if let Some(sdf_local_normal) = sdf.gradient(sdf_local) {
                            // The contact point is on the SDF surface
                            let sdf_surface_local = sdf_local + sdf_local_normal * distance;
                            let world_contact = sdf_pose.transform_point(&sdf_surface_local);
                            let world_normal = sdf_pose.rotation * sdf_local_normal;

                            deepest_contact = Some(SdfContact {
                                point: world_contact,
                                normal: world_normal,
                                penetration,
                            });
                        }
                    }
                }
            }
        }
    }

    deepest_contact
}

// =============================================================================
// Tests
// =============================================================================

#[cfg(test)]
#[allow(
    clippy::unwrap_used,
    clippy::expect_used,
    clippy::float_cmp,
    clippy::similar_names,
    clippy::cast_precision_loss,
    clippy::items_after_statements
)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;

    fn sphere_sdf(resolution: usize) -> SdfGrid {
        SdfGrid::sphere(Point3::origin(), 1.0, resolution, 1.0)
    }

    #[test]
    fn test_sdf_sphere_contact_no_collision() {
        let sdf = sphere_sdf(32);
        let sdf_pose = Pose::identity();

        // Sphere outside SDF surface - no contact
        let center = Point3::new(3.0, 0.0, 0.0);
        let contact = sdf_sphere_contact(&sdf, &sdf_pose, center, 0.5);
        assert!(contact.is_none());
    }

    #[test]
    fn test_sdf_sphere_contact_collision() {
        let sdf = sphere_sdf(32);
        let sdf_pose = Pose::identity();

        // Sphere overlapping SDF surface
        let center = Point3::new(1.3, 0.0, 0.0); // 1.3 - 0.5 = 0.8 (inside unit sphere)
        let contact = sdf_sphere_contact(&sdf, &sdf_pose, center, 0.5);
        assert!(contact.is_some());

        let c = contact.unwrap();
        assert!(c.penetration > 0.0, "should have positive penetration");
        assert!(c.normal.x > 0.5, "normal should point outward (+X)");
    }

    #[test]
    fn test_sdf_point_contact() {
        let sdf = sphere_sdf(32);
        let sdf_pose = Pose::identity();

        // Point inside sphere
        let point = Point3::new(0.5, 0.0, 0.0);
        let contact = sdf_point_contact(&sdf, &sdf_pose, point);
        assert!(contact.is_some());
        let c = contact.unwrap();
        assert!(c.penetration > 0.0);

        // Point outside sphere
        let point = Point3::new(1.5, 0.0, 0.0);
        let contact = sdf_point_contact(&sdf, &sdf_pose, point);
        assert!(contact.is_none());
    }

    #[test]
    fn test_sdf_box_contact() {
        let sdf = sphere_sdf(32);
        let sdf_pose = Pose::identity();
        let half_extents = Vector3::new(0.3, 0.3, 0.3);

        // Box outside sphere
        let box_pose = Pose::from_position(Point3::new(2.0, 0.0, 0.0));
        let contact = sdf_box_contact(&sdf, &sdf_pose, &box_pose, &half_extents);
        assert!(contact.is_none());

        // Box intersecting sphere
        let box_pose = Pose::from_position(Point3::new(0.9, 0.0, 0.0));
        let contact = sdf_box_contact(&sdf, &sdf_pose, &box_pose, &half_extents);
        assert!(contact.is_some());
        let c = contact.unwrap();
        assert!(c.penetration > 0.0);
    }

    #[test]
    fn test_sdf_capsule_contact() {
        let sdf = sphere_sdf(32);
        let sdf_pose = Pose::identity();

        // Capsule outside sphere
        let start = Point3::new(3.0, -1.0, 0.0);
        let end = Point3::new(3.0, 1.0, 0.0);
        let contact = sdf_capsule_contact(&sdf, &sdf_pose, start, end, 0.2, 40);
        assert!(contact.is_none());

        // Capsule intersecting sphere
        let start = Point3::new(0.8, -0.5, 0.0);
        let end = Point3::new(0.8, 0.5, 0.0);
        let contact = sdf_capsule_contact(&sdf, &sdf_pose, start, end, 0.3, 40);
        assert!(contact.is_some());
        let c = contact.unwrap();
        assert!(c.penetration > 0.0);
    }

    #[test]
    fn test_sdf_with_pose() {
        let sdf = sphere_sdf(32);

        // Translate the SDF by (5, 0, 0)
        let sdf_pose = Pose::from_position(Point3::new(5.0, 0.0, 0.0));

        // Sphere at (5, 0, 0) should be at origin of SDF local space
        let center = Point3::new(5.0, 0.0, 0.0);
        let contact = sdf_sphere_contact(&sdf, &sdf_pose, center, 0.5);
        assert!(contact.is_some());
        let c = contact.unwrap();
        // Sphere center is at SDF center (inside), so should have penetration
        assert!(c.penetration > 0.0);
    }

    // =========================================================================
    // Cylinder contact tests
    // =========================================================================

    #[test]
    fn test_sdf_cylinder_contact_no_collision() {
        let sdf = sphere_sdf(32);
        let sdf_pose = Pose::identity();

        // Cylinder far outside sphere - no contact
        let cylinder_pose = Pose::from_position(Point3::new(3.0, 0.0, 0.0));
        let contact = sdf_cylinder_contact(&sdf, &sdf_pose, &cylinder_pose, 0.5, 0.3, 40);
        assert!(contact.is_none());
    }

    #[test]
    fn test_sdf_cylinder_contact_collision() {
        let sdf = sphere_sdf(32);
        let sdf_pose = Pose::identity();

        // Cylinder intersecting sphere (cylinder axis along Z)
        let cylinder_pose = Pose::from_position(Point3::new(0.7, 0.0, 0.0));
        let contact = sdf_cylinder_contact(&sdf, &sdf_pose, &cylinder_pose, 0.5, 0.3, 40);
        assert!(contact.is_some());

        let c = contact.unwrap();
        assert!(c.penetration > 0.0, "should have positive penetration");
        // Normal should point roughly outward from SDF center (+X direction)
        assert!(c.normal.x > 0.5, "normal should point outward");
    }

    #[test]
    fn test_sdf_cylinder_contact_deep_penetration() {
        let sdf = sphere_sdf(32);
        let sdf_pose = Pose::identity();

        // Cylinder at center of sphere (maximum penetration)
        let cylinder_pose = Pose::identity();
        let contact = sdf_cylinder_contact(&sdf, &sdf_pose, &cylinder_pose, 0.3, 0.2, 40);
        assert!(contact.is_some());

        let c = contact.unwrap();
        // Should have significant penetration (cylinder inside sphere)
        assert!(c.penetration > 0.5, "should have deep penetration");
    }

    #[test]
    fn test_sdf_cylinder_contact_with_rotation() {
        let sdf = sphere_sdf(32);
        let sdf_pose = Pose::identity();

        // Cylinder rotated 90 degrees (axis along X instead of Z)
        use nalgebra::UnitQuaternion;
        let rotation = UnitQuaternion::from_euler_angles(0.0, std::f64::consts::FRAC_PI_2, 0.0);
        let cylinder_pose = Pose::from_position_rotation(Point3::new(0.0, 0.7, 0.0), rotation);

        let contact = sdf_cylinder_contact(&sdf, &sdf_pose, &cylinder_pose, 0.5, 0.3, 40);
        assert!(contact.is_some());

        let c = contact.unwrap();
        assert!(c.penetration > 0.0);
    }

    #[test]
    fn test_sdf_cylinder_contact_cap_hit() {
        let sdf = sphere_sdf(32);
        let sdf_pose = Pose::identity();

        // Cylinder positioned so cap center contacts the sphere
        // Cylinder along Z-axis, cap should hit sphere from +Z direction
        let cylinder_pose = Pose::from_position(Point3::new(0.0, 0.0, 0.6));
        let contact = sdf_cylinder_contact(&sdf, &sdf_pose, &cylinder_pose, 0.5, 0.2, 40);
        assert!(contact.is_some());

        let c = contact.unwrap();
        assert!(c.penetration > 0.0);
        // Normal should point roughly in +Z direction
        assert!(c.normal.z > 0.5, "normal should point in +Z direction");
    }

    // =========================================================================
    // Ellipsoid contact tests
    // =========================================================================

    #[test]
    fn test_sdf_ellipsoid_contact_no_collision() {
        let sdf = sphere_sdf(32);
        let sdf_pose = Pose::identity();

        // Ellipsoid far outside sphere - no contact
        let ellipsoid_pose = Pose::from_position(Point3::new(3.0, 0.0, 0.0));
        let radii = Vector3::new(0.3, 0.3, 0.3);
        let contact = sdf_ellipsoid_contact(&sdf, &sdf_pose, &ellipsoid_pose, &radii, 40);
        assert!(contact.is_none());
    }

    #[test]
    fn test_sdf_ellipsoid_contact_collision() {
        let sdf = sphere_sdf(32);
        let sdf_pose = Pose::identity();

        // Ellipsoid intersecting sphere
        let ellipsoid_pose = Pose::from_position(Point3::new(0.8, 0.0, 0.0));
        let radii = Vector3::new(0.4, 0.3, 0.3);
        let contact = sdf_ellipsoid_contact(&sdf, &sdf_pose, &ellipsoid_pose, &radii, 40);
        assert!(contact.is_some());

        let c = contact.unwrap();
        assert!(c.penetration > 0.0, "should have positive penetration");
        // Normal should point roughly outward from SDF center (+X direction)
        assert!(c.normal.x > 0.5, "normal should point outward");
    }

    #[test]
    fn test_sdf_ellipsoid_contact_deep_penetration() {
        let sdf = sphere_sdf(32);
        let sdf_pose = Pose::identity();

        // Small ellipsoid at center of sphere (maximum penetration)
        let ellipsoid_pose = Pose::identity();
        let radii = Vector3::new(0.2, 0.2, 0.2);
        let contact = sdf_ellipsoid_contact(&sdf, &sdf_pose, &ellipsoid_pose, &radii, 40);
        assert!(contact.is_some());

        let c = contact.unwrap();
        // Should have significant penetration
        assert!(c.penetration > 0.5, "should have deep penetration");
    }

    #[test]
    fn test_sdf_ellipsoid_contact_elongated() {
        let sdf = sphere_sdf(32);
        let sdf_pose = Pose::identity();

        // Elongated ellipsoid (like a cigar shape)
        let ellipsoid_pose = Pose::from_position(Point3::new(0.0, 0.0, 0.6));
        let radii = Vector3::new(0.2, 0.2, 0.5);
        let contact = sdf_ellipsoid_contact(&sdf, &sdf_pose, &ellipsoid_pose, &radii, 40);
        assert!(contact.is_some());

        let c = contact.unwrap();
        assert!(c.penetration > 0.0);
        // The elongated Z-axis should produce a contact with normal roughly in Z direction
        assert!(c.normal.z.abs() > 0.5, "normal should point in Z direction");
    }

    #[test]
    fn test_sdf_ellipsoid_contact_with_rotation() {
        let sdf = sphere_sdf(32);
        let sdf_pose = Pose::identity();

        // Ellipsoid rotated 45 degrees around Z-axis
        use nalgebra::UnitQuaternion;
        let rotation = UnitQuaternion::from_euler_angles(0.0, 0.0, std::f64::consts::FRAC_PI_4);
        let ellipsoid_pose = Pose::from_position_rotation(Point3::new(0.8, 0.0, 0.0), rotation);
        let radii = Vector3::new(0.4, 0.2, 0.2);

        let contact = sdf_ellipsoid_contact(&sdf, &sdf_pose, &ellipsoid_pose, &radii, 40);
        assert!(contact.is_some());

        let c = contact.unwrap();
        assert!(c.penetration > 0.0);
    }

    #[test]
    fn test_sdf_ellipsoid_contact_flat_disk() {
        let sdf = sphere_sdf(32);
        let sdf_pose = Pose::identity();

        // Flat disk-like ellipsoid (like a pancake)
        let ellipsoid_pose = Pose::from_position(Point3::new(0.0, 0.0, 0.8));
        let radii = Vector3::new(0.4, 0.4, 0.1);
        let contact = sdf_ellipsoid_contact(&sdf, &sdf_pose, &ellipsoid_pose, &radii, 40);
        assert!(contact.is_some());

        let c = contact.unwrap();
        assert!(c.penetration > 0.0);
    }

    #[test]
    fn test_sdf_ellipsoid_with_translated_sdf() {
        let sdf = sphere_sdf(32);
        // Translate the SDF by (5, 0, 0)
        let sdf_pose = Pose::from_position(Point3::new(5.0, 0.0, 0.0));

        // Ellipsoid at (5, 0, 0) should be at origin of SDF local space
        let ellipsoid_pose = Pose::from_position(Point3::new(5.0, 0.0, 0.0));
        let radii = Vector3::new(0.3, 0.3, 0.3);

        let contact = sdf_ellipsoid_contact(&sdf, &sdf_pose, &ellipsoid_pose, &radii, 40);
        assert!(contact.is_some());

        let c = contact.unwrap();
        // Ellipsoid at SDF center should have penetration
        assert!(c.penetration > 0.0);
    }

    // =========================================================================
    // SDF option wiring tests
    // =========================================================================

    #[test]
    fn test_cylinder_initpoints_changes_sample_count() {
        // With sdf_initpoints=8 (minimum): n_per_ring = max(4, (8-2)/3) = 4
        // Total = 2 + 3*4 = 14 sample points.
        // With sdf_initpoints=40 (default): n_per_ring = max(4, (40-2)/3) = 12
        // Total = 2 + 3*12 = 38 sample points.
        // Higher initpoints should still find contacts (or find them more reliably).
        let sdf = sphere_sdf(32);
        let sdf_pose = Pose::identity();
        let cylinder_pose = Pose::from_position(Point3::new(0.5, 0.0, 0.0));

        // Cylinder overlapping with sphere — both low and high initpoints should detect
        let contact_low = sdf_cylinder_contact(&sdf, &sdf_pose, &cylinder_pose, 0.5, 0.3, 8);
        let contact_high = sdf_cylinder_contact(&sdf, &sdf_pose, &cylinder_pose, 0.5, 0.3, 80);
        assert!(
            contact_low.is_some(),
            "Low initpoints should still detect collision"
        );
        assert!(
            contact_high.is_some(),
            "High initpoints should detect collision"
        );
    }

    #[test]
    fn test_ellipsoid_initpoints_changes_sample_count() {
        // With sdf_initpoints=6: only 6 poles, no latitude rings.
        // With sdf_initpoints=40: 6 poles + 34 distributed across latitude rings.
        let sdf = sphere_sdf(32);
        let sdf_pose = Pose::identity();
        let ellipsoid_pose = Pose::from_position(Point3::new(0.5, 0.0, 0.0));
        let radii = Vector3::new(0.3, 0.3, 0.3);

        let contact_low = sdf_ellipsoid_contact(&sdf, &sdf_pose, &ellipsoid_pose, &radii, 6);
        let contact_high = sdf_ellipsoid_contact(&sdf, &sdf_pose, &ellipsoid_pose, &radii, 80);
        assert!(
            contact_low.is_some(),
            "Low initpoints should still detect collision"
        );
        assert!(
            contact_high.is_some(),
            "High initpoints should detect collision"
        );
    }

    #[test]
    fn test_capsule_initpoints_changes_sample_count() {
        // With sdf_initpoints=2: only 2 endpoint samples.
        // With sdf_initpoints=20: 20 samples along the axis.
        let sdf = sphere_sdf(32);
        let sdf_pose = Pose::identity();
        let start = Point3::new(0.8, -0.5, 0.0);
        let end = Point3::new(0.8, 0.5, 0.0);

        let contact_low = sdf_capsule_contact(&sdf, &sdf_pose, start, end, 0.3, 2);
        let contact_high = sdf_capsule_contact(&sdf, &sdf_pose, start, end, 0.3, 20);
        assert!(
            contact_low.is_some(),
            "Low initpoints should still detect collision"
        );
        assert!(
            contact_high.is_some(),
            "High initpoints should detect collision"
        );
    }

    // =========================================================================
    // Convex mesh contact tests
    // =========================================================================

    /// Create a simple tetrahedron for testing
    fn tetrahedron_vertices(size: f64) -> Vec<Point3<f64>> {
        // Regular tetrahedron centered at origin
        let a = size / 2.0_f64.sqrt();
        vec![
            Point3::new(a, a, a),
            Point3::new(a, -a, -a),
            Point3::new(-a, a, -a),
            Point3::new(-a, -a, a),
        ]
    }

    /// Create a simple cube for testing
    fn cube_vertices(half_extent: f64) -> Vec<Point3<f64>> {
        let h = half_extent;
        vec![
            Point3::new(-h, -h, -h),
            Point3::new(h, -h, -h),
            Point3::new(-h, h, -h),
            Point3::new(h, h, -h),
            Point3::new(-h, -h, h),
            Point3::new(h, -h, h),
            Point3::new(-h, h, h),
            Point3::new(h, h, h),
        ]
    }

    #[test]
    fn test_sdf_convex_mesh_contact_no_collision() {
        let sdf = sphere_sdf(32);
        let sdf_pose = Pose::identity();

        // Tetrahedron far outside sphere - no contact
        let mesh_pose = Pose::from_position(Point3::new(3.0, 0.0, 0.0));
        let vertices = tetrahedron_vertices(0.3);
        let contact = sdf_convex_mesh_contact(&sdf, &sdf_pose, &mesh_pose, &vertices);
        assert!(contact.is_none());
    }

    #[test]
    fn test_sdf_convex_mesh_contact_collision() {
        let sdf = sphere_sdf(32);
        let sdf_pose = Pose::identity();

        // Tetrahedron intersecting sphere
        let mesh_pose = Pose::from_position(Point3::new(0.8, 0.0, 0.0));
        let vertices = tetrahedron_vertices(0.4);
        let contact = sdf_convex_mesh_contact(&sdf, &sdf_pose, &mesh_pose, &vertices);
        assert!(contact.is_some());

        let c = contact.unwrap();
        assert!(c.penetration > 0.0, "should have positive penetration");
        // Normal should point roughly outward from SDF center (+X direction)
        assert!(c.normal.x > 0.5, "normal should point outward");
    }

    #[test]
    fn test_sdf_convex_mesh_contact_deep_penetration() {
        let sdf = sphere_sdf(32);
        let sdf_pose = Pose::identity();

        // Small tetrahedron at center of sphere (maximum penetration)
        let mesh_pose = Pose::identity();
        let vertices = tetrahedron_vertices(0.2);
        let contact = sdf_convex_mesh_contact(&sdf, &sdf_pose, &mesh_pose, &vertices);
        assert!(contact.is_some());

        let c = contact.unwrap();
        // Should have significant penetration (mesh inside sphere)
        assert!(c.penetration > 0.5, "should have deep penetration");
    }

    #[test]
    fn test_sdf_convex_mesh_contact_with_rotation() {
        let sdf = sphere_sdf(32);
        let sdf_pose = Pose::identity();

        // Cube rotated 45 degrees around Z-axis
        use nalgebra::UnitQuaternion;
        let rotation = UnitQuaternion::from_euler_angles(0.0, 0.0, std::f64::consts::FRAC_PI_4);
        let mesh_pose = Pose::from_position_rotation(Point3::new(0.8, 0.0, 0.0), rotation);
        let vertices = cube_vertices(0.3);

        let contact = sdf_convex_mesh_contact(&sdf, &sdf_pose, &mesh_pose, &vertices);
        assert!(contact.is_some());

        let c = contact.unwrap();
        assert!(c.penetration > 0.0);
    }

    #[test]
    fn test_sdf_convex_mesh_contact_cube() {
        let sdf = sphere_sdf(32);
        let sdf_pose = Pose::identity();

        // Cube outside sphere - no contact
        let mesh_pose = Pose::from_position(Point3::new(2.0, 0.0, 0.0));
        let vertices = cube_vertices(0.3);
        let contact = sdf_convex_mesh_contact(&sdf, &sdf_pose, &mesh_pose, &vertices);
        assert!(contact.is_none());

        // Cube intersecting sphere
        let mesh_pose = Pose::from_position(Point3::new(0.9, 0.0, 0.0));
        let contact = sdf_convex_mesh_contact(&sdf, &sdf_pose, &mesh_pose, &vertices);
        assert!(contact.is_some());
        let c = contact.unwrap();
        assert!(c.penetration > 0.0);
    }

    #[test]
    fn test_sdf_convex_mesh_with_translated_sdf() {
        let sdf = sphere_sdf(32);
        // Translate the SDF by (5, 0, 0)
        let sdf_pose = Pose::from_position(Point3::new(5.0, 0.0, 0.0));

        // Tetrahedron at (5, 0, 0) should be at origin of SDF local space
        let mesh_pose = Pose::from_position(Point3::new(5.0, 0.0, 0.0));
        let vertices = tetrahedron_vertices(0.3);

        let contact = sdf_convex_mesh_contact(&sdf, &sdf_pose, &mesh_pose, &vertices);
        assert!(contact.is_some());

        let c = contact.unwrap();
        // Mesh at SDF center should have penetration
        assert!(c.penetration > 0.0);
    }

    // =========================================================================
    // Plane contact tests
    // =========================================================================

    #[test]
    fn test_sdf_plane_contact_no_collision() {
        let sdf = sphere_sdf(32);
        let sdf_pose = Pose::identity();

        let plane_normal = Vector3::z();
        let plane_offset = -5.0;

        let contacts = sdf_plane_contact(&sdf, &sdf_pose, &plane_normal, plane_offset);
        assert!(
            contacts.is_empty(),
            "SDF entirely above plane should have no contact"
        );
    }

    #[test]
    fn test_sdf_plane_contact_collision() {
        let sdf = sphere_sdf(32);
        let sdf_pose = Pose::identity();

        let plane_normal = Vector3::z();
        let plane_offset = 0.0;

        let contacts = sdf_plane_contact(&sdf, &sdf_pose, &plane_normal, plane_offset);
        assert!(
            !contacts.is_empty(),
            "SDF intersecting plane should have contact"
        );

        let c = &contacts[0]; // deepest contact (sorted)
        assert!(c.penetration > 0.0, "should have positive penetration");
        assert_relative_eq!(c.normal.z, 1.0, epsilon = 0.01);
        assert!(
            c.point.z < 0.5,
            "contact point should be in lower hemisphere"
        );
        // Sphere-plane should produce few contacts (bottom cluster)
        assert!(
            contacts.len() <= MAX_SDF_PLANE_CONTACTS,
            "sphere/plane should produce at most {} contacts (got {})",
            MAX_SDF_PLANE_CONTACTS,
            contacts.len()
        );
    }

    #[test]
    fn test_sdf_plane_contact_deep_penetration() {
        let sdf = sphere_sdf(32);
        let sdf_pose = Pose::identity();

        let plane_normal = Vector3::z();
        let plane_offset = 0.5;

        let contacts = sdf_plane_contact(&sdf, &sdf_pose, &plane_normal, plane_offset);
        assert!(
            !contacts.is_empty(),
            "SDF mostly below plane should have contact"
        );

        let c = &contacts[0];
        assert!(
            c.penetration > 1.0,
            "should have deep penetration (got {})",
            c.penetration
        );
    }

    #[test]
    fn test_sdf_plane_contact_tilted_plane() {
        let sdf = sphere_sdf(32);
        let sdf_pose = Pose::identity();

        let plane_normal = Vector3::new(1.0, 0.0, 1.0).normalize();
        let plane_offset = 0.0;

        let contacts = sdf_plane_contact(&sdf, &sdf_pose, &plane_normal, plane_offset);
        assert!(
            !contacts.is_empty(),
            "SDF intersecting tilted plane should have contact"
        );

        let c = &contacts[0];
        assert!(c.penetration > 0.0, "should have positive penetration");
        assert_relative_eq!(c.normal, plane_normal, epsilon = 0.01);
    }

    #[test]
    fn test_sdf_plane_contact_with_translated_sdf() {
        let sdf = sphere_sdf(32);
        let sdf_pose = Pose::from_position(Point3::new(0.0, 0.0, 2.0));

        let plane_normal = Vector3::z();
        let plane_offset = 0.0;

        let contacts = sdf_plane_contact(&sdf, &sdf_pose, &plane_normal, plane_offset);
        assert!(
            contacts.is_empty(),
            "Translated SDF above plane should have no contact"
        );

        let plane_offset = 1.5;
        let contacts = sdf_plane_contact(&sdf, &sdf_pose, &plane_normal, plane_offset);
        assert!(
            !contacts.is_empty(),
            "Translated SDF intersecting plane should have contact"
        );
        assert!(contacts[0].penetration > 0.0);
    }

    #[test]
    fn test_sdf_plane_contact_inverted_normal() {
        let sdf = sphere_sdf(32);
        let sdf_pose = Pose::identity();

        let plane_normal = -Vector3::z();
        let plane_offset = 0.0;

        let contacts = sdf_plane_contact(&sdf, &sdf_pose, &plane_normal, plane_offset);
        assert!(
            !contacts.is_empty(),
            "SDF intersecting inverted plane should have contact"
        );

        let c = &contacts[0];
        assert!(c.penetration > 0.0, "should have positive penetration");
        assert_relative_eq!(c.normal.z, -1.0, epsilon = 0.01);
    }

    #[test]
    fn test_sdf_plane_contact_box_sdf() {
        let sdf = SdfGrid::box_shape(Point3::origin(), Vector3::new(0.5, 0.5, 0.5), 32, 0.5);
        let sdf_pose = Pose::identity();

        let plane_normal = Vector3::z();
        let plane_offset = 0.0;

        let contacts = sdf_plane_contact(&sdf, &sdf_pose, &plane_normal, plane_offset);
        assert!(
            !contacts.is_empty(),
            "Box SDF intersecting plane should have contact"
        );
        assert!(contacts[0].penetration > 0.0);
    }

    // =========================================================================
    // Triangle mesh contact tests
    // =========================================================================

    /// Create a simple tetrahedron mesh for testing
    fn create_tetrahedron_mesh() -> crate::mesh::TriangleMeshData {
        let vertices = vec![
            Point3::new(0.0, 0.0, 0.0),
            Point3::new(1.0, 0.0, 0.0),
            Point3::new(0.5, 1.0, 0.0),
            Point3::new(0.5, 0.33, 0.8),
        ];
        let indices = vec![
            0, 1, 2, // bottom
            0, 1, 3, // front
            1, 2, 3, // right
            0, 2, 3, // left
        ];
        crate::mesh::TriangleMeshData::new(vertices, indices)
    }

    /// Create a simple cube mesh for testing
    fn create_cube_mesh() -> crate::mesh::TriangleMeshData {
        let vertices = vec![
            // Bottom face
            Point3::new(-0.5, -0.5, -0.5),
            Point3::new(0.5, -0.5, -0.5),
            Point3::new(0.5, 0.5, -0.5),
            Point3::new(-0.5, 0.5, -0.5),
            // Top face
            Point3::new(-0.5, -0.5, 0.5),
            Point3::new(0.5, -0.5, 0.5),
            Point3::new(0.5, 0.5, 0.5),
            Point3::new(-0.5, 0.5, 0.5),
        ];
        // Two triangles per face
        let indices = vec![
            // Bottom (-Z)
            0, 1, 2, 0, 2, 3, // Top (+Z)
            4, 6, 5, 4, 7, 6, // Front (-Y)
            0, 5, 1, 0, 4, 5, // Back (+Y)
            2, 7, 3, 2, 6, 7, // Left (-X)
            0, 7, 4, 0, 3, 7, // Right (+X)
            1, 6, 2, 1, 5, 6,
        ];
        crate::mesh::TriangleMeshData::new(vertices, indices)
    }

    #[test]
    fn test_sdf_triangle_mesh_contact_no_collision() {
        let sdf = sphere_sdf(32);
        let sdf_pose = Pose::identity();

        // Mesh far outside sphere - no contact
        let mesh = create_tetrahedron_mesh();
        let mesh_pose = Pose::from_position(Point3::new(5.0, 0.0, 0.0));

        let contact = sdf_triangle_mesh_contact(&sdf, &sdf_pose, &mesh, &mesh_pose);
        assert!(
            contact.is_none(),
            "Mesh far from SDF should have no contact"
        );
    }

    #[test]
    fn test_sdf_triangle_mesh_contact_collision() {
        let sdf = sphere_sdf(32);
        let sdf_pose = Pose::identity();

        // Tetrahedron intersecting the sphere
        let mesh = create_tetrahedron_mesh();
        let mesh_pose = Pose::from_position(Point3::new(0.5, 0.0, 0.0));

        let contact = sdf_triangle_mesh_contact(&sdf, &sdf_pose, &mesh, &mesh_pose);
        assert!(
            contact.is_some(),
            "Mesh intersecting SDF should have contact"
        );

        let c = contact.unwrap();
        assert!(c.penetration > 0.0, "should have positive penetration");
    }

    #[test]
    fn test_sdf_triangle_mesh_contact_deep_penetration() {
        let sdf = sphere_sdf(32);
        let sdf_pose = Pose::identity();

        // Small tetrahedron at center of sphere (maximum penetration)
        let mesh = create_tetrahedron_mesh();
        let mesh_pose = Pose::from_position(Point3::new(-0.3, -0.3, -0.2));

        let contact = sdf_triangle_mesh_contact(&sdf, &sdf_pose, &mesh, &mesh_pose);
        assert!(contact.is_some(), "Mesh inside SDF should have contact");

        let c = contact.unwrap();
        // Should have significant penetration
        assert!(
            c.penetration > 0.3,
            "should have deep penetration (got {})",
            c.penetration
        );
    }

    #[test]
    fn test_sdf_triangle_mesh_contact_with_transformed_poses() {
        let sdf = sphere_sdf(32);
        // Translate the SDF by (5, 0, 0)
        let sdf_pose = Pose::from_position(Point3::new(5.0, 0.0, 0.0));

        // Tetrahedron at (5, 0, 0) should be at origin of SDF local space
        let mesh = create_tetrahedron_mesh();
        let mesh_pose = Pose::from_position(Point3::new(5.5, 0.0, 0.0));

        let contact = sdf_triangle_mesh_contact(&sdf, &sdf_pose, &mesh, &mesh_pose);
        assert!(
            contact.is_some(),
            "Mesh at translated SDF location should have contact"
        );

        let c = contact.unwrap();
        assert!(c.penetration > 0.0);
    }

    #[test]
    fn test_sdf_triangle_mesh_contact_with_rotation() {
        let sdf = sphere_sdf(32);
        let sdf_pose = Pose::identity();

        // Cube rotated 45 degrees around Z-axis
        use nalgebra::UnitQuaternion;
        let rotation = UnitQuaternion::from_euler_angles(0.0, 0.0, std::f64::consts::FRAC_PI_4);
        let mesh = create_cube_mesh();
        let mesh_pose = Pose::from_position_rotation(Point3::new(0.8, 0.0, 0.0), rotation);

        let contact = sdf_triangle_mesh_contact(&sdf, &sdf_pose, &mesh, &mesh_pose);
        assert!(
            contact.is_some(),
            "Rotated mesh intersecting SDF should have contact"
        );

        let c = contact.unwrap();
        assert!(c.penetration > 0.0);
    }

    #[test]
    fn test_sdf_triangle_mesh_contact_cube_mesh() {
        let sdf = sphere_sdf(32);
        let sdf_pose = Pose::identity();

        // Cube outside sphere - no contact
        let mesh = create_cube_mesh();
        let mesh_pose = Pose::from_position(Point3::new(3.0, 0.0, 0.0));
        let contact = sdf_triangle_mesh_contact(&sdf, &sdf_pose, &mesh, &mesh_pose);
        assert!(contact.is_none(), "Cube far from sphere should not contact");

        // Cube intersecting sphere
        let mesh_pose = Pose::from_position(Point3::new(0.9, 0.0, 0.0));
        let contact = sdf_triangle_mesh_contact(&sdf, &sdf_pose, &mesh, &mesh_pose);
        assert!(
            contact.is_some(),
            "Cube intersecting sphere should have contact"
        );
        let c = contact.unwrap();
        assert!(c.penetration > 0.0);
    }

    #[test]
    fn test_sdf_triangle_mesh_contact_box_sdf() {
        // Test with a box SDF instead of sphere
        let sdf = SdfGrid::box_shape(Point3::origin(), Vector3::new(0.5, 0.5, 0.5), 32, 0.5);
        let sdf_pose = Pose::identity();

        // Tetrahedron intersecting the box
        let mesh = create_tetrahedron_mesh();
        let mesh_pose = Pose::from_position(Point3::new(0.3, 0.0, 0.0));

        let contact = sdf_triangle_mesh_contact(&sdf, &sdf_pose, &mesh, &mesh_pose);
        assert!(
            contact.is_some(),
            "Mesh intersecting box SDF should have contact"
        );

        let c = contact.unwrap();
        assert!(c.penetration > 0.0);
    }

    // =========================================================================
    // HeightField contact tests
    // =========================================================================

    #[test]
    fn test_sdf_heightfield_contact_no_collision() {
        let sdf = sphere_sdf(32);
        let sdf_pose = Pose::identity();

        // Height field entirely below the sphere (sphere at z=0 with radius 1, lowest point z=-1)
        // Height field at z=-5, so entirely below
        let hf = crate::heightfield::HeightFieldData::flat(10, 10, 1.0, -5.0);
        let hf_pose = Pose::identity();

        let contact = sdf_heightfield_contact(&sdf, &sdf_pose, &hf, &hf_pose);
        assert!(
            contact.is_none(),
            "Height field far below SDF should have no contact"
        );
    }

    #[test]
    fn test_sdf_heightfield_contact_collision() {
        let sdf = sphere_sdf(32);
        let sdf_pose = Pose::identity();

        // Height field at z=0.5, should intersect the sphere (radius 1, centered at origin)
        let hf = crate::heightfield::HeightFieldData::flat(10, 10, 0.5, 0.5);
        // Position height field so it overlaps with sphere center area
        let hf_pose = Pose::from_position(Point3::new(-2.0, -2.0, 0.0));

        let contact = sdf_heightfield_contact(&sdf, &sdf_pose, &hf, &hf_pose);
        assert!(
            contact.is_some(),
            "Height field intersecting SDF should have contact"
        );

        let c = contact.unwrap();
        assert!(c.penetration > 0.0, "should have positive penetration");
    }

    #[test]
    fn test_sdf_heightfield_contact_deep_penetration() {
        let sdf = sphere_sdf(32);
        let sdf_pose = Pose::identity();

        // Height field at z=0 (passing through sphere center)
        let hf = crate::heightfield::HeightFieldData::flat(10, 10, 0.5, 0.0);
        let hf_pose = Pose::from_position(Point3::new(-2.0, -2.0, 0.0));

        let contact = sdf_heightfield_contact(&sdf, &sdf_pose, &hf, &hf_pose);
        assert!(
            contact.is_some(),
            "Height field at sphere center should have contact"
        );

        let c = contact.unwrap();
        // At the center of the sphere, penetration should be approximately the radius
        assert!(
            c.penetration > 0.5,
            "should have deep penetration (got {})",
            c.penetration
        );
    }

    #[test]
    fn test_sdf_heightfield_contact_with_transformed_poses() {
        let sdf = sphere_sdf(32);
        // Translate the SDF by (5, 0, 0)
        let sdf_pose = Pose::from_position(Point3::new(5.0, 0.0, 0.0));

        // Height field at z=0.5, positioned to overlap with SDF at (5, 0, 0)
        let hf = crate::heightfield::HeightFieldData::flat(10, 10, 0.5, 0.5);
        let hf_pose = Pose::from_position(Point3::new(3.0, -2.0, 0.0));

        let contact = sdf_heightfield_contact(&sdf, &sdf_pose, &hf, &hf_pose);
        assert!(
            contact.is_some(),
            "Height field intersecting translated SDF should have contact"
        );

        let c = contact.unwrap();
        assert!(c.penetration > 0.0);
    }

    #[test]
    fn test_sdf_heightfield_contact_tilted_terrain() {
        let sdf = sphere_sdf(32);
        let sdf_pose = Pose::identity();

        // Create a sloped height field: z = 0.5 * x
        // This creates terrain that varies in height
        let hf = crate::heightfield::HeightFieldData::from_fn(10, 10, 0.5, |x, _y| x * 0.3);
        let hf_pose = Pose::from_position(Point3::new(-2.0, -2.0, 0.0));

        let contact = sdf_heightfield_contact(&sdf, &sdf_pose, &hf, &hf_pose);
        assert!(
            contact.is_some(),
            "Sloped height field intersecting SDF should have contact"
        );

        let c = contact.unwrap();
        assert!(c.penetration > 0.0);
    }

    #[test]
    fn test_sdf_heightfield_contact_entirely_above() {
        let sdf = sphere_sdf(32);
        let sdf_pose = Pose::identity();

        // Height field at z=5, entirely above the sphere
        let hf = crate::heightfield::HeightFieldData::flat(10, 10, 1.0, 5.0);
        let hf_pose = Pose::from_position(Point3::new(-2.0, -2.0, 0.0));

        let contact = sdf_heightfield_contact(&sdf, &sdf_pose, &hf, &hf_pose);
        assert!(
            contact.is_none(),
            "Height field entirely above SDF should have no contact"
        );
    }

    #[test]
    fn test_sdf_heightfield_contact_box_sdf() {
        // Test with a box SDF instead of sphere
        let sdf = SdfGrid::box_shape(Point3::origin(), Vector3::new(0.5, 0.5, 0.5), 32, 0.5);
        let sdf_pose = Pose::identity();

        // Height field at z=0.3, should intersect the box (extends from -0.5 to 0.5 in Z)
        let hf = crate::heightfield::HeightFieldData::flat(10, 10, 0.3, 0.3);
        let hf_pose = Pose::from_position(Point3::new(-1.0, -1.0, 0.0));

        let contact = sdf_heightfield_contact(&sdf, &sdf_pose, &hf, &hf_pose);
        assert!(
            contact.is_some(),
            "Height field intersecting box SDF should have contact"
        );

        let c = contact.unwrap();
        assert!(c.penetration > 0.0);
    }

    #[test]
    fn test_sdf_heightfield_contact_outside_xy_bounds() {
        let sdf = sphere_sdf(32);
        let sdf_pose = Pose::identity();

        // Height field positioned far away in XY plane
        let hf = crate::heightfield::HeightFieldData::flat(5, 5, 1.0, 0.0);
        let hf_pose = Pose::from_position(Point3::new(10.0, 10.0, 0.0));

        let contact = sdf_heightfield_contact(&sdf, &sdf_pose, &hf, &hf_pose);
        assert!(
            contact.is_none(),
            "Height field outside XY bounds should have no contact"
        );
    }
}
