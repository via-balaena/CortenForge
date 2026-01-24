//! Mesh generation for collision shape visualization.
//!
//! Converts sim-core collision shapes to Bevy meshes.

#![allow(clippy::cast_possible_truncation)] // f64 -> f32 is intentional for Bevy meshes

use bevy::asset::RenderAssetUsages;
use bevy::mesh::{PrimitiveTopology, VertexAttributeValues};
use bevy::prelude::*;
use sim_core::CollisionShape;

/// Generate a Bevy mesh from a sim-core collision shape.
///
/// Returns `None` for shapes that cannot be easily visualized (e.g., SDF).
#[must_use]
pub fn mesh_from_collision_shape(shape: &CollisionShape) -> Option<Mesh> {
    match shape {
        CollisionShape::Sphere { radius } => Some(sphere_mesh(*radius)),
        CollisionShape::Box { half_extents } => Some(box_mesh(half_extents)),
        CollisionShape::Capsule {
            half_length,
            radius,
        } => Some(capsule_mesh(*half_length, *radius)),
        CollisionShape::Cylinder {
            half_length,
            radius,
        } => Some(cylinder_mesh(*half_length, *radius)),
        CollisionShape::Ellipsoid { radii } => Some(ellipsoid_mesh(radii)),
        CollisionShape::Plane { .. } => Some(plane_mesh()),
        CollisionShape::ConvexMesh { vertices } => Some(convex_mesh_from_vertices(vertices)),
        CollisionShape::TriangleMesh { .. } => {
            // TODO: Implement triangle mesh visualization
            None
        }
        CollisionShape::HeightField { .. } => {
            // TODO: Implement height field visualization
            None
        }
        CollisionShape::Sdf { .. } => {
            // SDFs don't have a simple mesh representation
            None
        }
    }
}

/// Create a sphere mesh.
#[must_use]
pub fn sphere_mesh(radius: f64) -> Mesh {
    Sphere::new(radius as f32).mesh().build()
}

/// Create a box mesh from half extents.
#[must_use]
pub fn box_mesh(half_extents: &nalgebra::Vector3<f64>) -> Mesh {
    Cuboid::new(
        half_extents.x as f32 * 2.0,
        half_extents.y as f32 * 2.0,
        half_extents.z as f32 * 2.0,
    )
    .mesh()
    .build()
}

/// Create a capsule mesh.
#[must_use]
pub fn capsule_mesh(half_length: f64, radius: f64) -> Mesh {
    Capsule3d::new(radius as f32, half_length as f32 * 2.0)
        .mesh()
        .build()
}

/// Create a cylinder mesh.
#[must_use]
pub fn cylinder_mesh(half_length: f64, radius: f64) -> Mesh {
    Cylinder::new(radius as f32, half_length as f32 * 2.0)
        .mesh()
        .build()
}

/// Create an ellipsoid mesh by scaling a sphere.
#[must_use]
pub fn ellipsoid_mesh(radii: &nalgebra::Vector3<f64>) -> Mesh {
    // Use a unit sphere and scale it
    // The actual scaling is applied via Transform, not mesh
    // For now, we create a sphere with radius 1 and expect the caller to scale
    let mut mesh = Sphere::new(1.0).mesh().build();

    // Scale the vertices directly for accurate collision visualization
    if let Some(VertexAttributeValues::Float32x3(positions)) =
        mesh.attribute_mut(Mesh::ATTRIBUTE_POSITION)
    {
        for pos in positions.iter_mut() {
            pos[0] *= radii.x as f32;
            pos[1] *= radii.y as f32;
            pos[2] *= radii.z as f32;
        }
    }

    // Recalculate normals after scaling
    // For an ellipsoid, normals are not simply scaled - they need proper transformation
    // This is an approximation; for accurate normals, we'd need to transform by inverse transpose
    if let Some(VertexAttributeValues::Float32x3(normals)) =
        mesh.attribute_mut(Mesh::ATTRIBUTE_NORMAL)
    {
        let inv_radii = nalgebra::Vector3::new(
            1.0 / radii.x as f32,
            1.0 / radii.y as f32,
            1.0 / radii.z as f32,
        );
        for normal in normals.iter_mut() {
            // Transform normal by inverse transpose of scale matrix
            normal[0] *= inv_radii.x;
            normal[1] *= inv_radii.y;
            normal[2] *= inv_radii.z;
            // Renormalize
            let len =
                (normal[0] * normal[0] + normal[1] * normal[1] + normal[2] * normal[2]).sqrt();
            if len > 1e-6 {
                normal[0] /= len;
                normal[1] /= len;
                normal[2] /= len;
            }
        }
    }

    mesh
}

/// Create a large plane mesh for visualization.
#[must_use]
pub fn plane_mesh() -> Mesh {
    // Create a large quad for plane visualization
    Plane3d::new(Vec3::Y, Vec2::splat(100.0)).mesh().build()
}

/// Create a mesh from convex hull vertices.
///
/// This creates a simple point cloud visualization. For proper convex hull
/// rendering, we'd need to compute the hull faces.
#[must_use]
pub fn convex_mesh_from_vertices(vertices: &[nalgebra::Point3<f64>]) -> Mesh {
    if vertices.is_empty() {
        // Return an empty mesh
        return Mesh::new(
            PrimitiveTopology::TriangleList,
            RenderAssetUsages::default(),
        );
    }

    // For now, compute a simple bounding box visualization
    // A proper implementation would compute the convex hull faces
    let mut min = vertices[0];
    let mut max = vertices[0];
    for v in vertices.iter().skip(1) {
        min.x = min.x.min(v.x);
        min.y = min.y.min(v.y);
        min.z = min.z.min(v.z);
        max.x = max.x.max(v.x);
        max.y = max.y.max(v.y);
        max.z = max.z.max(v.z);
    }

    let half_extents = (max - min) * 0.5;
    let center = min + half_extents;

    // Create a box at the centroid with the bounding dimensions
    let mut mesh = box_mesh(&nalgebra::Vector3::new(
        half_extents.x,
        half_extents.y,
        half_extents.z,
    ));

    // Offset to center
    if let Some(VertexAttributeValues::Float32x3(positions)) =
        mesh.attribute_mut(Mesh::ATTRIBUTE_POSITION)
    {
        for pos in positions.iter_mut() {
            pos[0] += center.x as f32;
            pos[1] += center.y as f32;
            pos[2] += center.z as f32;
        }
    }

    mesh
}

#[cfg(test)]
mod tests {
    use super::*;

    #[test]
    fn sphere_mesh_has_vertices() {
        let mesh = sphere_mesh(1.0);
        assert!(mesh.attribute(Mesh::ATTRIBUTE_POSITION).is_some());
    }

    #[test]
    fn box_mesh_has_vertices() {
        let half_extents = nalgebra::Vector3::new(1.0, 2.0, 3.0);
        let mesh = box_mesh(&half_extents);
        assert!(mesh.attribute(Mesh::ATTRIBUTE_POSITION).is_some());
    }

    #[test]
    fn capsule_mesh_has_vertices() {
        let mesh = capsule_mesh(1.0, 0.5);
        assert!(mesh.attribute(Mesh::ATTRIBUTE_POSITION).is_some());
    }

    #[test]
    fn cylinder_mesh_has_vertices() {
        let mesh = cylinder_mesh(1.0, 0.5);
        assert!(mesh.attribute(Mesh::ATTRIBUTE_POSITION).is_some());
    }

    #[test]
    fn ellipsoid_mesh_has_vertices() {
        let radii = nalgebra::Vector3::new(1.0, 2.0, 0.5);
        let mesh = ellipsoid_mesh(&radii);
        assert!(mesh.attribute(Mesh::ATTRIBUTE_POSITION).is_some());
    }
}
