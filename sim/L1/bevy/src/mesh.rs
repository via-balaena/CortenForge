//! Mesh generation for shape visualization.
//!
//! Converts cf-geometry shapes to Bevy meshes.
//!
//! ## Coordinate System
//!
//! All coordinate conversion between physics (Z-up) and Bevy (Y-up) is handled
//! through the functions in [`crate::convert`]. This module uses those functions
//! to ensure consistent coordinate transformation across all mesh types.

#![allow(clippy::cast_possible_truncation)] // f64 -> f32 is intentional for Bevy meshes

use std::sync::Arc;

use bevy::asset::RenderAssetUsages;
use bevy::mesh::{PrimitiveTopology, VertexAttributeValues};
use bevy::prelude::*;
use cf_geometry::{IndexedMesh, Shape};

use crate::convert::{
    dimensions_to_bevy, normal_to_bevy, transform_from_physics, vec3_from_point,
    vertex_positions_from_points,
};

/// Spawn a design mesh entity from an [`IndexedMesh`] positioned in physics space.
///
/// Takes a **physics-space** position (`Point3<f64>`, Z-up) and handles the
/// coordinate conversion to Bevy (Y-up) automatically. This eliminates manual
/// Y↔Z swapping in example/application code.
///
/// Returns the spawned [`Entity`] so callers can attach additional components.
pub fn spawn_design_mesh(
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
    mesh_data: &IndexedMesh,
    position: nalgebra::Point3<f64>,
    color: Color,
) -> Entity {
    let indexed = Arc::new(mesh_data.clone());
    let bevy_mesh = triangle_mesh_from_indexed(&indexed);

    commands
        .spawn((
            Mesh3d(meshes.add(bevy_mesh)),
            MeshMaterial3d(materials.add(StandardMaterial {
                base_color: color,
                metallic: 0.15,
                perceptual_roughness: 0.7,
                double_sided: true,
                cull_mode: None,
                ..default()
            })),
            transform_from_physics(&position),
        ))
        .id()
}

/// Generate a Bevy mesh from a cf-geometry [`Shape`].
///
/// Returns `None` for shapes that cannot be easily visualized (e.g., SDF).
#[must_use]
pub fn mesh_from_shape(shape: &Shape) -> Option<Mesh> {
    match shape {
        Shape::Sphere { radius } => Some(sphere_mesh(*radius)),
        Shape::Box { half_extents } => Some(box_mesh(half_extents)),
        Shape::Capsule {
            half_length,
            radius,
        } => Some(capsule_mesh(*half_length, *radius)),
        Shape::Cylinder {
            half_length,
            radius,
        } => Some(cylinder_mesh(*half_length, *radius)),
        Shape::Ellipsoid { radii } => Some(ellipsoid_mesh(radii)),
        Shape::Plane { .. } => Some(plane_mesh()),
        Shape::ConvexMesh { hull } => Some(convex_mesh_from_vertices(&hull.vertices)),
        Shape::TriangleMesh { mesh, .. } => Some(triangle_mesh_from_indexed(mesh)),
        Shape::HeightField { .. } => {
            // DT-178: Implement height field visualization
            None
        }
        Shape::Sdf { .. } => {
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
///
/// Converts from physics Z-up coordinates to Bevy Y-up using [`dimensions_to_bevy`].
#[must_use]
pub fn box_mesh(half_extents: &nalgebra::Vector3<f64>) -> Mesh {
    let (x, y, z) = dimensions_to_bevy(half_extents);
    Cuboid::new(x * 2.0, y * 2.0, z * 2.0).mesh().build()
}

/// Create a capsule mesh.
///
/// Bevy's `Capsule3d` is oriented along the Y-axis, which corresponds to
/// the physics Z-axis after coordinate conversion (Z-up -> Y-up).
#[must_use]
pub fn capsule_mesh(half_length: f64, radius: f64) -> Mesh {
    Capsule3d::new(radius as f32, half_length as f32 * 2.0)
        .mesh()
        .build()
}

/// Create a cylinder mesh.
///
/// Bevy's `Cylinder` is oriented along the Y-axis, which corresponds to
/// the physics Z-axis after coordinate conversion (Z-up -> Y-up).
#[must_use]
pub fn cylinder_mesh(half_length: f64, radius: f64) -> Mesh {
    Cylinder::new(radius as f32, half_length as f32 * 2.0)
        .mesh()
        .build()
}

/// Create an ellipsoid mesh by scaling a sphere.
///
/// Converts from physics Z-up coordinates to Bevy Y-up using [`dimensions_to_bevy`].
#[must_use]
pub fn ellipsoid_mesh(radii: &nalgebra::Vector3<f64>) -> Mesh {
    // Convert radii from physics to Bevy coordinate system
    let (rx, ry, rz) = dimensions_to_bevy(radii);

    let mut mesh = Sphere::new(1.0).mesh().build();

    // Scale the vertices directly for accurate collision visualization
    if let Some(VertexAttributeValues::Float32x3(positions)) =
        mesh.attribute_mut(Mesh::ATTRIBUTE_POSITION)
    {
        for pos in positions.iter_mut() {
            pos[0] *= rx;
            pos[1] *= ry;
            pos[2] *= rz;
        }
    }

    // Recalculate normals after scaling
    // For an ellipsoid, normals are not simply scaled - they need proper transformation
    // This is an approximation; for accurate normals, we'd need to transform by inverse transpose
    if let Some(VertexAttributeValues::Float32x3(normals)) =
        mesh.attribute_mut(Mesh::ATTRIBUTE_NORMAL)
    {
        // Inverse radii for normal transformation (inverse transpose of scale matrix)
        let inv_radii = (1.0 / rx, 1.0 / ry, 1.0 / rz);
        for normal in normals.iter_mut() {
            // Transform normal by inverse transpose of scale matrix
            normal[0] *= inv_radii.0;
            normal[1] *= inv_radii.1;
            normal[2] *= inv_radii.2;
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
/// This creates a bounding box visualization of the convex hull.
/// For proper convex hull rendering, we'd need to compute the hull faces.
///
/// Coordinates are converted from physics Z-up to Bevy Y-up using [`vec3_from_point`].
#[must_use]
pub fn convex_mesh_from_vertices(vertices: &[nalgebra::Point3<f64>]) -> Mesh {
    if vertices.is_empty() {
        // Return an empty mesh
        return Mesh::new(
            PrimitiveTopology::TriangleList,
            RenderAssetUsages::default(),
        );
    }

    // Compute bounding box in physics coordinates
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
    let center = nalgebra::Point3::new(
        min.x + half_extents.x,
        min.y + half_extents.y,
        min.z + half_extents.z,
    );

    // Create a box at the centroid with the bounding dimensions
    // box_mesh handles the coordinate conversion for dimensions
    let mut mesh = box_mesh(&nalgebra::Vector3::new(
        half_extents.x,
        half_extents.y,
        half_extents.z,
    ));

    // Offset to center using proper coordinate conversion
    let bevy_center = vec3_from_point(&center);
    if let Some(VertexAttributeValues::Float32x3(positions)) =
        mesh.attribute_mut(Mesh::ATTRIBUTE_POSITION)
    {
        for pos in positions.iter_mut() {
            pos[0] += bevy_center.x;
            pos[1] += bevy_center.y;
            pos[2] += bevy_center.z;
        }
    }

    mesh
}

/// Create a mesh from a cf-geometry [`IndexedMesh`].
///
/// Converts the `IndexedMesh` to a Bevy mesh with proper vertices, indices,
/// and normals.
///
/// Coordinates are converted from physics Z-up to Bevy Y-up using
/// [`vertex_positions_from_points`] and [`normal_to_bevy`].
#[must_use]
pub fn triangle_mesh_from_indexed(mesh_data: &Arc<IndexedMesh>) -> Mesh {
    let vertices = &mesh_data.vertices;
    let triangles = &mesh_data.faces;

    if vertices.is_empty() || triangles.is_empty() {
        return Mesh::new(
            PrimitiveTopology::TriangleList,
            RenderAssetUsages::default(),
        );
    }

    // Convert vertices from physics (Z-up) to Bevy (Y-up) coordinates
    let positions = vertex_positions_from_points(vertices);

    // Convert triangle indices (already u32 from cf-geometry)
    let indices: Vec<u32> = triangles
        .iter()
        .flat_map(|face| [face[0], face[1], face[2]])
        .collect();

    // Compute normals per vertex (average of adjacent face normals) in physics space,
    // then convert to Bevy space
    let mut physics_normals = vec![nalgebra::Vector3::zeros(); vertices.len()];
    for face in triangles {
        let v0 = vertices[face[0] as usize];
        let v1 = vertices[face[1] as usize];
        let v2 = vertices[face[2] as usize];

        let e1 = v1 - v0;
        let e2 = v2 - v0;
        let face_normal = e1.cross(&e2);

        // Add face normal to each vertex's accumulated normal (in physics space)
        for &idx in &[face[0] as usize, face[1] as usize, face[2] as usize] {
            physics_normals[idx] += face_normal;
        }
    }

    // Normalize and convert to Bevy coordinates
    let normals: Vec<[f32; 3]> = physics_normals
        .iter()
        .map(|n| {
            let len = n.norm();
            if len > 1e-6 {
                let normalized = n / len;
                normal_to_bevy(&normalized)
            } else {
                // Default to up (Y-up in Bevy)
                [0.0, 1.0, 0.0]
            }
        })
        .collect();

    let mut mesh = Mesh::new(
        PrimitiveTopology::TriangleList,
        RenderAssetUsages::default(),
    );
    mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, positions);
    mesh.insert_attribute(Mesh::ATTRIBUTE_NORMAL, normals);
    mesh.insert_indices(bevy::mesh::Indices::U32(indices));

    mesh
}

#[cfg(test)]
mod tests {
    use super::*;
    use bevy::ecs::world::CommandQueue;

    #[test]
    #[allow(clippy::expect_used)]
    fn spawn_design_mesh_converts_z_up_to_y_up() {
        // Minimal triangle mesh (single triangle)
        let mesh_data = IndexedMesh {
            vertices: vec![
                nalgebra::Point3::new(0.0, 0.0, 0.0),
                nalgebra::Point3::new(1.0, 0.0, 0.0),
                nalgebra::Point3::new(0.0, 1.0, 0.0),
            ],
            faces: vec![[0, 1, 2]],
        };

        let mut world = bevy::prelude::World::new();
        let mut meshes = Assets::<Mesh>::default();
        let mut materials = Assets::<StandardMaterial>::default();

        // Physics position: X=1, Y=2, Z=3 (Z is up)
        let position = nalgebra::Point3::new(1.0, 2.0, 3.0);

        let mut queue = CommandQueue::default();
        let entity;
        {
            let mut commands = Commands::new(&mut queue, &world);
            entity = spawn_design_mesh(
                &mut commands,
                &mut meshes,
                &mut materials,
                &mesh_data,
                position,
                Color::WHITE,
            );
        }
        queue.apply(&mut world);

        // Verify: physics (1, 2, 3) → Bevy (1, 3, 2) (Y↔Z swap)
        let transform = world
            .get::<Transform>(entity)
            .expect("entity should have Transform");
        let t = transform.translation;
        assert!(
            (t.x - 1.0).abs() < 1e-6 && (t.y - 3.0).abs() < 1e-6 && (t.z - 2.0).abs() < 1e-6,
            "Expected Bevy translation (1, 3, 2), got ({}, {}, {})",
            t.x,
            t.y,
            t.z
        );
    }

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
