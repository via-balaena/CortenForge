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

use bevy::asset::RenderAssetUsages;
use bevy::mesh::{PrimitiveTopology, VertexAttributeValues};
use bevy::prelude::*;
use cf_geometry::{IndexedMesh, Shape};
use mesh_types::AttributedMesh;

use crate::convert::{
    dimensions_to_bevy, normal_to_bevy, transform_from_physics, vec3_from_point,
    vertex_positions_from_points,
};

/// Spawn a design mesh entity from an [`AttributedMesh`] positioned in physics space.
///
/// Takes a **physics-space** position (`Point3<f64>`, Z-up) and handles the
/// coordinate conversion to Bevy (Y-up) automatically. This eliminates manual
/// Y↔Z swapping in example/application code.
///
/// If the mesh carries per-vertex normals (typical for SDF-meshed output via
/// `cf_design::Solid::mesh*`), those normals are used directly for smooth
/// shading. Otherwise the renderer falls back to crease-angle splitting on
/// the underlying geometry.
///
/// Returns the spawned [`Entity`] so callers can attach additional components.
pub fn spawn_design_mesh(
    commands: &mut Commands,
    meshes: &mut Assets<Mesh>,
    materials: &mut Assets<StandardMaterial>,
    mesh_data: &AttributedMesh,
    position: nalgebra::Point3<f64>,
    color: Color,
) -> Entity {
    let bevy_mesh = triangle_mesh_from_attributed(mesh_data);

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
        Shape::TriangleMesh { mesh, .. } => Some(triangle_mesh_from_indexed(mesh.as_ref())),
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

/// Create a Bevy mesh from a positions+faces [`IndexedMesh`].
///
/// No per-vertex normals available: the renderer computes them from triangle
/// geometry using a 30° crease-angle split — sharp edges get duplicated
/// vertices with per-face normals; gentle edges share smoothed vertex
/// normals. Use [`triangle_mesh_from_attributed`] when analytical
/// per-vertex normals (e.g. SDF gradient normals from a marching-cubes pass)
/// are available — that path skips the splitting and renders smoothly.
///
/// Coordinates are converted from physics Z-up to Bevy Y-up using
/// [`vertex_positions_from_points`] and [`normal_to_bevy`].
#[must_use]
pub fn triangle_mesh_from_indexed(mesh_data: &IndexedMesh) -> Mesh {
    // Crease-angle vertex splitting: sharp edges (> 30°) get split vertices
    // with per-face normals, gentle edges share vertices with smooth averaged
    // normals. This gives flat shading on cuboid faces and smooth shading on
    // curved SDF surfaces. 30° matches the industry standard auto-smooth
    // threshold (Blender, Maya).
    const CREASE_COS: f64 = 0.866; // cos(30°)

    let vertices = &mesh_data.vertices;
    let triangles = &mesh_data.faces;

    if vertices.is_empty() || triangles.is_empty() {
        return Mesh::new(
            PrimitiveTopology::TriangleList,
            RenderAssetUsages::default(),
        );
    }

    // Pre-convert all positions to Bevy coordinates
    let bevy_positions = vertex_positions_from_points(vertices);

    // Compute unit face normals in physics space
    let face_normals: Vec<nalgebra::Vector3<f64>> = triangles
        .iter()
        .map(|face| {
            let v0 = vertices[face[0] as usize];
            let v1 = vertices[face[1] as usize];
            let v2 = vertices[face[2] as usize];
            let n = (v1 - v0).cross(&(v2 - v0));
            let len = n.norm();
            if len > 1e-10 {
                n / len
            } else {
                nalgebra::Vector3::z()
            }
        })
        .collect();

    // For each original vertex, track output splits: (output_index, representative_normal).
    // A face joins an existing split if its normal is within the crease angle of the
    // representative (first face that created the split). Otherwise a new split is created.
    //
    // Winding fix: the Y↔Z coordinate swap (Z-up → Y-up) reverses handedness,
    // flipping triangle winding. We emit vertices as (v0, v2, v1) instead of
    // (v0, v1, v2) to restore correct front-face orientation in Bevy.
    let mut positions: Vec<[f32; 3]> = Vec::new();
    let mut normal_sums: Vec<nalgebra::Vector3<f64>> = Vec::new();
    let mut indices: Vec<u32> = Vec::with_capacity(triangles.len() * 3);
    let mut splits: Vec<Vec<(u32, nalgebra::Vector3<f64>)>> = vec![Vec::new(); vertices.len()];

    for (fi, face) in triangles.iter().enumerate() {
        let fn_ = face_normals[fi];
        for &vi in &[face[0], face[2], face[1]] {
            let vi_usize = vi as usize;
            let found = splits[vi_usize]
                .iter()
                .find(|(_, rep)| fn_.dot(rep) >= CREASE_COS)
                .map(|&(idx, _)| idx);

            if let Some(out_idx) = found {
                normal_sums[out_idx as usize] += fn_;
                indices.push(out_idx);
            } else {
                let out_idx = positions.len() as u32;
                positions.push(bevy_positions[vi_usize]);
                normal_sums.push(fn_);
                splits[vi_usize].push((out_idx, fn_));
                indices.push(out_idx);
            }
        }
    }

    // Normalize accumulated normals and convert to Bevy coordinates
    let normals: Vec<[f32; 3]> = normal_sums
        .iter()
        .map(|n| {
            let len = n.norm();
            if len > 1e-6 {
                normal_to_bevy(&(n / len))
            } else {
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

/// Create a Bevy mesh from an [`AttributedMesh`].
///
/// If `mesh_data.normals` is `Some`, the per-vertex normals are used directly
/// (the analytical-normals fast path: typical for SDF marching-cubes output
/// where the normals come from the field gradient). The Y↔Z winding swap
/// still applies. If `normals` is `None`, the renderer falls back to
/// [`triangle_mesh_from_indexed`] on the underlying geometry, which derives
/// smooth/flat normals via crease-angle splitting.
#[must_use]
pub fn triangle_mesh_from_attributed(mesh_data: &AttributedMesh) -> Mesh {
    let vertices = &mesh_data.geometry.vertices;
    let triangles = &mesh_data.geometry.faces;

    if vertices.is_empty() || triangles.is_empty() {
        return Mesh::new(
            PrimitiveTopology::TriangleList,
            RenderAssetUsages::default(),
        );
    }

    // No per-vertex normals: defer to the geometry-only path, which handles
    // crease-angle splitting and Y↔Z winding.
    let Some(ref vertex_normals) = mesh_data.normals else {
        return triangle_mesh_from_indexed(&mesh_data.geometry);
    };

    let positions: Vec<[f32; 3]> = vertex_positions_from_points(vertices);
    let normals: Vec<[f32; 3]> = vertex_normals
        .iter()
        .map(|n| {
            let len = n.norm();
            if len > 1e-10 {
                normal_to_bevy(&(n / len))
            } else {
                [0.0, 1.0, 0.0]
            }
        })
        .collect();
    // Winding fix: Y↔Z swap reverses handedness → emit (v0, v2, v1)
    let indices: Vec<u32> = triangles
        .iter()
        .flat_map(|face| [face[0], face[2], face[1]])
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

// ── Spring coil mesh ────────────────────────────────────────────────────────

/// Parameters for a helical coil spring mesh.
///
/// Used with [`spring_coil`] to generate a tube-swept helix along the +X axis.
/// The `Default` provides sensible values for meter-scale simulations.
#[derive(Debug, Clone)]
pub struct SpringCoilParams {
    /// Number of coil turns.
    pub turns: u32,
    /// Radius of the helix (distance from axis to wire center).
    pub radius: f32,
    /// Radius of the wire tube cross-section.
    pub tube_radius: f32,
    /// Segments per full turn (controls smoothness). Default: 24.
    pub segments_per_turn: u32,
    /// Tube cross-section segments. Default: 8.
    pub tube_segments: u32,
    /// Minimum length clamp to avoid degenerate geometry. Default: 0.05.
    /// Adjust for your simulation scale (e.g. 0.001 for mm-scale sims).
    pub min_length: f32,
}

impl Default for SpringCoilParams {
    fn default() -> Self {
        Self {
            turns: 10,
            radius: 0.06,
            tube_radius: 0.008,
            segments_per_turn: 24,
            tube_segments: 8,
            min_length: 0.05,
        }
    }
}

/// Generate a helical coil mesh from `x=0` to `x=length`.
///
/// The coil axis is along +X (Bevy space). Negative or very small lengths
/// are clamped to `params.min_length` — never panics, always returns a valid mesh.
///
/// Intended for dynamic use: regenerate each frame with a new length to
/// animate spring compression/extension.
///
/// `usage` controls GPU/CPU asset residency (pass `RenderAssetUsages::default()`
/// for typical use, or `RENDER_WORLD` for GPU-only).
// Floating-point form chosen for numerical stability, not micro-optimization.
#[allow(clippy::suboptimal_flops, clippy::cast_precision_loss)]
#[must_use]
pub fn spring_coil(params: &SpringCoilParams, length: f32, usage: RenderAssetUsages) -> Mesh {
    use std::f32::consts::TAU;

    let length = length.max(params.min_length);
    let total_segs = params.turns * params.segments_per_turn;

    // Handle degenerate case (zero turns)
    if total_segs == 0 {
        return Mesh::new(PrimitiveTopology::TriangleList, usage);
    }

    let n_verts = ((total_segs + 1) * (params.tube_segments + 1)) as usize;
    let mut positions = Vec::with_capacity(n_verts);
    let mut normals = Vec::with_capacity(n_verts);
    let mut indices = Vec::new();

    for i in 0..=total_segs {
        let t = i as f32 / total_segs as f32;
        let cx = t * length;
        let helix_angle = t * params.turns as f32 * TAU;
        let cy = params.radius * helix_angle.cos();
        let cz = params.radius * helix_angle.sin();

        // Radial direction (outward from helix axis)
        let radial_y = helix_angle.cos();
        let radial_z = helix_angle.sin();

        // Tangent of the helix (Frenet frame)
        let tangent_x = length / total_segs as f32;
        let tangent_y =
            -params.radius * helix_angle.sin() * params.turns as f32 * TAU / total_segs as f32;
        let tangent_z =
            params.radius * helix_angle.cos() * params.turns as f32 * TAU / total_segs as f32;
        let tang_len = (tangent_x * tangent_x + tangent_y * tangent_y + tangent_z * tangent_z)
            .sqrt()
            .max(1e-6);
        let tx = tangent_x / tang_len;
        let ty = tangent_y / tang_len;
        let tz = tangent_z / tang_len;

        // Binormal = tangent × radial
        let bx = ty * radial_z - tz * radial_y;
        let by = -tx * radial_z;
        let bz = tx * radial_y;
        let b_len = (bx * bx + by * by + bz * bz).sqrt().max(1e-6);
        let bx = bx / b_len;
        let by = by / b_len;
        let bz = bz / b_len;

        for j in 0..=params.tube_segments {
            let tube_angle = j as f32 / params.tube_segments as f32 * TAU;
            let cos_t = tube_angle.cos();
            let sin_t = tube_angle.sin();

            let nx = sin_t * bx;
            let ny = cos_t * radial_y + sin_t * by;
            let nz = cos_t * radial_z + sin_t * bz;

            let px = cx + params.tube_radius * (sin_t * bx);
            let py = cy + params.tube_radius * (cos_t * radial_y + sin_t * by);
            let pz = cz + params.tube_radius * (cos_t * radial_z + sin_t * bz);

            positions.push([px, py, pz]);
            let n_len = (nx * nx + ny * ny + nz * nz).sqrt().max(1e-6);
            normals.push([nx / n_len, ny / n_len, nz / n_len]);
        }
    }

    // Triangle indices: quads between adjacent rings
    let ring = params.tube_segments + 1;
    for i in 0..total_segs {
        for j in 0..params.tube_segments {
            let a = i * ring + j;
            let b = a + ring;
            let c = b + 1;
            let d = a + 1;
            indices.extend_from_slice(&[a, b, d, b, c, d]);
        }
    }

    let mut mesh = Mesh::new(PrimitiveTopology::TriangleList, usage);
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
    // Localized expect: invariant guarantees the value is present.
    #[allow(clippy::expect_used)]
    fn spawn_design_mesh_converts_z_up_to_y_up() {
        // Minimal triangle mesh (single triangle)
        let geometry = IndexedMesh {
            vertices: vec![
                nalgebra::Point3::new(0.0, 0.0, 0.0),
                nalgebra::Point3::new(1.0, 0.0, 0.0),
                nalgebra::Point3::new(0.0, 1.0, 0.0),
            ],
            faces: vec![[0, 1, 2]],
        };
        let mesh_data = AttributedMesh::from(geometry);

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

    #[test]
    fn spring_coil_default_params() {
        let params = SpringCoilParams::default();
        let mesh = spring_coil(&params, 1.0, RenderAssetUsages::default());
        assert!(mesh.attribute(Mesh::ATTRIBUTE_POSITION).is_some());
        assert!(mesh.attribute(Mesh::ATTRIBUTE_NORMAL).is_some());
        assert!(mesh.indices().is_some());
    }

    #[test]
    fn spring_coil_negative_length_clamps() {
        let params = SpringCoilParams::default();
        let mesh = spring_coil(&params, -5.0, RenderAssetUsages::default());
        // Should not panic, returns valid mesh at min_length
        assert!(mesh.attribute(Mesh::ATTRIBUTE_POSITION).is_some());
    }

    #[test]
    fn spring_coil_zero_turns() {
        let params = SpringCoilParams {
            turns: 0,
            ..Default::default()
        };
        let mesh = spring_coil(&params, 1.0, RenderAssetUsages::default());
        // Should return an empty but valid mesh
        assert!(mesh.indices().is_none_or(bevy::mesh::Indices::is_empty));
    }
}
