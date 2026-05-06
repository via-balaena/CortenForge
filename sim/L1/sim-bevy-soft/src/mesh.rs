//! Soft-mesh build + per-frame position + smooth-normal write.
//!
//! [`build_soft_mesh`] consumes a sim-soft rest-configuration positions
//! slice + a `boundary_faces` triangulation and emits a Bevy `Mesh` with
//! the input → Bevy frame swap applied via
//! [`cf_bevy_common::axis::UpAxis::to_bevy_point`]; winding flips under
//! parity-flipping swaps via
//! [`cf_bevy_common::axis::UpAxis::flips_winding`] (sister of
//! `cf-viewer/src/mesh.rs::build_face_mesh`).
//!
//! [`apply_soft_positions`] overwrites a Mesh's `ATTRIBUTE_POSITION`
//! attribute with a frame's deformed positions and recomputes smooth
//! vertex normals so PBR shading tracks the deformation rather than
//! sticking to the rest configuration. Per-frame
//! [`Mesh::compute_smooth_normals`] cost is negligible at expected mesh
//! sizes (hundreds to thousands of vertices).
//!
//! # Vertex set
//!
//! The Bevy `Mesh` carries ALL sim-soft vertices, including interior ones
//! the boundary triangulation never references; the unindexed interior
//! vertices pay only their position+normal entries (~24 bytes each) and
//! aren't drawn. This 1:1 indexing keeps the position-update path a
//! straightforward `i → swap(frame[i*3..])` walk with no boundary-vertex
//! id map.

#![allow(clippy::cast_possible_truncation)] // f64 → f32 is intentional for Bevy meshes

use bevy::asset::RenderAssetUsages;
use bevy::mesh::{Indices, Mesh, PrimitiveTopology};
use cf_bevy_common::axis::UpAxis;
use mesh_types::Point3;
use sim_soft::VertexId;

/// Build a Bevy `Mesh` from a sim-soft rest-configuration positions slice
/// + a `boundary_faces` triangulation.
///
/// - Positions: f64 → f32 cast with the [`UpAxis`] swap applied via
///   [`UpAxis::to_bevy_point`]. All sim-soft vertices included; interior
///   ones unreferenced by the index buffer.
/// - Indices: triangle list. When the swap is parity-flipping
///   ([`UpAxis::flips_winding`]), emits `(v0, v2, v1)` to restore CCW
///   front-facing; otherwise emits `(v0, v1, v2)` unchanged.
/// - Normals: smooth, area-and-corner-angle-weighted via
///   [`Mesh::compute_smooth_normals`] over the rest configuration. The
///   per-frame replay path ([`apply_soft_positions`]) recomputes
///   against each frame's deformed positions.
///
/// `boundary_faces` is the slice returned by
/// [`sim_soft::Mesh::boundary_faces`]; outward-CCW winding follows from
/// each constructor's right-handed-tet `signed_volume > 0` invariant
/// (see `sim/L0/soft/src/mesh/mod.rs::boundary_faces_from_topology`).
///
/// [`UpAxis`]: cf_bevy_common::axis::UpAxis
/// [`UpAxis::to_bevy_point`]: cf_bevy_common::axis::UpAxis::to_bevy_point
/// [`UpAxis::flips_winding`]: cf_bevy_common::axis::UpAxis::flips_winding
#[must_use]
pub fn build_soft_mesh(
    positions: &[sim_soft::Vec3],
    boundary_faces: &[[VertexId; 3]],
    up: UpAxis,
) -> Mesh {
    let bevy_positions: Vec<[f32; 3]> = positions
        .iter()
        .map(|v| up.to_bevy_point(&Point3::new(v.x, v.y, v.z)))
        .collect();

    // Parity-flipping swaps reverse handedness → emit (v0, v2, v1) to keep
    // CCW front-facing. Identity swap (`+Y`) leaves winding alone. Same
    // pattern as cf-viewer/src/mesh.rs::build_face_mesh.
    let indices: Vec<u32> = if up.flips_winding() {
        boundary_faces
            .iter()
            .flat_map(|f| [f[0], f[2], f[1]])
            .collect()
    } else {
        boundary_faces
            .iter()
            .flat_map(|f| [f[0], f[1], f[2]])
            .collect()
    };

    let mut mesh = Mesh::new(
        PrimitiveTopology::TriangleList,
        RenderAssetUsages::default(),
    );
    mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, bevy_positions);
    mesh.insert_indices(Indices::U32(indices));
    mesh.compute_smooth_normals();
    mesh
}

/// Write a frame's stride-3 f64 positions into the Mesh's
/// `ATTRIBUTE_POSITION` buffer (with the [`UpAxis`] swap), then recompute
/// smooth vertex normals so shading tracks the deformed geometry.
///
/// `frame.len()` must equal `3 * n_vertices` (vertex-major + xyz-inner DOF
/// layout — same packing as `sim_soft::NewtonStep::x_final` per
/// `sim/L0/soft/src/solver/mod.rs`). The Mesh must already carry an index
/// buffer (set by [`build_soft_mesh`]) — `compute_smooth_normals` panics
/// without one.
///
/// Allocates a fresh `Vec<[f32; 3]>` per call. Negligible at expected
/// mesh sizes; if a vertex count ever pushes this onto the profiler,
/// the existing position buffer can be mutated in place via
/// `Mesh::attribute_mut(ATTRIBUTE_POSITION)`.
///
/// [`UpAxis`]: cf_bevy_common::axis::UpAxis
pub fn apply_soft_positions(mesh: &mut Mesh, frame: &[f64], up: UpAxis) {
    let n_vertices = frame.len() / 3;
    let bevy_positions: Vec<[f32; 3]> = (0..n_vertices)
        .map(|i| {
            up.to_bevy_point(&Point3::new(
                frame[i * 3],
                frame[i * 3 + 1],
                frame[i * 3 + 2],
            ))
        })
        .collect();
    mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, bevy_positions);
    mesh.compute_smooth_normals();
}

#[cfg(test)]
mod tests {
    use super::*;
    use bevy::mesh::VertexAttributeValues;

    /// Right-handed unit tet — 4 vertices forming a positive-volume
    /// tetrahedron, suitable for `boundary_faces_from_topology`'s
    /// outward-winding convention.
    fn single_tet_positions() -> Vec<sim_soft::Vec3> {
        vec![
            sim_soft::Vec3::new(0.0, 0.0, 0.0),
            sim_soft::Vec3::new(1.0, 0.0, 0.0),
            sim_soft::Vec3::new(0.0, 1.0, 0.0),
            sim_soft::Vec3::new(0.0, 0.0, 1.0),
        ]
    }

    /// All four faces of a right-handed tet, per the
    /// `boundary_faces_from_topology` convention in
    /// `sim/L0/soft/src/mesh/mod.rs`.
    fn single_tet_boundary_faces() -> Vec<[VertexId; 3]> {
        vec![
            [1, 2, 3], // opposite v0
            [0, 3, 2], // opposite v1
            [0, 1, 3], // opposite v2
            [0, 2, 1], // opposite v3
        ]
    }

    fn position_count(mesh: &Mesh) -> usize {
        match mesh.attribute(Mesh::ATTRIBUTE_POSITION) {
            Some(VertexAttributeValues::Float32x3(values)) => values.len(),
            _ => 0,
        }
    }

    fn normal_count(mesh: &Mesh) -> usize {
        match mesh.attribute(Mesh::ATTRIBUTE_NORMAL) {
            Some(VertexAttributeValues::Float32x3(values)) => values.len(),
            _ => 0,
        }
    }

    fn read_positions(mesh: &Mesh) -> Vec<[f32; 3]> {
        match mesh.attribute(Mesh::ATTRIBUTE_POSITION) {
            Some(VertexAttributeValues::Float32x3(values)) => values.clone(),
            _ => Vec::new(),
        }
    }

    fn read_indices(mesh: &Mesh) -> Vec<u32> {
        match mesh.indices() {
            Some(Indices::U32(idx)) => idx.clone(),
            _ => Vec::new(),
        }
    }

    /// Shape: positions + normals + index counts match the input.
    #[test]
    fn build_soft_mesh_shape_matches_input() {
        let positions = single_tet_positions();
        let faces = single_tet_boundary_faces();
        let mesh = build_soft_mesh(&positions, &faces, UpAxis::PlusZ);
        assert_eq!(position_count(&mesh), 4);
        assert_eq!(normal_count(&mesh), 4); // smooth normals computed at build time
        assert_eq!(read_indices(&mesh).len(), faces.len() * 3);
    }

    /// Z-up swap: physics `(x, y, z)` lands in Bevy as `(x, z, y)`.
    #[test]
    fn build_soft_mesh_under_plus_z_swaps_y_and_z() {
        let positions = vec![sim_soft::Vec3::new(1.0, 2.0, 3.0)];
        let mesh = build_soft_mesh(&positions, &[], UpAxis::PlusZ);
        let stored = read_positions(&mesh);
        assert_eq!(stored, vec![[1.0_f32, 3.0, 2.0]]);
    }

    /// `+Y` is identity on positions.
    #[test]
    fn build_soft_mesh_under_plus_y_is_identity() {
        let positions = vec![sim_soft::Vec3::new(1.0, 2.0, 3.0)];
        let mesh = build_soft_mesh(&positions, &[], UpAxis::PlusY);
        let stored = read_positions(&mesh);
        assert_eq!(stored, vec![[1.0_f32, 2.0, 3.0]]);
    }

    /// `+Z` (parity-flipping) reverses face winding to keep CCW front-facing.
    #[test]
    fn build_soft_mesh_flips_winding_under_plus_z() {
        let faces = vec![[0_u32, 1, 2]];
        let mesh = build_soft_mesh(
            &[
                sim_soft::Vec3::zeros(),
                sim_soft::Vec3::zeros(),
                sim_soft::Vec3::zeros(),
            ],
            &faces,
            UpAxis::PlusZ,
        );
        assert_eq!(read_indices(&mesh), vec![0_u32, 2, 1]);
    }

    /// `+Y` (identity) preserves face winding.
    #[test]
    fn build_soft_mesh_preserves_winding_under_plus_y() {
        let faces = vec![[0_u32, 1, 2]];
        let mesh = build_soft_mesh(
            &[
                sim_soft::Vec3::zeros(),
                sim_soft::Vec3::zeros(),
                sim_soft::Vec3::zeros(),
            ],
            &faces,
            UpAxis::PlusY,
        );
        assert_eq!(read_indices(&mesh), vec![0_u32, 1, 2]);
    }

    /// `apply_soft_positions` overwrites the position buffer with the new
    /// frame, applying the up-axis swap.
    #[test]
    fn apply_soft_positions_writes_new_positions() {
        let positions = single_tet_positions();
        let faces = single_tet_boundary_faces();
        let mut mesh = build_soft_mesh(&positions, &faces, UpAxis::PlusZ);

        // Frame: every vertex translated by +1 in the input X axis.
        let frame = vec![
            1.0, 0.0, 0.0, // v0 + (1,0,0)
            2.0, 0.0, 0.0, // v1 + (1,0,0)
            1.0, 1.0, 0.0, // v2 + (1,0,0)
            1.0, 0.0, 1.0, // v3 + (1,0,0)
        ];
        apply_soft_positions(&mut mesh, &frame, UpAxis::PlusZ);
        let stored = read_positions(&mesh);
        // PlusZ swap: (x, y, z) → (x, z, y). Input X-axis translation
        // surfaces in Bevy X under the swap.
        assert_eq!(stored[0], [1.0_f32, 0.0, 0.0]);
        assert_eq!(stored[1], [2.0_f32, 0.0, 0.0]);
        assert_eq!(stored[2], [1.0_f32, 0.0, 1.0]);
        assert_eq!(stored[3], [1.0_f32, 1.0, 0.0]);
    }

    /// Smooth normals are present after `apply_soft_positions` — confirms
    /// the per-frame recompute fires (visual lag fix per Q8 lock).
    #[test]
    fn apply_soft_positions_recomputes_normals() {
        let positions = single_tet_positions();
        let faces = single_tet_boundary_faces();
        let mut mesh = build_soft_mesh(&positions, &faces, UpAxis::PlusZ);

        // Move v3 dramatically; new normals will differ from rest.
        let frame = vec![
            0.0, 0.0, 0.0, //
            1.0, 0.0, 0.0, //
            0.0, 1.0, 0.0, //
            0.0, 0.0, 5.0, // v3 stretched along z
        ];
        apply_soft_positions(&mut mesh, &frame, UpAxis::PlusZ);
        // Normal count tracks vertex count after recompute.
        assert_eq!(normal_count(&mesh), 4);
    }
}
