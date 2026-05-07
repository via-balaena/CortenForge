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
//! [`Mesh::compute_area_weighted_normals`] cost is negligible at
//! expected mesh sizes (hundreds to thousands of vertices).
//!
//! # Why area-weighted instead of corner-angle smoothing
//!
//! Bevy 0.18's [`Mesh::compute_smooth_normals`] (corner-angle weighted)
//! has a hardcoded `EPS = f32::EPSILON` check on `(edge_a² × edge_c²)`
//! that gates each triangle's contribution. At sim-soft's typical
//! cell-size scale (e.g. `3 mm` for V-5 / row 12), the squared-edge
//! product is `~8e-11` — below `f32::EPSILON ≈ 1.2e-7`, so EVERY
//! per-vertex weight zeros out and `compute_smooth_normals` produces
//! all-zero normals (PBR sees `NdotL = 0` everywhere → ambient-only
//! shading regardless of light direction). [`Mesh::compute_area_weighted_normals`]
//! uses raw cross-product magnitudes (un-normalized; magnitude = `2 ×
//! triangle_area`) and only normalizes at the end, so the accumulator
//! stays well above any normalization threshold at our scales. Both
//! methods produce smooth Gouraud-shading-grade per-vertex normals;
//! the weighting difference (corner-angle vs area) is imperceptible
//! for fairly uniform tet-boundary triangulations.
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
/// - Normals: smooth, area-weighted via
///   [`Mesh::compute_area_weighted_normals`] over the rest
///   configuration. The per-frame replay path
///   ([`apply_soft_positions`]) recomputes against each frame's
///   deformed positions. See module-level "Why area-weighted instead of
///   corner-angle smoothing" for the small-scale-mesh
///   `compute_smooth_normals`-zero-out gotcha that selects this
///   variant.
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
    mesh.compute_area_weighted_normals();
    mesh
}

/// Write a frame's stride-3 f64 positions into the Mesh's
/// `ATTRIBUTE_POSITION` buffer (with the [`UpAxis`] swap), then recompute
/// area-weighted vertex normals so shading tracks the deformed geometry.
///
/// `frame.len()` must equal `3 * n_vertices` (vertex-major + xyz-inner DOF
/// layout — same packing as `sim_soft::NewtonStep::x_final` per
/// `sim/L0/soft/src/solver/mod.rs`). The Mesh must already carry an index
/// buffer (set by [`build_soft_mesh`]) — `compute_area_weighted_normals`
/// panics without one.
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
    mesh.compute_area_weighted_normals();
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

    /// Read normals out of the mesh as `Vec<[f32; 3]>`.
    fn read_normals(mesh: &Mesh) -> Vec<[f32; 3]> {
        match mesh.attribute(Mesh::ATTRIBUTE_NORMAL) {
            Some(VertexAttributeValues::Float32x3(values)) => values.clone(),
            _ => Vec::new(),
        }
    }

    /// **Regression test for the Bevy `compute_smooth_normals` zero-out
    /// bug at sim-soft scales.**
    ///
    /// Bevy 0.18's [`Mesh::compute_smooth_normals`] (corner-angle
    /// weighted) gates each triangle's contribution by an absolute
    /// `f32::EPSILON ≈ 1.2e-7` check on `(edge_a² × edge_c²)`. At
    /// sim-soft's typical cell-size scale (e.g. `3 mm` for V-5 / row
    /// 12), the squared-edge product is `~8e-11` — below the EPS, so
    /// every triangle's per-vertex weight zeros out and the resulting
    /// normals are all `(0, 0, 0)`. PBR sees `NdotL = 0` everywhere
    /// (ambient-only shading regardless of light direction).
    ///
    /// [`build_soft_mesh`] uses [`Mesh::compute_area_weighted_normals`]
    /// to side-step this — area-weighted contribution is the un-normalized
    /// cross product (magnitude `2 × triangle_area`); the accumulator
    /// stays well above any normalization threshold at our scales.
    ///
    /// This test scales [`single_tet_positions`] down to `1 cm`-edge
    /// (sim-soft's V-5 sphere radius, smaller than V-5's `3 mm` cell
    /// size — even harsher condition than row 12) and asserts every
    /// indexed-vertex normal has unit magnitude. Catches a regression
    /// that would re-introduce the smooth-normals zero-out (e.g.,
    /// swapping back to `compute_smooth_normals` for "perceived
    /// shading quality" reasons).
    #[test]
    fn build_soft_mesh_normals_unit_magnitude_at_sim_soft_scales() {
        // Tet positions scaled to ~1 cm — well below Bevy
        // compute_smooth_normals' EPS = f32::EPSILON breakpoint.
        let scale: f64 = 1.0e-2;
        let positions: Vec<sim_soft::Vec3> = single_tet_positions()
            .into_iter()
            .map(|p| sim_soft::Vec3::new(p.x * scale, p.y * scale, p.z * scale))
            .collect();
        let faces = single_tet_boundary_faces();
        let mesh = build_soft_mesh(&positions, &faces, UpAxis::PlusZ);

        let normals = read_normals(&mesh);
        let indices = read_indices(&mesh);
        assert!(!indices.is_empty());

        // Every INDEXED vertex must have a unit-length normal. Orphan
        // vertices (none in this single-tet fixture, but kept for
        // generality) are exempt.
        for idx in &indices {
            let n = normals[*idx as usize];
            let mag = (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt();
            assert!(
                (mag - 1.0).abs() < 1.0e-5,
                "indexed vertex {idx} has normal {n:?} with |n| = {mag} (expected 1.0)",
            );
        }
    }
}
