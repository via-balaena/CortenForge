//! `AttributedMesh` → Bevy `Mesh` conversion + point-cloud sizing.
//!
//! `build_face_mesh` consumes an `AttributedMesh` (PLY-loaded via
//! [`mesh_io::load_ply_attributed`]) and emits a Bevy `Mesh` ready for
//! the rendering pipeline. The axis-aware coordinate swap (input frame →
//! Bevy Y-up) is delegated to [`UpAxis::to_bevy_point`] /
//! [`UpAxis::to_bevy_normal`] in `cf-bevy-common`; winding is flipped via
//! [`UpAxis::flips_winding`] under parity-flipping swaps.
//!
//! [`UpAxis::to_bevy_point`]: cf_bevy_common::axis::UpAxis::to_bevy_point
//! [`UpAxis::to_bevy_normal`]: cf_bevy_common::axis::UpAxis::to_bevy_normal
//! [`UpAxis::flips_winding`]: cf_bevy_common::axis::UpAxis::flips_winding
//!
//! # Normals
//!
//! `build_face_mesh` dispatches on `mesh.normals`:
//!
//! - **Stored normals path** (`mesh.normals` is `Some`): use the
//!   analytical normals as-is (typical for SDF marching-cubes output
//!   where they come from the field gradient — higher fidelity than
//!   any face-derived approximation). No crease splitting.
//! - **Compute path** (`mesh.normals` is `None`): delegate to
//!   [`cf_bevy_common::mesh::triangle_mesh_with_crease_splitting`],
//!   which uses cos(30°) crease-angle vertex splitting. Sharp creases
//!   (cuboid edges, mesh-shell faces) get per-face normals; smooth
//!   regions (curved SDF boundaries) get share-group-averaged normals.
//!   This replaces v1's `compute_vertex_normals` smooth-only fallback
//!   per Q7 retrofit PR.

#![allow(clippy::cast_possible_truncation)] // f64 → f32 is intentional for Bevy meshes

use bevy::asset::RenderAssetUsages;
use bevy::mesh::{Indices, Mesh, PrimitiveTopology};
use cf_bevy_common::axis::UpAxis;
use cf_bevy_common::mesh::triangle_mesh_with_crease_splitting;
use mesh_types::AttributedMesh;

/// Radius of point-cloud sphere markers, expressed as a fraction of the
/// AABB diagonal.
///
/// Empirically tuned against the sphere-sdf-eval fixture (1331 verts on an
/// 11³ grid, ~3.5 diagonal): 0.5% renders discrete dots without occluding
/// neighbors. Commit 4's colormap pipeline can refine once shading provides
/// a second visibility cue.
pub const POINT_RADIUS_FRACTION: f32 = 0.005;

/// Build a Bevy `Mesh` from an `AttributedMesh` whose `geometry.faces` is
/// non-empty.
///
/// Dispatches on `mesh.normals`:
///
/// - `Some(stored)` → analytical-normals fast path. Positions / normals
///   pass through the [`UpAxis`] swap; indices flip per
///   [`UpAxis::flips_winding`] to keep CCW front-facing. Optional
///   `vertex_colors` attach as `Mesh::ATTRIBUTE_COLOR` (`Float32x4`).
///   No crease splitting — stored normals come from the SDF gradient
///   (or analogous source) and have higher fidelity than any face-
///   derived approximation could give.
/// - `None` → delegate to
///   [`cf_bevy_common::mesh::triangle_mesh_with_crease_splitting`] for
///   cos(30°) crease-angle splitting + per-face / share-group normals
///   + Bevy-frame projection in one pass.
///
/// `vertex_colors.len()` must match `geometry.vertices.len()` (debug-
/// asserted in both paths).
///
/// Caller responsibility: check `mesh.face_count() > 0` before calling. The
/// faces-empty path is point-cloud rendering, handled by the scene-setup
/// system in `main.rs` (one sphere entity per vertex).
///
/// [`UpAxis::to_bevy_point`]: cf_bevy_common::axis::UpAxis::to_bevy_point
/// [`UpAxis::to_bevy_normal`]: cf_bevy_common::axis::UpAxis::to_bevy_normal
/// [`UpAxis::flips_winding`]: cf_bevy_common::axis::UpAxis::flips_winding
#[must_use]
pub fn build_face_mesh(
    mesh: &AttributedMesh,
    vertex_colors: Option<&[[f32; 4]]>,
    up: UpAxis,
) -> Mesh {
    let geometry = &mesh.geometry;
    debug_assert!(
        !geometry.faces.is_empty(),
        "build_face_mesh: caller must guarantee non-empty faces",
    );
    if let Some(colors) = vertex_colors {
        debug_assert_eq!(
            colors.len(),
            geometry.vertices.len(),
            "build_face_mesh: vertex_colors length must match vertex count",
        );
    }

    // Compute path: delegate to the shared crease-splitting helper.
    let Some(stored_normals) = mesh.normals.as_ref() else {
        return triangle_mesh_with_crease_splitting(geometry, vertex_colors, up);
    };

    // Stored-normals path: analytical normals come straight through the
    // axis swap. Crease splitting would only degrade the gradient-based
    // fidelity here, so we keep the simple per-source-vertex emission.
    let positions: Vec<[f32; 3]> = geometry
        .vertices
        .iter()
        .map(|p| up.to_bevy_point(p))
        .collect();

    let normals: Vec<[f32; 3]> = stored_normals
        .iter()
        .map(|n| up.to_bevy_normal(n))
        .collect();

    // Parity-flipping swaps reverse handedness → emit (v0, v2, v1) to keep
    // CCW front-facing. Identity swap (`+Y`) leaves winding alone.
    let indices: Vec<u32> = if up.flips_winding() {
        geometry
            .faces
            .iter()
            .flat_map(|face| [face[0], face[2], face[1]])
            .collect()
    } else {
        geometry
            .faces
            .iter()
            .flat_map(|face| [face[0], face[1], face[2]])
            .collect()
    };

    let mut bevy_mesh = Mesh::new(
        PrimitiveTopology::TriangleList,
        RenderAssetUsages::default(),
    );
    bevy_mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, positions);
    bevy_mesh.insert_attribute(Mesh::ATTRIBUTE_NORMAL, normals);
    if let Some(colors) = vertex_colors {
        bevy_mesh.insert_attribute(Mesh::ATTRIBUTE_COLOR, colors.to_vec());
    }
    bevy_mesh.insert_indices(Indices::U32(indices));
    bevy_mesh
}

#[cfg(test)]
mod tests {
    use super::*;

    use bevy::mesh::VertexAttributeValues;
    use mesh_types::{IndexedMesh, Point3, Vector3};

    fn unit_triangle() -> AttributedMesh {
        let geometry = IndexedMesh::from_parts(
            vec![
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(1.0, 0.0, 0.0),
                Point3::new(0.0, 1.0, 0.0),
            ],
            vec![[0, 1, 2]],
        );
        AttributedMesh::new(geometry)
    }

    /// Triangle with an analytical `Vector3::z()` normal stored on each
    /// vertex — exercises the stored-normals fast path.
    fn unit_triangle_with_stored_normals() -> AttributedMesh {
        let mut mesh = unit_triangle();
        mesh.normals = Some(vec![Vector3::z(); 3]);
        mesh
    }

    /// Compute path on a single coplanar face: crease splitting collapses
    /// to one split per source vertex (no creases to fire on); position +
    /// normal + index counts each match the source `vertex_count` /
    /// `face_count * 3`.
    #[test]
    fn build_face_mesh_compute_path_shape_matches_input() {
        let mesh = unit_triangle();
        let bevy_mesh = build_face_mesh(&mesh, None, UpAxis::PlusZ);

        let position_count = match bevy_mesh.attribute(Mesh::ATTRIBUTE_POSITION) {
            Some(VertexAttributeValues::Float32x3(values)) => values.len(),
            _ => 0,
        };
        let normal_count = match bevy_mesh.attribute(Mesh::ATTRIBUTE_NORMAL) {
            Some(VertexAttributeValues::Float32x3(values)) => values.len(),
            _ => 0,
        };
        let index_count = match bevy_mesh.indices() {
            Some(Indices::U32(idx)) => idx.len(),
            _ => 0,
        };

        assert_eq!(position_count, 3, "coplanar face: 3 splits = 3 positions");
        assert_eq!(normal_count, 3);
        assert_eq!(index_count, 3, "indices = face_count * 3");
    }

    /// Stored-normals path: `mesh.normals = Some(_)` skips crease
    /// splitting and emits one position / normal per source vertex.
    #[test]
    fn build_face_mesh_stored_normals_path_shape_matches_input() {
        let mesh = unit_triangle_with_stored_normals();
        let bevy_mesh = build_face_mesh(&mesh, None, UpAxis::PlusZ);

        let position_count = match bevy_mesh.attribute(Mesh::ATTRIBUTE_POSITION) {
            Some(VertexAttributeValues::Float32x3(values)) => values.len(),
            _ => 0,
        };
        let index_count = match bevy_mesh.indices() {
            Some(Indices::U32(idx)) => idx.len(),
            _ => 0,
        };
        assert_eq!(position_count, 3);
        assert_eq!(index_count, 3);
    }

    /// `vertex_colors = None` must NOT attach `ATTRIBUTE_COLOR` — the
    /// shared template `StandardMaterial`'s `base_color` only applies when
    /// the shader's `VERTEX_COLORS` def stays off, which requires the
    /// attribute to be absent from the mesh layout. Tested on both
    /// dispatch branches.
    #[test]
    fn build_face_mesh_no_colors_omits_attribute() {
        for mesh in [unit_triangle(), unit_triangle_with_stored_normals()] {
            let bevy_mesh = build_face_mesh(&mesh, None, UpAxis::PlusZ);
            assert!(
                bevy_mesh.attribute(Mesh::ATTRIBUTE_COLOR).is_none(),
                "no vertex colors → ATTRIBUTE_COLOR must be absent",
            );
        }
    }

    /// Stored-normals path: `vertex_colors = Some(_)` attaches
    /// `ATTRIBUTE_COLOR` as `Float32x4` with the supplied values
    /// preserved bit-for-bit (no splits → no permutation).
    #[test]
    fn build_face_mesh_stored_normals_attaches_vertex_colors() {
        let mesh = unit_triangle_with_stored_normals();
        let colors = [
            [1.0_f32, 0.0, 0.0, 1.0],
            [0.0, 1.0, 0.0, 1.0],
            [0.0, 0.0, 1.0, 1.0],
        ];
        let bevy_mesh = build_face_mesh(&mesh, Some(&colors), UpAxis::PlusZ);

        let stored: Vec<[f32; 4]> = match bevy_mesh.attribute(Mesh::ATTRIBUTE_COLOR) {
            Some(VertexAttributeValues::Float32x4(values)) => values.clone(),
            _ => Vec::new(),
        };
        assert_eq!(stored, colors.to_vec());
    }

    /// Stored-normals path winding: input face `[0, 1, 2]` under `+Z`
    /// emits indices `[0, 2, 1]` to keep front-faces visible after the
    /// Y/Z swap's handedness reversal. (Compute path uses output-index
    /// semantics — `[0, 1, 2]` always — and is covered by
    /// `cf_bevy_common::mesh::tests::flips_winding_under_plus_z`.)
    #[test]
    fn build_face_mesh_stored_normals_flips_winding_under_plus_z() {
        let mesh = unit_triangle_with_stored_normals();
        let bevy_mesh = build_face_mesh(&mesh, None, UpAxis::PlusZ);
        let indices: Vec<u32> = match bevy_mesh.indices() {
            Some(Indices::U32(idx)) => idx.clone(),
            _ => Vec::new(),
        };
        assert_eq!(indices, vec![0_u32, 2, 1]);
    }

    /// Stored-normals path: `+Y` is identity → no winding flip.
    #[test]
    fn build_face_mesh_stored_normals_preserves_winding_under_plus_y() {
        let mesh = unit_triangle_with_stored_normals();
        let bevy_mesh = build_face_mesh(&mesh, None, UpAxis::PlusY);
        let indices: Vec<u32> = match bevy_mesh.indices() {
            Some(Indices::U32(idx)) => idx.clone(),
            _ => Vec::new(),
        };
        assert_eq!(indices, vec![0_u32, 1, 2]);
    }

    /// Stored-normals path: `+X` is parity-flipping → winding flips.
    #[test]
    fn build_face_mesh_stored_normals_flips_winding_under_plus_x() {
        let mesh = unit_triangle_with_stored_normals();
        let bevy_mesh = build_face_mesh(&mesh, None, UpAxis::PlusX);
        let indices: Vec<u32> = match bevy_mesh.indices() {
            Some(Indices::U32(idx)) => idx.clone(),
            _ => Vec::new(),
        };
        assert_eq!(indices, vec![0_u32, 2, 1]);
    }
}
