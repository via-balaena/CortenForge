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
//! # Normals (smooth-only for v1)
//!
//! `build_face_mesh` uses stored `mesh.normals` if `Some` (analytical fast
//! path — typical for SDF marching-cubes output), else falls back to
//! area-weighted smooth normals via `IndexedMesh::compute_vertex_normals`.
//! Crease-angle splitting (sim-bevy's `triangle_mesh_from_indexed`) is
//! deferred to a polish commit before the Q7 mesh-v1.0 retrofit PR — see
//! iter-2 still-open #5 in `docs/VIEWER_DESIGN.md`.

#![allow(clippy::cast_possible_truncation)] // f64 → f32 is intentional for Bevy meshes

use bevy::asset::RenderAssetUsages;
use bevy::mesh::{Indices, Mesh, PrimitiveTopology};
use cf_bevy_common::axis::UpAxis;
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
/// - Positions: f64 → f32 cast with the [`UpAxis`] swap applied via
///   [`UpAxis::to_bevy_point`].
/// - Normals: stored `mesh.normals` if `Some`, else area-weighted smooth
///   normals from `IndexedMesh::compute_vertex_normals`. Same swap as
///   positions via [`UpAxis::to_bevy_normal`].
/// - Indices: triangle list. When the swap is parity-flipping
///   ([`UpAxis::flips_winding`]), emits `(v0, v2, v1)` to restore CCW
///   front-facing; otherwise emits `(v0, v1, v2)` unchanged.
/// - Vertex colors: if `vertex_colors` is `Some`, attached as
///   `Mesh::ATTRIBUTE_COLOR` (`Float32x4`). The PBR fragment shader
///   overwrites `material.base_color` from this, so the bound
///   `StandardMaterial`'s `base_color` is moot when colors are present.
///   Length must match `geometry.vertices.len()` (debug-asserted).
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

    let positions: Vec<[f32; 3]> = geometry
        .vertices
        .iter()
        .map(|p| up.to_bevy_point(p))
        .collect();

    let normals: Vec<[f32; 3]> = match mesh.normals.as_ref() {
        Some(stored) => stored.iter().map(|n| up.to_bevy_normal(n)).collect(),
        None => geometry
            .compute_vertex_normals()
            .iter()
            .map(|n| up.to_bevy_normal(n))
            .collect(),
    };

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
        debug_assert_eq!(
            colors.len(),
            geometry.vertices.len(),
            "build_face_mesh: vertex_colors length must match vertex count",
        );
        bevy_mesh.insert_attribute(Mesh::ATTRIBUTE_COLOR, colors.to_vec());
    }
    bevy_mesh.insert_indices(Indices::U32(indices));
    bevy_mesh
}

#[cfg(test)]
mod tests {
    use super::*;

    use bevy::mesh::VertexAttributeValues;
    use mesh_types::{IndexedMesh, Point3};

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

    /// Single-triangle conversion — verifies the conversion's *shape*
    /// invariants without spinning up a Bevy app. Position + normal counts
    /// match input vertex count; index count matches `face_count * 3`.
    #[test]
    fn build_face_mesh_shape_matches_input() {
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

        assert_eq!(position_count, 3, "positions match vertex count");
        assert_eq!(normal_count, 3, "normals match vertex count");
        assert_eq!(index_count, 3, "indices = face_count * 3");
    }

    /// `vertex_colors = None` must NOT attach `ATTRIBUTE_COLOR` — the
    /// shared template `StandardMaterial`'s `base_color` only applies when
    /// the shader's `VERTEX_COLORS` def stays off, which requires the
    /// attribute to be absent from the mesh layout.
    #[test]
    fn build_face_mesh_no_colors_omits_attribute() {
        let mesh = unit_triangle();
        let bevy_mesh = build_face_mesh(&mesh, None, UpAxis::PlusZ);
        assert!(
            bevy_mesh.attribute(Mesh::ATTRIBUTE_COLOR).is_none(),
            "no vertex colors → ATTRIBUTE_COLOR must be absent",
        );
    }

    /// `vertex_colors = Some(_)` attaches `ATTRIBUTE_COLOR` as `Float32x4`
    /// with the supplied values preserved.
    #[test]
    fn build_face_mesh_attaches_vertex_colors() {
        let mesh = unit_triangle();
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

    /// Winding flip under `+Z` (today's default): input face `[0, 1, 2]`
    /// emits indices `[0, 2, 1]`. Keeps front-faces visible after the
    /// handedness reversal from the Y/Z swap.
    #[test]
    fn build_face_mesh_flips_winding_under_plus_z() {
        let mesh = unit_triangle();
        let bevy_mesh = build_face_mesh(&mesh, None, UpAxis::PlusZ);

        let indices: Vec<u32> = match bevy_mesh.indices() {
            Some(Indices::U32(idx)) => idx.clone(),
            _ => Vec::new(),
        };
        assert_eq!(indices, vec![0_u32, 2, 1]);
    }

    /// `+Y` is identity → no winding flip.
    #[test]
    fn build_face_mesh_preserves_winding_under_plus_y() {
        let mesh = unit_triangle();
        let bevy_mesh = build_face_mesh(&mesh, None, UpAxis::PlusY);

        let indices: Vec<u32> = match bevy_mesh.indices() {
            Some(Indices::U32(idx)) => idx.clone(),
            _ => Vec::new(),
        };
        assert_eq!(indices, vec![0_u32, 1, 2]);
    }

    /// `+X` is parity-flipping → winding flips.
    #[test]
    fn build_face_mesh_flips_winding_under_plus_x() {
        let mesh = unit_triangle();
        let bevy_mesh = build_face_mesh(&mesh, None, UpAxis::PlusX);

        let indices: Vec<u32> = match bevy_mesh.indices() {
            Some(Indices::U32(idx)) => idx.clone(),
            _ => Vec::new(),
        };
        assert_eq!(indices, vec![0_u32, 2, 1]);
    }
}
