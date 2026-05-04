//! `AttributedMesh` → Bevy `Mesh` conversion + point-cloud sizing.
//!
//! Inline copy-adapted from `sim/L1/bevy/src/{mesh,convert}.rs` per
//! `docs/VIEWER_DESIGN.md` iter-1.1 N1 — cf-viewer must not depend on
//! `sim-bevy` (whose transitive deps are physics-specific).
//!
//! # Coordinate convention
//!
//! Workspace input PLYs are Z-up (mesh-v1.0 build-plate convention; sim-soft
//! sphere-sdf-eval is rotation-symmetric, so any up-axis works). Bevy is
//! internally Y-up. The swap `[x, z, y]` lives in this module so that every
//! downstream consumer (scene placement, lights, future orbit camera) can
//! treat positions as Bevy-y-up without re-deriving the conversion. The
//! `--up=<axis>` CLI flag (commit 6) parameterizes which input axis maps to
//! Bevy-y; Q6's `+Z` default is hard-coded for now.
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
use mesh_types::{AttributedMesh, Point3, Vector3};

/// Radius of point-cloud sphere markers, expressed as a fraction of the
/// AABB diagonal.
///
/// Empirically tuned against the sphere-sdf-eval fixture (1331 verts on an
/// 11³ grid, ~3.5 diagonal): 0.5% renders discrete dots without occluding
/// neighbors. Commit 4's colormap pipeline can refine once shading provides
/// a second visibility cue.
pub const POINT_RADIUS_FRACTION: f32 = 0.005;

/// Convert a physics-space `Point3<f64>` (Z-up) to a Bevy `[f32; 3]` (Y-up).
///
/// Swaps Y and Z: `(x, y, z) → (x, z, y)`. Mirrors sim-bevy's
/// `convert::vertex_positions_from_points` element-wise.
#[inline]
#[must_use]
pub fn vertex_position(p: &Point3<f64>) -> [f32; 3] {
    [p.x as f32, p.z as f32, p.y as f32]
}

/// Convert a physics-space `Vector3<f64>` (Z-up) normal to a Bevy `[f32; 3]`
/// (Y-up). Same swap as [`vertex_position`].
#[inline]
fn vertex_normal(n: &Vector3<f64>) -> [f32; 3] {
    [n.x as f32, n.z as f32, n.y as f32]
}

/// Build a Bevy `Mesh` from an `AttributedMesh` whose `geometry.faces` is
/// non-empty.
///
/// - Positions: f64 → f32 cast with Z-up → Y-up swap.
/// - Normals: stored `mesh.normals` if `Some`, else area-weighted smooth
///   normals from `IndexedMesh::compute_vertex_normals`.
/// - Indices: triangle list with `(v0, v2, v1)` winding flip — the Y/Z swap
///   reverses handedness, so emit reversed triangles to keep front-faces
///   facing the camera.
///
/// Caller responsibility: check `mesh.face_count() > 0` before calling. The
/// faces-empty path is point-cloud rendering, handled by the scene-setup
/// system in `main.rs` (one sphere entity per vertex).
#[must_use]
pub fn build_face_mesh(mesh: &AttributedMesh) -> Mesh {
    let geometry = &mesh.geometry;
    debug_assert!(
        !geometry.faces.is_empty(),
        "build_face_mesh: caller must guarantee non-empty faces",
    );

    let positions: Vec<[f32; 3]> = geometry.vertices.iter().map(vertex_position).collect();

    let normals: Vec<[f32; 3]> = match mesh.normals.as_ref() {
        Some(stored) => stored.iter().map(vertex_normal).collect(),
        None => geometry
            .compute_vertex_normals()
            .iter()
            .map(vertex_normal)
            .collect(),
    };

    // Y/Z swap reverses handedness → emit (v0, v2, v1) to restore CCW front-facing.
    let indices: Vec<u32> = geometry
        .faces
        .iter()
        .flat_map(|face| [face[0], face[2], face[1]])
        .collect();

    let mut bevy_mesh = Mesh::new(
        PrimitiveTopology::TriangleList,
        RenderAssetUsages::default(),
    );
    bevy_mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, positions);
    bevy_mesh.insert_attribute(Mesh::ATTRIBUTE_NORMAL, normals);
    bevy_mesh.insert_indices(Indices::U32(indices));
    bevy_mesh
}

#[cfg(test)]
mod tests {
    use super::*;

    use bevy::mesh::VertexAttributeValues;
    use mesh_types::IndexedMesh;

    /// Single-triangle conversion — verifies the conversion's *shape*
    /// invariants without spinning up a Bevy app. Position + normal counts
    /// match input vertex count; index count matches `face_count * 3`.
    #[test]
    fn build_face_mesh_shape_matches_input() {
        let geometry = IndexedMesh::from_parts(
            vec![
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(1.0, 0.0, 0.0),
                Point3::new(0.0, 1.0, 0.0),
            ],
            vec![[0, 1, 2]],
        );
        let mesh = AttributedMesh::new(geometry);
        let bevy_mesh = build_face_mesh(&mesh);

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

    /// Z-up → Y-up swap on a known position. Physics `(1, 2, 3)` should
    /// land in Bevy as `(1, 3, 2)` (Y/Z swapped).
    #[test]
    fn vertex_position_swaps_y_and_z() {
        let p = Point3::new(1.0, 2.0, 3.0);
        let bevy = vertex_position(&p);
        assert_eq!(bevy, [1.0_f32, 3.0, 2.0]);
    }

    /// Winding flip: input face `[0, 1, 2]` emits indices `[0, 2, 1]`. Keeps
    /// front-faces visible after the handedness reversal from the Y/Z swap.
    #[test]
    fn build_face_mesh_flips_winding() {
        let geometry = IndexedMesh::from_parts(
            vec![
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(1.0, 0.0, 0.0),
                Point3::new(0.0, 1.0, 0.0),
            ],
            vec![[0, 1, 2]],
        );
        let mesh = AttributedMesh::new(geometry);
        let bevy_mesh = build_face_mesh(&mesh);

        let indices: Vec<u32> = match bevy_mesh.indices() {
            Some(Indices::U32(idx)) => idx.clone(),
            _ => Vec::new(),
        };
        assert_eq!(indices, vec![0_u32, 2, 1]);
    }
}
