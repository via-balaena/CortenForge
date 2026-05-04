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
//! internally Y-up. The swap from input frame to Bevy frame lives in this
//! module so that every downstream consumer (scene placement, lights,
//! orbit camera) can treat positions as Bevy-y-up without re-deriving
//! the conversion.
//!
//! Commit 6 parameterized the swap by [`UpAxis`]: callers thread the
//! resource into `vertex_position` / `vertex_normal` / `build_face_mesh`
//! so the mapping varies per-run via `--up=<+X|+Y|+Z>`. `+Z` reproduces
//! today's `[x, z, y]` swap exactly.
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

use crate::UpAxis;

/// Radius of point-cloud sphere markers, expressed as a fraction of the
/// AABB diagonal.
///
/// Empirically tuned against the sphere-sdf-eval fixture (1331 verts on an
/// 11³ grid, ~3.5 diagonal): 0.5% renders discrete dots without occluding
/// neighbors. Commit 4's colormap pipeline can refine once shading provides
/// a second visibility cue.
pub const POINT_RADIUS_FRACTION: f32 = 0.005;

/// Convert a physics-space `Point3<f64>` (input frame) to a Bevy `[f32; 3]`
/// (Y-up) under the supplied [`UpAxis`].
///
/// - [`UpAxis::PlusZ`] — `(x, y, z)` → `(x, z, y)` (today's default).
/// - [`UpAxis::PlusY`] — identity.
/// - [`UpAxis::PlusX`] — `(x, y, z)` → `(y, x, z)`.
#[inline]
#[must_use]
pub fn vertex_position(p: &Point3<f64>, up: UpAxis) -> [f32; 3] {
    let (x, y, z) = (p.x as f32, p.y as f32, p.z as f32);
    match up {
        UpAxis::PlusX => [y, x, z],
        UpAxis::PlusY => [x, y, z],
        UpAxis::PlusZ => [x, z, y],
    }
}

/// Convert a physics-space `Vector3<f64>` normal to a Bevy `[f32; 3]`
/// (Y-up). Same swap as [`vertex_position`].
#[inline]
fn vertex_normal(n: &Vector3<f64>, up: UpAxis) -> [f32; 3] {
    let (x, y, z) = (n.x as f32, n.y as f32, n.z as f32);
    match up {
        UpAxis::PlusX => [y, x, z],
        UpAxis::PlusY => [x, y, z],
        UpAxis::PlusZ => [x, z, y],
    }
}

/// Build a Bevy `Mesh` from an `AttributedMesh` whose `geometry.faces` is
/// non-empty.
///
/// - Positions: f64 → f32 cast with the [`UpAxis`] swap applied.
/// - Normals: stored `mesh.normals` if `Some`, else area-weighted smooth
///   normals from `IndexedMesh::compute_vertex_normals`. Same swap as
///   positions.
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
        .map(|p| vertex_position(p, up))
        .collect();

    let normals: Vec<[f32; 3]> = match mesh.normals.as_ref() {
        Some(stored) => stored.iter().map(|n| vertex_normal(n, up)).collect(),
        None => geometry
            .compute_vertex_normals()
            .iter()
            .map(|n| vertex_normal(n, up))
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
    use mesh_types::IndexedMesh;

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

    /// Z-up → Y-up swap on a known position. Physics `(1, 2, 3)` should
    /// land in Bevy as `(1, 3, 2)` (Y/Z swapped).
    #[test]
    fn vertex_position_plus_z_swaps_y_and_z() {
        let p = Point3::new(1.0, 2.0, 3.0);
        assert_eq!(vertex_position(&p, UpAxis::PlusZ), [1.0_f32, 3.0, 2.0]);
    }

    /// `+Y` is identity: the input is already Bevy-Y-up, no swap.
    #[test]
    fn vertex_position_plus_y_is_identity() {
        let p = Point3::new(1.0, 2.0, 3.0);
        assert_eq!(vertex_position(&p, UpAxis::PlusY), [1.0_f32, 2.0, 3.0]);
    }

    /// `+X` swaps X↔Y so input X (the up axis) lands in Bevy +Y.
    #[test]
    fn vertex_position_plus_x_swaps_x_and_y() {
        let p = Point3::new(1.0, 2.0, 3.0);
        assert_eq!(vertex_position(&p, UpAxis::PlusX), [2.0_f32, 1.0, 3.0]);
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
