//! `IndexedMesh` → `bevy::Mesh` adapters with optional per-vertex
//! colors.
//!
//! Lifted from `tools/cf-device-design/src/main.rs` per
//! `docs/archive/SIM_DECOUPLE_PHASE_3_RECON.md` §2.5.c (§1.7 "Duplicate") so
//! cf-device-design (cavity + validations) and cf-sim-research
//! (Phase 3 cavity + per-layer + intruder rendering) consume the
//! same adapter. Plain (no-color) consumers: cf-device-design's
//! rest-frame cavity spawner + palette-tinted per-layer surface
//! meshes, plus cf-sim-research's rest-frame cavity. Colored
//! consumers (cf-sim-research only): heat-map color path on
//! per-layer surfaces + the deformed-cavity / intruder render in
//! the sim panel.

use bevy::prelude::*;
use cf_bevy_common::axis::UpAxis;
use mesh_types::IndexedMesh;

/// Build a Bevy `Mesh` directly from an [`IndexedMesh`] — no
/// per-vertex displacement. Used by the SDF-extracted cavity + per-
/// layer surfaces ([`crate::sdf_layers::extract_layer_surface`]).
/// Maps physics-frame vertices through the cast-frame `UpAxis` swap
/// plus the `render_scale` lift to Bevy frame; computes smooth
/// per-vertex normals from face winding.
///
/// The SDF iso path extracts geometry where it naturally lives, so
/// there is no displacement step at the bevy-mesh-build boundary —
/// the adapter is a plain vertex-position + index passthrough.
pub fn build_bevy_mesh_from_indexed(mesh: &IndexedMesh, up: UpAxis, render_scale: f32) -> Mesh {
    build_bevy_mesh_from_indexed_with_colors(mesh, up, render_scale, None)
}

/// Same as [`build_bevy_mesh_from_indexed`] but with optional per-
/// vertex RGBA colors — the SDF-path heat-map analog of the
/// retired `build_displaced_proxy_mesh_with_colors`.
///
/// # Panics
///
/// `vertex_colors` must match `mesh.vertices.len()` when `Some`;
/// mismatched lengths panic via the `Mesh::ATTRIBUTE_COLOR` insert
/// invariant — a construction-side bug, not a runtime data
/// dependence.
pub fn build_bevy_mesh_from_indexed_with_colors(
    mesh: &IndexedMesh,
    up: UpAxis,
    render_scale: f32,
    vertex_colors: Option<&[[f32; 4]]>,
) -> Mesh {
    let positions: Vec<[f32; 3]> = mesh
        .vertices
        .iter()
        .map(|v| {
            let bevy = up.to_bevy_point(v);
            #[allow(clippy::cast_possible_truncation)] // f64 → f32 for Bevy.
            [
                bevy[0] * render_scale,
                bevy[1] * render_scale,
                bevy[2] * render_scale,
            ]
        })
        .collect();
    let indices: Vec<u32> = mesh.faces.iter().flatten().copied().collect();

    let mut bevy_mesh = Mesh::new(
        bevy::mesh::PrimitiveTopology::TriangleList,
        bevy::asset::RenderAssetUsages::default(),
    );
    bevy_mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, positions);
    bevy_mesh.insert_indices(bevy::mesh::Indices::U32(indices));
    bevy_mesh.compute_smooth_normals();
    if let Some(colors) = vertex_colors {
        assert_eq!(
            colors.len(),
            mesh.vertices.len(),
            "build_bevy_mesh_from_indexed_with_colors: vertex_colors.len() = {} \
             must match mesh.vertices.len() = {}",
            colors.len(),
            mesh.vertices.len(),
        );
        bevy_mesh.insert_attribute(Mesh::ATTRIBUTE_COLOR, colors.to_vec());
    }
    bevy_mesh
}
