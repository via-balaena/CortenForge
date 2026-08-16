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
/// `vertex_colors` must match `mesh.vertices.len()` when `Some`. The
/// `assert_eq!` below is what enforces that, and it is **load-bearing
/// rather than defensive**: measured 2026-08-16 by making the assertion
/// vacuous, `Mesh::insert_attribute` accepts a colour buffer shorter
/// than the position buffer WITHOUT panicking, and the adapter returns a
/// mesh whose attributes disagree. (This doc previously attributed the
/// panic to a `Mesh::ATTRIBUTE_COLOR` insert invariant; there is no such
/// check at insert time.) A mismatch is a construction-side bug, not a
/// runtime data dependence, so failing here beats tinting the wrong
/// vertices downstream.
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

#[cfg(test)]
mod tests {
    // `panic!` is denied at the crate level for production safety; allow it
    // inside tests so an assertion can reject a wrong attribute VARIANT rather
    // than falling back to a default the next assertion would silently compare
    // against. Same posture as the `unwrap`/`expect` allows in `sdf_layers`
    // and `clip_plane`.
    #![allow(clippy::panic)]

    use super::*;
    use bevy::mesh::VertexAttributeValues;
    use nalgebra::Point3;

    /// One triangle whose three vertices are distinguishable on every axis,
    /// so an axis swap or a dropped scale changes the numbers rather than
    /// landing back on the input.
    ///
    /// ⚠ Deliberately NOT collinear. An arithmetic-progression fixture
    /// ((1,2,3), (4,5,6), (7,8,9)) is a degenerate triangle with zero area and
    /// no defined normal — it reads like a triangle and silently breaks
    /// `compute_smooth_normals`.
    fn one_triangle() -> IndexedMesh {
        IndexedMesh {
            vertices: vec![
                Point3::new(1.0, 2.0, 3.0),
                Point3::new(4.0, 5.0, 6.0),
                Point3::new(7.0, 2.0, 9.0),
            ],
            faces: vec![[0, 1, 2]],
        }
    }

    fn positions(mesh: &Mesh) -> Vec<[f32; 3]> {
        match mesh.attribute(Mesh::ATTRIBUTE_POSITION) {
            Some(VertexAttributeValues::Float32x3(v)) => v.clone(),
            _ => Vec::new(),
        }
    }

    /// ★★ Vertices pass through the axis swap AND the render-scale lift, and
    /// the scale applies to the SWAPPED coordinates.
    ///
    /// `PlusZ` maps `(x, y, z) -> (x, z, y)`; at scale 2 the first vertex
    /// `(1, 2, 3)` must land at `(2, 6, 4)`. Asserting a swapped-and-scaled
    /// value rather than a round-trip is what separates "both transforms
    /// happened" from "neither did" — an identity axis or a unit scale would
    /// pass a laxer check.
    #[test]
    fn vertices_are_axis_swapped_then_scaled() {
        let mesh = build_bevy_mesh_from_indexed(&one_triangle(), UpAxis::PlusZ, 2.0);

        assert_eq!(
            positions(&mesh),
            vec![[2.0, 6.0, 4.0], [8.0, 12.0, 10.0], [14.0, 18.0, 4.0]],
            "PlusZ swaps y/z, then render_scale multiplies the swapped coords"
        );
    }

    /// `PlusY` is the identity swap, so only the scale acts — the control that
    /// proves the previous test's numbers come from the swap and not from the
    /// scale alone.
    #[test]
    fn plus_y_applies_the_scale_without_swapping() {
        let mesh = build_bevy_mesh_from_indexed(&one_triangle(), UpAxis::PlusY, 3.0);

        assert_eq!(positions(&mesh)[0], [3.0, 6.0, 9.0]);
    }

    /// Faces flatten into the index buffer in order; a mesh that lost its
    /// indices still renders (as nothing) rather than erroring.
    #[test]
    fn faces_flatten_into_the_index_buffer() {
        let mut m = one_triangle();
        m.faces.push([2, 1, 0]);
        let mesh = build_bevy_mesh_from_indexed(&m, UpAxis::PlusY, 1.0);

        match mesh.indices() {
            Some(bevy::mesh::Indices::U32(idx)) => assert_eq!(idx, &vec![0, 1, 2, 2, 1, 0]),
            _ => panic!("expected a U32 index buffer"),
        }
    }

    /// Normals are computed, not left to Bevy's default — an unlit surface is
    /// the symptom if this is dropped.
    #[test]
    fn smooth_normals_are_computed() {
        let mesh = build_bevy_mesh_from_indexed(&one_triangle(), UpAxis::PlusY, 1.0);

        let normals = match mesh.attribute(Mesh::ATTRIBUTE_NORMAL) {
            Some(VertexAttributeValues::Float32x3(v)) => v.clone(),
            _ => Vec::new(),
        };
        assert_eq!(normals.len(), 3, "one normal per vertex");
        for n in normals {
            let len = (n[0] * n[0] + n[1] * n[1] + n[2] * n[2]).sqrt();
            assert!((len - 1.0).abs() < 1e-5, "normals must be unit, got {len}");
        }
    }

    /// The plain adapter must not attach a COLOR attribute — a mesh carrying
    /// one binds a different shader path than the palette-tinted consumers
    /// expect.
    #[test]
    fn the_plain_adapter_attaches_no_color_attribute() {
        let mesh = build_bevy_mesh_from_indexed(&one_triangle(), UpAxis::PlusY, 1.0);

        assert!(mesh.attribute(Mesh::ATTRIBUTE_COLOR).is_none());
    }

    #[test]
    fn supplied_vertex_colors_are_attached_verbatim() {
        let colors = [
            [1.0, 0.0, 0.0, 1.0],
            [0.0, 1.0, 0.0, 1.0],
            [0.0, 0.0, 1.0, 1.0],
        ];
        let mesh = build_bevy_mesh_from_indexed_with_colors(
            &one_triangle(),
            UpAxis::PlusY,
            1.0,
            Some(&colors),
        );

        match mesh.attribute(Mesh::ATTRIBUTE_COLOR) {
            Some(VertexAttributeValues::Float32x4(v)) => assert_eq!(v, &colors.to_vec()),
            _ => panic!("expected an RGBA color attribute"),
        }
    }

    /// ★ The documented `# Panics` contract. A colour buffer shorter than the
    /// vertex list would otherwise be attached and misread downstream, tinting
    /// the wrong vertices — so it must fail loudly at construction.
    #[test]
    #[should_panic(expected = "must match mesh.vertices.len()")]
    fn a_short_color_buffer_panics_rather_than_tinting_the_wrong_vertices() {
        let short = [[1.0, 0.0, 0.0, 1.0]];
        let _ = build_bevy_mesh_from_indexed_with_colors(
            &one_triangle(),
            UpAxis::PlusY,
            1.0,
            Some(&short),
        );
    }
}
