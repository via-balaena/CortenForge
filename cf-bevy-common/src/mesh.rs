//! Flat-per-triangle shading for `IndexedMesh` → Bevy `Mesh` conversion.
//!
//! [`triangle_mesh_flat_shaded`] is the shared mesh-conversion path for
//! both `cf-viewer` (PLY visual review — WYSIWYP rendering for 3D-print
//! preview) and `sim-bevy` (rigid-body scene rendering). Every triangle
//! emits three distinct output vertices, each carrying that triangle's
//! face normal. There is no smoothing or vertex sharing across faces:
//! adjacent triangles render with their own face normal, exposing the
//! mesh's actual triangulation under PBR lighting.
//!
//! # Why flat-per-triangle (not crease-angle splitting, not smooth)
//!
//! cf-viewer is a "what you see is what you print" preview tool. The
//! input mesh is the geometry that gets sliced + printed; the printer
//! materializes each triangle as a flat surface in the build. Smooth
//! shading (area-weighted vertex normals) hides the actual print
//! tessellation behind a procedurally-blended look — pretty for visual
//! review but misleading about what will physically come out of the
//! printer. Crease-angle splitting (cos(30°) Blender default,
//! cos(45°) intermediate) is a halfway compromise that smooths some
//! face groups while sharpening others; it requires threshold tuning
//! per mesh class and STILL hides print reality on smooth-curvature
//! regions of TPMS / shell / SDF outputs. Flat-per-triangle is the
//! truthful default — coarse meshes look coarse, fine meshes look
//! fine, and the viewer matches print quality 1:1.
//!
//! For meshes that DO want smooth shading (e.g., SDF marching-cubes
//! outputs that ship analytical gradient normals), the consumer can
//! bypass this helper entirely and build the Bevy mesh from the
//! stored `mesh.normals` directly — see `cf-viewer`'s `build_face_mesh`
//! which dispatches on `Option<&[Vector3<f64>]>` for that path.
//!
//! # Coordinate convention
//!
//! Positions and normals are projected through [`UpAxis::to_bevy_point`]
//! / [`UpAxis::to_bevy_normal`] at the boundary. When the chosen
//! `UpAxis` is parity-flipping ([`UpAxis::flips_winding`]), the per-
//! face vertex emission order is reversed (`(v0, v2, v1)` instead of
//! `(v0, v1, v2)`) so the post-swap winding stays CCW front-facing in
//! Bevy.
//!
//! # Vertex colors
//!
//! When `vertex_colors` is `Some`, each emitted output vertex inherits
//! the colour of its source vertex (so colormap-painted scalars
//! propagate through the per-triangle expansion bit-equally). Length
//! must match `mesh.vertices.len()`; mismatches are caught by
//! `debug_assert!`.

use bevy::asset::RenderAssetUsages;
use bevy::mesh::{Indices, Mesh, PrimitiveTopology};
use mesh_types::{IndexedMesh, Vector3};

use crate::axis::UpAxis;

/// Build a Bevy `Mesh` from an `IndexedMesh` with flat-per-triangle
/// shading + optional per-vertex colors.
///
/// - **Positions / normals**: every triangle emits three output
///   vertices, each at its source position (Bevy Y-up via
///   [`UpAxis::to_bevy_point`]) and each carrying that triangle's
///   unit face normal (via [`UpAxis::to_bevy_normal`]). No averaging
///   across faces — adjacent triangles always render as separate flat
///   surfaces, matching the input mesh's truthful triangulation.
/// - **Indices**: CCW-correct in Bevy. When [`UpAxis::flips_winding`],
///   each face emits as `(v0, v2, v1)`; otherwise `(v0, v1, v2)`.
/// - **Vertex colors**: when `vertex_colors` is `Some`, the output's
///   `Mesh::ATTRIBUTE_COLOR` carries one `[f32; 4]` per emitted
///   vertex, copied from the corresponding source vertex.
///
/// Empty meshes (zero vertices or zero faces) return an empty Bevy
/// `Mesh` with `TriangleList` topology — same shape as the non-empty
/// path, callable downstream without special-casing.
#[must_use]
pub fn triangle_mesh_flat_shaded(
    mesh: &IndexedMesh,
    vertex_colors: Option<&[[f32; 4]]>,
    up: UpAxis,
) -> Mesh {
    if let Some(colors) = vertex_colors {
        debug_assert_eq!(
            colors.len(),
            mesh.vertices.len(),
            "vertex_colors length must match mesh vertex count",
        );
    }

    let vertices = &mesh.vertices;
    let triangles = &mesh.faces;

    if vertices.is_empty() || triangles.is_empty() {
        return Mesh::new(
            PrimitiveTopology::TriangleList,
            RenderAssetUsages::default(),
        );
    }

    let flips = up.flips_winding();

    let n_emitted = triangles.len() * 3;
    let mut positions: Vec<[f32; 3]> = Vec::with_capacity(n_emitted);
    let mut normals: Vec<[f32; 3]> = Vec::with_capacity(n_emitted);
    let mut colors_out: Vec<[f32; 4]> = if vertex_colors.is_some() {
        Vec::with_capacity(n_emitted)
    } else {
        Vec::new()
    };
    let mut indices: Vec<u32> = Vec::with_capacity(n_emitted);

    for face in triangles {
        // Compute the unit face normal in input (pre-swap) space, then
        // project to Bevy. Same swap as positions keeps the lighting
        // invariant under the up-axis convention.
        let v0 = vertices[face[0] as usize];
        let v1 = vertices[face[1] as usize];
        let v2 = vertices[face[2] as usize];
        let n = (v1 - v0).cross(&(v2 - v0));
        let len = n.norm();
        let face_n_input = if len > 1e-10 {
            n / len
        } else {
            // Degenerate (zero-area) face: arbitrary axis-aligned
            // fallback. The triangle won't render visibly, but the
            // mesh stays well-formed.
            Vector3::z()
        };
        let face_n_bevy = up.to_bevy_normal(&face_n_input);

        // Emit three unique vertices, in winding-correct order.
        let order: [u32; 3] = if flips {
            [face[0], face[2], face[1]]
        } else {
            [face[0], face[1], face[2]]
        };

        for vi in order {
            let vi_usize = vi as usize;
            let out_idx = positions.len() as u32;
            positions.push(up.to_bevy_point(&vertices[vi_usize]));
            normals.push(face_n_bevy);
            if let Some(colors) = vertex_colors {
                colors_out.push(colors[vi_usize]);
            }
            indices.push(out_idx);
        }
    }

    let mut bevy_mesh = Mesh::new(
        PrimitiveTopology::TriangleList,
        RenderAssetUsages::default(),
    );
    bevy_mesh.insert_attribute(Mesh::ATTRIBUTE_POSITION, positions);
    bevy_mesh.insert_attribute(Mesh::ATTRIBUTE_NORMAL, normals);
    if vertex_colors.is_some() {
        bevy_mesh.insert_attribute(Mesh::ATTRIBUTE_COLOR, colors_out);
    }
    bevy_mesh.insert_indices(Indices::U32(indices));
    bevy_mesh
}

#[cfg(test)]
mod tests {
    use super::*;

    use bevy::mesh::VertexAttributeValues;
    use mesh_types::Point3;

    /// Unit cube centred at the origin. Six faces, each a square emitted
    /// as two triangles with consistent CCW outward winding under the
    /// input-space (right-handed Z-up) frame.
    fn unit_cube() -> IndexedMesh {
        let v = [
            Point3::new(-0.5, -0.5, -0.5),
            Point3::new(0.5, -0.5, -0.5),
            Point3::new(0.5, 0.5, -0.5),
            Point3::new(-0.5, 0.5, -0.5),
            Point3::new(-0.5, -0.5, 0.5),
            Point3::new(0.5, -0.5, 0.5),
            Point3::new(0.5, 0.5, 0.5),
            Point3::new(-0.5, 0.5, 0.5),
        ];
        let faces = vec![
            // -Z face
            [0, 2, 1],
            [0, 3, 2],
            // +Z face
            [4, 5, 6],
            [4, 6, 7],
            // -Y face
            [0, 1, 5],
            [0, 5, 4],
            // +Y face
            [3, 7, 6],
            [3, 6, 2],
            // -X face
            [0, 4, 7],
            [0, 7, 3],
            // +X face
            [1, 2, 6],
            [1, 6, 5],
        ];
        IndexedMesh::from_parts(v.to_vec(), faces)
    }

    fn flat_triangle() -> IndexedMesh {
        IndexedMesh::from_parts(
            vec![
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(1.0, 0.0, 0.0),
                Point3::new(0.0, 1.0, 0.0),
            ],
            vec![[0, 1, 2]],
        )
    }

    fn position_count(mesh: &Mesh) -> usize {
        match mesh.attribute(Mesh::ATTRIBUTE_POSITION) {
            Some(VertexAttributeValues::Float32x3(values)) => values.len(),
            _ => 0,
        }
    }

    fn index_vec(mesh: &Mesh) -> Vec<u32> {
        match mesh.indices() {
            Some(Indices::U32(idx)) => idx.clone(),
            _ => Vec::new(),
        }
    }

    /// Every triangle emits three unique vertices with that triangle's
    /// face normal — total output count is `triangles.len() × 3` for any
    /// input mesh. Cuboid: 12 triangles × 3 = 36 output vertices.
    #[test]
    fn cuboid_emits_three_vertices_per_triangle() {
        let mesh = triangle_mesh_flat_shaded(&unit_cube(), None, UpAxis::PlusZ);
        assert_eq!(
            position_count(&mesh),
            36,
            "12 triangles × 3 vertices = 36 output positions",
        );
    }

    /// Single-face mesh: 1 triangle × 3 vertices = 3 outputs.
    #[test]
    fn single_face_emits_three_vertices() {
        let mesh = triangle_mesh_flat_shaded(&flat_triangle(), None, UpAxis::PlusZ);
        assert_eq!(position_count(&mesh), 3);
    }

    /// Vertex colors propagate by source-vertex lookup: each emitted
    /// vertex carries its source vertex's colour bit-equally. For the
    /// unit cube, sum-over-source-vertices of (colors_out matching
    /// that source) must equal the source's incidence count in the
    /// triangle list. Source vertices 0 and 6 are corner-of-three-
    /// faces hubs (incident to 6 triangles each); the other six
    /// corners are incident to 4 triangles each.
    #[test]
    fn vertex_colors_propagate_to_per_triangle_emissions() {
        let mesh = unit_cube();
        let colors: Vec<[f32; 4]> = (0..mesh.vertices.len())
            .map(|i| [i as f32 / 8.0, 0.0, 0.0, 1.0])
            .collect();

        let bevy_mesh = triangle_mesh_flat_shaded(&mesh, Some(&colors), UpAxis::PlusZ);

        let out_colors: Vec<[f32; 4]> = match bevy_mesh.attribute(Mesh::ATTRIBUTE_COLOR) {
            Some(VertexAttributeValues::Float32x4(values)) => values.clone(),
            _ => Vec::new(),
        };
        assert_eq!(out_colors.len(), 36, "12 triangles × 3 = 36 colors");

        // Decode source vertex from each output color and tally.
        let mut counts = vec![0usize; mesh.vertices.len()];
        for c in &out_colors {
            let src = (c[0] * 8.0).round() as usize;
            assert!(src < mesh.vertices.len(), "color decodes to valid source");
            assert!(
                (c[0] - colors[src][0]).abs() < 1e-6,
                "color preserved bit-equally through emission",
            );
            counts[src] += 1;
        }
        // Per-vertex incidence in the unit_cube fixture: corners 0 and
        // 6 appear in 6 triangles each; the other six in 4 each.
        // 2 × 6 + 6 × 4 = 36 ✓.
        assert_eq!(counts[0], 6);
        assert_eq!(counts[6], 6);
        for &i in &[1usize, 2, 3, 4, 5, 7] {
            assert_eq!(counts[i], 4, "corner {i} should be in 4 triangles");
        }
    }

    /// `UpAxis::PlusZ` flips winding → output indices reference splits
    /// in `(face[0], face[2], face[1])` emission order. For a single
    /// face, the index buffer reads `[0, 1, 2]` (positions are
    /// permuted, indices count emission order linearly).
    #[test]
    fn flips_winding_under_plus_z() {
        let mesh = triangle_mesh_flat_shaded(&flat_triangle(), None, UpAxis::PlusZ);
        let indices = index_vec(&mesh);
        assert_eq!(indices, vec![0_u32, 1, 2]);
    }

    /// `UpAxis::PlusY` is identity → no winding flip.
    #[test]
    fn preserves_winding_under_plus_y() {
        let mesh = triangle_mesh_flat_shaded(&flat_triangle(), None, UpAxis::PlusY);
        let indices = index_vec(&mesh);
        assert_eq!(indices, vec![0_u32, 1, 2]);
    }

    /// Empty mesh — both vertices and faces empty — returns a Bevy
    /// mesh with `TriangleList` topology and zero positions / indices.
    /// No panic; downstream consumers can treat the return value
    /// uniformly.
    #[test]
    fn empty_mesh_returns_empty_bevy_mesh() {
        let mesh = IndexedMesh::from_parts(Vec::new(), Vec::new());
        let bevy_mesh = triangle_mesh_flat_shaded(&mesh, None, UpAxis::PlusZ);
        assert_eq!(position_count(&bevy_mesh), 0);
        assert_eq!(index_vec(&bevy_mesh).len(), 0);
    }

    /// `vertex_colors = None` must NOT attach `ATTRIBUTE_COLOR` —
    /// downstream PBR materials' `base_color` only applies when the
    /// shader's `VERTEX_COLORS` def is off, which requires the
    /// attribute to be absent from the mesh layout.
    #[test]
    fn no_colors_omits_attribute() {
        let mesh = triangle_mesh_flat_shaded(&flat_triangle(), None, UpAxis::PlusZ);
        assert!(
            mesh.attribute(Mesh::ATTRIBUTE_COLOR).is_none(),
            "no vertex colors → ATTRIBUTE_COLOR must be absent",
        );
    }

    /// Within a single triangle, all three emitted vertices carry the
    /// same face normal (since all three reference one face's normal,
    /// not any per-vertex average).
    #[test]
    fn single_face_all_normals_match_face_normal() {
        let mesh = triangle_mesh_flat_shaded(&flat_triangle(), None, UpAxis::PlusY);
        let normals: Vec<[f32; 3]> = match mesh.attribute(Mesh::ATTRIBUTE_NORMAL) {
            Some(VertexAttributeValues::Float32x3(values)) => values.clone(),
            _ => Vec::new(),
        };
        assert_eq!(normals.len(), 3);
        // flat_triangle lies in the z=0 plane → face normal = +z.
        // Under UpAxis::PlusY (identity), +z stays +z.
        for n in &normals {
            assert!((n[2] - 1.0).abs() < 1e-6, "normal should be +z, got {n:?}");
        }
    }
}
