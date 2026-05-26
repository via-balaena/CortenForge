//! Scan-mesh-direct routing for scan-derived plug bodies.
//!
//! S1 of `docs/CF_CAST_SCAN_MESH_DIRECT_RECON.md` — replaces the
//! `pinned_floor_shell → SDF → marching cubes` re-meshing pipeline
//! with a direct copy of the cf-scan-prep cleaned scan mesh for the
//! zero-offset plug body case (`plug_layer_0` with
//! `cavity_inset_m == 0`).
//!
//! ## Why scan-mesh-direct
//!
//! Per S0a + S0b empirical spikes (`tests/s0_scan_mesh_direct_probe.rs`,
//! dev `46354c79`): the cf-scan-prep cleaned scan mesh already
//! matches the workshop-desired smoothness for the plug body (~0.66 mm
//! avg edge length; workshop cf-view + Orca smoke confirms). The
//! existing SDF→MC pipeline at 2 mm cell size coarsens to ~15k faces
//! and produces workshop-visible radial striping + cap-plane perimeter
//! stair-step — both quantization artifacts from MC sampling the
//! pinned-floor-shell SDF at coarse grid, not from the input scan.
//!
//! ## Unit boundary
//!
//! The scan [`IndexedMesh`] arrives from
//! [`SharedScanSdf::mesh()`][shared-scan-sdf-mesh] in **meters** (cf-design /
//! cf-scan-prep frame). `apply_mating_transforms` consumes meshes in
//! **millimeters** (the convention [`solid_to_mm_mesh`] emits — see
//! `mesher.rs:METERS_TO_MM`). [`build_plug_body_mesh`] performs the
//! ×1000 scale at this paradigm boundary exactly once so the resulting
//! mesh composes directly with the post-MC mating-feature transforms
//! from [`add_plug_pins`].
//!
//! ## Scope (S1)
//!
//! S1 ships [`build_plug_body_mesh`] + a feature flag on
//! [`CastSpec::scan_mesh_for_plug_layer_0`][scan-mesh-field]. The
//! consumer wiring in [`CastSpec::export_molds_v2`] routes `plug_layer_0`
//! through this path when the flag is `Some`; layers 1+ continue
//! through the SDF/MC pipeline. The §SMD-4 S2/S3 offset cases
//! (per-layer body + cup-wall outer) are out of scope here.
//!
//! [shared-scan-sdf-mesh]: `scan::SharedScanSdf::mesh` (cf-cast-cli)
//! [scan-mesh-field]: `crate::CastSpec::scan_mesh_for_plug_layer_0`
//!
//! [`CastSpec::export_molds_v2`]: crate::CastSpec::export_molds_v2

use mesh_types::IndexedMesh;

use crate::mesher::METERS_TO_MM;

/// Build the plug-body mesh for the zero-offset plug case directly
/// from the cf-scan-prep cleaned scan mesh.
///
/// The input mesh is expected in **meters** (cf-design / cf-scan-prep
/// frame — same allocation [`SharedScanSdf::mesh()`] returns). The
/// returned mesh is in **millimeters** so it flows through
/// [`crate::apply_mating_transforms`] alongside meshes emitted by
/// [`crate::mesher::solid_to_mm_mesh`].
///
/// Faces are copied unchanged (topology preserved); vertices are
/// scaled by [`METERS_TO_MM`]. Self-intersections, manifoldness, and
/// winding direction are inherited from the input scan — cf-scan-prep
/// is responsible for those upstream.
///
/// The cap-plane truncation that [`crate::build_plug_cap_trim_transform`]
/// re-asserts post-MC also flattens any cap-plane material below the
/// trim plane on a scan-mesh-direct input (the scan mesh extends only
/// to the cap-plane via cf-scan-prep's `trim_floor` workflow; if it
/// over-extends, the `SeamTrim` removes the excess at scan resolution
/// rather than MC resolution).
#[must_use]
pub fn build_plug_body_mesh(scan_mesh: &IndexedMesh) -> IndexedMesh {
    let mut out = IndexedMesh::new();
    out.vertices = scan_mesh
        .vertices
        .iter()
        .map(|v| {
            mesh_types::Point3::new(v.x * METERS_TO_MM, v.y * METERS_TO_MM, v.z * METERS_TO_MM)
        })
        .collect();
    out.faces.clone_from(&scan_mesh.faces);
    out
}

#[cfg(test)]
#[allow(clippy::unwrap_used, clippy::float_cmp)]
mod tests {
    use super::*;
    use approx::assert_relative_eq;
    use mesh_types::Point3;

    fn synthetic_scan_mesh() -> IndexedMesh {
        // Small tetrahedron in meters with non-zero offset to verify
        // the scale isn't accidentally only applied near origin. Face
        // count + vertex count are pinned so the regression catches
        // any accidental topology mutation.
        let mut mesh = IndexedMesh::new();
        mesh.vertices.push(Point3::new(0.001, 0.002, 0.003));
        mesh.vertices.push(Point3::new(0.011, 0.002, 0.003));
        mesh.vertices.push(Point3::new(0.001, 0.012, 0.003));
        mesh.vertices.push(Point3::new(0.001, 0.002, 0.013));
        mesh.faces.push([0, 1, 2]);
        mesh.faces.push([0, 2, 3]);
        mesh.faces.push([0, 3, 1]);
        mesh.faces.push([1, 3, 2]);
        mesh
    }

    #[test]
    fn build_plug_body_mesh_scales_vertices_meters_to_millimeters() {
        let scan = synthetic_scan_mesh();
        let out = build_plug_body_mesh(&scan);

        assert_eq!(out.vertices.len(), scan.vertices.len());
        for (in_v, out_v) in scan.vertices.iter().zip(out.vertices.iter()) {
            assert_relative_eq!(out_v.x, in_v.x * 1000.0);
            assert_relative_eq!(out_v.y, in_v.y * 1000.0);
            assert_relative_eq!(out_v.z, in_v.z * 1000.0);
        }
    }

    #[test]
    fn build_plug_body_mesh_preserves_face_count_and_indices() {
        // Scan resolution preservation is the load-bearing property
        // of the pivot — any future refactor that decimates here
        // would silently void the §SMD-1-a win.
        let scan = synthetic_scan_mesh();
        let out = build_plug_body_mesh(&scan);

        assert_eq!(out.faces.len(), scan.faces.len());
        for (a, b) in scan.faces.iter().zip(out.faces.iter()) {
            assert_eq!(a, b);
        }
    }

    #[test]
    fn build_plug_body_mesh_does_not_mutate_input() {
        // The helper takes a `&IndexedMesh` so this is enforced by
        // the borrow checker; pinning a behavioural test here keeps
        // a future API drift (e.g. accidental `&mut` taken via
        // shadowing) from going unnoticed.
        let scan = synthetic_scan_mesh();
        let scan_clone = scan.clone();
        let out = build_plug_body_mesh(&scan);
        assert!(!out.vertices.is_empty());
        assert_eq!(scan.vertices.len(), scan_clone.vertices.len());
        for (a, b) in scan.vertices.iter().zip(scan_clone.vertices.iter()) {
            assert_relative_eq!(a.x, b.x);
            assert_relative_eq!(a.y, b.y);
            assert_relative_eq!(a.z, b.z);
        }
    }

    #[test]
    fn build_plug_body_mesh_handles_empty_mesh() {
        let empty = IndexedMesh::new();
        let out = build_plug_body_mesh(&empty);
        assert!(out.vertices.is_empty());
        assert!(out.faces.is_empty());
    }
}
