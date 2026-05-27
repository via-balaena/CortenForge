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
//! `SharedScanSdf::mesh()` (cf-cast-cli) in **meters** (cf-design /
//! cf-scan-prep frame). `apply_mating_transforms` consumes meshes in
//! **millimeters** (the convention `solid_to_mm_mesh` emits — see
//! `mesher.rs:METERS_TO_MM`). [`build_plug_body_mesh`] performs the
//! ×1000 scale at this paradigm boundary exactly once so the resulting
//! mesh composes directly with the post-MC mating-feature transforms
//! from `add_plug_pins`.
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
//! [scan-mesh-field]: `crate::CastSpec::scan_mesh_for_plug_layer_0`
//!
//! [`CastSpec::export_molds_v2`]: crate::CastSpec::export_molds_v2

use mesh_repair::{
    RepairError, RepairSummary, remove_duplicate_faces, remove_unreferenced_vertices, weld_vertices,
};
use mesh_types::IndexedMesh;

use crate::mesher::METERS_TO_MM;

/// Vertex-weld epsilon (mm) applied by [`repair_scan_mesh_for_mesh_csg`]
/// to merge the per-triangle vertex slots that `mesh_io::load_stl`
/// produces (STL is a vertex-soup format: 3 unique vertex slots per
/// triangle). 0.01 mm = 10 µm — well below the workshop iter-1 scan's
/// ~0.66 mm avg edge length so distinct features are preserved, well
/// above the f32 → STL roundtrip precision (~10 nm at 150 mm scale)
/// so coincident vertices reliably merge.
const SCAN_MESH_WELD_EPSILON_MM: f64 = 0.01;

/// Build the plug-body mesh for the zero-offset plug case directly
/// from the cf-scan-prep cleaned scan mesh.
///
/// The input mesh is expected in **meters** (cf-design / cf-scan-prep
/// frame — same allocation `SharedScanSdf::mesh()` returns). The
/// returned mesh is in **millimeters** so it flows through
/// [`crate::apply_mating_transforms`] alongside meshes emitted by
/// `crate::mesher::solid_to_mm_mesh`.
///
/// Faces are copied unchanged (topology preserved); vertices are
/// scaled by `METERS_TO_MM`. Winding direction is inherited from the
/// input scan. **Manifoldness is NOT guaranteed by this helper** —
/// cf-scan-prep's cleaned scan mesh is watertight enough for STL
/// distribution but does not satisfy manifold3d's stricter "every edge
/// shared by exactly two faces" precondition (workshop regen
/// 2026-05-26 confirmed `manifold3d status: NotManifold` on the
/// scan's raw `mesh_to_manifold` conversion). Callers that hand the
/// returned mesh to [`crate::apply_mating_transforms`] (which calls
/// manifold3d) MUST run [`repair_scan_mesh_for_mesh_csg`] first.
///
/// **Cap-plane geometry note**: the cleaned scan mesh is a **CLOSED,
/// watertight** mesh — cf-scan-prep's `auto_cap_open_boundaries` adds
/// a cap-fan polygon (projected onto the fit plane) to close the
/// open bottom, then `taubin_smooth_vertices` smooths the whole mesh
/// (which can drift cap-fan vertices up to ~6 mm off the cap plane;
/// see `cf-cap-planes::dome_wall_only_mesh` docstring for the
/// rationale). The flatness of the resulting plug bottom is therefore
/// owned upstream by cf-scan-prep's cap-fan placement, not by any
/// truncation here.
///
/// The post-MC [`crate::build_plug_cap_trim_transform`] `SeamTrim`
/// applies to scan-mesh-direct input the same way it applies to the
/// SDF/MC output: it removes everything on the `+cap_normal` side of
/// the parsed `.prep.toml` cap-plane (i.e., anything below cap-plane
/// in pour orientation). For a properly-capped scan this is a near-
/// no-op (the cap-fan vertices already sit near the cap-plane). Any
/// pre-existing material below cap-plane is trimmed at scan resolution
/// rather than MC resolution.
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

/// Summary of [`repair_scan_mesh_for_mesh_csg`]'s work.
///
/// Stripped down 2026-05-26 (S1.1 post-baby_shark + post-centroid-fan
/// fix) — see [`repair_scan_mesh_for_mesh_csg`] docstring for the
/// before/after. Holes-filled + degens-removed are gone since the
/// minimal repair no longer runs aspect-ratio degen-removal or
/// `fill_holes`.
#[derive(Debug, Clone)]
pub struct ScanMeshRepairSummary {
    /// Stats from the inline weld-then-strip-unreferenced-then-
    /// dedupe pass. `vertices_welded` is the load-bearing count
    /// (turns the STL vertex-soup back into shared-index topology
    /// that manifold3d will accept).
    pub base: RepairSummary,
    /// Held at 0; field retained for backward-compat with the
    /// production-regen log line. Future S2-style expansions might
    /// re-introduce a hole-fill stage if cf-scan-prep ever ships
    /// non-watertight meshes again.
    pub holes_filled: usize,
}

/// Re-weld the cf-scan-prep cleaned-scan STL back to shared-index
/// topology so it satisfies manifold3d's preconditions.
///
/// **Required upstream of [`crate::apply_mating_transforms`] on the
/// scan-mesh-direct path** — `mesh_io::load_stl` produces a vertex
/// soup (3 unique vertex slots per triangle), where manifold3d sees
/// every edge as a boundary edge. The weld merges coincident vertex
/// slots back into shared indices, restoring the topology
/// cf-scan-prep emitted before STL serialization.
///
/// **History — why this is a single 3-step pass instead of a full
/// `repair_mesh` + `fill_holes` pipeline**: pre-baby_shark (S1.1
/// 2026-05-26), cf-scan-prep's meshopt-based decimation produced
/// non-manifold edges + duplicate faces in the dome wall, and its
/// ear-clip-based cap-fan produced similar artifacts at the cap-plane.
/// cf-cast worked around both bugs with `RepairParams::for_scans` +
/// `fill_holes`. After (a) cf-scan-prep switched to `baby_shark` and
/// (b) `auto_cap_open_boundaries` + `build_cleaned_mesh` switched to
/// a centroid-fan cap, cf-scan-prep emits a manifold mesh. cf-cast's
/// own aspect-ratio sliver removal + ear-clip `fill_holes` then
/// INTRODUCED the very artifacts we were defending against
/// (`mesh-repair::fill_holes` uses the same ear-clip pattern
/// internally). Slimming the repair to weld-only avoids re-introducing
/// the bug while still fixing the genuine STL-soup issue.
///
/// Three steps:
/// 1. [`weld_vertices`] at `SCAN_MESH_WELD_EPSILON_MM` = 0.01 mm —
///    merges per-triangle vertex slots back to shared indices.
/// 2. [`remove_unreferenced_vertices`] — compacts the vertex array
///    after weld.
/// 3. [`remove_duplicate_faces`] — defensive cleanup for the rare
///    case where cf-scan-prep emits exactly-coincident triangles
///    (e.g., a tiny capped scanner artifact that the centroid-fan
///    fans over twice).
///
/// The mesh is expected in **millimeters** (i.e., post-
/// [`build_plug_body_mesh`] scaling).
///
/// Returns the [`ScanMeshRepairSummary`] for production-regen
/// diagnostics. Idempotent — a second pass over a clean mesh is a
/// no-op.
///
/// # Errors
///
/// Currently infallible — the `Result` return type is preserved for
/// API stability with the pre-2026-05-26 `fill_holes`-bearing
/// signature so the spec.rs call site doesn't churn.
pub fn repair_scan_mesh_for_mesh_csg(
    mesh: &mut IndexedMesh,
) -> Result<ScanMeshRepairSummary, RepairError> {
    let initial_vertices = mesh.vertices.len();
    let initial_faces = mesh.faces.len();
    let vertices_welded = weld_vertices(mesh, SCAN_MESH_WELD_EPSILON_MM);
    let unreferenced_removed = remove_unreferenced_vertices(mesh);
    let duplicates_removed = remove_duplicate_faces(mesh);
    let base = RepairSummary {
        initial_vertices,
        initial_faces,
        final_vertices: mesh.vertices.len(),
        final_faces: mesh.faces.len(),
        vertices_welded,
        degenerates_removed: 0,
        duplicates_removed,
        unreferenced_removed,
    };
    Ok(ScanMeshRepairSummary {
        base,
        holes_filled: 0,
    })
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

    #[test]
    fn repair_scan_mesh_for_mesh_csg_welds_sub_epsilon_duplicate_vertices() {
        // Two coincident triangles sharing a hypothetical edge that
        // sits 1 µm apart via two duplicate vertex slots — exactly the
        // shape of failure manifold3d's `NotManifold` flags (the edge
        // is technically a boundary edge because each "side" sees a
        // different vertex index). The 10 µm weld closes the gap.
        let mut mesh = IndexedMesh::new();
        mesh.vertices.push(Point3::new(0.0, 0.0, 0.0));
        mesh.vertices.push(Point3::new(10.0, 0.0, 0.0));
        mesh.vertices.push(Point3::new(0.0, 10.0, 0.0));
        mesh.vertices.push(Point3::new(10.0, 10.0, 0.0));
        // Duplicate of vertex 1, 1 µm offset (below 10 µm weld_epsilon).
        mesh.vertices.push(Point3::new(10.0 + 1e-3, 0.0, 0.0));
        // Duplicate of vertex 2.
        mesh.vertices.push(Point3::new(0.0, 10.0 + 1e-3, 0.0));
        mesh.faces.push([0, 1, 2]);
        mesh.faces.push([4, 3, 5]);

        let summary = repair_scan_mesh_for_mesh_csg(&mut mesh).unwrap();
        assert!(
            summary.base.vertices_welded >= 2,
            "expected at least 2 welds, got {}",
            summary.base.vertices_welded
        );
        assert_eq!(mesh.vertices.len(), 4);
        // Face count NOT pinned here — `remove_duplicate_faces` may
        // collapse the two near-coincident triangles after weld; the
        // load-bearing property is the weld count.
    }

    #[test]
    fn repair_scan_mesh_for_mesh_csg_is_idempotent_on_clean_mesh() {
        // A second pass over an already-clean mesh should be a no-op
        // (no welds, no duplicate-face removals). Pins the idempotency
        // property documented on the helper.
        let mut mesh = build_plug_body_mesh(&synthetic_scan_mesh());

        let _first = repair_scan_mesh_for_mesh_csg(&mut mesh).unwrap();
        let verts_after_first = mesh.vertices.len();
        let faces_after_first = mesh.faces.len();
        let second = repair_scan_mesh_for_mesh_csg(&mut mesh).unwrap();

        assert_eq!(second.base.vertices_welded, 0);
        assert_eq!(second.base.duplicates_removed, 0);
        assert_eq!(second.holes_filled, 0);
        assert_eq!(mesh.vertices.len(), verts_after_first);
        assert_eq!(mesh.faces.len(), faces_after_first);
    }

    #[test]
    fn repair_scan_mesh_for_mesh_csg_preserves_open_boundary() {
        // Cube missing its top face: 5 faces × 4 unique verts. Post-
        // S1.1-cleanup the repair NO LONGER calls fill_holes (per
        // function docstring: that was the source of the bugs we
        // moved upstream). The top face must STAY open — caller
        // (cf-scan-prep) is responsible for emitting watertight
        // meshes; cf-cast just re-welds the STL soup.
        let mut mesh = IndexedMesh::new();
        let p = |x: f64, y: f64, z: f64| Point3::new(x, y, z);
        let v = [
            p(0.0, 0.0, 0.0),
            p(10.0, 0.0, 0.0),
            p(10.0, 10.0, 0.0),
            p(0.0, 10.0, 0.0),
            p(0.0, 0.0, 10.0),
            p(10.0, 0.0, 10.0),
            p(10.0, 10.0, 10.0),
            p(0.0, 10.0, 10.0),
        ];
        mesh.vertices.extend_from_slice(&v);
        // Bottom + 4 side walls; top deliberately omitted.
        mesh.faces.push([0, 2, 1]);
        mesh.faces.push([0, 3, 2]);
        mesh.faces.push([0, 1, 5]);
        mesh.faces.push([0, 5, 4]);
        mesh.faces.push([1, 2, 6]);
        mesh.faces.push([1, 6, 5]);
        mesh.faces.push([2, 3, 7]);
        mesh.faces.push([2, 7, 6]);
        mesh.faces.push([3, 0, 4]);
        mesh.faces.push([3, 4, 7]);
        let faces_before = mesh.faces.len();
        let summary = repair_scan_mesh_for_mesh_csg(&mut mesh).unwrap();
        assert_eq!(summary.holes_filled, 0, "S1.1-cleanup removes fill_holes");
        assert_eq!(
            mesh.faces.len(),
            faces_before,
            "repair must not add or drop faces on a clean input"
        );
    }
}
