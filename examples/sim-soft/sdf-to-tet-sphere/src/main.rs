//! sdf-to-tet-sphere — `SdfMeshedTetMesh::from_sdf` on a solid `SphereSdf`.
//!
//! Wraps the BCC + Labelle-Shewchuk Isosurface Stuffing pipeline
//! (SIGGRAPH 2007 Theorem 1 — `α_long = 0.24999`, `α_short = 0.41189`)
//! on the canonical Phase 3 sphere scene (`R = 0.1` m, `cell_size =
//! 0.02` m, `bbox = [-0.12, 0.12]³`). This is the first triangle-mesh
//! `cf-view` consumer in the sim-soft examples arc — rows 1+2 emitted
//! point clouds, this row emits a closed triangulated boundary surface
//! (the boundary faces of the tet mesh).
//!
//! Per `sdf_pipeline_determinism.rs` (III-1), every count and metric
//! the mesher produces is bit-stable across runs on a fixed
//! hardware/libm. The example pins all counts EXACTLY and asserts
//! every Theorem-1 quality bound, so any future regression surfaces at
//! the integer count layer (cheap to read, cheap to diagnose). If a
//! future CI matrix expansion (different libm) trips a count, that is
//! a real discovery — relax only after diagnosing.
//!
//! Anchor groups (all assertions exit-0 on success):
//!
//! - **Determinism** — second `from_sdf` call produces an
//!   `equals_structurally` mesh.
//! - **Counts** — `n_tets`, `n_vertices`, `referenced_vertices.len()`
//!   (orphan-rejection invariant — strictly less than `n_vertices`),
//!   boundary-face count, boundary-vertex count, boundary-edge count,
//!   all exact-pinned. Euler characteristic of the boundary equals 2
//!   (topological sphere).
//! - **Quality floors (Theorem 1)** — per-tet `signed_volume > 0`
//!   strict (D-10 detector); `aspect_ratio ≥ 0.05`; `dihedral_min ≥
//!   5°`; `dihedral_max ≤ 175°`.
//! - **Boundary geometry** — every boundary face is outward-wound
//!   (`normal · centroid > 0`, since the sphere is centred at the
//!   origin); every boundary edge appears exactly twice across
//!   boundary faces (closed-manifold invariant); per-vertex residual
//!   `|‖p‖ − R|` ≤ `cell_size` (loose Eikonal band); warp-snapped
//!   (residual ≤ `1e-12`) and cut-point (residual > `1e-12`) bucket
//!   counts are pinned exact (proves both stuffing paths fire and
//!   catches a warp-threshold drift that shifts a vertex between
//!   buckets without changing the total).
//! - **Volume convergence** — `|Σ signed_volume − (4/3) π R³| /
//!   (4/3) π R³` ≤ 0.15 (mirrors `sdf_quality_bounds.rs` III-4
//!   soft-bound).

// PLY field-data is single-precision on disk; converting f64 boundary
// residuals to f32 for `extras["boundary_residual"]` is intrinsic to
// the PLY format. Same precedent as rows 1+2. Covers the index-cast
// sites (`mesh.n_tets() as TetId`, `i as u32` for PLY index).
#![allow(clippy::cast_possible_truncation)]
// `usize as i64` casts in `verify_boundary_counts_and_topology` —
// sphere-scale boundary counts (~10³) stay astronomically below
// `i64::MAX`. Signed arithmetic is required so the χ subtraction
// can surface a negative result if V < E rather than wrapping.
#![allow(clippy::cast_possible_wrap)]
// `from_sdf(...).expect(...)` in `build_sphere_mesh`: the canonical
// Phase 3 sphere scene either succeeds by construction or surfaces a
// regression worth investigating — the `.expect()` IS the run-time
// failure signal. Same precedent as `sdf_pipeline_determinism.rs`
// line 41 inline-test convention.
#![allow(clippy::expect_used)]

use std::collections::{BTreeMap, BTreeSet};
use std::path::Path;

use anyhow::Result;
use mesh_io::save_ply_attributed;
use mesh_types::{AttributedMesh, IndexedMesh, Point3};
use sim_soft::{
    Aabb3, Mesh, MeshingHints, SdfMeshedTetMesh, SphereSdf, TetId, Vec3, VertexId,
    referenced_vertices,
};

// =============================================================================
// Constants
// =============================================================================

/// Sphere radius (m) — III-1 canonical scene parameter.
const RADIUS: f64 = 0.1;

/// BCC lattice spacing (m) — III-1 canonical scene parameter.
const CELL_SIZE: f64 = 0.02;

/// Bounding-box half-extent (m). 1.2 × R clearance band per III-1 / III-2.
const BBOX_HALF_EXTENT: f64 = 0.12;

/// Theorem-1 sanity floor for inscribed-over-circumscribed sphere
/// ratio. Mirrors `sdf_quality_bounds.rs` `RHO_MIN`.
const RHO_MIN: f64 = 0.05;

/// Theorem-1 sanity floor for minimum dihedral angle (degrees).
/// Mirrors `sdf_quality_bounds.rs` `ALPHA_LO_DEG`.
const ALPHA_LO_DEG: f64 = 5.0;

/// Theorem-1 sanity ceiling for maximum dihedral angle (degrees).
/// Mirrors `sdf_quality_bounds.rs` `ALPHA_HI_DEG`.
const ALPHA_HI_DEG: f64 = 175.0;

/// Eikonal-band bound on per-vertex boundary residual `|‖p‖ − R|`.
/// Loose bound: warp-snapped vertices land at residual ≈ 0; cut-point
/// vertices at the linear-interp secant zero are `O(cell_size²)` ≈
/// `4e-4` off the analytic surface — both regimes sit comfortably
/// below `cell_size = 0.02`.
const BOUNDARY_RESIDUAL_BOUND: f64 = CELL_SIZE;

/// Threshold separating "warp-snapped" (residual ≈ 0) from
/// "cut-point" (residual > 0) buckets in
/// `verify_residual_buckets_both_non_empty`. `1e-12` is comfortably
/// above the FP noise floor for `‖p‖ − R` arithmetic on order-`1e-1`
/// inputs (~1 ULP ≈ `1e-17`) and orders of magnitude below the
/// `O(cell_size²)` ≈ `4e-4` cut-point separation.
const WARP_SNAP_THRESHOLD: f64 = 1e-12;

/// Volume-convergence relative-error bound vs analytic `(4/3) π R³`.
/// Mirrors `sdf_quality_bounds.rs` III-4
/// `VOLUME_REL_ERROR_BOUND_AT_R_OVER_8`.
const VOLUME_REL_ERROR_BOUND: f64 = 0.15;

// -----------------------------------------------------------------------------
// Exact-pin counts — III-1 bit-stability contract (same hardware/libm).
//
// Every count below is run-to-run deterministic per
// `sdf_pipeline_determinism.rs`. Pinning exactly gives the tightest
// regression net — any future change to BCC + warp + stuffing that
// shifts a count surfaces here, not in a downstream test that has to
// re-derive a band.
// -----------------------------------------------------------------------------

/// Total emitted tet count (post-stuffing, pre-FEM).
const N_TETS_EXACT: usize = 6768;

/// Total mesh vertex count, including orphan BCC lattice corners.
/// `4634 − 1483 = 3151` orphans (≈ 68 % of the lattice — the bbox
/// clearance band sits mostly outside the sphere by design).
const N_VERTICES_EXACT: usize = 4634;

/// Vertices referenced by at least one tet — strictly less than
/// `N_VERTICES_EXACT` because BCC lattice corners outside the SDF
/// retain `positions()` slots but no tet references them.
const N_REFERENCED_VERTICES_EXACT: usize = 1483;

/// Boundary face count — closed triangulated surface enclosing the
/// solid tet body.
const N_BOUNDARY_FACES_EXACT: usize = 1224;

/// Boundary vertex count — vertices appearing in at least one
/// boundary face; subset of referenced vertices (`1483 − 614 = 869`
/// interior tet vertices).
const N_BOUNDARY_VERTICES_EXACT: usize = 614;

/// Boundary edge count — distinct sorted vertex pairs across all
/// boundary faces; each appears exactly twice, so `3·F/2 = 3·1224/2 =
/// 1836` matches by closed-manifold invariant.
const N_BOUNDARY_EDGES_EXACT: usize = 1836;

/// Warp-snapped boundary vertex count — vertices with residual ≤
/// `WARP_SNAP_THRESHOLD`. Originally BCC lattice corners pulled
/// exactly onto the analytic sphere by the warp step. Pinned exact
/// (not just `> 0`) so a warp-threshold drift surfaces here even if
/// the total boundary-vertex count stays the same.
const N_WARP_SNAPPED_EXACT: usize = 38;

/// Cut-point boundary vertex count — vertices with residual >
/// `WARP_SNAP_THRESHOLD`. Generated by linear interpolation along
/// edges where the SDF sign changes; sit at `O(cell_size²) ≈ 4e-4`
/// off the analytic surface. `N_WARP_SNAPPED_EXACT + N_CUT_POINT_EXACT
/// = 38 + 576 = 614 = N_BOUNDARY_VERTICES_EXACT` (the buckets
/// partition the boundary vertex set).
const N_CUT_POINT_EXACT: usize = 576;

// =============================================================================
// Constructor
// =============================================================================

/// Build the canonical sphere mesh. `material_field: None` selects
/// the `MaterialField::skeleton_default` fallback inside `from_sdf`;
/// this example exercises the meshing pipeline only — FEM consumption
/// is the concern of later rows.
fn build_sphere_mesh() -> SdfMeshedTetMesh {
    let half = BBOX_HALF_EXTENT;
    let hints = MeshingHints {
        bbox: Aabb3::new(Vec3::new(-half, -half, -half), Vec3::new(half, half, half)),
        cell_size: CELL_SIZE,
        material_field: None,
    };
    SdfMeshedTetMesh::from_sdf(&SphereSdf { radius: RADIUS }, &hints)
        .expect("canonical sphere scene meshes successfully — see III-1")
}

// =============================================================================
// verify_determinism — second from_sdf call equals_structurally
// =============================================================================

/// Two consecutive `from_sdf` calls produce meshes with identical
/// (`n_vertices`, `n_tets`, per-tet vertex IDs). Anchors the
/// bit-stability contract that backs every exact-pin assertion below
/// — III-1 covers this at the test layer; here we exercise it
/// user-facing as well so a regression surfaces at the example layer
/// too.
fn verify_determinism(mesh: &SdfMeshedTetMesh) {
    let other = build_sphere_mesh();
    assert!(
        mesh.equals_structurally(&other),
        "second from_sdf call drifted — III-1 determinism contract violated",
    );
}

// =============================================================================
// verify_counts — exact pins + orphan-rejection invariant
// =============================================================================

/// `n_tets`, `n_vertices`, and orphan-aware referenced-vertex count
/// are pinned exactly per III-1 contract. Strict inequality
/// `referenced.len() < n_vertices` IS the orphan-rejection invariant
/// the spec calls out — BCC lattice retains corners outside the SDF
/// in `positions()`, but they aren't referenced by any tet.
fn verify_counts(mesh: &SdfMeshedTetMesh, referenced: &[VertexId]) {
    assert_eq!(mesh.n_tets(), N_TETS_EXACT, "n_tets drift");
    assert_eq!(mesh.n_vertices(), N_VERTICES_EXACT, "n_vertices drift");
    assert_eq!(
        referenced.len(),
        N_REFERENCED_VERTICES_EXACT,
        "referenced vertex count drift",
    );
    assert!(
        referenced.len() < mesh.n_vertices(),
        "orphan-rejection invariant vacuous: referenced ({}) == n_vertices ({}) — \
         BCC lattice corners outside the SDF should be retained as orphans",
        referenced.len(),
        mesh.n_vertices(),
    );
}

// =============================================================================
// verify_signed_volume_strictly_positive — III-1 structural-bug detector
// =============================================================================

/// Every emitted sub-tet has positive signed volume. Right-handed by
/// construction (Decision H); the D-10 backstop drops sub-volume-floor
/// sub-tets, so any surviving non-positive-volume tet is a structural
/// bug surfaced here. Mirrors III-1's
/// `output_mesh_is_strictly_right_handed_post_filter`.
fn verify_signed_volume_strictly_positive(mesh: &SdfMeshedTetMesh) {
    for (tet_id, &v) in mesh.quality().signed_volume.iter().enumerate() {
        assert!(
            v > 0.0,
            "tet {tet_id}: signed_volume {v:e} ≤ 0 — D-10 filter or post-hoc \
             orientation swap regressed",
        );
    }
}

// =============================================================================
// verify_aspect_ratio_clears_floor — Theorem 1 sanity floor
// =============================================================================

/// Every tet has `aspect_ratio ≥ RHO_MIN = 0.05`. Theorem 1 with the
/// pinned "min dihedral, safe" parameters guarantees ≈ `0.08` actual,
/// so the `0.05` floor leaves ≈ `0.03` headroom. Mirrors III-2's
/// `aspect_ratio_clears_sanity_floor_for_every_tet`.
fn verify_aspect_ratio_clears_floor(mesh: &SdfMeshedTetMesh) {
    for (tet_id, &a) in mesh.quality().aspect_ratio.iter().enumerate() {
        assert!(
            a >= RHO_MIN,
            "tet {tet_id}: aspect_ratio {a:.4} < RHO_MIN {RHO_MIN}",
        );
    }
}

// =============================================================================
// verify_dihedral_within_floor — Theorem 1 sanity floor
// =============================================================================

/// Every tet's dihedral extrema sit inside `[5°, 175°]`. Theorem 1
/// guarantees `[9.32°, 161.64°]` actual, so the floor leaves
/// `4.32°` / `13.36°` headroom. Mirrors III-2's
/// `dihedral_angles_stay_within_sanity_floor_for_every_tet`.
fn verify_dihedral_within_floor(mesh: &SdfMeshedTetMesh) {
    let alpha_lo = ALPHA_LO_DEG.to_radians();
    let alpha_hi = ALPHA_HI_DEG.to_radians();
    let q = mesh.quality();
    for tet_id in 0..q.dihedral_min.len() {
        let lo = q.dihedral_min[tet_id];
        let hi = q.dihedral_max[tet_id];
        assert!(
            lo >= alpha_lo,
            "tet {tet_id}: dihedral_min {:.2}° below ALPHA_LO {ALPHA_LO_DEG}°",
            lo.to_degrees(),
        );
        assert!(
            hi <= alpha_hi,
            "tet {tet_id}: dihedral_max {:.2}° above ALPHA_HI {ALPHA_HI_DEG}°",
            hi.to_degrees(),
        );
    }
}

// =============================================================================
// Boundary face extraction
// =============================================================================

/// Sort a vertex pair into ascending order — canonical edge key for
/// matching boundary edges across faces.
const fn sorted_pair(a: VertexId, b: VertexId) -> [VertexId; 2] {
    if a <= b { [a, b] } else { [b, a] }
}

/// Extract the boundary triangles of the tet mesh.
///
/// For each tet `(v0, v1, v2, v3)` (right-handed by III-1 contract),
/// emit four outward-oriented faces:
///
/// - `(v1, v2, v3)` opposite `v0`
/// - `(v0, v3, v2)` opposite `v1`
/// - `(v0, v1, v3)` opposite `v2`
/// - `(v0, v2, v1)` opposite `v3`
///
/// (Standard right-handed-tet convention: each face's vertices wind
/// such that the cross product of consecutive edges points away from
/// the missing vertex.) Index by the sorted-triple key into a
/// `BTreeMap` carrying `(count, ordered_face)`. A face shared between
/// two tets appears twice and is interior; a face appearing once is on
/// the boundary. Returns the unique outward-oriented triangles in
/// sorted-key order (deterministic via `BTreeMap` traversal).
fn extract_boundary_faces(mesh: &SdfMeshedTetMesh) -> Vec<[VertexId; 3]> {
    let mut face_count: BTreeMap<[VertexId; 3], (usize, [VertexId; 3])> = BTreeMap::new();

    // `as TetId` cast: `n_tets` returns `usize` while the trait-side
    // walk takes `TetId = u32`. III-1 sphere meshes stay ≪ `u32::MAX`
    // (canonical N_TETS_EXACT = 6768); covered by the file-level
    // `cast_possible_truncation` allow.
    let n_tets_id = mesh.n_tets() as TetId;
    for tet_id in 0..n_tets_id {
        let [v0, v1, v2, v3] = mesh.tet_vertices(tet_id);
        let outward = [[v1, v2, v3], [v0, v3, v2], [v0, v1, v3], [v0, v2, v1]];
        for face in outward {
            let mut sorted = face;
            sorted.sort_unstable();
            face_count
                .entry(sorted)
                .and_modify(|(c, _)| *c += 1)
                .or_insert((1, face));
        }
    }

    face_count
        .into_iter()
        .filter_map(|(_, (count, face))| (count == 1).then_some(face))
        .collect()
}

/// Vertices appearing in at least one boundary face, sorted ascending.
fn boundary_vertices_of(boundary_faces: &[[VertexId; 3]]) -> Vec<VertexId> {
    let mut set: BTreeSet<VertexId> = BTreeSet::new();
    for face in boundary_faces {
        for &v in face {
            set.insert(v);
        }
    }
    set.into_iter().collect()
}

/// Per-edge incidence count across boundary faces. Each edge of a
/// closed-manifold boundary appears exactly twice.
fn boundary_edge_counts(boundary_faces: &[[VertexId; 3]]) -> BTreeMap<[VertexId; 2], usize> {
    let mut counts: BTreeMap<[VertexId; 2], usize> = BTreeMap::new();
    for face in boundary_faces {
        let edges = [
            sorted_pair(face[0], face[1]),
            sorted_pair(face[1], face[2]),
            sorted_pair(face[0], face[2]),
        ];
        for e in edges {
            *counts.entry(e).or_insert(0) += 1;
        }
    }
    counts
}

// =============================================================================
// verify_boundary_counts_and_topology
// =============================================================================

/// Boundary-face / vertex / edge counts pinned exact, plus the Euler
/// characteristic `χ = V − E + F = 2` (topological sphere). The Euler
/// check catches a topology bug that the per-edge-counted-twice check
/// might miss — e.g., disjoint genus-1 components whose Euler sum
/// lands at `2 − 2g`.
fn verify_boundary_counts_and_topology(
    boundary_faces: &[[VertexId; 3]],
    boundary_vertices: &[VertexId],
    edge_counts: &BTreeMap<[VertexId; 2], usize>,
) {
    assert_eq!(
        boundary_faces.len(),
        N_BOUNDARY_FACES_EXACT,
        "boundary face count drift",
    );
    assert_eq!(
        boundary_vertices.len(),
        N_BOUNDARY_VERTICES_EXACT,
        "boundary vertex count drift",
    );
    assert_eq!(
        edge_counts.len(),
        N_BOUNDARY_EDGES_EXACT,
        "boundary edge count drift",
    );

    // i64 casts are loss-free: sphere-scale boundary counts stay
    // below `i64::MAX`. Using signed arithmetic for the χ subtraction
    // so a V < E case surfaces as a negative χ ≠ 2 rather than
    // wrapping. `usize as i64` covered by the file-level
    // `cast_possible_wrap` allow.
    let v = boundary_vertices.len() as i64;
    let e = edge_counts.len() as i64;
    let f = boundary_faces.len() as i64;
    let chi = v - e + f;
    assert_eq!(
        chi, 2,
        "Euler characteristic V − E + F = {v} − {e} + {f} = {chi} ≠ 2 — \
         boundary is not a topological sphere",
    );
}

// =============================================================================
// verify_boundary_edges_appear_exactly_twice — closed-manifold invariant
// =============================================================================

/// Every interior edge of the boundary triangulation is shared by
/// exactly two faces. Catches a hole (1 incident face) or a
/// non-manifold flap (≥ 3).
fn verify_boundary_edges_appear_exactly_twice(edge_counts: &BTreeMap<[VertexId; 2], usize>) {
    for (&[a, b], &count) in edge_counts {
        assert_eq!(
            count, 2,
            "boundary edge ({a}, {b}) appears {count} times — closed-manifold \
             invariant requires exactly twice",
        );
    }
}

// =============================================================================
// verify_boundary_outward_winding
// =============================================================================

/// Every boundary face's normal `(p1 − p0) × (p2 − p0)` points
/// radially outward — equivalent on this scene to `normal · centroid >
/// 0` since the sphere is centred at the origin and the boundary
/// centroid lies near the analytic surface. Catches a regression that
/// flips the canonical right-handed-tet face convention used in
/// `extract_boundary_faces`.
fn verify_boundary_outward_winding(mesh: &SdfMeshedTetMesh, boundary_faces: &[[VertexId; 3]]) {
    let positions = mesh.positions();
    for (i, face) in boundary_faces.iter().enumerate() {
        let p0 = positions[face[0] as usize];
        let p1 = positions[face[1] as usize];
        let p2 = positions[face[2] as usize];
        let normal = (p1 - p0).cross(&(p2 - p0));
        let centroid = (p0 + p1 + p2) / 3.0;
        let dot = normal.dot(&centroid);
        assert!(
            dot > 0.0,
            "boundary face {i} {face:?} wound inward — normal · centroid = {dot:e} ≤ 0",
        );
    }
}

// =============================================================================
// Boundary residual computation
// =============================================================================

/// Per-boundary-vertex residual `|‖p‖ − R|`. Bimodal under BCC +
/// stuffing: warp-snapped vertices land on the analytic sphere
/// (residual ≈ 0); cut-point vertices land at the linear-interp
/// secant zero, off the analytic surface by `O(cell_size²)`.
fn boundary_residuals(mesh: &SdfMeshedTetMesh, boundary_vertices: &[VertexId]) -> Vec<f64> {
    boundary_vertices
        .iter()
        .map(|&v| (mesh.positions()[v as usize].norm() - RADIUS).abs())
        .collect()
}

// =============================================================================
// verify_boundary_residuals_within_eikonal_band
// =============================================================================

/// Every boundary vertex lies within `cell_size` of the analytic
/// sphere. Loose bound — cut-points are `O(cell_size²) ≈ 4e-4` off, two
/// orders below the band. Catches a regression that detaches a
/// boundary vertex from the SDF zero set.
fn verify_boundary_residuals_within_eikonal_band(residuals: &[f64]) {
    let max_res = residuals.iter().copied().fold(0.0_f64, f64::max);
    assert!(
        max_res <= BOUNDARY_RESIDUAL_BOUND,
        "max boundary residual {max_res:e} exceeds Eikonal-band bound \
         {BOUNDARY_RESIDUAL_BOUND}",
    );
}

// =============================================================================
// verify_residual_buckets_exact — both stuffing paths exercised, exact-pinned
// =============================================================================

/// Bimodal-distribution: warp-snapped vertices (residual ≤
/// `WARP_SNAP_THRESHOLD = 1e-12`) and cut-point vertices (residual >
/// `WARP_SNAP_THRESHOLD`) are each pinned to their exact deterministic
/// counts per III-1. Proves both stuffing paths fire on this
/// canonical scene; absent the bimodal split the example would
/// silently lose coverage of one path. The exact pin (vs. `> 0`)
/// catches a warp-threshold drift that shifts a vertex from one
/// bucket to the other without changing the total.
fn verify_residual_buckets_exact(residuals: &[f64]) -> (usize, usize) {
    let n_warp = residuals
        .iter()
        .filter(|&&r| r <= WARP_SNAP_THRESHOLD)
        .count();
    let n_cut = residuals
        .iter()
        .filter(|&&r| r > WARP_SNAP_THRESHOLD)
        .count();
    assert_eq!(
        n_warp, N_WARP_SNAPPED_EXACT,
        "warp-snapped bucket count drift — warp step regressed?",
    );
    assert_eq!(
        n_cut, N_CUT_POINT_EXACT,
        "cut-point bucket count drift — stuffing dispatch regressed?",
    );
    (n_warp, n_cut)
}

// =============================================================================
// verify_volume_within_analytic_band — III-4 soft-bound
// =============================================================================

/// `|Σ signed_volume − (4/3) π R³| / (4/3) π R³ ≤ 0.15` —
/// volume-convergence bound at canonical resolution. Mirrors III-4.
fn verify_volume_within_analytic_band(mesh: &SdfMeshedTetMesh) -> (f64, f64, f64) {
    let total: f64 = mesh.quality().signed_volume.iter().sum();
    let analytic = 4.0 / 3.0 * std::f64::consts::PI * RADIUS.powi(3);
    let rel_err = (total - analytic).abs() / analytic;
    assert!(
        rel_err <= VOLUME_REL_ERROR_BOUND,
        "volume relative error {rel_err:.4} exceeds bound {VOLUME_REL_ERROR_BOUND} \
         (total = {total:e}, analytic = {analytic:e})",
    );
    (total, analytic, rel_err)
}

// =============================================================================
// PLY emit — boundary triangle mesh + per-vertex residual scalar
// =============================================================================

/// Build an `AttributedMesh` over the boundary surface only:
/// vertices = boundary vertices (subset of `mesh.positions()` with
/// index remapping into the PLY's contiguous `0..N` index space);
/// faces = boundary faces with remapped indices; per-vertex scalar =
/// `boundary_residual` for cf-view colormap. PLY is binary LE per
/// rows 1+2 convention.
fn save_boundary_ply(
    mesh: &SdfMeshedTetMesh,
    boundary_vertices: &[VertexId],
    boundary_faces: &[[VertexId; 3]],
    residuals: &[f64],
    path: &Path,
) -> Result<()> {
    // VertexId → contiguous PLY index. Keys are sorted ascending
    // (`boundary_vertices` is a sorted Vec collected from a BTreeSet);
    // values are `0..boundary_vertices.len()`. BTreeMap keeps the
    // lookup deterministic and reads cleanly at the call site.
    // `i as u32` cast: PLY indices fit easily in u32 — boundary
    // vertex count is N_BOUNDARY_VERTICES_EXACT = 614 ≪ u32::MAX.
    // Covered by the file-level `cast_possible_truncation` allow.
    let remap: BTreeMap<VertexId, u32> = boundary_vertices
        .iter()
        .enumerate()
        .map(|(i, &v)| (v, i as u32))
        .collect();

    let positions = mesh.positions();
    let vertices: Vec<Point3<f64>> = boundary_vertices
        .iter()
        .map(|&v| {
            let p = positions[v as usize];
            Point3::new(p.x, p.y, p.z)
        })
        .collect();

    let faces: Vec<[u32; 3]> = boundary_faces
        .iter()
        .map(|f| [remap[&f[0]], remap[&f[1]], remap[&f[2]]])
        .collect();

    let geometry = IndexedMesh::from_parts(vertices, faces);
    let mut attributed = AttributedMesh::new(geometry);
    let residuals_f32: Vec<f32> = residuals.iter().map(|&r| r as f32).collect();
    attributed.insert_extra("boundary_residual", residuals_f32)?;
    save_ply_attributed(&attributed, path, true)?;
    Ok(())
}

// =============================================================================
// print_summary — museum-plaque stdout
// =============================================================================

#[allow(clippy::too_many_arguments)]
fn print_summary(
    mesh: &SdfMeshedTetMesh,
    referenced: &[VertexId],
    boundary_faces: &[[VertexId; 3]],
    boundary_vertices: &[VertexId],
    edge_counts: &BTreeMap<[VertexId; 2], usize>,
    residual_buckets: (usize, usize),
    volume_stats: (f64, f64, f64),
    path: &Path,
) {
    let (n_warp, n_cut) = residual_buckets;
    let (total_vol, analytic_vol, rel_err) = volume_stats;
    let n_orphans = mesh.n_vertices() - referenced.len();
    println!("==== sdf-to-tet-sphere ====");
    println!();
    println!("input  : SphereSdf {{ radius: {RADIUS} }}");
    println!("         MeshingHints {{");
    println!("           bbox: [-{BBOX_HALF_EXTENT}, {BBOX_HALF_EXTENT}]³,");
    println!("           cell_size: {CELL_SIZE},");
    println!("           material_field: None  // skeleton_default fallback");
    println!("         }}");
    println!("         pipeline: BCC lattice → SDF sample (sign-flipped per Decision A)");
    println!("                   → warp_lattice → stuffing dispatch (Labelle-Shewchuk Thm 1)");
    println!();
    println!("Anchor groups (all assertions exit-0 on success):");
    println!("  determinism                : second from_sdf call equals_structurally");
    println!("  counts                     : n_tets / n_vertices / referenced exact-pin +");
    println!("                               orphan-rejection invariant strict");
    println!("  signed_volume              : every tet > 0 (D-10 detector)");
    println!("  aspect_ratio               : every tet ≥ {RHO_MIN} (Theorem 1 floor)");
    println!("  dihedral                   : every tet ∈ [{ALPHA_LO_DEG}°, {ALPHA_HI_DEG}°]");
    println!("  boundary_counts_topology   : F/V/E exact-pin + Euler χ = 2");
    println!("  boundary_edges_twice       : every edge incident to exactly 2 faces");
    println!("  boundary_outward_winding   : every face's normal · centroid > 0");
    println!("  boundary_residual_band     : every vertex |‖p‖ − R| ≤ {BOUNDARY_RESIDUAL_BOUND}");
    println!("  residual_buckets_exact     : warp-snapped + cut-point exact-pin");
    println!("  volume_convergence         : rel_err ≤ {VOLUME_REL_ERROR_BOUND}");
    println!();
    println!("Mesh:");
    println!("  n_tets                     : {:>7}", mesh.n_tets());
    println!("  n_vertices                 : {:>7}", mesh.n_vertices());
    println!("  referenced (≤ n_vertices)  : {:>7}", referenced.len());
    println!("  orphans (n_vertices − ref) : {n_orphans:>7}");
    println!();
    println!("Boundary surface (closed manifold, χ = V − E + F = 2):");
    println!("  faces                      : {:>7}", boundary_faces.len());
    println!(
        "  vertices                   : {:>7}",
        boundary_vertices.len()
    );
    println!("  edges                      : {:>7}", edge_counts.len());
    println!("  warp-snapped (res ≤ 1e-12) : {n_warp:>7}");
    println!("  cut-point    (res >  1e-12): {n_cut:>7}");
    println!();
    println!("Volume:");
    println!("  total     Σ signed_volume  : {total_vol:.6e} m³");
    println!("  analytic  (4/3) π R³       : {analytic_vol:.6e} m³");
    println!("  relative  |Δ| / analytic   : {rel_err:.4}");
    println!();
    println!("PLY    : {}", path.display());
    println!(
        "         triangle mesh ({} vertices + {} faces) + 1 per-vertex scalar:",
        boundary_vertices.len(),
        boundary_faces.len(),
    );
    println!("           extras[\"boundary_residual\"] — |‖p‖ − R|, sequential");
    println!("         open in cf-view, the workspace's unified visual-review viewer:");
    println!("           cargo run -p cf-viewer --release -- <path>");
    println!("         default-picks boundary_residual (only scalar present); colormap");
    println!("         renders the bimodal split as visible structure on the surface —");
    println!("         warp-snapped vertices at original BCC lattice positions, cut-points");
    println!("         along stuffing-emitted edges at O(cell_size²) ≈ 4e-4 secant offset.");
}

// =============================================================================
// main
// =============================================================================

fn main() -> Result<()> {
    let mesh = build_sphere_mesh();

    verify_determinism(&mesh);
    verify_signed_volume_strictly_positive(&mesh);
    verify_aspect_ratio_clears_floor(&mesh);
    verify_dihedral_within_floor(&mesh);

    let referenced = referenced_vertices(&mesh);
    verify_counts(&mesh, &referenced);

    let boundary_faces = extract_boundary_faces(&mesh);
    let boundary_vertices = boundary_vertices_of(&boundary_faces);
    let edge_counts = boundary_edge_counts(&boundary_faces);

    verify_boundary_counts_and_topology(&boundary_faces, &boundary_vertices, &edge_counts);
    verify_boundary_edges_appear_exactly_twice(&edge_counts);
    verify_boundary_outward_winding(&mesh, &boundary_faces);

    let residuals = boundary_residuals(&mesh, &boundary_vertices);
    verify_boundary_residuals_within_eikonal_band(&residuals);
    let residual_buckets = verify_residual_buckets_exact(&residuals);

    let volume_stats = verify_volume_within_analytic_band(&mesh);

    let out_dir = Path::new(env!("CARGO_MANIFEST_DIR")).join("out");
    std::fs::create_dir_all(&out_dir)?;
    let out_path = out_dir.join("sphere_boundary.ply");
    save_boundary_ply(
        &mesh,
        &boundary_vertices,
        &boundary_faces,
        &residuals,
        &out_path,
    )?;

    print_summary(
        &mesh,
        &referenced,
        &boundary_faces,
        &boundary_vertices,
        &edge_counts,
        residual_buckets,
        volume_stats,
        &out_path,
    );

    Ok(())
}
