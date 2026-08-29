//! Decimation, welding and the on-disk cleanup pass.
//!
//! Moved verbatim from the flat `lib.rs` (pure module split).

use baby_shark::{
    decimation::{AlwaysDecimate, EdgeDecimator},
    mesh::{corner_table::CornerTableF, traits::TriangleMesh},
};
use mesh_repair::{
    TAUBIN_DEFAULT_LAMBDA, TAUBIN_DEFAULT_MU, remove_degenerate_triangles, remove_small_components,
    remove_unreferenced_vertices, taubin_smooth_vertices, weld_vertices,
};
use mesh_types::{IndexedMesh, Point3};
use nalgebra::Vector3;
use std::time::Instant;

/// Heuristic: does `mesh` look like raw, unwelded STL soup?
///
/// `mesh_io::load_stl` emits 3 unshared vertices per triangle, so a
/// freshly-loaded scan has `vertex_count == 3 × face_count`. A welded
/// mesh has `vertex_count ≈ 0.5 × face_count` (Euler: V ≈ F/2). The
/// `≥ 2 ×` threshold cleanly separates the two and stays robust to
/// partial welds. Used to surface the "Weld first" warning + to make
/// the Cap → Scan message actionable when it detects per-triangle loops.
pub fn mesh_looks_unwelded(vertex_count: usize, face_count: usize) -> bool {
    face_count > 0 && vertex_count >= face_count * 2
}

/// Default target face count for the Simplify panel slider per spec
/// §Architectural decisions §Simplify algorithm: 10× over the 50k
/// surface-continuity floor, well below the 500k "may be sluggish"
/// threshold, matches cf-cast's 2 mm-cell SDF sampling density
/// × surface-continuity headroom.
pub const SIMPLIFY_TARGET_DEFAULT: usize = 200_000;

/// Lower slider bound. Below this the simplified mesh loses too much
/// surface continuity for cast purposes.
pub const SIMPLIFY_TARGET_MIN: usize = 1_000;

/// Upper slider bound. Above this we're not really simplifying typical
/// scans anymore.
pub const SIMPLIFY_TARGET_MAX: usize = 1_000_000;

/// Spatial-hash welding tolerance applied pre-`meshopt::simplify` so
/// the STL's 3N unshared vertices collapse to ~N shared. 1 µm in
/// meters; tighter than any practical scan precision.
pub const SIMPLIFY_WELD_EPSILON_M: f64 = 1e-6;

/// Area threshold for `remove_degenerate_triangles` in the
/// `build_cleaned_mesh` cleanup pass (CSP.3). Square meters; 1e-15 m²
/// is well below cf-cast's 2 mm-cell SDF resolution (4e-6 m² per cell
/// face) and below the f32 quantization floor meshopt operates at,
/// so anything we strip here is FP noise the downstream couldn't have
/// used anyway. Catches zero-area triangles from cap ear-clip
/// degeneracies, clip-intersection sliver triangles, and any
/// preexisting degenerate faces the raw STL carried in. The latter
/// is what made cf-device-design's `simplify_decoder` retain its
/// full 3.34M face count on the iter-1 fixture
/// (`tools/cf-device-design/src/main.rs:419-425`).
pub const CLEANUP_DEGENERATE_AREA_M2: f64 = 1e-15;

/// Minimum face count for a connected component to survive the
/// `build_cleaned_mesh` cleanup pass. Components smaller than this
/// are dropped as scanner noise. 10 is conservative — a meaningful
/// shell (even a small cyst-like protrusion) has dozens of faces;
/// noise islands are typically 1-3 stray triangles from scanner
/// registration glitches or self-intersection artifacts. Spec
/// §Strategic context assumes the user has pre-trimmed to a single
/// shell externally; this threshold is the safety net for the
/// "user forgot" case.
pub const CLEANUP_MIN_COMPONENT_FACES: usize = 10;

/// Mesh hygiene cleanup pass applied at save time (CSP.3b).
///
/// Before this function existed the cleaned STL inherited the raw
/// scan's vertex layout — 3N unshared verts from `mesh_io::load_stl`,
/// no degenerate-triangle strip, no smallest-component drop — plus
/// whatever new geometry the cap + clip steps appended. Downstream
/// `simplify_decoder` choked on that (cf-device-design
/// `main.rs:419-425` documents the iter-1 fixture retaining its full
/// 3.34M face count under topology-preserving simplification, forcing
/// a switch to `simplify_sloppy_decoder`).
///
/// This pass runs in `handle_save_action` between `build_cleaned_mesh`
/// and the save-time simplify (slice 9.8). The fix-set lands here,
/// not inside `build_cleaned_mesh`, so the cap-construction concern
/// (loop-vertex projection) stays isolated from the disk-hygiene
/// concern, and so unit-test fixtures with tiny face counts don't
/// have to fight the `min_component_faces` cutoff.
///
/// Pipeline:
///
///   1. **weld_vertices** — collapse 3N STL-unshared verts to ~N
///      shared so collapse-edge algorithms can find topology.
///   2. **remove_degenerate_triangles** — drop FP-noise zero-area
///      triangles from cap ear-clip + clip intersection slivers.
///   3. **remove_small_components** — drop scanner-noise islands
///      (`< CLEANUP_MIN_COMPONENT_FACES` per shell). Spec assumes
///      the user pre-trimmed to single shell; this is the safety net.
///   4. **remove_unreferenced_vertices** — tighten memory layout
///      after the other passes orphan vertices.
///
/// Order matters: weld first (degenerate detection needs shared
/// indices), then degenerate, then component analysis (so components
/// don't bridge via zero-area triangles), then unreferenced cleanup
/// last.
///
/// Mutates `mesh` in place. Returns a [`CleanupReport`] summarizing
/// each step's effect so the Save status message can surface how
/// aggressive the cleanup was for the user's confidence.
///
/// `smoothing_iterations` controls a final Taubin-smoothing pass
/// (added 2026-05-16, user-driven) that suppresses sub-mm scanner
/// noise so the cleaned STL represents the silicone cast's actual
/// outcome (surface tension during cure smooths sub-mm features
/// physically), not the noisy scan capture.
/// `0` = skip smoothing entirely (back-compat with pre-fix behavior).
/// Default per the Save panel slider is `SMOOTHING_DEFAULT_ITERATIONS`.
pub fn cleanup_cleaned_mesh_for_disk(
    mesh: &mut IndexedMesh,
    smoothing_iterations: usize,
) -> CleanupReport {
    let welded = weld_vertices(mesh, SIMPLIFY_WELD_EPSILON_M);
    let degenerate = remove_degenerate_triangles(mesh, CLEANUP_DEGENERATE_AREA_M2);
    let small_components = remove_small_components(mesh, CLEANUP_MIN_COMPONENT_FACES);
    let unreferenced = remove_unreferenced_vertices(mesh);
    let smoothing = taubin_smooth_vertices(
        mesh,
        smoothing_iterations,
        TAUBIN_DEFAULT_LAMBDA,
        TAUBIN_DEFAULT_MU,
    );
    CleanupReport {
        welded,
        degenerate,
        small_components,
        unreferenced,
        smoothing,
    }
}

/// Per-pass counts from [`cleanup_cleaned_mesh_for_disk`].
///
/// Sums are surfaced in the Save panel status message when non-zero,
/// so the user can tell whether the cleanup pass did meaningful work
/// (workshop-iter-1 fixture: high counts mean the raw scan needed
/// hygiene help; future cleaned scans approaching ~zero counts
/// indicate the pre-prep pipeline is producing cleaner inputs).
#[derive(Debug, Clone, Copy, Default)]
pub struct CleanupReport {
    pub welded: usize,
    pub degenerate: usize,
    pub small_components: usize,
    pub unreferenced: usize,
    /// Number of Taubin-smoothing iterations actually applied
    /// (returned by [`taubin_smooth_vertices`]; matches the
    /// `smoothing_iterations` parameter unless the mesh was
    /// degenerate). Surfaced in the Save status so the user
    /// can see how much smoothing landed in the file.
    pub smoothing: usize,
}

impl CleanupReport {
    /// Total operations across all five passes. Zero when the input
    /// was already disk-ready (no cleanup needed) AND the smoothing
    /// slider was at 0.
    pub fn total(self) -> usize {
        self.welded + self.degenerate + self.small_components + self.unreferenced + self.smoothing
    }
}

/// Format an integer count with `k` / `M` suffixes for compact display
/// in the Scan Info panel + Simplify panel + load-time auto-suggest
/// banner. Mirrors the spec's wording (`"18.4k"`, `"3.35M"`).
pub fn human_count(n: usize) -> String {
    // f64 keeps 53 bits of precision; usize → f64 is lossless for the
    // counts cf-scan-prep operates on (typical scans 10k-10M faces; the
    // 2^53 boundary is ~9×10^15).
    #[allow(clippy::cast_precision_loss)]
    if n >= 1_000_000 {
        format!("{:.2}M", n as f64 / 1_000_000.0)
    } else if n >= 1_000 {
        format!("{:.1}k", n as f64 / 1_000.0)
    } else {
        n.to_string()
    }
}

/// Result of [`simplify_mesh`]: the decimated mesh + wall-clock elapsed
/// for the status-bar achievement message.
pub struct SimplifyResult {
    pub mesh: IndexedMesh,
    pub elapsed_secs: f64,
}

/// Run boundary-preserving quadric edge collapse decimation on `original`
/// down to (approximately) `target_face_count` faces. Returns the
/// simplified mesh + wall-clock elapsed time.
///
/// Pipeline:
///
/// 1. **Weld vertices** ([`mesh_repair::weld_vertices`] with epsilon
///    `SIMPLIFY_WELD_EPSILON_M`). STL load produces 3N unshared
///    vertices (one set per triangle); meshopt operates on indexed
///    buffers and needs shared vertex indices across adjacent triangles
///    to find collapsible edges. Without this step `meshopt::simplify`
///    would see every triangle as topologically disconnected and would
///    refuse to decimate.
/// 2. **Convert positions to `[f32; 3]`** for meshopt's
///    `DecodePosition` impl. f64 → f32 loses ~16 ulps of precision at
///    the scan's mm scale; below the 1 µm weld tolerance.
/// 3. **`meshopt::simplify_decoder`** with
///    `SimplifyOptions::LockBorder`. `target_count` is in **indices**
///    (`target_face_count × 3`); the C-side `meshopt_simplify` takes
///    `target_index_count`. `LockBorder` pins open-boundary-loop
///    vertices in place so the Cap panel (commit #9) sees the same
///    boundary topology after simplification (the spec's load-bearing
///    boundary-preservation requirement).
/// 4. **Reassemble `IndexedMesh`** + strip unreferenced vertices left
///    over from collapse so the simplified mesh has tight memory shape.
///
/// The output mesh shares its vertex coordinates with the welded
/// intermediate (no further conversion); faces reference the surviving
/// vertex indices.
///
/// # Precondition: `target_face_count < original.faces.len()`
///
/// meshopt is a reduction-only algorithm; its C++ side asserts
/// `target_index_count <= index_count` in `meshopt_simplifyEdge`
/// (simplifier.cpp:2286) and SIGABRTs the process if violated. We
/// defensively early-return the cloned input unchanged when the target
/// is at or above the current face count, so callers that forget the
/// precondition see a no-op + finite `elapsed_secs` instead of a
/// process abort. `handle_simplify_actions` pre-checks this case and
/// surfaces a user-facing status message before invoking us; the guard
/// here is belt-and-suspenders.
pub fn simplify_mesh(original: &IndexedMesh, target_face_count: usize) -> SimplifyResult {
    let start = Instant::now();

    if target_face_count >= original.faces.len() {
        return SimplifyResult {
            mesh: original.clone(),
            elapsed_secs: start.elapsed().as_secs_f64(),
        };
    }

    // Step 1: weld unshared vertices into shared indices, then
    // compact the vertex array. STL load produces 3 unique vertex
    // slots per triangle (vertex soup); the decimator needs shared
    // vertex indices across adjacent triangles to find collapsible
    // edges. `remove_unreferenced_vertices` compacts away the
    // unreferenced entries `weld_vertices` leaves behind, which makes
    // the decimator's vertex-index space dense.
    let mut welded = original.clone();
    weld_vertices(&mut welded, SIMPLIFY_WELD_EPSILON_M);
    remove_unreferenced_vertices(&mut welded);

    // Env-var-gated diagnostics. Set `CF_SCAN_PREP_SIMPLIFY_DIAG=1` to
    // see pre/post stats + timing on stderr.
    let diag = std::env::var("CF_SCAN_PREP_SIMPLIFY_DIAG").is_ok_and(|v| !v.is_empty());
    if diag {
        eprintln!(
            "[simplify_mesh] start: target={} welded_faces={} welded_vertices={}",
            target_face_count,
            welded.faces.len(),
            welded.vertices.len(),
        );
    }

    // Step 2: convert `IndexedMesh` -> `CornerTableF` for baby_shark.
    // CornerTableF stores positions as f32 (matches the prior meshopt
    // path's f32 conversion; sub-1 µm precision at scan mm scale).
    #[allow(clippy::cast_possible_truncation)]
    let positions_f32: Vec<Vector3<f32>> = welded
        .vertices
        .iter()
        .map(|p| Vector3::new(p.x as f32, p.y as f32, p.z as f32))
        .collect();
    let flat_indices: Vec<usize> = welded
        .faces
        .iter()
        .flat_map(|tri| [tri[0] as usize, tri[1] as usize, tri[2] as usize])
        .collect();
    let mut ct_mesh = CornerTableF::from_vertex_and_face_slices(&positions_f32, &flat_indices);

    // Step 3: decimate with baby_shark's incremental QEM edge collapse.
    // `keep_boundary(true)` is the equivalent of meshopt's
    // `SimplifyOptions::LockBorder` — pins open-boundary loop vertices
    // so the downstream Cap panel sees the same loop topology after
    // simplification (the spec's load-bearing boundary-preservation
    // requirement). Unlike meshopt, baby_shark's `EdgeDecimator`
    // refuses any collapse that would create a non-manifold edge —
    // see S1.1 probe-8 (2026-05-26) for the empirical confirmation
    // on the workshop iter-1 scan.
    let decimate_start = Instant::now();
    let mut decimator: EdgeDecimator<f32, AlwaysDecimate> = EdgeDecimator::default()
        .decimation_criteria(AlwaysDecimate)
        .min_faces_count(Some(target_face_count))
        .keep_boundary(true);
    decimator.decimate(&mut ct_mesh);
    let decimate_elapsed = decimate_start.elapsed().as_secs_f64();

    // Step 4: convert CornerTableF back to IndexedMesh. baby_shark's
    // `VertexId` is opaque (an internal handle, possibly with gaps
    // after edge collapse); walk `vertices()` once to assign each VID
    // a dense `[0..n)` index, then walk `faces()` to emit triangles
    // using the dense indices.
    let mut out_mesh = IndexedMesh::new();
    let mut vid_to_dense: std::collections::HashMap<_, u32> = std::collections::HashMap::new();
    for vid in <CornerTableF as TriangleMesh>::vertices(&ct_mesh) {
        let pos = <CornerTableF as TriangleMesh>::position(&ct_mesh, vid);
        let dense_idx = out_mesh.vertices.len() as u32;
        out_mesh.vertices.push(Point3::new(
            f64::from(pos[0]),
            f64::from(pos[1]),
            f64::from(pos[2]),
        ));
        vid_to_dense.insert(vid, dense_idx);
    }
    for face_vids in <CornerTableF as TriangleMesh>::faces(&ct_mesh) {
        // Invariant: each `face_vids[i]` was emitted by `vertices()`
        // above, so the map lookup always succeeds. Use `if let` over
        // `expect` to satisfy the crate's `expect_used = deny` lint
        // without changing behavior — the no-match arm is unreachable
        // under the documented baby_shark TriangleMesh contract.
        if let (Some(&a), Some(&b), Some(&c)) = (
            vid_to_dense.get(&face_vids[0]),
            vid_to_dense.get(&face_vids[1]),
            vid_to_dense.get(&face_vids[2]),
        ) {
            out_mesh.faces.push([a, b, c]);
        }
    }

    if diag {
        eprintln!(
            "[simplify_mesh] decimated: in_faces={} out_faces={} out_verts={} \
             decimate_elapsed={:.3}s",
            welded.faces.len(),
            out_mesh.faces.len(),
            out_mesh.vertices.len(),
            decimate_elapsed,
        );
    }

    if diag {
        eprintln!(
            "[simplify_mesh] end: final_faces={} final_vertices={} elapsed={:.3}s",
            out_mesh.faces.len(),
            out_mesh.vertices.len(),
            start.elapsed().as_secs_f64(),
        );
    }

    SimplifyResult {
        mesh: out_mesh,
        elapsed_secs: start.elapsed().as_secs_f64(),
    }
}
