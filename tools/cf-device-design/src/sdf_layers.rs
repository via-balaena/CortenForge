//! `sdf_layers` — uniform-offset isosurfaces of the cleaned scan's SDF.
//!
//! Replaces the per-vertex radial displacement that `EnvelopeProxyMesh`
//! drives with a cached SDF + per-iso marching-cubes extraction. The
//! two properties this delivers that the per-vertex path cannot:
//!
//! 1. **No dome-apex nipple on Layer 1.** Uniform-offset isosurfaces
//!    are exactly perpendicular to the source surface by construction;
//!    apex vertices stop racing past their neighbors.
//! 2. **Correct geometry inputs for the FEM insertion sim.**
//!
//! Architecture (per `docs/CF_DEVICE_DESIGN_SDF_LAYERS_SPEC.md`):
//!
//! - Decimate the cleaned scan to `SDF_SOURCE_TARGET_FACES` triangles
//!   via the local `decimate_scan_for_sdf` (regular `simplify_decoder`
//!   with sloppy fallback + degenerate hygiene — mirrors
//!   `compute_envelope_proxy_mesh`'s pipeline, NOT
//!   `insertion_sim::decimate_for_sdf`'s sloppy-only path which the
//!   FEM BCC-mesher resamples past anyway). mesh-sdf has no spatial
//!   acceleration; brute-force O(faces) per query, so the raw
//!   3 M-face scan is non-viable — measured 39 s grid fill at 5 mm.
//! - Build a `SignedDistanceField` over the decimated mesh; wrap in
//!   `Arc` for cheap cloning (matches `cf-cast-cli::scan::SharedScanSdf`).
//! - Allocate a `ScalarGrid` over the scan AABB + `LAYER_GRID_MARGIN_M`
//!   at `LAYER_PREVIEW_CELL_SIZE_M` cell pitch; fill ONCE with raw SDF
//!   values (NOT iso-shifted).
//! - Per layer: `MarchingCubesConfig::at_iso_value` with `iso = +T`
//!   (outward offset for layer outer surfaces) or `iso = -T` (inward
//!   offset for the cavity).
//!
//! The fill is the dominant cost (324 ms on iter-1, dec-2500 @ 5 mm);
//! per-layer extraction is < 1 ms.
//!
//! Sign convention: `SignedDistanceField::distance` is positive
//! outside the scan, negative inside. So:
//! - `iso = +T` (T > 0) → outward offset by T meters (layer outer
//!   surfaces).
//! - `iso = -T` (T > 0) → inward offset by T meters (cavity surface).
//! - `iso =  0` → original scan surface.
//!
//! Module-level `#![allow(dead_code)]` matches the `insertion_sim`
//! precedent: this surface (`CachedScanSdf` + `build_cached_scan_sdf`
//! + `extract_layer_surface` + the three constants) is consumed by
//! later sub-leaves (2: Bevy resource wiring; 3: cavity preview; 4:
//! per-layer surfaces; 5: Validations panel). Each sub-leaf un-shades
//! parts of the surface; the allowance keeps the bin's compile clean
//! at the in-flight slice boundary.
#![allow(dead_code)]

use std::sync::Arc;

use anyhow::{Context, Result};
use bevy::prelude::Resource;
use cf_cap_planes::{CapPlane, dome_wall_only_mesh, report_cap_face_classification};
use mesh_offset::{MarchingCubesConfig, ScalarGrid, marching_cubes};
use mesh_repair::{remove_degenerate_triangles, remove_unreferenced_vertices, weld_vertices};
use mesh_sdf::SignedDistanceField;
use mesh_types::{Bounded, IndexedMesh};
use meshopt::{SimplifyOptions, simplify_decoder, simplify_sloppy_decoder};
use nalgebra::Point3;

/// Bevy resource holding the parsed + baked cap planes. Built once at
/// scan load by the `.prep.toml` parser; consumed by the two-SDF
/// `build_cached_scan_sdf` and the post-MC half-space clip in
/// `extract_layer_surface`.
///
/// Empty default = "no caps" — `Default` makes the resource insertable
/// without conditional wiring at app startup (the parser inserts the
/// fully-populated value, but the resource type always exists so
/// systems can `Res<CapPlanes>` it unconditionally).
#[derive(Resource, Debug, Clone, Default)]
pub(crate) struct CapPlanes {
    pub(crate) planes: Vec<CapPlane>,
}

impl CapPlanes {
    pub(crate) fn is_empty(&self) -> bool {
        self.planes.is_empty()
    }
}

/// Decimation target for the SDF source mesh. mesh-sdf queries are
/// brute-force O(faces); 2500 is the spike-measured sweet spot for
/// iter-1 (324 ms one-time grid fill at 5 mm cell pitch vs 39 s on
/// the raw 167 k-face cleaned scan).
///
/// Generous for the smooth iter-1 sock fixture; body parts with
/// fingertip / nostril / knuckle detail may need 5000+. See spec
/// §"Open risks #2" — revisit when iter-2 / iter-3 scans surface a
/// visible quality regression.
pub(crate) const SDF_SOURCE_TARGET_FACES: usize = 2_500;

/// Weld epsilon (meters) for the pre-decimation vertex weld — mirrors
/// `ENVELOPE_PROXY_WELD_EPSILON_M` (1 µm). cf-scan-prep's STL loader
/// produces 3-per-triangle unshared vertices; meshopt needs them
/// welded to find collapsible edges.
const SDF_WELD_EPSILON_M: f64 = 1e-6;

/// `meshopt::simplify_decoder`'s target-error cap — same posture as
/// `ENVELOPE_PROXY_SIMPLIFY_TARGET_ERROR`: relaxed (10.0 ≈ 1000 % of
/// half-extent) so the face-count target is the binding constraint.
const SDF_SIMPLIFY_TARGET_ERROR: f32 = 10.0;

/// If topology-preserving `simplify_decoder` stops short with more
/// than `SDF_SLOPPY_FALLBACK_MULTIPLIER × target` faces, fall back to
/// `simplify_sloppy_decoder` so the SDF source stays in the
/// O(2 500-face) cost envelope. 4× target mirrors
/// `ENVELOPE_PROXY_SLOPPY_FALLBACK_MULTIPLIER`.
const SDF_SLOPPY_FALLBACK_MULTIPLIER: usize = 4;

/// Area threshold for the post-decimation `remove_degenerate_triangles`
/// hygiene pass. 1 e-15 m² mirrors `ENVELOPE_PROXY_DEGENERATE_AREA_M2`
/// + cf-scan-prep's `CLEANUP_DEGENERATE_AREA_M2`.
const SDF_DEGENERATE_AREA_M2: f64 = 1e-15;

/// Cell pitch (meters) for the cached `ScalarGrid`. 5 mm = the spike's
/// production setting: < 1 ms MC per extraction, ~20 k grid cells on
/// the iter-1 AABB (≈ 150 mm × 150 mm × 250 mm). Consumed by the
/// scan-load wiring in sub-leaf 2.
pub(crate) const LAYER_PREVIEW_CELL_SIZE_M: f64 = 0.005;

/// Margin (meters) added on every side of the scan AABB when
/// allocating the cached grid. Must cover the maximum-outward layer
/// offset so the marching-cubes extraction has enough grid headroom
/// to find the iso surface; 40 mm covers 6 layers × ~5 mm cumulative
/// plus a safety pad. Hard upper bound on extractable offsets — see
/// the `debug_assert!` in [`extract_layer_surface`].
pub(crate) const LAYER_GRID_MARGIN_M: f64 = 0.040;

// CAP_FACE_PLANARITY_EPS_M / CAP_FACE_NORMAL_DOT_MIN /
// CAP_FACE_CENTROID_DIST_M lifted to cf-cap-planes alongside
// `dome_wall_only_mesh` + `report_cap_face_classification` so cf-cast-cli
// + insertion_sim can consume the same classifier without duplication.
// Tests below import `cf_cap_planes::CAP_FACE_PLANARITY_EPS_M`
// directly where the original value was used as an assertion bound.

/// Decimated-scan SDF(s) + a pre-filled `ScalarGrid` of cached
/// modulated distance values, plus the AABB the grid covers.
///
/// Built ONCE per scan load via [`build_cached_scan_sdf`]. Every
/// per-tick layer extraction reads from the same cached grid via
/// [`extract_layer_surface`]; the per-iso `MarchingCubesConfig::at_iso_value`
/// trick lets every layer share the single fill. Contrast with
/// [`mesh_offset::offset_mesh`], which subtracts the offset inside
/// `sample_sdf_to_grid` and extracts at iso 0 — one fill per offset.
///
/// **Two-SDF construction** (cavity-mouth spec sub-leaf 3): the
/// `grid` cells store a *modulated* distance
/// `sd_mod = sign(sd_closed) * |sd_open|`, where
/// - `sdf_closed` is the SDF of the full closed cleaned mesh (cap
///   polygons included). Its sign correctly distinguishes body-
///   interior from body-exterior even for points outside the body
///   laterally (e.g. far above the dome).
/// - `sdf_open` is the SDF of the cleaned mesh with `included` cap
///   polygons stripped (via [`dome_wall_only_mesh`]). Its unsigned
///   distance does NOT see the cap polygon, so the inward iso
///   surface has no cap-polygon offset to enclose — the cavity
///   terminates naturally at the cap plane.
///
/// When there are no cap planes the two SDFs are the same `Arc` and
/// the fill loop skips the second query — single-SDF fast path. See
/// `docs/CF_DEVICE_DESIGN_CAVITY_MOUTH_SPEC.md` §1 Q2 for the full
/// derivation.
///
/// Both SDFs are retained (not just consumed by the fill) for two
/// future uses: (a) the heat-map re-projection (per-tet → per-layer-
/// vertex closest-point lookup, sub-leaf 7) and (b) a higher-fidelity
/// Save path. `bounds` is held for the same future Save-side
/// consumer + for `cf_design::Sdf` adapters that need an outer
/// bounding interval.
#[derive(Resource, Clone)]
pub(crate) struct CachedScanSdf {
    /// Reference-counted SDF over the decimated CLOSED scan (cap
    /// polygons included). Provides the SIGN for the cached grid's
    /// modulated distance. `Arc<T>` is `Send + Sync` whenever `T` is,
    /// and `SignedDistanceField` holds plain `Vec`s of
    /// `f64`/`u32`/`Vector3<f64>` — so this satisfies Bevy's
    /// `Resource: Send + Sync` requirement.
    pub(crate) sdf_closed: Arc<SignedDistanceField>,
    /// Reference-counted SDF over the decimated OPEN scan (cap
    /// polygons stripped). Provides the unsigned MAGNITUDE for the
    /// cached grid's modulated distance. Equals `sdf_closed` (same
    /// `Arc`) when no cap planes are present — the fill loop skips
    /// the second query in that case.
    pub(crate) sdf_open: Arc<SignedDistanceField>,
    /// Grid pre-filled with the modulated distance
    /// `sign(sdf_closed.distance(p)) * sdf_open.unsigned_distance(p)`
    /// at every cell (or just `sdf_closed.distance(p)` when no caps).
    /// NOT iso-shifted — per-layer extraction picks the iso value at
    /// extract-time.
    pub(crate) grid: ScalarGrid,
    /// Margin-expanded AABB the grid covers, in physics-frame meters.
    pub(crate) bounds: (Point3<f64>, Point3<f64>),
    /// Most-negative cached grid value (= negation of the cavity-
    /// collapse threshold). When the user-requested cavity inset goes
    /// below this, the inward iso lies past the modulated minimum
    /// and the extracted cavity mesh is empty — the SDF analog of
    /// the prior `min_radial_distance_m` check.
    pub(crate) min_sdf_value: f64,
}

/// Decimate the cleaned scan to roughly `SDF_SOURCE_TARGET_FACES`
/// triangles for SDF source consumption — topology-preserving with a
/// sloppy fallback, mirroring `compute_envelope_proxy_mesh`'s
/// pipeline (which switched FROM sloppy TO regular on 2026-05-16
/// after the iter-1 sock fixture surfaced sliver triangles that
/// produced visual artifacts in the preview).
///
/// Why a dedicated decimator here rather than reusing
/// `insertion_sim::decimate_for_sdf`: that path is sloppy-only by
/// design — sized for raw uncleaned scans with disconnected
/// components / degenerates that block topology-preserving collapse,
/// and tuned for the FEM sim's BCC-mesher (which tolerates slivers
/// in the SDF source because the BCC lattice resamples
/// independently). The layer-surface MC extraction is more sensitive
/// — slivers in the decimated source produce spurious axis-aligned
/// face-plane segments, which then trigger
/// `mesh-sdf::SignedDistanceField::compute_sign`'s tie behavior at
/// grid points landing on those planes, surfacing as floating
/// "needle" fragments in the extracted layer. Same hygiene as
/// `compute_envelope_proxy_mesh` keeps the SDF source clean enough
/// that MC produces a single connected iso surface per layer.
///
/// Pipeline (per `compute_envelope_proxy_mesh` precedent):
/// 1. Weld unshared STL vertices, then `remove_unreferenced_vertices`
///    (the cf-scan-prep `e314d713` lesson — `weld_vertices` leaves a
///    sparse positions array which meshopt silently no-ops on).
/// 2. `simplify_decoder` with `LockBorder` (topology-preserving).
/// 3. If regular leaves > `SDF_SLOPPY_FALLBACK_MULTIPLIER × target`
///    faces, fall back to `simplify_sloppy_decoder` (defensive for
///    uncleaned scans that bypass cf-scan-prep).
/// 4. `remove_degenerate_triangles` + `remove_unreferenced_vertices`
///    post-decimation hygiene pass.
fn decimate_scan_for_sdf(scan: &IndexedMesh, target_faces: usize) -> IndexedMesh {
    let mut welded = scan.clone();
    weld_vertices(&mut welded, SDF_WELD_EPSILON_M);
    remove_unreferenced_vertices(&mut welded);

    let mut proxy = welded;
    if proxy.faces.len() > target_faces {
        #[allow(clippy::cast_possible_truncation)] // meshopt's C API is f32.
        let positions: Vec<[f32; 3]> = proxy
            .vertices
            .iter()
            .map(|p| [p.x as f32, p.y as f32, p.z as f32])
            .collect();
        let indices: Vec<u32> = proxy.faces.iter().flatten().copied().collect();
        let target_index_count = target_faces.saturating_mul(3);

        let mut result_error = 0.0_f32;
        let simplified_indices = if target_index_count < indices.len() {
            let regular = simplify_decoder(
                &indices,
                &positions,
                target_index_count,
                SDF_SIMPLIFY_TARGET_ERROR,
                SimplifyOptions::LockBorder,
                Some(&mut result_error),
            );
            let regular_face_count = regular.len() / 3;
            let fallback_threshold = target_faces.saturating_mul(SDF_SLOPPY_FALLBACK_MULTIPLIER);
            if regular_face_count > fallback_threshold {
                eprintln!(
                    "[sdf_layers] simplify_decoder stopped at {regular_face_count} faces \
                     (target {target_faces}, fallback {fallback_threshold}); \
                     falling back to simplify_sloppy_decoder",
                );
                simplify_sloppy_decoder(
                    &indices,
                    &positions,
                    target_index_count,
                    SDF_SIMPLIFY_TARGET_ERROR,
                    Some(&mut result_error),
                )
            } else {
                regular
            }
        } else {
            indices
        };
        proxy.faces = simplified_indices
            .chunks_exact(3)
            .map(|tri| [tri[0], tri[1], tri[2]])
            .collect();
    }

    remove_degenerate_triangles(&mut proxy, SDF_DEGENERATE_AREA_M2);
    remove_unreferenced_vertices(&mut proxy);
    proxy
}

// `dome_wall_only_mesh` + `report_cap_face_classification` (plus the
// internal DIAG_* sweep constants) lifted to cf-cap-planes. Callers in
// this module now invoke `cf_cap_planes::dome_wall_only_mesh` +
// `cf_cap_planes::report_cap_face_classification` directly.

/// Build the cached SDF(s) + grid from a cleaned-scan mesh.
///
/// Decimates the scan to [`SDF_SOURCE_TARGET_FACES`] via
/// [`decimate_scan_for_sdf`], builds the closed-body
/// [`SignedDistanceField`], optionally builds a second SDF over the
/// open mesh (cap polygons stripped via [`dome_wall_only_mesh`]) when
/// `cap_planes` is non-empty, allocates a [`ScalarGrid`] over the
/// scan AABB expanded by `margin_m`, and fills the grid with the
/// modulated distance `sign(sd_closed) * |sd_open|` at every cell
/// (or just `sd_closed` when no caps).
///
/// **Cost**: the no-caps fast path matches the legacy single-SDF
/// performance (1 SDF build + 1 query per cell). The cap path adds a
/// second SDF build at startup + a second query per cell — for the
/// iter-1 sock fixture this measured ~324 ms → ~648 ms (one-time at
/// scan load); per-extraction cost is unchanged.
///
/// Use [`LAYER_PREVIEW_CELL_SIZE_M`] for `cell_size_m` and
/// [`LAYER_GRID_MARGIN_M`] for `margin_m` in production calls; the
/// parameters are spelled out so tests can vary them.
///
/// # Errors
///
/// Propagates [`SignedDistanceField::new`] failures (empty mesh, or
/// open mesh empty after stripping all faces as cap faces) with
/// context.
pub(crate) fn build_cached_scan_sdf(
    scan: &IndexedMesh,
    cap_planes: &[CapPlane],
    cell_size_m: f64,
    margin_m: f64,
) -> Result<CachedScanSdf> {
    let decimated = decimate_scan_for_sdf(scan, SDF_SOURCE_TARGET_FACES);
    // Diagnostic: report cap-face classification on the decimated mesh
    // (the actual `dome_wall_only_mesh` input). Compares the current
    // `vertex-distance < CAP_FACE_PLANARITY_EPS_M` rule with a face-
    // normal-based counterfactual; surfaces the hypothesis that
    // post-Taubin-smoothing the vertex-distance rule under-strips on
    // real cleaned scans. See `docs/CF_DEVICE_DESIGN_CAVITY_MOUTH_SPEC.md`
    // §1 Q2. Permanent startup log — two integers + a couple
    // f64s — useful as a regression sentinel for every scan load.
    report_cap_face_classification(&decimated, cap_planes);

    let sdf_closed_raw = SignedDistanceField::new(decimated.clone())
        .context("build closed-body SignedDistanceField from the decimated scan")?;
    let sdf_closed = Arc::new(sdf_closed_raw);
    // No-caps fast path: share the same Arc so the per-cell fill
    // loop's `cap_planes.is_empty()` branch can skip the second
    // query without leaving `sdf_open` empty for downstream
    // consumers.
    let sdf_open = if cap_planes.is_empty() {
        Arc::clone(&sdf_closed)
    } else {
        let open_mesh = dome_wall_only_mesh(&decimated, cap_planes);
        let sdf_open_raw = SignedDistanceField::new(open_mesh)
            .context("build open-body SignedDistanceField (caps stripped)")?;
        Arc::new(sdf_open_raw)
    };

    let aabb = scan.aabb();
    let min = Point3::new(
        aabb.min.x - margin_m,
        aabb.min.y - margin_m,
        aabb.min.z - margin_m,
    );
    let max = Point3::new(
        aabb.max.x + margin_m,
        aabb.max.y + margin_m,
        aabb.max.z + margin_m,
    );
    let mut grid = ScalarGrid::from_bounds(min, max, cell_size_m, 0);

    let (nx, ny, nz) = grid.dimensions();
    let mut min_sdf_value = f64::INFINITY;
    let has_caps = !cap_planes.is_empty();
    for iz in 0..nz {
        for iy in 0..ny {
            for ix in 0..nx {
                let p = grid.position(ix, iy, iz);
                let sd_closed = sdf_closed.distance(p);
                let d = if has_caps {
                    // Two-SDF magnitude: cap polygon is invisible to
                    // `sdf_open.unsigned_distance`, so the inward iso
                    // has no cap-polygon offset to enclose. Sign
                    // comes from `sd_closed` (correct sign for points
                    // outside the body laterally — see spec §1 Q2).
                    // `signum` returns 0.0 when sd_closed is exactly
                    // zero; for grid cells landing on the closed-body
                    // surface this collapses to 0, which is the
                    // correct iso-zero value either way.
                    sd_closed.signum() * sdf_open.unsigned_distance(p)
                } else {
                    sd_closed
                };
                grid.set(ix, iy, iz, d);
                if d < min_sdf_value {
                    min_sdf_value = d;
                }
            }
        }
    }

    Ok(CachedScanSdf {
        sdf_closed,
        sdf_open,
        grid,
        bounds: (min, max),
        min_sdf_value,
    })
}

/// Clip a triangle mesh against ONE cap-plane half-space, keeping
/// only the body-interior side (where `(p - centroid) · normal ≤ 0`).
///
/// Standard Sutherland-Hodgman triangle-vs-plane clip: walk the
/// triangle's three edges in cyclic order; for each edge classify
/// the endpoint pair against the plane and emit kept vertices +
/// edge-plane intersection points into an output polygon. Fan-
/// triangulate the result from the first vertex.
///
/// Result cases per triangle:
/// - All 3 vertices inside → triangle kept verbatim.
/// - All 3 outside → discarded.
/// - 1 inside, 2 outside → 1 sub-triangle (cut corner kept).
/// - 2 inside, 1 outside → quad → 2 sub-triangles.
///
/// New intersection vertices are appended to the output mesh's
/// vertex array. After clipping all faces, unreferenced vertices
/// are stripped via [`remove_unreferenced_vertices`] so the result
/// stays tight (the original vertex array is preserved as a prefix
/// when no clipping happens, otherwise the post-clip vertex set is
/// what survives).
///
/// Plane convention: `plane.normal` points OUTWARD (away from body
/// interior, as cf-scan-prep's `orient_cap_normal_outward` flips
/// raw fit-plane normals before recording). The kept half-space is
/// therefore `signed ≤ 0`.
fn clip_mesh_against_cap_plane(mesh: &IndexedMesh, plane: &CapPlane) -> IndexedMesh {
    let centroid = plane.centroid.coords;
    let normal = plane.normal;
    let signed = |p: Point3<f64>| (p.coords - centroid).dot(&normal);

    let mut out_vertices = mesh.vertices.clone();
    let mut out_faces: Vec<[u32; 3]> = Vec::with_capacity(mesh.faces.len());

    // Linear interpolation between an inside endpoint `a` (with
    // signed distance `da ≤ 0`) and an outside endpoint `b` (with
    // signed distance `db > 0`). Solves `da + t·(db - da) = 0` →
    // `t = da / (da - db)`. Since `da ≤ 0 < db`, denominator is
    // strictly negative and numerator non-positive, so `t ∈ [0, 1]`.
    // `da == db` is unreachable (one is positive, the other non-
    // positive); no zero-divide guard needed.
    let interp = |a: Point3<f64>, da: f64, b: Point3<f64>, db: f64| -> Point3<f64> {
        let t = da / (da - db);
        Point3::from(a.coords + (b.coords - a.coords) * t)
    };

    for face in &mesh.faces {
        let idx = [face[0] as usize, face[1] as usize, face[2] as usize];
        let v = [
            mesh.vertices[idx[0]],
            mesh.vertices[idx[1]],
            mesh.vertices[idx[2]],
        ];
        let d = [signed(v[0]), signed(v[1]), signed(v[2])];

        // Walk edges (0→1), (1→2), (2→0); for each edge apply the
        // Sutherland-Hodgman rule. Output is up to 4 vertices.
        // Track whether each output vertex is original (face-index
        // reusable) or interpolated (needs a new vertex appended).
        enum OutVtx {
            Original(u32),
            Interp(Point3<f64>),
        }
        let mut polygon: Vec<OutVtx> = Vec::with_capacity(4);
        for edge in 0..3 {
            let a_local = edge;
            let b_local = (edge + 1) % 3;
            let da = d[a_local];
            let db = d[b_local];
            let a_in = da <= 0.0;
            let b_in = db <= 0.0;
            let b_face_idx = face[b_local];
            match (a_in, b_in) {
                (true, true) => {
                    polygon.push(OutVtx::Original(b_face_idx));
                }
                (true, false) => {
                    polygon.push(OutVtx::Interp(interp(v[a_local], da, v[b_local], db)));
                }
                (false, true) => {
                    polygon.push(OutVtx::Interp(interp(v[a_local], da, v[b_local], db)));
                    polygon.push(OutVtx::Original(b_face_idx));
                }
                (false, false) => {}
            }
        }

        if polygon.len() < 3 {
            // Fully clipped (0 or degenerate polygon).
            continue;
        }

        // Materialize indices for each output vertex (allocating new
        // ones for interpolated points), then fan-triangulate from
        // index 0.
        let resolved: Vec<u32> = polygon
            .into_iter()
            .map(|v| match v {
                OutVtx::Original(idx) => idx,
                OutVtx::Interp(p) => {
                    // Vertex count overflowing u32 needs > 4 billion
                    // vertices, physically impossible for any
                    // marching-cubes output the cached 5 mm grid can
                    // produce — and the underlying `IndexedMesh::faces`
                    // is `Vec<[u32; 3]>` so the input mesh itself
                    // already proves the count fits.
                    #[allow(clippy::expect_used)]
                    let new_idx = u32::try_from(out_vertices.len())
                        .expect("clipped mesh vertex count must fit in u32");
                    out_vertices.push(p);
                    new_idx
                }
            })
            .collect();
        for i in 1..(resolved.len() - 1) {
            out_faces.push([resolved[0], resolved[i], resolved[i + 1]]);
        }
    }

    let mut out = IndexedMesh {
        vertices: out_vertices,
        faces: out_faces,
    };
    remove_unreferenced_vertices(&mut out);
    out
}

/// Extract one layer's iso surface from the cached grid as a CLOSED
/// pinned-floor shell.
///
/// `offset_m` is the iso value in meters: positive = outward (layer
/// outer surfaces), negative = inward (cavity surface), zero = original
/// scan surface.
///
/// **Pinned-floor construction** (pinned-floor scope-C sub-leaf 2):
/// clone the cached grid, compose
/// `max(scan_sd(p) - offset_m, max over caps of (p - centroid)·normal)`
/// per cell, then extract at iso 0. The composition is a per-cell SDF-
/// level intersection of the offset dome-wall body with every cap's
/// body-interior half-space (outward cap normals mean `(p - c)·n > 0`
/// is the cap-extension side); marching cubes naturally triangulates a
/// flat floor at each cap plane where the half-space SDF crosses zero.
///
/// **No-caps fast path** (regression sentinel): when `cap_planes` is
/// empty, skip the grid clone + per-cell loop and extract directly at
/// `MarchingCubesConfig::at_iso_value(offset_m)`. Byte-identical to
/// the pre-pinned-floor behavior — the post-rewrite path collapses to
/// the original single MC call.
///
/// **Cost**: no-caps path matches the legacy single-MC cost. The
/// pinned-floor path adds one grid clone (`O(cells)` alloc; ~160 kB on
/// iter-1's 20 k cells) + an `O(cells × n_caps)` compose pass. iter-1
/// measures ~1.5 ms total (was ~0.5 ms); negligible against the frame
/// budget. In-place composition (no clone) is a Followup F6 optimization
/// if profiling surfaces a real cost.
///
/// Pure: does not mutate the cache.
///
/// # Panics
///
/// `debug_assert!`s that `|offset_m| < LAYER_GRID_MARGIN_M` — the
/// grid AABB was sized for that envelope; extractions past the
/// margin would silently clip against the bounds.
#[must_use]
pub(crate) fn extract_layer_surface(
    cache: &CachedScanSdf,
    cap_planes: &[CapPlane],
    offset_m: f64,
) -> IndexedMesh {
    debug_assert!(
        offset_m.abs() < LAYER_GRID_MARGIN_M,
        "extract_layer_surface: |offset_m|={:.4} exceeds LAYER_GRID_MARGIN_M={:.4}; \
         grid AABB cannot represent this iso value",
        offset_m.abs(),
        LAYER_GRID_MARGIN_M,
    );
    if cap_planes.is_empty() {
        // No-caps fast path: byte-identical to pre-pinned-floor.
        let config = MarchingCubesConfig::at_iso_value(offset_m);
        return marching_cubes(&cache.grid, &config);
    }
    let mut composed = cache.grid.clone();
    let (nx, ny, nz) = composed.dimensions();
    for iz in 0..nz {
        for iy in 0..ny {
            for ix in 0..nx {
                let p = composed.position(ix, iy, iz);
                let shifted_scan_sd = composed.get(ix, iy, iz) - offset_m;
                // Outward cap normals: (p - c)·n > 0 outside the body
                // (cap-extension side). Intersection of body-interior
                // half-spaces is max-fold across cap planes; on the
                // body-interior side every cap_sd is ≤ 0 so the
                // intersection collapses to shifted_scan_sd.
                let cap_sd = cap_planes
                    .iter()
                    .map(|c| (p.coords - c.centroid.coords).dot(&c.normal))
                    .fold(f64::NEG_INFINITY, f64::max);
                composed.set(ix, iy, iz, shifted_scan_sd.max(cap_sd));
            }
        }
    }
    marching_cubes(&composed, &MarchingCubesConfig::at_iso_value(0.0))
}

#[cfg(test)]
mod tests {
    // `unwrap()` + `expect()` are denied at the crate level for
    // production safety; allow them inside tests so fixtures can pull
    // values out of `Result` returns without multi-line `match`
    // ceremony.
    #![allow(clippy::unwrap_used, clippy::expect_used)]

    use std::collections::HashMap;

    use super::*;
    use mesh_types::IndexedMesh;
    use nalgebra::{Point3, Vector3};

    /// Translation applied to analytical fixtures so the fixture
    /// origin isn't co-located with the world origin. Does NOT defeat
    /// grid-cell sign-tie on its own — the grid origin tracks the
    /// scan AABB, so they shift together. The defense against the
    /// sign-tie is choosing a `margin_m` that isn't an integer
    /// multiple of the cell pitch, which the affected tests do.
    /// See [`MARGIN_OFFSET_M`].
    const FIXTURE_OFFSET: Vector3<f64> = Vector3::new(0.0031, 0.0027, 0.0023);

    /// Grid margin (meters) used by the cube + cylinder tests that
    /// otherwise have axis-aligned faces sitting on grid lines.
    ///
    /// Background (2026-05-16, sub-leaf 1 testing):
    /// [`mesh_sdf::SignedDistanceField::distance`]'s sign branch is
    /// `to_point.dot(face_normal) >= 0.0`. At a grid point exactly
    /// on an axis-aligned mesh face, the f64 roundoff can produce a
    /// tiny negative dot product, flipping the sign — producing
    /// spurious "inside" cells at the boundary plane that marching
    /// cubes then renders as phantom needle surfaces.
    ///
    /// The defense: pick a margin such that `(face_position -
    /// (face_position - margin)) / cell_size = margin / cell_size`
    /// is NOT an integer. Then no grid point lands on a face plane
    /// and the sign-tie path is never reached.
    ///
    /// 0.213 / 0.02 = 10.65 (non-integer) for the cube tests.
    /// This is a per-test workaround for a latent mesh-sdf issue,
    /// NOT a property of our cached-SDF code under test.
    const MARGIN_OFFSET_M: f64 = 0.213;

    // ----- Analytical-fixture mesh builders -----------------------

    /// Unit icosphere centered at origin, refined `subdivisions`
    /// times. Vertices land on the unit sphere; faces are CCW-outward
    /// (matches cf-scan-prep's cleaned-mesh winding convention).
    fn unit_icosphere(subdivisions: u32) -> IndexedMesh {
        let phi = (1.0 + 5.0_f64.sqrt()) / 2.0;
        let mut verts = vec![
            Point3::new(-1.0, phi, 0.0),
            Point3::new(1.0, phi, 0.0),
            Point3::new(-1.0, -phi, 0.0),
            Point3::new(1.0, -phi, 0.0),
            Point3::new(0.0, -1.0, phi),
            Point3::new(0.0, 1.0, phi),
            Point3::new(0.0, -1.0, -phi),
            Point3::new(0.0, 1.0, -phi),
            Point3::new(phi, 0.0, -1.0),
            Point3::new(phi, 0.0, 1.0),
            Point3::new(-phi, 0.0, -1.0),
            Point3::new(-phi, 0.0, 1.0),
        ];
        for v in &mut verts {
            let len = v.coords.norm();
            *v = Point3::from(v.coords / len);
        }
        let mut faces: Vec<[u32; 3]> = vec![
            [0, 11, 5],
            [0, 5, 1],
            [0, 1, 7],
            [0, 7, 10],
            [0, 10, 11],
            [1, 5, 9],
            [5, 11, 4],
            [11, 10, 2],
            [10, 7, 6],
            [7, 1, 8],
            [3, 9, 4],
            [3, 4, 2],
            [3, 2, 6],
            [3, 6, 8],
            [3, 8, 9],
            [4, 9, 5],
            [2, 4, 11],
            [6, 2, 10],
            [8, 6, 7],
            [9, 8, 1],
        ];
        let mut cache: HashMap<(u32, u32), u32> = HashMap::new();
        for _ in 0..subdivisions {
            let mut next: Vec<[u32; 3]> = Vec::with_capacity(faces.len() * 4);
            for &[a, b, c] in &faces {
                let ab = midpoint(&mut verts, &mut cache, a, b);
                let bc = midpoint(&mut verts, &mut cache, b, c);
                let ca = midpoint(&mut verts, &mut cache, c, a);
                next.push([a, ab, ca]);
                next.push([b, bc, ab]);
                next.push([c, ca, bc]);
                next.push([ab, bc, ca]);
            }
            faces = next;
        }
        IndexedMesh {
            vertices: verts,
            faces,
        }
    }

    fn midpoint(
        verts: &mut Vec<Point3<f64>>,
        cache: &mut HashMap<(u32, u32), u32>,
        a: u32,
        b: u32,
    ) -> u32 {
        let key = if a < b { (a, b) } else { (b, a) };
        if let Some(&idx) = cache.get(&key) {
            return idx;
        }
        let mid = Point3::from((verts[a as usize].coords + verts[b as usize].coords) * 0.5);
        let len = mid.coords.norm();
        let normalized = Point3::from(mid.coords / len);
        #[allow(clippy::cast_possible_truncation)]
        let idx = verts.len() as u32;
        verts.push(normalized);
        cache.insert(key, idx);
        idx
    }

    /// Axis-aligned cube of half-extent `h`, translated by `center`,
    /// CCW-outward winding (matches cf-scan-prep's cleaned mesh).
    fn axis_aligned_cube(h: f64, center: Vector3<f64>) -> IndexedMesh {
        let v: Vec<Point3<f64>> = [
            (-h, -h, -h),
            (h, -h, -h),
            (h, h, -h),
            (-h, h, -h),
            (-h, -h, h),
            (h, -h, h),
            (h, h, h),
            (-h, h, h),
        ]
        .iter()
        .map(|&(x, y, z)| Point3::new(x + center.x, y + center.y, z + center.z))
        .collect();
        let f: Vec<[u32; 3]> = vec![
            [0, 2, 1],
            [0, 3, 2], // -z
            [4, 5, 6],
            [4, 6, 7], // +z
            [0, 1, 5],
            [0, 5, 4], // -y
            [2, 3, 7],
            [2, 7, 6], // +y
            [0, 4, 7],
            [0, 7, 3], // -x
            [1, 2, 6],
            [1, 6, 5], // +x
        ];
        IndexedMesh {
            vertices: v,
            faces: f,
        }
    }

    /// Z-axis cylinder with flat circular caps, CCW-outward winding,
    /// translated by `center`. `radius` and `half_height`; `n`
    /// segments around the circumference.
    fn capped_cylinder(radius: f64, half_height: f64, n: u32, center: Vector3<f64>) -> IndexedMesh {
        let mut verts = Vec::with_capacity((2 * n + 2) as usize);
        for i in 0..n {
            let t = f64::from(i) / f64::from(n) * std::f64::consts::TAU;
            verts.push(Point3::new(
                radius * t.cos() + center.x,
                radius * t.sin() + center.y,
                -half_height + center.z,
            ));
        }
        for i in 0..n {
            let t = f64::from(i) / f64::from(n) * std::f64::consts::TAU;
            verts.push(Point3::new(
                radius * t.cos() + center.x,
                radius * t.sin() + center.y,
                half_height + center.z,
            ));
        }
        let bot_center = u32::try_from(verts.len()).expect("vertex count fits in u32");
        verts.push(Point3::new(center.x, center.y, -half_height + center.z));
        let top_center = u32::try_from(verts.len()).expect("vertex count fits in u32");
        verts.push(Point3::new(center.x, center.y, half_height + center.z));

        let mut faces: Vec<[u32; 3]> = Vec::new();
        // Side wall — outward normal points radially out at angle θ.
        // Triangle winding is [b0, t1, t0] / [b0, b1, t1] (cross-checked
        // by hand: previous [b0,t0,t1]/[b0,t1,b1] winding produced an
        // INWARD normal — the cylinder SDF then returned positive
        // inside the cylinder, and iso=+T extracted an inset rather
        // than an offset, which surfaced as test failure during
        // sub-leaf 1 testing).
        for i in 0..n {
            let i_next = (i + 1) % n;
            let b0 = i;
            let b1 = i_next;
            let t0 = n + i;
            let t1 = n + i_next;
            faces.push([b0, t1, t0]);
            faces.push([b0, b1, t1]);
        }
        // Bottom cap (CCW from below = outward-down).
        for i in 0..n {
            let i_next = (i + 1) % n;
            faces.push([bot_center, i_next, i]);
        }
        // Top cap (CCW from above = outward-up).
        for i in 0..n {
            let i_next = (i + 1) % n;
            faces.push([top_center, n + i, n + i_next]);
        }
        IndexedMesh {
            vertices: verts,
            faces,
        }
    }

    // ----- Property tests ----------------------------------------

    #[test]
    fn sphere_isosurface_outward_offset_lies_on_radius_one_plus_t() {
        // Unit sphere SDF (radius 1.0), outward offset T = 0.1, cell
        // 0.05 m, margin 0.2 m. Every extracted vertex should sit at
        // radius 1.1 ± 0.5 × cell_size from origin — the uniform-
        // offset property the SDF path delivers that the per-vertex
        // radial path could not.
        let sphere = unit_icosphere(3);
        let cache = build_cached_scan_sdf(&sphere, &[], 0.05, 0.2).expect("build cache");
        let surf = extract_layer_surface(&cache, &[], 0.1);
        assert!(!surf.vertices.is_empty(), "surface must extract");

        let r_expected = 1.1_f64;
        let tol = 0.5 * 0.05; // half a cell
        for v in &surf.vertices {
            let r = v.coords.norm();
            assert!(
                (r - r_expected).abs() < tol,
                "vertex radius {r} not within {tol} of {r_expected}",
            );
        }
    }

    #[test]
    fn sphere_isosurface_inward_offset_lies_on_radius_one_minus_t() {
        // Same fixture, inward offset T = 0.1 → vertices at radius 0.9.
        let sphere = unit_icosphere(3);
        let cache = build_cached_scan_sdf(&sphere, &[], 0.05, 0.2).expect("build cache");
        let surf = extract_layer_surface(&cache, &[], -0.1);
        assert!(!surf.vertices.is_empty(), "inward surface must extract");

        let r_expected = 0.9_f64;
        let tol = 0.5 * 0.05;
        for v in &surf.vertices {
            let r = v.coords.norm();
            assert!(
                (r - r_expected).abs() < tol,
                "vertex radius {r} not within {tol} of {r_expected}",
            );
        }
    }

    #[test]
    fn cube_outward_offset_face_centers_reach_h_plus_t() {
        // Cube half-extent 0.5, outward offset T = 0.1. Face-center
        // direction (e.g. +X axis) extends to x = 0.6 ± cell_size:
        // the +X face plane moves outward by exactly T, so the max
        // x-coord across extracted vertices should be ≈ 0.6 + the
        // off-grid offset (see [`FIXTURE_OFFSET`] for the rationale).
        let cube = axis_aligned_cube(0.5, FIXTURE_OFFSET);
        // Non-integer-multiple margin defeats mesh-sdf's sign-tie on
        // axis-aligned face planes; see [`MARGIN_OFFSET_M`].
        let cache = build_cached_scan_sdf(&cube, &[], 0.02, MARGIN_OFFSET_M).expect("build cache");
        let surf = extract_layer_surface(&cache, &[], 0.1);
        assert!(!surf.vertices.is_empty(), "cube offset must extract");

        let max_x = surf.vertices.iter().map(|v| v.x).fold(f64::MIN, f64::max);
        let expected = 0.5 + 0.1 + FIXTURE_OFFSET.x;
        assert!(
            (max_x - expected).abs() < 0.025,
            "max-x = {max_x}; expected ≈ {expected} (h=0.5 + T=0.1 + shift)",
        );
    }

    #[test]
    fn cube_corners_are_rounded_not_minkowski() {
        // Corner of a uniform-offset cube should be ROUNDED (spherical
        // cap of radius T at the corner). Max vertex norm should be
        // ≈ h*sqrt(3) + T, NOT the Minkowski-sum corner norm (h+T)*sqrt(3).
        // For h=0.5, T=0.1: rounded ≈ 0.5*sqrt(3) + 0.1 ≈ 0.966 (shift
        // adds another ~0.005); Minkowski cube corner = 0.6*sqrt(3)
        // ≈ 1.039 — meaningfully larger. This test fails if we
        // accidentally extracted a box-Minkowski-sum offset (the
        // failure mode of the prior per-vertex radial path).
        let cube = axis_aligned_cube(0.5, FIXTURE_OFFSET);
        let cache = build_cached_scan_sdf(&cube, &[], 0.02, MARGIN_OFFSET_M).expect("build cache");
        let surf = extract_layer_surface(&cache, &[], 0.1);

        let max_norm = surf
            .vertices
            .iter()
            .map(|v| v.coords.norm())
            .fold(f64::MIN, f64::max);
        let h = 0.5_f64;
        let t = 0.1_f64;
        let shifted_corner_norm = ((h + FIXTURE_OFFSET.x).powi(2)
            + (h + FIXTURE_OFFSET.y).powi(2)
            + (h + FIXTURE_OFFSET.z).powi(2))
        .sqrt();
        let rounded_upper = shifted_corner_norm + t;
        let minkowski = (h + t) * 3.0_f64.sqrt();
        assert!(
            max_norm < rounded_upper + 0.03,
            "max_norm = {max_norm}; expected ≲ {rounded_upper} (rounded-corner upper bound)",
        );
        // Clear gap to Minkowski (~0.07): if we get into Minkowski
        // territory we are not extracting an SDF iso surface.
        assert!(
            max_norm < minkowski - 0.04,
            "max_norm = {max_norm}; would imply Minkowski-cube corner ({minkowski})",
        );
    }

    #[test]
    fn cylinder_outward_offset_radial_reach() {
        // Cylinder radius 0.3, half-height 0.4 along Z, outward
        // offset T = 0.05. The radial extent at z = center.z should
        // reach r + T = 0.35; axial extent reaches half_h + T = 0.45.
        // Use a fine grid (3 mm) and tight tolerance to catch any
        // axis-alignment bug in the fill.
        let cyl = capped_cylinder(0.3, 0.4, 48, FIXTURE_OFFSET);
        let cache = build_cached_scan_sdf(&cyl, &[], 0.003, 0.1).expect("build cache");
        let surf = extract_layer_surface(&cache, &[], 0.05);
        assert!(!surf.vertices.is_empty(), "cylinder offset must extract");

        // Pick vertices near the mid-plane (z ≈ center.z ± 1.5 mm)
        // and check their distance from the cylinder axis reaches
        // r + T.
        let mid_radii: Vec<f64> = surf
            .vertices
            .iter()
            .filter(|v| (v.z - FIXTURE_OFFSET.z).abs() < 0.003)
            .map(|v| {
                let dx = v.x - FIXTURE_OFFSET.x;
                let dy = v.y - FIXTURE_OFFSET.y;
                (dx * dx + dy * dy).sqrt()
            })
            .collect();
        let max_radius = mid_radii.iter().copied().fold(f64::MIN, f64::max);
        assert!(
            (max_radius - 0.35).abs() < 0.01,
            "mid-plane max radius = {max_radius}; expected ≈ 0.35 (r=0.3 + T=0.05)",
        );

        // Axial reach: top vertices at z = center.z + (half_h + T)
        // = center.z + 0.45.
        let max_z = surf.vertices.iter().map(|v| v.z).fold(f64::MIN, f64::max);
        let expected_z = FIXTURE_OFFSET.z + 0.45;
        assert!(
            (max_z - expected_z).abs() < 0.01,
            "max-z = {max_z}; expected ≈ {expected_z} (half_h=0.4 + T=0.05 + shift)",
        );
    }

    #[test]
    fn deep_inward_offset_collapses_cavity_to_empty() {
        // Sphere radius 0.02 m (20 mm): center SDF ≈ -0.02. Asking
        // for an inward offset deeper than that (iso = -0.025) puts
        // the iso below every grid cell's value → marching cubes
        // finds no surface → empty mesh. This is the SDF analog of
        // the prior `cavity_self_intersects` check; the Validations
        // panel (sub-leaf 5) consumes [`CachedScanSdf::min_sdf_value`]
        // for the same gate.
        //
        // Sphere fixture (not cube) deliberately: the mesh-sdf sign-
        // tie issue at axis-aligned face planes makes cube-on-grid
        // tests fragile near boundary cells. The sphere has no
        // axis-aligned faces, so its SDF is well-behaved everywhere.
        let sphere = unit_icosphere(3);
        // Scale the unit icosphere down to radius 0.02 m and apply
        // the test fixture offset.
        let small_sphere = IndexedMesh {
            vertices: sphere
                .vertices
                .iter()
                .map(|v| Point3::from(v.coords * 0.02 + FIXTURE_OFFSET))
                .collect(),
            faces: sphere.faces.clone(),
        };
        let cache = build_cached_scan_sdf(&small_sphere, &[], 0.001, 0.02).expect("build cache");
        // Sanity: min SDF should be ≈ -0.02 (sphere center distance).
        assert!(
            cache.min_sdf_value < -0.018 && cache.min_sdf_value > -0.022,
            "sphere min_sdf_value = {}; expected ≈ -0.02",
            cache.min_sdf_value,
        );

        // Past the min SDF: completely empty.
        // -0.025 < cache.min_sdf_value (≈ -0.02), so every grid cell
        // has value >= -0.025; cube-index = 0 in every cell; empty.
        let surf_empty = extract_layer_surface(&cache, &[], -0.025);
        assert!(
            surf_empty.faces.is_empty(),
            "iso past min SDF should yield empty mesh; got {} faces",
            surf_empty.faces.len(),
        );

        // Just within the min SDF: should still produce a non-empty
        // surface (sanity that the "empty" gate above is meaningful).
        let surf_alive = extract_layer_surface(&cache, &[], -0.015);
        assert!(
            !surf_alive.faces.is_empty(),
            "iso just inside min SDF should yield a non-empty mesh",
        );
    }

    #[test]
    fn build_cached_scan_sdf_is_deterministic() {
        // Same input mesh + same parameters → bit-identical grid
        // values. Guards against any iterator-order non-determinism
        // creeping into the fill loop.
        let sphere = unit_icosphere(2);
        let a = build_cached_scan_sdf(&sphere, &[], 0.05, 0.2).expect("build a");
        let b = build_cached_scan_sdf(&sphere, &[], 0.05, 0.2).expect("build b");
        assert_eq!(a.grid.dimensions(), b.grid.dimensions());
        let (nx, ny, nz) = a.grid.dimensions();
        for iz in 0..nz {
            for iy in 0..ny {
                for ix in 0..nx {
                    let va = a.grid.get(ix, iy, iz);
                    let vb = b.grid.get(ix, iy, iz);
                    assert!(
                        (va - vb).abs() < 1e-12,
                        "grid mismatch at ({ix},{iy},{iz}): {va} vs {vb}",
                    );
                }
            }
        }
        assert!((a.min_sdf_value - b.min_sdf_value).abs() < 1e-12);
    }

    #[test]
    fn extract_layer_surface_does_not_mutate_grid() {
        // Two extractions at different iso values from the same
        // cache: the second result should be identical whether or not
        // the first one ran. Pins the no-mutation property of
        // `extract_layer_surface` (vs `mesh-offset::offset_mesh`,
        // which rewrites the grid each call).
        let sphere = unit_icosphere(3);
        let cache = build_cached_scan_sdf(&sphere, &[], 0.05, 0.2).expect("build cache");

        let _first = extract_layer_surface(&cache, &[], 0.1);
        let second_after = extract_layer_surface(&cache, &[], -0.1);

        let fresh_cache = build_cached_scan_sdf(&sphere, &[], 0.05, 0.2).expect("fresh");
        let second_fresh = extract_layer_surface(&fresh_cache, &[], -0.1);

        assert_eq!(second_after.vertices.len(), second_fresh.vertices.len());
        assert_eq!(second_after.faces.len(), second_fresh.faces.len());
        for (a, b) in second_after
            .vertices
            .iter()
            .zip(second_fresh.vertices.iter())
        {
            assert!((a.coords - b.coords).norm() < 1e-12);
        }
    }

    // ----- Sub-leaf 2: dome_wall_only_mesh ------------------------

    /// Helper: build a `CapPlane` for tests (drops vertex_count /
    /// loop_index — the helper under test only reads centroid + normal).
    fn cap_plane_at(centroid: Point3<f64>, normal: Vector3<f64>) -> CapPlane {
        CapPlane {
            centroid,
            normal: normal.normalize(),
            vertex_count: 0,
            loop_index: 0,
        }
    }

    #[test]
    fn dome_wall_only_mesh_empty_caps_returns_mesh_unchanged() {
        // Legacy fast path: no caps → no face stripping.
        let cube = axis_aligned_cube(0.5, Vector3::zeros());
        let original_face_count = cube.faces.len();
        let stripped = dome_wall_only_mesh(&cube, &[]);
        assert_eq!(stripped.faces.len(), original_face_count);
        assert_eq!(stripped.vertices.len(), cube.vertices.len());
    }

    #[test]
    fn dome_wall_only_mesh_removes_top_face_of_cube() {
        // Cube of half-extent 0.5 centered at origin. Top face is the
        // +z face at z = 0.5; caller flags that plane as a cap. The
        // cube has two triangles per face, so exactly two are removed.
        let cube = axis_aligned_cube(0.5, Vector3::zeros());
        let cap = cap_plane_at(Point3::new(0.0, 0.0, 0.5), Vector3::new(0.0, 0.0, 1.0));
        let stripped = dome_wall_only_mesh(&cube, &[cap]);
        assert_eq!(
            stripped.faces.len(),
            cube.faces.len() - 2,
            "expected 2 top-face triangles removed; got {} (was {})",
            stripped.faces.len(),
            cube.faces.len(),
        );
        // No surviving face has all 3 vertices at z = 0.5 (the top cap
        // was the only such face cluster).
        for face in &stripped.faces {
            let zs = [
                stripped.vertices[face[0] as usize].z,
                stripped.vertices[face[1] as usize].z,
                stripped.vertices[face[2] as usize].z,
            ];
            assert!(
                !zs.iter()
                    .all(|&z| (z - 0.5).abs() < cf_cap_planes::CAP_FACE_PLANARITY_EPS_M),
                "expected no top-face triangle to survive; got {face:?} at z={zs:?}",
            );
        }
    }

    #[test]
    fn dome_wall_only_mesh_removes_both_caps_of_cylinder() {
        // Capped cylinder, bottom cap at z = -h, top cap at z = +h.
        // Flag both planes as caps → both cap fans removed (2n triangles
        // per cap → 4n total), side wall (2n triangles) survives.
        let n = 12;
        let half_h = 0.4;
        let cyl = capped_cylinder(0.3, half_h, n, Vector3::zeros());
        let bot = cap_plane_at(Point3::new(0.0, 0.0, -half_h), Vector3::new(0.0, 0.0, -1.0));
        let top = cap_plane_at(Point3::new(0.0, 0.0, half_h), Vector3::new(0.0, 0.0, 1.0));
        let stripped = dome_wall_only_mesh(&cyl, &[bot, top]);
        // Side wall: 2 triangles per circumference segment × n segments
        // = 2n triangles. Each cap contributes n fan triangles.
        let expected = 2 * (n as usize);
        assert_eq!(
            stripped.faces.len(),
            expected,
            "expected side wall only ({expected} tris); got {}",
            stripped.faces.len(),
        );
    }

    #[test]
    fn dome_wall_only_mesh_keeps_face_grazing_cap_with_only_one_vertex() {
        // A triangle with ONE vertex on a cap plane and two vertices
        // off the plane is NOT a cap face; only the all-three-vertices
        // case triggers removal. Guards against over-aggressive
        // stripping in scans where the dome wall meets the cap rim.
        let mut mesh = IndexedMesh::new();
        // Two vertices off-plane, one on the cap plane (z = 0.5).
        mesh.vertices.push(Point3::new(0.0, 0.0, 0.5));
        mesh.vertices.push(Point3::new(0.1, 0.0, 0.3));
        mesh.vertices.push(Point3::new(0.0, 0.1, 0.3));
        mesh.faces.push([0, 1, 2]);
        let cap = cap_plane_at(Point3::new(0.0, 0.0, 0.5), Vector3::new(0.0, 0.0, 1.0));
        let stripped = dome_wall_only_mesh(&mesh, &[cap]);
        assert_eq!(
            stripped.faces.len(),
            1,
            "single-vertex graze must NOT trigger removal",
        );
    }

    #[test]
    fn dome_wall_only_mesh_vertex_array_unchanged() {
        // Pin the "no compaction" invariant — face indices stay valid
        // against the original vertex array.
        let cube = axis_aligned_cube(0.5, Vector3::zeros());
        let cap = cap_plane_at(Point3::new(0.0, 0.0, 0.5), Vector3::new(0.0, 0.0, 1.0));
        let stripped = dome_wall_only_mesh(&cube, &[cap]);
        assert_eq!(stripped.vertices.len(), cube.vertices.len());
        for (i, v) in stripped.vertices.iter().enumerate() {
            assert!((v.coords - cube.vertices[i].coords).norm() < 1e-12);
        }
    }

    #[test]
    fn dome_wall_only_mesh_strips_taubin_drifted_cap_face() {
        // Regression for the iter-1-falsification bug fixed
        // 2026-05-16: cf-scan-prep's Taubin smoothing drifts cap-face
        // vertices off the cap plane by hundreds of µm to several mm,
        // which the original `vdist < 1 µm` rule under-stripped (0
        // faces caught) and the closed-cavity visual exposed.
        //
        // Synthetic fixture mimicking the post-smoothing state:
        // cube top face vertices pushed off the cap plane by 1 mm
        // (alternating ±). Face normal is still nearly +Z so the
        // face-normal rule catches them.
        let h = 0.5_f64;
        let drift = 1e-3_f64; // 1 mm — well above 1 µm legacy eps.
        let v = vec![
            Point3::new(-h, -h, -h),        // 0
            Point3::new(h, -h, -h),         // 1
            Point3::new(h, h, -h),          // 2
            Point3::new(-h, h, -h),         // 3
            Point3::new(-h, -h, h - drift), // 4 (drifted)
            Point3::new(h, -h, h + drift),  // 5 (drifted)
            Point3::new(h, h, h - drift),   // 6 (drifted)
            Point3::new(-h, h, h + drift),  // 7 (drifted)
        ];
        let faces: Vec<[u32; 3]> = vec![
            [0, 2, 1],
            [0, 3, 2], // -z
            [4, 5, 6],
            [4, 6, 7], // +z (top — to be stripped, vertices drifted ±1mm)
            [0, 1, 5],
            [0, 5, 4], // -y
            [2, 3, 7],
            [2, 7, 6], // +y
            [0, 4, 7],
            [0, 7, 3], // -x
            [1, 2, 6],
            [1, 6, 5], // +x
        ];
        let drifted_cube = IndexedMesh { vertices: v, faces };
        let cap = cap_plane_at(Point3::new(0.0, 0.0, h), Vector3::new(0.0, 0.0, 1.0));
        let stripped = dome_wall_only_mesh(&drifted_cube, &[cap]);
        assert_eq!(
            stripped.faces.len(),
            drifted_cube.faces.len() - 2,
            "expected 2 top-face triangles stripped despite 1 mm vertex drift; \
             got {} (was {})",
            stripped.faces.len(),
            drifted_cube.faces.len(),
        );
    }

    #[test]
    fn dome_wall_only_mesh_excludes_dome_apex_with_aligned_normal() {
        // Regression: on iter-1 sock the cap normal points roughly
        // -Z and the dome-apex face normals point +Z (|dot| ≈ 0.96).
        // Without the centroid-distance gate those dome-apex faces
        // would be falsely stripped. Pin that the centroid check
        // excludes them.
        //
        // Synthetic fixture: cube of half-extent 50 mm with cap plane
        // at the BOTTOM face (z = -50 mm, outward normal -Z). The
        // top face vertices have normals (0, 0, +1) — |dot| with
        // cap normal (0,0,-1) = 1.0 > 0.95 (aligned). But the top
        // face centroid sits at z = +50 mm, which is 100 mm from
        // the bottom cap plane — well outside `CAP_FACE_CENTROID_DIST_M`
        // (25 mm). Top face must be KEPT.
        let cube = axis_aligned_cube(0.05, Vector3::zeros());
        let bottom_cap = cap_plane_at(Point3::new(0.0, 0.0, -0.05), Vector3::new(0.0, 0.0, -1.0));
        let stripped = dome_wall_only_mesh(&cube, &[bottom_cap]);
        // Only the BOTTOM face (2 triangles) should be stripped — top
        // face survives despite aligned normal because its centroid
        // is too far from the bottom cap plane.
        assert_eq!(
            stripped.faces.len(),
            cube.faces.len() - 2,
            "expected only 2 bottom-face triangles stripped (not the top); \
             got {} (was {})",
            stripped.faces.len(),
            cube.faces.len(),
        );
        // Verify NO surviving face has all vertices at z ≈ -0.05
        // (the would-be-stripped bottom face).
        for face in &stripped.faces {
            let zs = [
                stripped.vertices[face[0] as usize].z,
                stripped.vertices[face[1] as usize].z,
                stripped.vertices[face[2] as usize].z,
            ];
            assert!(
                !zs.iter().all(|&z| (z - -0.05).abs() < 1e-9),
                "expected no bottom-face triangle to survive; got {face:?} at z={zs:?}",
            );
        }
        // And verify the top face DID survive (would have been
        // stripped under a normal-only rule with no centroid gate).
        let top_face_survivors = stripped
            .faces
            .iter()
            .filter(|face| {
                [face[0], face[1], face[2]]
                    .iter()
                    .all(|&i| (stripped.vertices[i as usize].z - 0.05).abs() < 1e-9)
            })
            .count();
        assert_eq!(
            top_face_survivors, 2,
            "expected both top-face triangles to survive (aligned normal but far centroid)",
        );
    }

    // ----- Sub-leaf 3: two-SDF build_cached_scan_sdf --------------

    #[test]
    fn build_cached_scan_sdf_no_caps_shares_sdf_arc() {
        // The no-caps fast path is meant to skip the second SDF build
        // by sharing the Arc. Pin that we DON'T build a second SDF
        // unnecessarily.
        let sphere = unit_icosphere(2);
        let cache = build_cached_scan_sdf(&sphere, &[], 0.05, 0.2).expect("build cache");
        assert!(
            Arc::ptr_eq(&cache.sdf_closed, &cache.sdf_open),
            "no-caps fast path must share the same Arc for sdf_closed/sdf_open",
        );
    }

    #[test]
    fn build_cached_scan_sdf_with_caps_builds_distinct_sdfs() {
        // With caps the two SDFs are different objects (different
        // underlying meshes — one with cap faces, one without). The
        // Arcs must be distinct.
        let cube = axis_aligned_cube(0.05, Vector3::zeros());
        let cap = cap_plane_at(Point3::new(0.0, 0.0, 0.05), Vector3::new(0.0, 0.0, 1.0));
        let cache = build_cached_scan_sdf(&cube, &[cap], 0.005, 0.043).expect("build cache");
        assert!(
            !Arc::ptr_eq(&cache.sdf_closed, &cache.sdf_open),
            "cap-present path must build a distinct sdf_open",
        );
    }

    #[test]
    fn two_sdf_cavity_opens_at_cap_plane_on_cube() {
        // Cube of half-extent 0.05 m centered at origin, top face
        // (z = +0.05) flagged as the cap plane. Without the two-SDF
        // construction the cavity at iso = -0.005 would be a smaller
        // closed box at z ∈ [-0.045, +0.045]. With two-SDF the cavity
        // has no floor at z = +0.045 — its top vertices reach up to
        // z ≈ +0.05 (the cap plane) because the open mesh (top face
        // stripped) has no surface up there for the unsigned distance
        // to be 5 mm from.
        //
        // Assertion: max z across extracted cavity vertices is closer
        // to the cap-plane height (+0.05) than to the would-be inward
        // floor (+0.045). With a 5 mm grid the MC vertex placement
        // can drift by half a cell — we accept up to 2.5 mm from the
        // cap plane, which is still well above the closed-cavity
        // floor.
        let cube = axis_aligned_cube(0.05, Vector3::zeros());
        let cap = cap_plane_at(Point3::new(0.0, 0.0, 0.05), Vector3::new(0.0, 0.0, 1.0));
        let cell = 0.005;
        let cache = build_cached_scan_sdf(&cube, &[cap], cell, 0.043).expect("build cache");
        let cavity = extract_layer_surface(&cache, &[], -0.005);
        assert!(!cavity.vertices.is_empty(), "cavity must extract");
        let max_z = cavity.vertices.iter().map(|v| v.z).fold(f64::MIN, f64::max);
        let floor_z = 0.045_f64;
        let cap_z = 0.05_f64;
        assert!(
            max_z > floor_z + 0.002,
            "cavity top reached only z = {max_z}; expected > {} (above closed-cavity floor at {floor_z})",
            floor_z + 0.002,
        );
        assert!(
            max_z <= cap_z + 0.5 * cell,
            "cavity top z = {max_z} exceeded cap plane (+0.05) by more than half a cell",
        );
    }

    #[test]
    fn two_sdf_no_caps_path_is_bit_identical_to_legacy_fast_path() {
        // Determinism + backwards-compat: the no-caps modulated fill
        // collapses to `sd_closed.distance(p)` for every cell. Any
        // grid-value drift between the two-SDF path with empty caps
        // and a hypothetical single-SDF fill would indicate the
        // branch wasn't actually short-circuiting.
        //
        // We exercise the property by hand here: re-run the fill via
        // direct SDF queries and compare cell-by-cell with the cached
        // grid. Used as the regression gate that the no-caps branch
        // matches the pre-spec single-SDF behaviour.
        let sphere = unit_icosphere(2);
        let cache = build_cached_scan_sdf(&sphere, &[], 0.05, 0.2).expect("build cache");
        let (nx, ny, nz) = cache.grid.dimensions();
        for iz in 0..nz {
            for iy in 0..ny {
                for ix in 0..nx {
                    let cached = cache.grid.get(ix, iy, iz);
                    let direct = cache.sdf_closed.distance(cache.grid.position(ix, iy, iz));
                    assert!(
                        (cached - direct).abs() < 1e-12,
                        "no-caps grid value at ({ix},{iy},{iz}) drifted: cached={cached} vs direct sd_closed={direct}",
                    );
                }
            }
        }
    }

    // ----- Sub-leaf 4: post-MC half-space clip --------------------

    /// Triangle-only fixture: build a one-triangle `IndexedMesh`
    /// for half-space clip tests.
    fn single_triangle(a: Point3<f64>, b: Point3<f64>, c: Point3<f64>) -> IndexedMesh {
        IndexedMesh {
            vertices: vec![a, b, c],
            faces: vec![[0, 1, 2]],
        }
    }

    #[test]
    fn clip_keeps_triangle_entirely_inside() {
        // All three vertices on the body-interior side (signed < 0)
        // → triangle kept verbatim.
        let cap = cap_plane_at(Point3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0));
        let tri = single_triangle(
            Point3::new(0.0, 0.0, -0.1),
            Point3::new(0.1, 0.0, -0.05),
            Point3::new(0.0, 0.1, -0.1),
        );
        let clipped = clip_mesh_against_cap_plane(&tri, &cap);
        assert_eq!(clipped.faces.len(), 1);
        // Vertex coordinates preserved (modulo `remove_unreferenced_vertices`
        // permutation — for an all-kept face the order is unchanged).
        assert_eq!(clipped.vertices.len(), 3);
    }

    #[test]
    fn clip_discards_triangle_entirely_outside() {
        // All three vertices on the cap-extension side (signed > 0)
        // → triangle dropped.
        let cap = cap_plane_at(Point3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0));
        let tri = single_triangle(
            Point3::new(0.0, 0.0, 0.1),
            Point3::new(0.1, 0.0, 0.05),
            Point3::new(0.0, 0.1, 0.1),
        );
        let clipped = clip_mesh_against_cap_plane(&tri, &cap);
        assert!(clipped.faces.is_empty());
        assert!(clipped.vertices.is_empty());
    }

    #[test]
    fn clip_one_outside_two_inside_emits_quad_as_two_triangles() {
        // V0 outside (z = +0.1), V1 + V2 inside (z = -0.1) → output
        // polygon is a quad → 2 triangles. The two interpolated
        // points sit exactly on the cap plane (z = 0).
        let cap = cap_plane_at(Point3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0));
        let tri = single_triangle(
            Point3::new(0.0, 0.0, 0.1),  // outside
            Point3::new(0.2, 0.0, -0.1), // inside
            Point3::new(0.0, 0.2, -0.1), // inside
        );
        let clipped = clip_mesh_against_cap_plane(&tri, &cap);
        assert_eq!(clipped.faces.len(), 2, "quad → 2 sub-triangles");
        // Every surviving vertex must have z ≤ 0 (body-interior).
        for v in &clipped.vertices {
            assert!(v.z <= 1e-12, "vertex z = {} should be ≤ 0", v.z);
        }
        // The two interpolated points sit exactly on z = 0 (cap
        // plane); the two original-inside vertices sit at z = -0.1.
        let on_plane = clipped.vertices.iter().filter(|v| v.z.abs() < 1e-9).count();
        assert_eq!(on_plane, 2, "expected 2 vertices on the cap plane");
    }

    #[test]
    fn clip_two_outside_one_inside_emits_single_triangle() {
        // V0 inside, V1 + V2 outside → output is one triangle (the
        // kept vertex + 2 interpolated points).
        let cap = cap_plane_at(Point3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0));
        let tri = single_triangle(
            Point3::new(0.0, 0.0, -0.1), // inside
            Point3::new(0.2, 0.0, 0.1),  // outside
            Point3::new(0.0, 0.2, 0.1),  // outside
        );
        let clipped = clip_mesh_against_cap_plane(&tri, &cap);
        assert_eq!(clipped.faces.len(), 1, "tri-out-tri-in → 1 sub-triangle");
        for v in &clipped.vertices {
            assert!(v.z <= 1e-12, "vertex z = {} should be ≤ 0", v.z);
        }
        // One vertex stays at the original (0, 0, -0.1); two new
        // intersections sit on z = 0.
        let on_plane = clipped.vertices.iter().filter(|v| v.z.abs() < 1e-9).count();
        assert_eq!(on_plane, 2);
    }

    #[test]
    fn clip_against_two_cap_planes_keeps_intersection_region() {
        // Sphere clipped by TWO planes: top (z > 0) AND lateral
        // (x > 0). The surviving region is the lower-left quadrant
        // of the sphere surface. Pin that both clips applied
        // independently produce a non-empty mesh with no vertex on
        // the wrong side of either plane.
        let sphere = unit_icosphere(2);
        let cap_z = cap_plane_at(Point3::new(0.0, 0.0, 0.0), Vector3::new(0.0, 0.0, 1.0));
        let cap_x = cap_plane_at(Point3::new(0.0, 0.0, 0.0), Vector3::new(1.0, 0.0, 0.0));
        let after_z = clip_mesh_against_cap_plane(&sphere, &cap_z);
        let after_both = clip_mesh_against_cap_plane(&after_z, &cap_x);
        assert!(!after_both.faces.is_empty());
        // No vertex should land on the wrong (positive) side of
        // either plane, beyond the float tolerance the interpolation
        // can introduce.
        for v in &after_both.vertices {
            assert!(v.z <= 1e-9, "vertex z = {} survived top clip", v.z);
            assert!(v.x <= 1e-9, "vertex x = {} survived lateral clip", v.x);
        }
    }

    #[test]
    fn extract_layer_surface_with_outer_clip_trims_skirt_on_cube() {
        // Cube of half-extent 50 mm, top face flagged as cap plane,
        // outward offset T = 10 mm. The pinned-floor composition (sub-
        // leaf 2) intersects the offset shell with the cap-plane half-
        // space at extract time, so the outer iso terminates at the
        // cap plane (z ≤ +50 mm modulo half a cell) with a CLOSED flat
        // floor — the pre-pinned-floor post-MC clip used to leave a
        // knife-edge brim; the new path produces a closed shell.
        let cube = axis_aligned_cube(0.05, Vector3::zeros());
        let cap = cap_plane_at(Point3::new(0.0, 0.0, 0.05), Vector3::new(0.0, 0.0, 1.0));
        let cell = 0.005;
        let caps = [cap];
        let cache = build_cached_scan_sdf(&cube, &caps, cell, 0.043).expect("build");
        let outer = extract_layer_surface(&cache, &caps, 0.01);
        assert!(!outer.vertices.is_empty(), "outer iso must extract");
        let max_z = outer.vertices.iter().map(|v| v.z).fold(f64::MIN, f64::max);
        assert!(
            max_z <= 0.05 + 1e-9,
            "outer iso top z = {max_z} must be clipped to cap plane (z = 0.05)",
        );
    }

    // ----- Pinned-floor scope-C sub-leaf 2 ----------------------------

    #[test]
    fn extract_pinned_floor_cube_cavity_has_flat_floor() {
        // Cube of half-extent 50 mm, cap at top face (z=+0.05). Cavity
        // offset T = -5 mm. Cavity iso surface should be a CLOSED shell
        // with a flat floor pinned at z = +0.05 (the cap plane). Bottom
        // of cavity extends down to z = -0.045 (cube floor + |T|).
        let cube = axis_aligned_cube(0.05, Vector3::zeros());
        let cap = cap_plane_at(Point3::new(0.0, 0.0, 0.05), Vector3::new(0.0, 0.0, 1.0));
        let cell = 0.005;
        let caps = [cap];
        let cache = build_cached_scan_sdf(&cube, &caps, cell, 0.043).expect("build");
        let cavity = extract_layer_surface(&cache, &caps, -0.005);
        assert!(!cavity.vertices.is_empty(), "cavity must extract");
        // Flat floor pinned at cap plane: max z ≈ 0.05 (modulo half a
        // cell of MC vertex placement jitter).
        let max_z = cavity.vertices.iter().map(|v| v.z).fold(f64::MIN, f64::max);
        assert!(
            (max_z - 0.05).abs() < 0.5 * cell + 1e-9,
            "cavity top z = {max_z} not within half a cell of cap plane (0.05)",
        );
        // Some vertices should land near z = 0.05 (the floor itself —
        // not just the rim where the dome wall meets the cap plane).
        let near_cap = cavity
            .vertices
            .iter()
            .filter(|v| (v.z - 0.05).abs() < cell)
            .count();
        assert!(
            near_cap >= 4,
            "expected ≥ 4 vertices on the floor at z = 0.05; got {near_cap}",
        );
    }

    #[test]
    fn extract_pinned_floor_cube_outer_grown_has_flat_floor() {
        // Same cube + cap; outer offset T = +5 mm. Outer iso surface
        // is a CLOSED shell with flat floor at cap plane (z = +0.05).
        // Outer surface reaches outward to (h + T) = 0.055 on the side
        // faces; capped at z = 0.05 from above.
        let cube = axis_aligned_cube(0.05, Vector3::zeros());
        let cap = cap_plane_at(Point3::new(0.0, 0.0, 0.05), Vector3::new(0.0, 0.0, 1.0));
        let cell = 0.005;
        let caps = [cap];
        let cache = build_cached_scan_sdf(&cube, &caps, cell, 0.043).expect("build");
        let outer = extract_layer_surface(&cache, &caps, 0.005);
        assert!(!outer.vertices.is_empty(), "outer must extract");
        // Cap plane bounds max z.
        let max_z = outer.vertices.iter().map(|v| v.z).fold(f64::MIN, f64::max);
        assert!(
            max_z <= 0.05 + 1e-9,
            "outer max z = {max_z}; must be ≤ cap plane (0.05)",
        );
        // Some vertices on the flat floor.
        let near_cap = outer
            .vertices
            .iter()
            .filter(|v| (v.z - 0.05).abs() < cell)
            .count();
        assert!(near_cap >= 4, "outer must have flat-floor vertices");
        // Side extent reaches (h + T) = 0.055.
        let max_x = outer.vertices.iter().map(|v| v.x).fold(f64::MIN, f64::max);
        assert!(
            (max_x - 0.055).abs() < cell,
            "outer max x = {max_x}; expected ≈ 0.055",
        );
    }

    #[test]
    fn extract_pinned_floor_no_caps_fast_path_byte_identical() {
        // Regression sentinel: empty cap_planes must produce
        // byte-identical output to a direct MC at_iso_value call. The
        // no-caps fast path is what every existing consumer relies on
        // when no .prep.toml caps are present.
        let sphere = unit_icosphere(3);
        let cache = build_cached_scan_sdf(&sphere, &[], 0.05, 0.2).expect("build");
        let from_extract = extract_layer_surface(&cache, &[], 0.1);
        let from_direct = marching_cubes(&cache.grid, &MarchingCubesConfig::at_iso_value(0.1));
        assert_eq!(
            from_extract.vertices.len(),
            from_direct.vertices.len(),
            "vertex count must match",
        );
        assert_eq!(
            from_extract.faces.len(),
            from_direct.faces.len(),
            "face count must match",
        );
        for (a, b) in from_extract
            .vertices
            .iter()
            .zip(from_direct.vertices.iter())
        {
            assert_eq!(a.coords, b.coords, "vertex coords must be bit-identical");
        }
        for (a, b) in from_extract.faces.iter().zip(from_direct.faces.iter()) {
            assert_eq!(a, b, "face indices must be bit-identical");
        }
    }

    #[test]
    fn extract_pinned_floor_multi_cap_closes_both_floors() {
        // Cube with TWO opposed caps (top + bottom face). Cavity offset
        // T = -5 mm. Iso surface should be a closed shell with flat
        // floors at BOTH cap planes (z = -0.05 and z = +0.05).
        let cube = axis_aligned_cube(0.05, Vector3::zeros());
        let top = cap_plane_at(Point3::new(0.0, 0.0, 0.05), Vector3::new(0.0, 0.0, 1.0));
        let bot = cap_plane_at(Point3::new(0.0, 0.0, -0.05), Vector3::new(0.0, 0.0, -1.0));
        let cell = 0.005;
        let caps = [top, bot];
        let cache = build_cached_scan_sdf(&cube, &caps, cell, 0.043).expect("build");
        let cavity = extract_layer_surface(&cache, &caps, -0.005);
        assert!(!cavity.vertices.is_empty(), "cavity must extract");
        let max_z = cavity.vertices.iter().map(|v| v.z).fold(f64::MIN, f64::max);
        let min_z = cavity.vertices.iter().map(|v| v.z).fold(f64::MAX, f64::min);
        // Both caps pin the floors.
        assert!(
            (max_z - 0.05).abs() < 0.5 * cell + 1e-9,
            "top floor not at +0.05; max z = {max_z}",
        );
        assert!(
            (min_z - -0.05).abs() < 0.5 * cell + 1e-9,
            "bottom floor not at -0.05; min z = {min_z}",
        );
        // Vertices on both floors.
        let near_top = cavity
            .vertices
            .iter()
            .filter(|v| (v.z - 0.05).abs() < cell)
            .count();
        let near_bot = cavity
            .vertices
            .iter()
            .filter(|v| (v.z - -0.05).abs() < cell)
            .count();
        assert!(near_top >= 4, "missing top-floor vertices");
        assert!(near_bot >= 4, "missing bottom-floor vertices");
    }

    #[test]
    fn extract_pinned_floor_cube_taubin_drift_robustness() {
        // Synthetic Taubin-drift fixture: cube with top face vertices
        // pushed off the cap plane by ±1 mm. The face-normal cap-face
        // classifier strips the top face for the open SDF (regression
        // already pinned in `dome_wall_only_mesh_strips_taubin_drifted_cap_face`
        // in cf-cap-planes' tests). With the pinned-floor composition,
        // the floor at the cap plane is built by the cap-plane SDF
        // half-space — NOT the cleaned-scan vertices — so the iso
        // floor sits at the cap plane regardless of the vertex drift.
        let h = 0.05_f64;
        let drift = 1e-3_f64;
        let v = vec![
            Point3::new(-h, -h, -h),
            Point3::new(h, -h, -h),
            Point3::new(h, h, -h),
            Point3::new(-h, h, -h),
            Point3::new(-h, -h, h - drift),
            Point3::new(h, -h, h + drift),
            Point3::new(h, h, h - drift),
            Point3::new(-h, h, h + drift),
        ];
        let faces: Vec<[u32; 3]> = vec![
            [0, 2, 1],
            [0, 3, 2],
            [4, 5, 6],
            [4, 6, 7],
            [0, 1, 5],
            [0, 5, 4],
            [2, 3, 7],
            [2, 7, 6],
            [0, 4, 7],
            [0, 7, 3],
            [1, 2, 6],
            [1, 6, 5],
        ];
        let drifted_cube = IndexedMesh { vertices: v, faces };
        let cap = cap_plane_at(Point3::new(0.0, 0.0, h), Vector3::new(0.0, 0.0, 1.0));
        let cell = 0.005;
        let caps = [cap];
        let cache = build_cached_scan_sdf(&drifted_cube, &caps, cell, 0.043).expect("build");
        let cavity = extract_layer_surface(&cache, &caps, -0.005);
        assert!(
            !cavity.vertices.is_empty(),
            "cavity must extract on drifted cube"
        );
        let max_z = cavity.vertices.iter().map(|v| v.z).fold(f64::MIN, f64::max);
        // Floor pinned at the cap plane (z = h), NOT the drifted
        // vertices (some at h+drift, some at h-drift).
        assert!(
            (max_z - h).abs() < 0.5 * cell + 1e-9,
            "floor drifted from cap plane: max z = {max_z}; expected ≈ {h}",
        );
    }
}
