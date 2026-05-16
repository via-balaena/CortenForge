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
use mesh_offset::{MarchingCubesConfig, ScalarGrid, marching_cubes};
use mesh_repair::{remove_degenerate_triangles, remove_unreferenced_vertices, weld_vertices};
use mesh_sdf::SignedDistanceField;
use mesh_types::{Bounded, IndexedMesh};
use meshopt::{SimplifyOptions, simplify_decoder, simplify_sloppy_decoder};
use nalgebra::{Point3, Vector3};

/// One cap-polygon plane parsed out of cf-scan-prep's `[caps]` block
/// and baked into the cleaned-STL frame.
///
/// Cap planes drive the cavity-mouth opening (cavity-mouth spec): the
/// two-SDF construction uses them to strip cap-polygon faces from the
/// SDF source (so the cavity iso has no floor), and the post-MC half-
/// space clip uses them to trim outer-iso skirts that extend past the
/// cap plane on the cap-extension side.
///
/// Only loops with `included == true` survive the parse — excluded
/// loops keep their cap-polygon face in the cleaned STL and stay
/// closed in the device preview (consistent with "the user said leave
/// this hole as-is").
#[derive(Debug, Clone)]
pub(crate) struct CapPlane {
    /// Cap-polygon centroid in cleaned-STL physics-frame meters
    /// (post-`[transform]`-bake). cf-scan-prep emits this in the
    /// PRE-bake frame; the parser applies the user's rotation +
    /// translation before stuffing the value into here so all
    /// downstream geometric code can treat it as cleaned-STL-frame.
    pub(crate) centroid: Point3<f64>,
    /// Cap-polygon outward unit normal in cleaned-STL physics-frame
    /// coordinates. "Outward" = away-from-body-interior (cf-scan-prep
    /// flips raw fit-plane normals via `orient_cap_normal_outward`
    /// before recording). Re-normalized after rotation.
    pub(crate) normal: Vector3<f64>,
    /// Boundary-loop vertex count, recorded at scan time. Used by the
    /// volume-integral primary-cap heuristic (cap-centroid-origin) —
    /// the largest cap by vertex count is picked as the integration
    /// origin when multiple caps are present.
    pub(crate) vertex_count: usize,
    /// Loop index from cf-scan-prep's boundary-loop enumeration. Used
    /// as a secondary tiebreaker for the primary-cap selection (lowest
    /// loop_index wins on ties) and for log/diagnostic readouts.
    pub(crate) loop_index: usize,
}

/// Bevy resource holding the parsed + baked cap planes. Built once at
/// scan load by the `.prep.toml` parser; consumed by the two-SDF
/// `build_cached_scan_sdf` (sub-leaf 3) and the post-MC half-space
/// clip in `extract_layer_surface` (sub-leaf 4).
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

/// Decimated-scan SDF + a pre-filled `ScalarGrid` of its raw distance
/// values, plus the AABB the grid covers.
///
/// Built ONCE per scan load via [`build_cached_scan_sdf`]. Every
/// per-tick layer extraction reads from the same cached grid via
/// [`extract_layer_surface`]; the per-iso `MarchingCubesConfig::at_iso_value`
/// trick lets every layer share the single fill. Contrast with
/// [`mesh_offset::offset_mesh`], which subtracts the offset inside
/// `sample_sdf_to_grid` and extracts at iso 0 — one fill per offset.
///
/// `sdf` is retained (not just consumed by the fill) for two future
/// uses: (a) the heat-map re-projection (per-tet → per-layer-vertex
/// closest-point lookup, sub-leaf 7) and (b) a higher-fidelity Save
/// path (spec §"Open risks #3"). `bounds` is held for the same
/// future Save-side consumer + for `cf_design::Sdf` adapters that
/// need an outer bounding interval.
#[derive(Resource, Clone)]
pub(crate) struct CachedScanSdf {
    /// Reference-counted SDF over the decimated scan. `Arc<T>` is
    /// `Send + Sync` whenever `T` is, and `SignedDistanceField` holds
    /// plain `Vec`s of `f64`/`u32`/`Vector3<f64>` — so this satisfies
    /// Bevy's `Resource: Send + Sync` requirement.
    pub(crate) sdf: Arc<SignedDistanceField>,
    /// Grid pre-filled with `sdf.distance(grid.position(...))` at
    /// every cell. NOT iso-shifted — per-layer extraction picks the
    /// iso value at extract-time.
    pub(crate) grid: ScalarGrid,
    /// Margin-expanded AABB the grid covers, in physics-frame meters.
    pub(crate) bounds: (Point3<f64>, Point3<f64>),
    /// Most-negative SDF value across the cached grid (= negation of
    /// the cavity-collapse threshold). When the user-requested cavity
    /// inset goes below this, the inward iso lies past the SDF
    /// minimum and the extracted cavity mesh is empty — the
    /// SDF analog of the prior `min_radial_distance_m` check.
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

/// Build the cached SDF + grid from a cleaned-scan mesh.
///
/// Decimates the scan to [`SDF_SOURCE_TARGET_FACES`] via
/// [`decimate_scan_for_sdf`], builds a [`SignedDistanceField`],
/// allocates a [`ScalarGrid`] over the scan AABB expanded by
/// `margin_m`, and fills the grid with raw SDF values at every cell.
///
/// Use [`LAYER_PREVIEW_CELL_SIZE_M`] for `cell_size_m` and
/// [`LAYER_GRID_MARGIN_M`] for `margin_m` in production calls; the
/// parameters are spelled out so tests can vary them.
///
/// # Errors
///
/// Propagates [`SignedDistanceField::new`] failures (empty mesh) with
/// context.
pub(crate) fn build_cached_scan_sdf(
    scan: &IndexedMesh,
    cell_size_m: f64,
    margin_m: f64,
) -> Result<CachedScanSdf> {
    let decimated = decimate_scan_for_sdf(scan, SDF_SOURCE_TARGET_FACES);
    let sdf = SignedDistanceField::new(decimated)
        .context("build SignedDistanceField from the decimated scan")?;

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
    for iz in 0..nz {
        for iy in 0..ny {
            for ix in 0..nx {
                let d = sdf.distance(grid.position(ix, iy, iz));
                grid.set(ix, iy, iz, d);
                if d < min_sdf_value {
                    min_sdf_value = d;
                }
            }
        }
    }

    Ok(CachedScanSdf {
        sdf: Arc::new(sdf),
        grid,
        bounds: (min, max),
        min_sdf_value,
    })
}

/// Extract one layer's iso surface from the cached grid.
///
/// `offset_m` is the iso value in meters: positive = outward
/// (layer outer surfaces), negative = inward (cavity surface),
/// zero = original scan surface.
///
/// Cheap (sub-millisecond on a 5 mm grid). Pure: does not mutate the
/// cache.
///
/// # Panics
///
/// `debug_assert!`s that `|offset_m| < LAYER_GRID_MARGIN_M` — the
/// grid AABB was sized for that envelope; extractions past the
/// margin would silently clip against the bounds. See spec
/// §"Open risks #4".
#[must_use]
pub(crate) fn extract_layer_surface(cache: &CachedScanSdf, offset_m: f64) -> IndexedMesh {
    debug_assert!(
        offset_m.abs() < LAYER_GRID_MARGIN_M,
        "extract_layer_surface: |offset_m|={:.4} exceeds LAYER_GRID_MARGIN_M={:.4}; \
         grid AABB cannot represent this iso value",
        offset_m.abs(),
        LAYER_GRID_MARGIN_M,
    );
    let config = MarchingCubesConfig::at_iso_value(offset_m);
    marching_cubes(&cache.grid, &config)
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
        let cache = build_cached_scan_sdf(&sphere, 0.05, 0.2).expect("build cache");
        let surf = extract_layer_surface(&cache, 0.1);
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
        let cache = build_cached_scan_sdf(&sphere, 0.05, 0.2).expect("build cache");
        let surf = extract_layer_surface(&cache, -0.1);
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
        let cache = build_cached_scan_sdf(&cube, 0.02, MARGIN_OFFSET_M).expect("build cache");
        let surf = extract_layer_surface(&cache, 0.1);
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
        let cache = build_cached_scan_sdf(&cube, 0.02, MARGIN_OFFSET_M).expect("build cache");
        let surf = extract_layer_surface(&cache, 0.1);

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
        let cache = build_cached_scan_sdf(&cyl, 0.003, 0.1).expect("build cache");
        let surf = extract_layer_surface(&cache, 0.05);
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
        let cache = build_cached_scan_sdf(&small_sphere, 0.001, 0.02).expect("build cache");
        // Sanity: min SDF should be ≈ -0.02 (sphere center distance).
        assert!(
            cache.min_sdf_value < -0.018 && cache.min_sdf_value > -0.022,
            "sphere min_sdf_value = {}; expected ≈ -0.02",
            cache.min_sdf_value,
        );

        // Past the min SDF: completely empty.
        // -0.025 < cache.min_sdf_value (≈ -0.02), so every grid cell
        // has value >= -0.025; cube-index = 0 in every cell; empty.
        let surf_empty = extract_layer_surface(&cache, -0.025);
        assert!(
            surf_empty.faces.is_empty(),
            "iso past min SDF should yield empty mesh; got {} faces",
            surf_empty.faces.len(),
        );

        // Just within the min SDF: should still produce a non-empty
        // surface (sanity that the "empty" gate above is meaningful).
        let surf_alive = extract_layer_surface(&cache, -0.015);
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
        let a = build_cached_scan_sdf(&sphere, 0.05, 0.2).expect("build a");
        let b = build_cached_scan_sdf(&sphere, 0.05, 0.2).expect("build b");
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
        let cache = build_cached_scan_sdf(&sphere, 0.05, 0.2).expect("build cache");

        let _first = extract_layer_surface(&cache, 0.1);
        let second_after = extract_layer_surface(&cache, -0.1);

        let fresh_cache = build_cached_scan_sdf(&sphere, 0.05, 0.2).expect("fresh");
        let second_fresh = extract_layer_surface(&fresh_cache, -0.1);

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
}
