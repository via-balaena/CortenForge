//! Cap-plane abstraction shared by CortenForge's scan-prep / device-
//! design / cast tooling.
//!
//! cf-scan-prep emits a `[caps]` block in every `.prep.toml` recording
//! the detected open-boundary loops of a cleaned scan — for each loop,
//! the fit-plane centroid + outward normal, vertex count, and whether
//! the user marked the loop as "included" (worth treating as a cap, not
//! a small hole the device should keep sealed). cf-device-design,
//! `insertion_sim`, and cf-cast-cli all parse these records and then
//! compose pinned-floor closed shells against them.
//!
//! Before this crate, every consumer hand-rolled its own copy of the
//! parser + the [`dome_wall_only_mesh`] cap-stripper + the diagnostic
//! used to validate cap-face classification. This crate is the shared
//! home so the three consumers stay in lockstep — schema drift on one
//! side now surfaces as a compile error in the other.
//!
//! # Public surface
//!
//! - [`CapPlane`] — the post-bake runtime type (centroid + outward
//!   normal in cleaned-STL-frame meters, plus `vertex_count` +
//!   `loop_index` for diagnostic readouts).
//! - [`parse_cap_planes`] — parse a `.prep.toml` text into baked
//!   `CapPlane`s. Empty `Vec` when the block is absent or all loops
//!   are excluded.
//! - [`dome_wall_only_mesh`] — strip cap-polygon faces from a cleaned
//!   scan mesh so its `SignedDistanceField` has no cap-polygon offset
//!   to enclose.
//! - [`report_cap_face_classification`] — permanent regression-sentinel
//!   diagnostic. Emits a stderr table of cap-face classification stats
//!   so a Taubin-smoothing or decimation drift that breaks the cap-face
//!   detection rule surfaces at startup, not at the visual gate.
//!
//! # The `CapPlane` runtime frame (post-bake)
//!
//! cf-scan-prep records cap loops in the PRE-bake physics frame —
//! before the user's `[transform]` (rotation + translation) is applied
//! to the cleaned STL vertices. Every consumer needs cap planes in the
//! POST-bake frame so they line up with the cleaned-STL geometry on
//! disk. [`parse_cap_planes`] reads the `[transform]` block and bakes
//! the rotation + translation into every centroid/normal before
//! returning, so `CapPlane.centroid` / `.normal` are always in the
//! cleaned-STL-frame.
//!
//! # Cap-normal convention
//!
//! `CapPlane.normal` points OUTWARD — away from the body interior. This
//! matches cf-scan-prep's `orient_cap_normal_outward` heuristic, which
//! flips raw fit-plane normals to point away from the mesh-majority
//! side. Downstream code uses this convention to decide which half-
//! space to keep when intersecting an offset shell with a cap plane
//! (the body-interior half-space, where `(p - centroid) · normal ≤ 0`).
//!
//! # Schema lockstep risk (parse-only for v1)
//!
//! cf-scan-prep's `.prep.toml` emit path (`PrepCapsBlock` /
//! `PrepCapLoop` in `tools/cf-scan-prep/src/main.rs`) was NOT refactored
//! to use this crate's types — its wire format is flat `[f64;3]` arrays
//! in PRE-bake frame, fundamentally incompatible with this crate's
//! runtime [`CapPlane`] (nalgebra `Point3`/`Vector3` in POST-bake
//! frame). Sharing a serde-derived struct would require dual
//! wire/runtime types + a frame-transform adapter — a bigger refactor
//! than the pinned-floor arc's v1 scope. The lockstep is currently
//! maintained by hand: the field-name string constants in this crate's
//! parser MUST stay in sync with cf-scan-prep's `PrepCapLoop` serde
//! fields.

#![deny(clippy::unwrap_used, clippy::expect_used, clippy::panic)]

use anyhow::Result;
use mesh_types::IndexedMesh;
use nalgebra::{Point3, Quaternion, UnitQuaternion, Vector3};
use serde::Deserialize;

// ---- Constants ----------------------------------------------------------

/// Legacy vertex-distance threshold (meters), retained as a reference
/// value for [`report_cap_face_classification`]'s sweep table.
///
/// NOT consulted by the production cap-face classifier any longer. The
/// original cavity-mouth spec assumed cap-face vertices sit bit-exactly
/// on the cap plane (modulo float roundoff from cf-scan-prep's
/// `auto_cap_open_boundaries` projection). Empirically false: cf-scan-
/// prep also runs 8 iterations of `taubin_smooth_vertices` on the
/// cleaned mesh, which pulls cap-rim vertices toward their off-plane
/// wall neighbors. On the iter-1 sock fixture this drifts cap vertices
/// up to ~6 mm off the plane — 6000× this 1 µm threshold. See
/// [`report_cap_face_classification`] for the diagnostic that
/// surfaced this and [`CAP_FACE_NORMAL_DOT_MIN`] /
/// [`CAP_FACE_CENTROID_DIST_M`] for the replacement classifier.
pub const CAP_FACE_PLANARITY_EPS_M: f64 = 1e-6;

/// Minimum `|dot(face_normal, cap_normal)|` for cap-face classification.
///
/// 0.95 ≈ 18° tolerance — generous enough to absorb decimation jitter on
/// the cap fan, tight enough to exclude the dome wall's normals (which
/// are perpendicular to the cap normal so `|dot| ≈ 0`).
pub const CAP_FACE_NORMAL_DOT_MIN: f64 = 0.95;

/// Maximum face-centroid-to-cap-plane distance (meters) for cap-face
/// classification.
///
/// 25 mm sits squarely inside the plateau the diagnostic exposed on
/// iter-1 — between 10 mm (all 31 cap-fan triangles caught) and 50 mm
/// (no additional catches), with the next nearest false-positive
/// cluster at ~120 mm (the dome apex).
pub const CAP_FACE_CENTROID_DIST_M: f64 = 0.025;

/// Threshold (unitless) above which a face's normal is considered
/// "parallel" to a cap plane's normal for the counterfactual
/// diagnostic. 0.95 ≈ 18° tolerance — matches [`CAP_FACE_NORMAL_DOT_MIN`].
const DIAG_PARALLEL_DOT_THRESHOLD: f64 = 0.95;

// ---- CapPlane runtime type ---------------------------------------------

/// One cap-polygon plane parsed out of cf-scan-prep's `[caps]` block
/// and baked into the cleaned-STL frame.
///
/// Cap planes drive the pinned-floor shell construction: the
/// `dome_wall_only_mesh` strips cap-polygon faces from the SDF source
/// (so the offset iso has no cap-polygon offset to enclose), and the
/// cf-design `pinned_floor_shell` primitive intersects each shell with
/// the body-interior half-space of every cap plane (closing every
/// shell with a flat floor pinned at the cap plane).
///
/// Only loops with `included == true` survive the parse — excluded
/// loops keep their cap-polygon face in the cleaned STL and stay closed
/// in the device preview (consistent with "the user said leave this
/// hole as-is").
#[derive(Debug, Clone)]
pub struct CapPlane {
    /// Cap-polygon centroid in cleaned-STL physics-frame meters
    /// (post-`[transform]`-bake). cf-scan-prep emits this in the PRE-
    /// bake frame; [`parse_cap_planes`] applies the user's rotation +
    /// translation before stuffing the value into here so all
    /// downstream geometric code can treat it as cleaned-STL-frame.
    pub centroid: Point3<f64>,
    /// Cap-polygon outward unit normal in cleaned-STL physics-frame
    /// coordinates. "Outward" = away-from-body-interior (cf-scan-prep
    /// flips raw fit-plane normals via `orient_cap_normal_outward`
    /// before recording). Re-normalized after rotation.
    pub normal: Vector3<f64>,
    /// Boundary-loop vertex count, recorded at scan time. Carried for
    /// diagnostic readouts; the v1 pinned-floor primitive does not
    /// consume it (closed shells are origin-invariant — the cap-
    /// centroid trick the cavity-mouth arc shipped is no longer
    /// needed).
    pub vertex_count: usize,
    /// Loop index from cf-scan-prep's boundary-loop enumeration. Used
    /// for log / diagnostic readouts.
    pub loop_index: usize,
}

impl CapPlane {
    /// Convenience for callers that consume cap planes as anonymous
    /// `(centroid, normal)` tuples — for example
    /// `cf_design::pinned_floor_shell`, which stays schema-agnostic by
    /// taking `&[(Point3, Vector3)]` rather than `&[CapPlane]`.
    #[must_use]
    pub const fn as_tuple(&self) -> (Point3<f64>, Vector3<f64>) {
        (self.centroid, self.normal)
    }
}

// ---- .prep.toml parsing ------------------------------------------------

/// Subset of the `.prep.toml` schema this parser cares about — just
/// `[transform]` + `[caps]`. Other blocks (`[centerline]`, `[scan_prep]`,
/// `[simplify]`, …) are tolerated and ignored: serde's default for
/// missing fields lets a tiny subset coexist with cf-scan-prep's full
/// emit.
#[derive(Deserialize)]
struct PrepTomlSubset {
    transform: Option<TransformBlock>,
    caps: Option<CapsBlock>,
}

#[derive(Deserialize)]
struct TransformBlock {
    rotation: TransformRotationBlock,
    translation: TransformTranslationBlock,
}

#[derive(Deserialize)]
struct TransformRotationBlock {
    /// Physics-frame unit quaternion in `(w, x, y, z)` order — matches
    /// `cf-scan-prep::PrepRotationBlock::quaternion`.
    quaternion: [f64; 4],
}

#[derive(Deserialize)]
struct TransformTranslationBlock {
    /// Physics-frame translation in meters — matches
    /// `cf-scan-prep::PrepTranslationBlock::m`.
    m: [f64; 3],
}

#[derive(Deserialize)]
struct CapsBlock {
    #[serde(default)]
    loops: Vec<CapsLoopRecord>,
}

#[derive(Deserialize)]
struct CapsLoopRecord {
    loop_index: usize,
    vertex_count: usize,
    /// Pre-bake physics-frame outward normal (matches
    /// `cf-scan-prep::PrepCapLoop::plane_normal`).
    plane_normal: [f64; 3],
    /// Pre-bake physics-frame centroid in meters (matches
    /// `cf-scan-prep::PrepCapLoop::plane_centroid_m`).
    plane_centroid_m: [f64; 3],
    included: bool,
}

/// Parse a `.prep.toml` string and return the cap planes baked into the
/// cleaned-STL frame.
///
/// Bake convention: cf-scan-prep records cap loops in the PRE-bake
/// physics frame (`PrepCapLoop::plane_centroid_m` / `plane_normal`); the
/// cleaned STL on disk has the user's `[transform]` (rotation +
/// translation) baked into vertex positions. To align cap planes with
/// the cleaned STL we apply the same rotation + translation here.
/// Excluded loops are silently dropped (consistent with cf-scan-prep
/// keeping their cap-polygon face in the cleaned STL — the device
/// treats the hole as closed).
///
/// **Pivot caveat**: `cf-scan-prep::build_cleaned_mesh` rotates vertices
/// around the working-scan AABB centroid (`bake_vertex_with_pivot`) to
/// feel like "rotate in place". This parser uses pivot = origin because
/// the working-scan AABB centroid is not recorded in `.prep.toml`.
/// After cf-scan-prep's `auto_center_offset_m` the working-scan AABB
/// centroid sits at origin to within sub-cm, so identity-transform
/// scans (the common case) and small manual reorients land cap planes
/// correctly; meaningful manual reorients drift by `‖(R − I) c_w‖` where
/// `c_w` is the residual working-scan AABB centroid. Adding the pivot
/// to `[scan_prep]` would tighten this — deferred (the user-pinned
/// position is "cf-scan-prep stays untouched").
///
/// # Errors
///
/// Returns the underlying `toml` parser error if `text` is not valid
/// TOML; returns `Ok(Vec::new())` for valid TOML with no `[caps]` block,
/// no loops, or all loops `included = false`.
pub fn parse_cap_planes(text: &str) -> Result<Vec<CapPlane>> {
    let subset: PrepTomlSubset = toml::from_str(text)?;
    let rotation = subset
        .transform
        .as_ref()
        .map_or_else(UnitQuaternion::identity, |t| {
            let q = t.rotation.quaternion;
            UnitQuaternion::from_quaternion(Quaternion::new(q[0], q[1], q[2], q[3]))
        });
    let translation = subset.transform.as_ref().map_or_else(Vector3::zeros, |t| {
        Vector3::new(t.translation.m[0], t.translation.m[1], t.translation.m[2])
    });

    let loops = subset.caps.map(|c| c.loops).unwrap_or_default();
    let mut planes = Vec::new();
    for record in loops {
        if !record.included {
            continue;
        }
        let centroid_pre = Point3::new(
            record.plane_centroid_m[0],
            record.plane_centroid_m[1],
            record.plane_centroid_m[2],
        );
        let normal_pre = Vector3::new(
            record.plane_normal[0],
            record.plane_normal[1],
            record.plane_normal[2],
        );
        // Pivot = origin (see docstring); rotation-only on the centroid
        // (translation added below), rotation-only on the normal
        // (directions ignore translation), then re-normalize to shed
        // any quaternion-roundoff drift.
        let centroid_baked =
            Point3::from(rotation.transform_point(&centroid_pre).coords + translation);
        let normal_baked = rotation.transform_vector(&normal_pre).normalize();
        planes.push(CapPlane {
            centroid: centroid_baked,
            normal: normal_baked,
            vertex_count: record.vertex_count,
            loop_index: record.loop_index,
        });
    }
    Ok(planes)
}

// ---- dome_wall_only_mesh ----------------------------------------------

/// Strip cap-polygon faces from a cleaned-scan mesh so the resulting
/// open surface can drive the dome-wall-signed SDF construction.
///
/// A face is a cap face iff its NORMAL is parallel to a cap plane's
/// normal (`|dot| > CAP_FACE_NORMAL_DOT_MIN`, ~18° tolerance) AND its
/// centroid lies within [`CAP_FACE_CENTROID_DIST_M`] of that cap plane.
///
/// **Why face-normal instead of vertex-distance**: cf-scan-prep applies
/// 8 iterations of `taubin_smooth_vertices` (λ=0.5, μ=-0.53) to the
/// cleaned mesh AFTER `auto_cap_open_boundaries` projects boundary
/// vertices onto the fit plane. Taubin smoothing pulls each vertex
/// toward its neighbors' centroid; for a cap-rim vertex surrounded by
/// off-plane wall vertices that's a meaningful normal component to the
/// cap plane. On the iter-1 sock fixture this drifts cap-fan vertices
/// up to ~6 mm off the cap plane — six thousand times the original
/// 1 µm vertex-distance threshold. Face-normal classification is robust
/// against vertex drift: cap-fan triangles still have normals parallel
/// to the cap normal even after every vertex moved a few mm.
///
/// Vertices are NOT compacted — the SDF construction tolerates
/// unreferenced vertices (`SignedDistanceField::new` walks faces, not
/// vertices); skipping the compaction step keeps face-index parity with
/// the input so the cap planes stay aligned with whatever downstream
/// code references both.
///
/// Empty `cap_planes` → mesh cloned unchanged (legacy fast path).
#[must_use]
pub fn dome_wall_only_mesh(cleaned_mesh: &IndexedMesh, cap_planes: &[CapPlane]) -> IndexedMesh {
    if cap_planes.is_empty() {
        return cleaned_mesh.clone();
    }
    let kept_faces: Vec<[u32; 3]> = cleaned_mesh
        .faces
        .iter()
        .filter(|face| {
            let v = [
                cleaned_mesh.vertices[face[0] as usize],
                cleaned_mesh.vertices[face[1] as usize],
                cleaned_mesh.vertices[face[2] as usize],
            ];
            // Compute face normal once per triangle. Degenerate (zero-
            // area) triangles can't be classified — keep them (SDF
            // construction tolerates them; downstream hygiene passes can
            // strip them later if needed).
            let e1 = v[1].coords - v[0].coords;
            let e2 = v[2].coords - v[0].coords;
            let face_normal_unnorm = e1.cross(&e2);
            let face_area2 = face_normal_unnorm.norm();
            if face_area2 < 1e-18 {
                return true;
            }
            let face_normal = face_normal_unnorm / face_area2;
            let face_centroid = (v[0].coords + v[1].coords + v[2].coords) / 3.0;
            // Cap face iff ANY cap plane matches both conditions
            // (normal aligned AND centroid near). Survives iff NO cap
            // plane matches both.
            !cap_planes.iter().any(|plane| {
                let normal_aligned = face_normal.dot(&plane.normal).abs() > CAP_FACE_NORMAL_DOT_MIN;
                let centroid_near = (face_centroid - plane.centroid.coords)
                    .dot(&plane.normal)
                    .abs()
                    < CAP_FACE_CENTROID_DIST_M;
                normal_aligned && centroid_near
            })
        })
        .copied()
        .collect();
    IndexedMesh {
        vertices: cleaned_mesh.vertices.clone(),
        faces: kept_faces,
    }
}

// ---- Diagnostic --------------------------------------------------------

/// Diagnostic helper: report cap-face classification statistics on the
/// given mesh (typically the decimated SDF source). For each cap plane,
/// emits to stderr:
///
/// - face count classified under the legacy vertex-distance rule
///   (`< CAP_FACE_PLANARITY_EPS_M`) — what the old rule would strip;
/// - face count classified under a counterfactual face-normal rule
///   (`|dot| > DIAG_PARALLEL_DOT_THRESHOLD`) at multiple centroid-
///   distance bounds — the sweep table that exposes the
///   smoothing-drift gap;
/// - centroid-distance percentile distribution among parallel-normal
///   faces — quantifies how much Taubin smoothing / decimation actually
///   moved cap-face vertices off the plane.
///
/// Permanent startup log — emits a couple of integers + a few f64s per
/// cap plane, useful as a regression sentinel for every scan load.
///
/// No-op on empty `cap_planes`.
pub fn report_cap_face_classification(mesh: &IndexedMesh, cap_planes: &[CapPlane]) {
    if cap_planes.is_empty() {
        return;
    }
    for plane in cap_planes {
        let n = plane.normal;
        let c = plane.centroid.coords;
        let signed = |p: Point3<f64>| (p.coords - c).dot(&n);
        let mut current_rule_strip = 0_usize;
        // Sweep parallel-normal counts over a centroid-distance ladder.
        // 1 m is effectively "no constraint" (caps lie within the mesh
        // AABB of ~16 cm diagonal, so 1 m bound never trips).
        let centroid_bins_m: [f64; 5] = [5e-3, 10e-3, 25e-3, 50e-3, 1.0];
        let mut parallel_by_bin: [usize; 5] = [0; 5];
        let mut centroid_dist_samples: Vec<f64> = Vec::new();
        for face in &mesh.faces {
            let v = [
                mesh.vertices[face[0] as usize],
                mesh.vertices[face[1] as usize],
                mesh.vertices[face[2] as usize],
            ];
            let d0 = signed(v[0]).abs();
            let d1 = signed(v[1]).abs();
            let d2 = signed(v[2]).abs();
            if d0 < CAP_FACE_PLANARITY_EPS_M
                && d1 < CAP_FACE_PLANARITY_EPS_M
                && d2 < CAP_FACE_PLANARITY_EPS_M
            {
                current_rule_strip += 1;
            }
            let e1 = v[1].coords - v[0].coords;
            let e2 = v[2].coords - v[0].coords;
            let face_normal_unnorm = e1.cross(&e2);
            let face_area2 = face_normal_unnorm.norm();
            if face_area2 < 1e-18 {
                continue;
            }
            let face_normal = face_normal_unnorm / face_area2;
            let dot_abs = face_normal.dot(&n).abs();
            if dot_abs > DIAG_PARALLEL_DOT_THRESHOLD {
                let face_centroid = (v[0].coords + v[1].coords + v[2].coords) / 3.0;
                let centroid_dist = (face_centroid - c).dot(&n).abs();
                centroid_dist_samples.push(centroid_dist);
                for (i, &bound) in centroid_bins_m.iter().enumerate() {
                    if centroid_dist < bound {
                        parallel_by_bin[i] += 1;
                    }
                }
            }
        }
        eprintln!(
            "[cap-planes diag] cap loop {}: decimated faces in = {}; \
             current rule (vdist < {:.0e}) strips {}; \
             face-normal sweep (|dot| > {}):",
            plane.loop_index,
            mesh.faces.len(),
            CAP_FACE_PLANARITY_EPS_M,
            current_rule_strip,
            DIAG_PARALLEL_DOT_THRESHOLD,
        );
        for (i, &bound) in centroid_bins_m.iter().enumerate() {
            eprintln!(
                "    centroid < {:>7.4} m: {:>4} faces",
                bound, parallel_by_bin[i],
            );
        }
        if !centroid_dist_samples.is_empty() {
            let mut sorted = centroid_dist_samples.clone();
            sorted.sort_by(|a, b| a.partial_cmp(b).unwrap_or(std::cmp::Ordering::Equal));
            let pct = |q: f64| {
                // `sorted.len()` is bounded by `mesh.faces.len()` (well
                // under `2^53`), so the f64 round-trip is exact; `idx`
                // is provably in `[0, len-1]` after the (n-1)*q
                // multiplication so the cast is sound.
                #[allow(
                    clippy::cast_possible_truncation,
                    clippy::cast_sign_loss,
                    clippy::cast_precision_loss
                )]
                let idx = ((sorted.len() as f64 - 1.0) * q).round() as usize;
                sorted[idx]
            };
            eprintln!(
                "    centroid-dist distribution among parallel-normal faces (n={}): \
                 p0={:.4} p25={:.4} p50={:.4} p75={:.4} p90={:.4} p100={:.4} m",
                sorted.len(),
                sorted[0],
                pct(0.25),
                pct(0.50),
                pct(0.75),
                pct(0.90),
                sorted[sorted.len() - 1],
            );
        }
    }
}

// ---- Tests -------------------------------------------------------------

#[cfg(test)]
mod tests {
    // `unwrap()` + `expect()` denied at crate level; allow inside tests.
    #![allow(clippy::unwrap_used, clippy::expect_used)]

    use super::*;

    // ----- parse_cap_planes -------------------------------------------

    #[test]
    fn parse_cap_planes_absent_block_returns_empty() {
        let text = r#"
            [scan_prep]
            tool_version = "1.0"
        "#;
        let planes = parse_cap_planes(text).unwrap();
        assert!(planes.is_empty());
    }

    #[test]
    fn parse_cap_planes_filters_excluded_loops() {
        let text = r"
            [caps]
            applied = true

            [[caps.loops]]
            loop_index = 0
            vertex_count = 128
            plane_fit_r_squared = 0.999
            plane_normal = [0.0, 0.0, 1.0]
            plane_centroid_m = [0.0, 0.0, 0.05]
            included = true

            [[caps.loops]]
            loop_index = 1
            vertex_count = 4
            plane_fit_r_squared = 0.5
            plane_normal = [1.0, 0.0, 0.0]
            plane_centroid_m = [0.1, 0.0, 0.0]
            included = false
        ";
        let planes = parse_cap_planes(text).unwrap();
        assert_eq!(planes.len(), 1, "excluded loops must be filtered out");
        assert_eq!(planes[0].loop_index, 0);
        assert_eq!(planes[0].vertex_count, 128);
    }

    #[test]
    fn parse_cap_planes_identity_transform_passes_through() {
        let text = r"
            [caps]
            applied = true

            [[caps.loops]]
            loop_index = 0
            vertex_count = 64
            plane_fit_r_squared = 0.99
            plane_normal = [0.0, 0.0, 1.0]
            plane_centroid_m = [0.01, 0.02, 0.03]
            included = true
        ";
        let planes = parse_cap_planes(text).unwrap();
        assert_eq!(planes.len(), 1);
        let p = &planes[0];
        assert!((p.centroid.x - 0.01).abs() < 1e-12);
        assert!((p.centroid.y - 0.02).abs() < 1e-12);
        assert!((p.centroid.z - 0.03).abs() < 1e-12);
        assert!((p.normal - Vector3::new(0.0, 0.0, 1.0)).norm() < 1e-12);
    }

    #[test]
    fn parse_cap_planes_bakes_90deg_x_rotation_plus_translation() {
        // 90° about X-axis: y → z, z → -y. Plus translation (0.1, 0.2, 0.3).
        // Pre-bake centroid (0, 0, 0.05) → post-rot (0, -0.05, 0) →
        // post-trans (0.1, 0.15, 0.3).
        // Pre-bake normal (0, 0, 1) → post-rot (0, -1, 0).
        let text = r"
            [transform]

            [transform.rotation]
            quaternion = [0.7071067811865476, 0.7071067811865475, 0.0, 0.0]
            roll_deg = 90.0
            pitch_deg = 0.0
            yaw_deg = 0.0

            [transform.translation]
            m = [0.1, 0.2, 0.3]

            [caps]
            applied = true

            [[caps.loops]]
            loop_index = 0
            vertex_count = 32
            plane_fit_r_squared = 1.0
            plane_normal = [0.0, 0.0, 1.0]
            plane_centroid_m = [0.0, 0.0, 0.05]
            included = true
        ";
        let planes = parse_cap_planes(text).unwrap();
        assert_eq!(planes.len(), 1);
        let p = &planes[0];
        assert!((p.centroid - Point3::new(0.1, 0.15, 0.3)).norm() < 1e-9);
        assert!((p.normal - Vector3::new(0.0, -1.0, 0.0)).norm() < 1e-9);
    }

    #[test]
    fn parse_cap_planes_missing_transform_defaults_to_identity() {
        let text = r"
            [caps]
            applied = true

            [[caps.loops]]
            loop_index = 0
            vertex_count = 16
            plane_fit_r_squared = 0.98
            plane_normal = [1.0, 0.0, 0.0]
            plane_centroid_m = [0.5, 0.0, 0.0]
            included = true
        ";
        let planes = parse_cap_planes(text).unwrap();
        assert_eq!(planes.len(), 1);
        assert!((planes[0].centroid - Point3::new(0.5, 0.0, 0.0)).norm() < 1e-12);
        assert!((planes[0].normal - Vector3::new(1.0, 0.0, 0.0)).norm() < 1e-12);
    }

    #[test]
    fn cap_plane_as_tuple_returns_centroid_and_normal() {
        let cp = CapPlane {
            centroid: Point3::new(1.0, 2.0, 3.0),
            normal: Vector3::new(0.0, 0.0, 1.0),
            vertex_count: 42,
            loop_index: 7,
        };
        let (c, n) = cp.as_tuple();
        assert_eq!(c, Point3::new(1.0, 2.0, 3.0));
        assert_eq!(n, Vector3::new(0.0, 0.0, 1.0));
    }

    // ----- dome_wall_only_mesh ----------------------------------------

    fn axis_aligned_cube(h: f64) -> IndexedMesh {
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
        .map(|&(x, y, z)| Point3::new(x, y, z))
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
        let cube = axis_aligned_cube(0.5);
        let original_face_count = cube.faces.len();
        let stripped = dome_wall_only_mesh(&cube, &[]);
        assert_eq!(stripped.faces.len(), original_face_count);
        assert_eq!(stripped.vertices.len(), cube.vertices.len());
    }

    #[test]
    fn dome_wall_only_mesh_removes_top_face_of_cube() {
        let cube = axis_aligned_cube(0.5);
        let cap = cap_plane_at(Point3::new(0.0, 0.0, 0.5), Vector3::new(0.0, 0.0, 1.0));
        let stripped = dome_wall_only_mesh(&cube, &[cap]);
        assert_eq!(stripped.faces.len(), cube.faces.len() - 2);
    }

    #[test]
    fn dome_wall_only_mesh_excludes_dome_apex_with_aligned_normal() {
        // Cube with cap plane at BOTTOM face. Top face vertices have
        // |dot| = 1.0 (aligned) but centroid sits at z = +0.05, which
        // is 100 mm from the bottom cap plane — well outside
        // CAP_FACE_CENTROID_DIST_M (25 mm). Top face must be KEPT.
        let cube = axis_aligned_cube(0.05);
        let bottom_cap = cap_plane_at(Point3::new(0.0, 0.0, -0.05), Vector3::new(0.0, 0.0, -1.0));
        let stripped = dome_wall_only_mesh(&cube, &[bottom_cap]);
        assert_eq!(
            stripped.faces.len(),
            cube.faces.len() - 2,
            "expected only bottom-face triangles stripped (not top)",
        );
    }

    #[test]
    fn dome_wall_only_mesh_vertex_array_unchanged() {
        let cube = axis_aligned_cube(0.5);
        let cap = cap_plane_at(Point3::new(0.0, 0.0, 0.5), Vector3::new(0.0, 0.0, 1.0));
        let stripped = dome_wall_only_mesh(&cube, &[cap]);
        assert_eq!(stripped.vertices.len(), cube.vertices.len());
        for (i, v) in stripped.vertices.iter().enumerate() {
            assert!((v.coords - cube.vertices[i].coords).norm() < 1e-12);
        }
    }

    #[test]
    fn dome_wall_only_mesh_strips_taubin_drifted_cap_face() {
        // Regression: cf-scan-prep's Taubin smoothing drifts cap-face
        // vertices off the cap plane by mm. Face-normal rule still
        // catches them.
        let h = 0.5_f64;
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
            [4, 6, 7], // +z (top — drifted)
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
        let stripped = dome_wall_only_mesh(&drifted_cube, &[cap]);
        assert_eq!(stripped.faces.len(), drifted_cube.faces.len() - 2);
    }

    // ----- report_cap_face_classification -----------------------------

    #[test]
    fn report_cap_face_classification_no_op_on_empty_caps() {
        // Empty caps short-circuits without touching the mesh. Smoke
        // test that it doesn't panic.
        let cube = axis_aligned_cube(0.5);
        report_cap_face_classification(&cube, &[]);
    }

    #[test]
    fn report_cap_face_classification_runs_on_cube_with_cap() {
        // Smoke test — the diagnostic walks the mesh, emits stderr.
        // We just need it not to panic on a typical input.
        let cube = axis_aligned_cube(0.5);
        let cap = cap_plane_at(Point3::new(0.0, 0.0, 0.5), Vector3::new(0.0, 0.0, 1.0));
        report_cap_face_classification(&cube, &[cap]);
    }
}
