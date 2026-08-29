//! The `.prep.toml` provenance schema and its serializer.
//!
//! Moved verbatim from the flat `lib.rs` (pure module split).

use crate::{
    AppliedReconstruct, DetectedCapLoop, ReconstructShape, ReconstructedFloorPlane,
    bake_vertex_with_pivot, chrono_like_timestamp, floor_loop_index, trim_centerline_polyline,
};
use anyhow::{Context, Result};
use mesh_repair::{TAUBIN_DEFAULT_LAMBDA, TAUBIN_DEFAULT_MU};
use mesh_types::{Aabb, Point3};
use nalgebra::{UnitQuaternion, Vector3};
use serde::Serialize;
use std::path::Path;

// ----- .prep.toml serializable structures -----
//
// Block names match `docs/SCAN_PREP_DESIGN.md` §Output format (v1.0
// completion rename CSP.1, 2026-05-15). Earlier as-built used
// `[reorient]` / `[recenter]` — those names exposed internal egui-panel
// nouns rather than the conceptual transform operation. Downstream
// consumers (cf-cast-cli `prep.rs`, cf-device-design `parse_centerline`)
// read only `[centerline]` and tolerate unknown sibling keys, so this
// rename is downstream-safe.

#[derive(Serialize)]
pub struct PrepToml {
    pub scan_prep: PrepScanPrepBlock,
    pub simplify: PrepSimplifyBlock,
    pub smoothing: PrepSmoothingBlock,
    pub transform: PrepTransformBlock,
    pub caps: PrepCapsBlock,
    #[serde(skip_serializing_if = "Option::is_none")]
    pub centerline: Option<PrepCenterlineBlock>,
    pub centerline_trim: PrepCenterlineTrimBlock,
    pub output: PrepOutputBlock,
}

#[derive(Serialize)]
pub struct PrepScanPrepBlock {
    pub source_stl: String,
    pub tool_version: &'static str,
    pub generated_at: String,
    pub stl_units_at_load: &'static str,
    /// Physics-frame translation auto-applied at load (CSP.3.5) so
    /// the scan's AABB centroid lands at origin. Provenance only —
    /// the cleaned STL is in the auto-centered frame (plus the
    /// user's transforms). To reconstruct the source-frame position
    /// of a cleaned-STL vertex, INVERT the user's Reorient +
    /// Recenter recorded in `[transform]`, then add this offset.
    pub auto_center_offset_m: [f64; 3],
    /// Auto-PCA rotation applied at load (CSP.4a) — quaternion in
    /// `(w, x, y, z)` order that takes the source's principal axis
    /// to `+Z`. Skipped (serialized as `null`) when PCA was
    /// degenerate (rare). Pairs with `auto_center_offset_m` for
    /// full source-frame reconstruction.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub auto_pca_quaternion: Option<[f64; 4]>,
}

/// `[simplify]` provenance — what decimation, if any, was applied at
/// save time. Per spec §Output format. Records both the user-targeted
/// budget (slice 9.8: Simplify panel slider IS the save-time budget)
/// and the actually-achieved face count, plus the originally-loaded
/// count so the reduction ratio is visible.
#[derive(Serialize)]
pub struct PrepSimplifyBlock {
    /// `true` when meshopt was invoked at save time (target < pre-
    /// simplify cleaned-mesh face count). `false` when the cleaned
    /// mesh's face count was already at or below the target (no
    /// decimation; achieved == original).
    pub applied: bool,
    /// Algorithm identifier — pinned literal so downstream audits know
    /// what code path produced this file.
    pub algorithm: &'static str,
    /// Version string for the algorithm dependency. Tracks the
    /// workspace `meshopt` Cargo dep; bump this constant in lockstep
    /// when that dep is updated. Hard-coded literal because the
    /// meshopt-rs crate doesn't expose its version at runtime and a
    /// build-script lift for one provenance line would be overkill.
    pub algorithm_version: &'static str,
    /// Target face count from the Simplify panel slider at save time.
    pub target_face_count: usize,
    /// Actual face count of the cleaned mesh on disk. Equals
    /// `target_face_count` ± boundary-locked vertex topology slack
    /// when `applied`; equals `original_face_count` when `!applied`.
    pub achieved_face_count: usize,
    /// Face count of the as-loaded scan (in `OriginalScanMesh`,
    /// before any decimation or transforms). Distinct from
    /// `achieved_face_count` whenever simplify ran at all.
    pub original_face_count: usize,
    /// Always `true` — cf-scan-prep uses
    /// `meshopt::SimplifyOptions::LockBorder` (NOT
    /// `simplify_sloppy`) per spec §Architectural decisions §Simplify
    /// algorithm. Surfaced in the TOML for audit purposes.
    pub boundary_preserved: bool,
}

/// `[smoothing]` provenance — what surface smoothing was
/// applied at save time. Added 2026-05-16 with the Taubin
/// smoothing pass in [`cleanup_cleaned_mesh_for_disk`](crate::cleanup_cleaned_mesh_for_disk).
///
/// Downstream consumers can use this to know how aggressively
/// the cleaned mesh has been smoothed — e.g., cf-cast's SDF
/// sampling math is unaffected (it just samples whatever
/// surface is on disk), but a future re-mesh / re-process
/// pipeline can decide whether to apply additional smoothing
/// based on what's already there.
#[derive(Serialize)]
pub struct PrepSmoothingBlock {
    /// Algorithm identifier — pinned literal so downstream
    /// audits know what produced the smoothed vertices.
    pub algorithm: &'static str,
    /// Number of Taubin (shrink + expand) pass pairs applied.
    /// `0` = smoothing disabled at save time.
    pub iterations: usize,
    /// Shrink-pass weight `λ` (positive Laplacian step).
    pub lambda: f64,
    /// Expand-pass weight `μ` (negative Laplacian step).
    pub mu: f64,
}

/// Algorithm identifier pinned literal for
/// [`PrepSmoothingBlock::algorithm`]. Matches the function
/// name in `mesh-repair`'s public API so downstream audit
/// tooling can trace back to the implementation.
pub const SMOOTHING_ALGORITHM_NAME: &str = "taubin_smooth_vertices";

/// `[transform]` umbrella — `rotation` + `translation` sub-tables
/// match spec §Output format. Each renders as `[transform.rotation]`
/// + `[transform.translation]` in TOML.
#[derive(Serialize)]
pub struct PrepTransformBlock {
    pub rotation: PrepRotationBlock,
    pub translation: PrepTranslationBlock,
}

#[derive(Serialize)]
pub struct PrepRotationBlock {
    /// Physics-frame unit quaternion `(w, x, y, z)`. Source of truth
    /// for downstream reconstruction; the Euler angles below mirror
    /// the cf-scan-prep slider source-of-truth for human readability.
    pub quaternion: [f64; 4],
    pub roll_deg: f64,
    pub pitch_deg: f64,
    pub yaw_deg: f64,
}

#[derive(Serialize)]
pub struct PrepTranslationBlock {
    /// Physics-frame translation in meters. Spec §Output format
    /// names this `m`; the panel state stores mm but the on-disk
    /// units convention is meters (matches cf-cast / the rest of
    /// the workspace).
    pub m: [f64; 3],
}

// CSP.4c — `PrepClipBlock` removed alongside `ClipState`. The
// `.prep.toml` no longer emits a `[clip]` block. Downstream
// consumers (cf-cast-cli, cf-device-design) read only
// `[centerline]` and tolerate unknown sibling keys, so the
// removal is downstream-safe.

#[derive(Serialize)]
pub struct PrepCapsBlock {
    /// Whether ear-clipping was applied at save time to close the
    /// included loops. Always `true` if any loops were detected
    /// AND included.
    pub applied: bool,
    pub loops: Vec<PrepCapLoop>,
}

#[derive(Serialize)]
pub struct PrepCapLoop {
    pub loop_index: usize,
    pub vertex_count: usize,
    pub plane_fit_r_squared: f64,
    /// Physics-frame outward normal at scan time (pre-transform bake).
    pub plane_normal: [f64; 3],
    /// Physics-frame centroid at scan time.
    pub plane_centroid_m: [f64; 3],
    pub included: bool,
}

#[derive(Serialize)]
pub struct PrepCenterlineBlock {
    /// Polyline in **post-bake, post-trim world-frame meters**
    /// (matches the cleaned STL's coordinate system; v2 cf-cast
    /// consumes directly). CSP.4b — when the user dialed centerline
    /// trim, this is the polyline between the trim cut planes, not
    /// the full pre-trim polyline.
    pub points_m: Vec<[f64; 3]>,
    pub algorithm: &'static str,
}

/// `[centerline_trim]` provenance — what user-driven centerline
/// trim was applied at save time (CSP.4b). Always emitted, even
/// when no trim was requested (the explicit `0.0 / 0.0` record
/// makes "saved without trim" indistinguishable from an audit
/// perspective).
#[derive(Serialize)]
pub struct PrepCenterlineTrimBlock {
    pub trim_tip_mm: f64,
    pub trim_floor_mm: f64,
    /// Number of boundary loops auto-capped after the trim cuts.
    /// For a "trim only the floor end" save on a closed-tip sock,
    /// this is 1; for "trim both ends" it's 2; for "no trim" it's
    /// 0. When floor reconstruction is applied this drops to 0 +
    /// 1 (the tip-end auto-cap survives, the floor end is
    /// extrusion + flat cap instead of an auto-cap).
    pub capped_loops: usize,
    /// CSP.4e.5 — present only when floor reconstruction was
    /// applied at save time. The cleaned STL has the extruded
    /// sidewall + flat cap baked in; this provenance block
    /// records WHAT shape + how much reference zone the user
    /// dialed.
    #[serde(skip_serializing_if = "Option::is_none")]
    pub reconstruct: Option<PrepReconstructSubBlock>,
}

/// `[centerline_trim.reconstruct]` provenance — emitted only
/// when the user clicked Apply reconstruct before Save. CSP.4e.5.
#[derive(Serialize)]
pub struct PrepReconstructSubBlock {
    /// "constant", "taper", or "extrapolate" — matches the
    /// `ReconstructShape` variant the user selected.
    pub shape: &'static str,
    /// Reference-zone length in mm that drove the radial profile
    /// sampling.
    pub reference_mm: f64,
}

#[derive(Serialize)]
pub struct PrepOutputBlock {
    pub cleaned_stl: String,
    /// AABB of the cleaned mesh on disk (post-transform, post-cap,
    /// post-save-time-simplify), in meters. Spec §Output format
    /// promised this so downstream tooling can sanity-check the
    /// cleaned scan extents without re-loading the STL.
    pub aabb_m: PrepAabbBlock,
}

#[derive(Serialize)]
pub struct PrepAabbBlock {
    pub min: [f64; 3],
    pub max: [f64; 3],
}

/// Algorithm identifier for `[simplify].algorithm`. Pinned literal so
/// downstream audits can distinguish cf-scan-prep's boundary-preserving
/// quadric collapse from other decimation strategies (e.g.,
/// cf-device-design's `simplify_sloppy` proxy).
pub const SIMPLIFY_ALGORITHM_NAME: &str = "meshopt_quadric_edge_collapse";

/// Tracks the workspace `meshopt` Cargo dep. Update in lockstep with
/// `Cargo.toml`'s `meshopt = "X.Y.Z"`.
pub const SIMPLIFY_ALGORITHM_VERSION: &str = "0.6.2";

/// Build the `.prep.toml` string from the current cf-scan-prep state.
/// Includes provenance for every transform / cap / centerline /
/// simplify-at-save so v2 cf-cast (or a future audit) can reconstruct
/// what cf-scan-prep did to produce the cleaned STL.
///
/// `pivot_centroid_m` is the raw scan AABB centroid (in physics-frame
/// meters). The centerline polyline is baked through the same
/// centroid-pivot transform `bake_vertex_with_pivot` uses for mesh
/// vertices, so the polyline coordinates emitted into `.prep.toml`
/// agree with the cleaned STL on disk.
///
/// `simplify_target_face_count`, `original_face_count`, and
/// `cleaned_aabb_m` feed the spec-promised `[simplify]` and
/// `[output.aabb_m]` blocks. The caller computes them from the live
/// `SimplifyState` / `OriginalScanMesh` / final cleaned mesh AABB so
/// this function stays pure (no Res lookups).
#[allow(clippy::too_many_arguments)]
pub fn build_prep_toml_string(
    source_stl: &Path,
    stl_units_label: &'static str,
    auto_center_offset_m: Vector3<f64>,
    auto_pca_quat: Option<UnitQuaternion<f64>>,
    rotation_physics: UnitQuaternion<f64>,
    euler_deg: [f64; 3],
    translation_m: Vector3<f64>,
    centerline_polyline: &[Point3<f64>],
    cap_loops: &[DetectedCapLoop],
    applied_tip_mm: f64,
    applied_floor_mm: f64,
    applied_reconstruct: Option<AppliedReconstruct>,
    centerline_trim_capped: usize,
    cleaned_stl_name: &str,
    rotation_for_centerline: UnitQuaternion<f64>,
    translation_for_centerline_m: Vector3<f64>,
    pivot_centroid_m: Point3<f64>,
    simplify_target_face_count: usize,
    simplify_ran: bool,
    original_face_count: usize,
    achieved_face_count: usize,
    smoothing_iterations: usize,
    cleaned_aabb_m: &Aabb,
    reconstructed_floor: Option<ReconstructedFloorPlane>,
) -> Result<String> {
    let q = rotation_physics;
    let timestamp = chrono_like_timestamp();

    // Project the centerline polyline into world frame so v2 cf-cast
    // can consume it directly without redoing transform math. Same
    // centroid-pivot transform as `build_cleaned_mesh` uses for mesh
    // vertices. CSP.4b — if the user dialed trim, emit the
    // POST-TRIM polyline so the TOML record matches what's actually
    // in the cleaned STL on disk.
    let baked_polyline: Vec<Point3<f64>> = centerline_polyline
        .iter()
        .map(|p| {
            bake_vertex_with_pivot(
                p,
                rotation_for_centerline,
                &pivot_centroid_m,
                translation_for_centerline_m,
            )
        })
        .collect();
    // CSP.4b.6 — use APPLIED trim values (what the displayed +
    // saved mesh was cut with), not the slider values (which may
    // be ahead of the user's last Apply click).
    let trimmed_polyline =
        trim_centerline_polyline(&baked_polyline, applied_tip_mm, applied_floor_mm);
    let centerline_world: Vec<[f64; 3]> =
        trimmed_polyline.iter().map(|p| [p.x, p.y, p.z]).collect();

    let toml_struct = PrepToml {
        scan_prep: PrepScanPrepBlock {
            source_stl: source_stl.display().to_string(),
            tool_version: env!("CARGO_PKG_VERSION"),
            generated_at: timestamp,
            stl_units_at_load: stl_units_label,
            auto_center_offset_m: [
                auto_center_offset_m.x,
                auto_center_offset_m.y,
                auto_center_offset_m.z,
            ],
            auto_pca_quaternion: auto_pca_quat.map(|q| [q.w, q.i, q.j, q.k]),
        },
        simplify: PrepSimplifyBlock {
            // `applied = true` iff the user clicked [Apply Simplify]
            // against the currently-loaded mesh — threaded directly
            // from `SimplifyState::was_applied`. Replaces a prior
            // face-count-inference (`achieved < original`) which
            // produced false positives once save-time simplify was
            // retired (commit `a66a3cda`, 2026-05-15) and the
            // save-time cleanup pass became the only face-dropper
            // for the "user never clicked Apply" case.
            applied: simplify_ran,
            algorithm: SIMPLIFY_ALGORITHM_NAME,
            algorithm_version: SIMPLIFY_ALGORITHM_VERSION,
            target_face_count: simplify_target_face_count,
            achieved_face_count,
            original_face_count,
            boundary_preserved: true,
        },
        smoothing: PrepSmoothingBlock {
            algorithm: SMOOTHING_ALGORITHM_NAME,
            iterations: smoothing_iterations,
            lambda: TAUBIN_DEFAULT_LAMBDA,
            mu: TAUBIN_DEFAULT_MU,
        },
        transform: PrepTransformBlock {
            rotation: PrepRotationBlock {
                quaternion: [q.w, q.i, q.j, q.k],
                roll_deg: euler_deg[0],
                pitch_deg: euler_deg[1],
                yaw_deg: euler_deg[2],
            },
            translation: PrepTranslationBlock {
                m: [translation_m.x, translation_m.y, translation_m.z],
            },
        },
        caps: PrepCapsBlock {
            applied: cap_loops.iter().any(|l| l.include),
            loops: cap_loops
                .iter()
                .enumerate()
                .map(|(i, cl)| {
                    // Reconstruction override: when the user applied
                    // centerline-driven floor reconstruction, the
                    // cleaned mesh's actual floor sits at the
                    // post-reconstruction plane (NOT at this loop's
                    // pre-reconstruction fit plane). Identify the
                    // floor loop with the same pick-by-count
                    // heuristic `find_floor_loop_index` uses (the
                    // largest valid loop is overwhelmingly the cut
                    // rim on practical scans) and override its plane.
                    // Other loops (top-end caps, scanner-noise
                    // stragglers below the size threshold) keep their
                    // detected planes verbatim.
                    let is_floor_loop =
                        reconstructed_floor.is_some() && floor_loop_index(cap_loops) == Some(i);
                    let (centroid, normal, r_squared) = match reconstructed_floor {
                        Some(rf) if is_floor_loop => (rf.centroid_m, rf.normal, 1.0_f64),
                        _ => (cl.plane_centroid, cl.plane_normal, cl.plane_fit_r_squared),
                    };
                    PrepCapLoop {
                        loop_index: i,
                        vertex_count: cl.vertex_indices.len(),
                        plane_fit_r_squared: r_squared,
                        plane_normal: [normal.x, normal.y, normal.z],
                        plane_centroid_m: [centroid.x, centroid.y, centroid.z],
                        included: cl.include,
                    }
                })
                .collect(),
        },
        centerline: if centerline_world.is_empty() {
            None
        } else {
            Some(PrepCenterlineBlock {
                points_m: centerline_world,
                algorithm: "cross_section_centroids",
            })
        },
        centerline_trim: PrepCenterlineTrimBlock {
            // CSP.4b.6 — record APPLIED values (what was actually
            // cut into the saved STL), not slider values (the
            // user's draft preview position).
            trim_tip_mm: applied_tip_mm,
            trim_floor_mm: applied_floor_mm,
            capped_loops: centerline_trim_capped,
            // CSP.4e.5 — reconstruct sub-block. Present only when
            // floor reconstruction was applied (and the floor was
            // chopped — the gate match in `handle_save_action`
            // ensures both).
            reconstruct: applied_reconstruct.and_then(|ar| {
                if applied_floor_mm > 0.0 {
                    Some(PrepReconstructSubBlock {
                        shape: match ar.shape {
                            ReconstructShape::Constant => "constant",
                            ReconstructShape::Taper => "taper",
                            ReconstructShape::Extrapolate => "extrapolate",
                        },
                        reference_mm: ar.reference_mm,
                    })
                } else {
                    None
                }
            }),
        },
        output: PrepOutputBlock {
            cleaned_stl: cleaned_stl_name.to_string(),
            aabb_m: PrepAabbBlock {
                min: [
                    cleaned_aabb_m.min.x,
                    cleaned_aabb_m.min.y,
                    cleaned_aabb_m.min.z,
                ],
                max: [
                    cleaned_aabb_m.max.x,
                    cleaned_aabb_m.max.y,
                    cleaned_aabb_m.max.z,
                ],
            },
        },
    };
    toml::to_string_pretty(&toml_struct).context("serialize PrepToml to TOML")
}
