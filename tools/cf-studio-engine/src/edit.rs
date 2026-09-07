//! The scan-editing session — the headless engine boundary for
//! interactive scan cleanup.
//!
//! Holds the working mesh + the original (for reset) + accumulated
//! edit/provenance state, and applies [`cortenforge::cf_scan_prep_core`] ops. The Bevy
//! `cf-scan-prep` tool and CortenForge Studio's cleanup editor are two
//! frontends over this same logic — the goal is identical function. The
//! session ultimately produces the cleaned STL + `.prep.toml` the cast
//! pipeline consumes.
//!
//! The full op set, mirroring the cf-scan-prep tool: load
//! (auto-center + auto-orient) / weld / simplify / reset; cap detection +
//! interior centerline (`detect_caps`); floor leveling (`level_to_floor`);
//! centerline trim + floor reconstruction (derived, display-only until
//! save); and `save` → the cleaned STL + `.prep.toml` the cast consumes.
//! Reorient/recenter are baked only at save (the `[transform]` block);
//! trim/reconstruct are applied to the displayed/saved mesh, never `working`.

use std::path::{Path, PathBuf};

use cortenforge::cf_scan_prep_core; // module import keeps call sites short
use cortenforge::cf_scan_prep_core::{AppliedReconstruct, DetectedCapLoop, ReconstructShape};
use cortenforge::mesh::repair::{remove_unreferenced_vertices, weld_vertices};
use cortenforge::mesh::types::{Aabb, Bounded, IndexedMesh, Point3};
use nalgebra::{UnitQuaternion, Vector3};

use crate::error::{EngineError, Result};

/// Weld tolerance in meters — matches cf-scan-prep's
/// `SIMPLIFY_WELD_EPSILON_M` so the two tools weld identically.
const WELD_EPSILON_M: f64 = 1e-6;
/// Cross-section slabs sampled along the spine for the centerline — matches
/// the cf-scan-prep tool's `handle_cap_actions` (30).
const CENTERLINE_SLICES: usize = 30;
/// Moving-average smoothing passes on the raw centerline centroids — matches
/// cf-scan-prep (3); tames the few-mm per-slab wobble on noisy scans.
const CENTERLINE_SMOOTH_ITERS: usize = 3;
/// Above this many detected loops on an unwelded mesh, flag "weld first"
/// rather than treat the vertex-soup triangles as real boundaries — matches
/// cf-scan-prep's guard (100).
const UNWELDED_LOOP_WARN: usize = 100;

/// An interactive scan-editing session over a working [`IndexedMesh`].
///
/// Each op mutates the working mesh in place (or replaces it) and records
/// provenance; [`EditSession::reset`] restores the originally-loaded scan.
/// Frontends read [`EditSession::working`] to render the live result after
/// every edit.
#[derive(Debug, Clone)]
pub struct EditSession {
    source_path: PathBuf,
    /// The pristine loaded+scaled scan — the reset target.
    original: IndexedMesh,
    /// The current edited mesh.
    working: IndexedMesh,
    /// Face count of the originally-loaded scan (for `[simplify]` provenance).
    original_face_count: usize,
    simplify_applied: bool,
    simplify_target: usize,
    /// Accumulated recenter offset in meters (auto-center provenance).
    auto_center_offset_m: Vector3<f64>,
    /// Accumulated PCA-orient rotation (auto-orient provenance).
    auto_pca_quat: Option<UnitQuaternion<f64>>,
    /// Detected open-boundary cap loops (from the last `detect_caps`).
    /// Cleared whenever a mesh-mutating op makes them stale.
    cap_loops: Vec<DetectedCapLoop>,
    /// The interior centerline polyline (from the last `detect_caps`).
    centerline: Vec<Point3<f64>>,
    /// The UNBAKED reorient rotation (e.g. from `level_to_floor`) — applied
    /// to the DISPLAY only; baked into the cleaned mesh + recorded as the
    /// `[transform]` block at save. Persists across mesh edits (it's an
    /// orientation intent, not geometry); only `reset` clears it.
    reorient_rotation: UnitQuaternion<f64>,
    /// Applied centerline trim at the tip end, in mm (0 = no trim). Like
    /// reorient, trim is a DERIVED op: it chops the displayed/saved mesh
    /// along the centerline, but never mutates `working`.
    trim_tip_mm: f64,
    /// Applied centerline trim at the floor end, in mm (0 = no trim).
    trim_floor_mm: f64,
    /// Applied floor reconstruction (closes the chopped floor with an
    /// extruded profile), or `None` for a flat cap. Gated on a floor trim.
    reconstruct: Option<AppliedReconstruct>,
}

/// Summary of a [`EditSession::detect_caps`] pass, for the frontend to
/// surface: how many open-boundary loops were found, how long the centerline
/// came out, and whether the mesh looks like unwelded vertex soup (in which
/// case the user should Weld first).
#[derive(Debug, Clone, Copy, PartialEq, Eq)]
pub struct CapScan {
    /// Number of valid open-boundary loops detected.
    pub loop_count: usize,
    /// Centerline segment count (`points - 1`; 0 if no centerline).
    pub centerline_segments: usize,
    /// The mesh looks like raw, unwelded STL soup (too many loops) — the
    /// frontend should prompt for a Weld before trusting the result.
    pub looks_unwelded: bool,
}

/// What [`EditSession::save`] wrote: the cleaned-scan STL + the `.prep.toml`
/// the cast pipeline consumes, plus the final face count.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct SaveReport {
    /// Path to the written `{stem}.cleaned.stl`.
    pub cleaned_stl: PathBuf,
    /// Path to the written `{stem}.prep.toml`.
    pub prep_toml: PathBuf,
    /// Face count of the cleaned mesh on disk.
    pub face_count: usize,
}

impl EditSession {
    /// Load a scan (STL / OBJ / PLY, auto-detected by extension) and
    /// **prepare** it the way the cf-scan-prep Bevy tool does at load:
    /// scale into meters (`scale_to_m` = meters per file unit; `0.001` for
    /// a millimeter scan, `1.0` if already meters), then auto-center the
    /// AABB centroid onto the origin, then PCA-orient to the cast frame.
    ///
    /// Order matters: centering first means the PCA rotation (which pivots
    /// about the origin) pivots about the centroid — matching the Bevy
    /// tool's `try_load_scan` exactly, so the two frontends start from an
    /// identical prepared mesh. The prepared mesh becomes both the reset
    /// target and the working mesh; the auto-center offset + PCA quaternion
    /// are recorded for the `[scan_prep]` provenance block.
    ///
    /// # Errors
    /// - [`EngineError::ScanLoad`] if the file is missing or unparseable.
    /// - [`EngineError::EmptyScan`] if it parses but has no geometry.
    pub fn load(path: &Path, scale_to_m: f64) -> Result<Self> {
        let mut mesh =
            cortenforge::mesh::io::load_mesh(path).map_err(|e| EngineError::ScanLoad {
                path: path.display().to_string(),
                reason: e.to_string(),
            })?;
        if mesh.vertices.is_empty() || mesh.faces.is_empty() {
            return Err(EngineError::EmptyScan {
                path: path.display().to_string(),
            });
        }
        if (scale_to_m - 1.0).abs() > f64::EPSILON {
            cf_scan_prep_core::scale_vertices_in_place(&mut mesh, scale_to_m);
        }
        // Prepare exactly as the Bevy tool's try_load_scan: center, then
        // orient (centering first so PCA's origin-pivot == centroid-pivot).
        let auto_center_offset_m = cf_scan_prep_core::auto_center_in_place(&mut mesh);
        let auto_pca_quat = cf_scan_prep_core::auto_pca_in_place(&mut mesh);

        let mut session = Self::from_mesh(path.to_path_buf(), mesh);
        session.auto_center_offset_m = auto_center_offset_m;
        session.auto_pca_quat = auto_pca_quat;
        Ok(session)
    }

    /// Build a session from an in-memory mesh already in meters (used by
    /// frontends that loaded the mesh themselves, and by tests).
    #[must_use]
    pub fn from_mesh(source_path: PathBuf, mesh: IndexedMesh) -> Self {
        let original_face_count = mesh.faces.len();
        Self {
            source_path,
            original: mesh.clone(),
            working: mesh,
            original_face_count,
            simplify_applied: false,
            simplify_target: 0,
            auto_center_offset_m: Vector3::zeros(),
            auto_pca_quat: None,
            cap_loops: Vec::new(),
            centerline: Vec::new(),
            reorient_rotation: UnitQuaternion::identity(),
            trim_tip_mm: 0.0,
            trim_floor_mm: 0.0,
            reconstruct: None,
        }
    }

    /// The current working mesh — what the viewport renders.
    #[must_use]
    pub fn working(&self) -> &IndexedMesh {
        &self.working
    }

    /// The scan file this session was loaded from.
    #[must_use]
    pub fn source_path(&self) -> &Path {
        &self.source_path
    }

    /// Working-mesh triangle count.
    #[must_use]
    pub fn face_count(&self) -> usize {
        self.working.faces.len()
    }

    /// Working-mesh vertex count.
    #[must_use]
    pub fn vertex_count(&self) -> usize {
        self.working.vertices.len()
    }

    /// Face count of the originally-loaded scan.
    #[must_use]
    pub fn original_face_count(&self) -> usize {
        self.original_face_count
    }

    /// Whether a simplify op has been applied to the working mesh.
    #[must_use]
    pub fn simplify_applied(&self) -> bool {
        self.simplify_applied
    }

    /// Axis-aligned bounding box of the working mesh.
    #[must_use]
    pub fn aabb(&self) -> Aabb {
        self.working.aabb()
    }

    /// The auto-center offset applied at load, in meters — the
    /// `[scan_prep].auto_center_offset_m` provenance.
    #[must_use]
    pub fn auto_center_offset_m(&self) -> Vector3<f64> {
        self.auto_center_offset_m
    }

    /// The accumulated PCA-orient rotation, if any — the
    /// `[scan_prep].auto_pca_quaternion` provenance.
    #[must_use]
    pub fn auto_pca_quat(&self) -> Option<UnitQuaternion<f64>> {
        self.auto_pca_quat
    }

    /// The cap loops found by the last [`detect_caps`](Self::detect_caps).
    #[must_use]
    pub fn cap_loops(&self) -> &[DetectedCapLoop] {
        &self.cap_loops
    }

    /// The interior centerline from the last [`detect_caps`](Self::detect_caps)
    /// (empty until one is run, or if no boundary loop was found).
    #[must_use]
    pub fn centerline(&self) -> &[Point3<f64>] {
        &self.centerline
    }

    /// Whether a centerline is available (needed for leveling + trim).
    #[must_use]
    pub fn has_centerline(&self) -> bool {
        !self.centerline.is_empty()
    }

    /// Total centerline arc length in mm (0 if no centerline) — the upper
    /// bound a frontend should clamp trim sliders to.
    #[must_use]
    pub fn centerline_arc_length_mm(&self) -> f64 {
        cf_scan_prep_core::polyline_arc_length_m(&self.centerline) * 1000.0
    }

    /// The centerline as the viewport should draw it: trimmed to match the
    /// displayed mesh, then baked through the same reorient (about the
    /// working centroid) as [`display_mesh`] — so the overlaid line tracks
    /// the rendered geometry. Empty until [`detect_caps`] runs.
    ///
    /// [`display_mesh`]: Self::display_mesh
    /// [`detect_caps`]: Self::detect_caps
    #[must_use]
    pub fn display_centerline(&self) -> Vec<Point3<f64>> {
        if self.centerline.len() < 2 {
            return Vec::new();
        }
        let trimmed = if self.trim_tip_mm > 0.0 || self.trim_floor_mm > 0.0 {
            cf_scan_prep_core::trim_centerline_polyline(
                &self.centerline,
                self.trim_tip_mm,
                self.trim_floor_mm,
            )
        } else {
            self.centerline.clone()
        };
        if self.reorient_rotation == UnitQuaternion::identity() {
            return trimmed;
        }
        let pivot = self.working.aabb().center();
        trimmed
            .iter()
            .map(|p| {
                cf_scan_prep_core::bake_vertex_with_pivot(
                    p,
                    self.reorient_rotation,
                    &pivot,
                    Vector3::zeros(),
                )
            })
            .collect()
    }

    /// Detect open-boundary cap loops + compute the interior centerline —
    /// the "Cap → Scan" step, mirroring the cf-scan-prep tool's
    /// `handle_cap_actions`: keep only valid loops, fit each loop's plane,
    /// then trace the cross-section centerline along the first loop's
    /// outward normal (smoothed). Replaces any previous cap/centerline
    /// state. Returns a [`CapScan`] summary.
    pub fn detect_caps(&mut self) -> CapScan {
        let raw_loops = cf_scan_prep_core::detect_boundary_loops(&self.working);
        let valid: Vec<_> = raw_loops
            .iter()
            .filter(|loop_data| loop_data.is_valid())
            .collect();

        // Raw unwelded soup → thousands of per-triangle "loops". Detecting
        // them is cheap, but fitting a plane to each + tracing a meaningless
        // centerline is not — and the result is useless until the user
        // welds. Short-circuit with the "weld first" flag instead (the
        // cf-scan-prep tool builds them anyway then warns; skipping the
        // wasted work is strictly better for a guided wizard). The
        // loop-count gate avoids false-positives on tiny meshes, where the
        // `v >= 2f` heuristic alone trips.
        if cf_scan_prep_core::mesh_looks_unwelded(
            self.working.vertices.len(),
            self.working.faces.len(),
        ) && valid.len() > UNWELDED_LOOP_WARN
        {
            self.clear_caps();
            return CapScan {
                loop_count: 0,
                centerline_segments: 0,
                looks_unwelded: true,
            };
        }

        self.cap_loops = valid
            .iter()
            .map(|&loop_data| cf_scan_prep_core::build_detected_cap_loop(&self.working, loop_data))
            .collect();
        self.centerline = match self.cap_loops.first() {
            Some(first) => {
                let raw = cf_scan_prep_core::compute_centerline_polyline(
                    &self.working,
                    first.plane_normal,
                    CENTERLINE_SLICES,
                );
                cf_scan_prep_core::smooth_polyline(&raw, CENTERLINE_SMOOTH_ITERS)
            }
            None => Vec::new(),
        };
        CapScan {
            loop_count: self.cap_loops.len(),
            centerline_segments: self.centerline.len().saturating_sub(1),
            looks_unwelded: false,
        }
    }

    /// Drop ALL centerline-derived state — cap loops, the centerline, and
    /// the centerline-relative trim + reconstruction — called by every
    /// mesh-mutating op. A mesh change invalidates the old centerline, so
    /// the trim (defined as arc-distances along it) is stale too; clearing
    /// it keeps the engine state and the frontend's trim controls in sync
    /// (re-run `detect_caps` + re-trim against the current geometry).
    fn clear_caps(&mut self) {
        self.cap_loops.clear();
        self.centerline.clear();
        self.trim_tip_mm = 0.0;
        self.trim_floor_mm = 0.0;
        self.reconstruct = None;
    }

    /// The unbaked reorient rotation (e.g. from [`level_to_floor`]). Apply
    /// it to the display; bake it into the cleaned mesh only at save.
    ///
    /// [`level_to_floor`]: Self::level_to_floor
    #[must_use]
    pub fn reorient_rotation(&self) -> UnitQuaternion<f64> {
        self.reorient_rotation
    }

    /// Level the scan onto its floor: set the (unbaked) reorient rotation so
    /// the floor cap's plane becomes horizontal, standing the piece upright.
    /// Mirrors the cf-scan-prep tool's `auto_level_to_floor` cap-loop path —
    /// it aligns the floor normal to the NEAREST Z pole (shortest rotation,
    /// so the piece isn't flipped). The reconstructed / predicted-cut floor
    /// paths arrive with trim in Phase 3.
    ///
    /// Requires a centerline + at least one cap loop (run [`detect_caps`]
    /// first), matching the Bevy tool's gate. Returns the corrected tilt in
    /// degrees, or `None` if there's nothing to level to.
    ///
    /// [`detect_caps`]: Self::detect_caps
    pub fn level_to_floor(&mut self) -> Option<f64> {
        if self.centerline.is_empty() {
            return None;
        }
        let floor_normal = self.floor_normal()?;
        let rotation = floor_leveling_rotation(floor_normal)?;
        self.reorient_rotation = rotation;
        let tilt_deg = floor_normal.z.abs().clamp(0.0, 1.0).acos().to_degrees();
        Some(tilt_deg)
    }

    /// The floor-plane normal to level by, in priority order (mirrors the
    /// cf-scan-prep tool's `auto_level_to_floor`): (1) the reconstructed /
    /// predicted-cut floor when a floor trim is applied — the plane the
    /// device actually seats on; (2) failing that, the centerline tangent
    /// at the predicted floor cut; (3) the raw detected cap-loop normal.
    fn floor_normal(&self) -> Option<Vector3<f64>> {
        // (1) the reconstructed floor plane (valid only with a floor trim).
        if let Some(plane) = cf_scan_prep_core::compute_reconstructed_floor_plane_physics(
            &self.centerline,
            self.trim_tip_mm,
            self.trim_floor_mm,
        ) {
            return Some(plane.normal.normalize());
        }
        // (2) the centerline tangent at the predicted floor cut.
        if self.centerline.len() >= 2 && self.trim_floor_mm > 0.0 {
            let total_m = cf_scan_prep_core::polyline_arc_length_m(&self.centerline);
            let floor_cut_m = (total_m - self.trim_floor_mm * 0.001).max(0.0);
            if let Some((_, tangent)) = cf_scan_prep_core::point_along_polyline_at_arc_distance(
                &self.centerline,
                floor_cut_m,
            ) {
                return Some(tangent.normalize());
            }
        }
        // (3) the raw detected cap-loop normal.
        self.cap_loops.first().map(|l| l.plane_normal.normalize())
    }

    /// Applied tip-end centerline trim, mm (0 = none).
    #[must_use]
    pub fn trim_tip_mm(&self) -> f64 {
        self.trim_tip_mm
    }

    /// Applied floor-end centerline trim, mm (0 = none).
    #[must_use]
    pub fn trim_floor_mm(&self) -> f64 {
        self.trim_floor_mm
    }

    /// The applied floor reconstruction, if any.
    #[must_use]
    pub fn reconstruct(&self) -> Option<AppliedReconstruct> {
        self.reconstruct
    }

    /// Whether floor reconstruction is available — it needs a committed
    /// floor trim to reconstruct down to.
    #[must_use]
    pub fn reconstruct_available(&self) -> bool {
        self.trim_floor_mm > 0.0
    }

    /// Set the applied centerline trim (mm from each end; negatives clamp
    /// to 0). Trim is a derived op — it chops the displayed/saved mesh
    /// along the centerline, never `working`. Changing the floor trim
    /// drops any reconstruction (it was fit to the old cut) — the
    /// directional-workflow trip-wire from the Bevy tool.
    pub fn apply_trim(&mut self, tip_mm: f64, floor_mm: f64) {
        let floor_mm = floor_mm.max(0.0);
        if (floor_mm - self.trim_floor_mm).abs() > f64::EPSILON {
            self.reconstruct = None;
        }
        self.trim_tip_mm = tip_mm.max(0.0);
        self.trim_floor_mm = floor_mm;
    }

    /// Apply floor reconstruction (`reference_mm` = the zone above the cut
    /// to sample the cross-section from; `shape` = constant/taper/
    /// extrapolate). No-op returning `false` if there's no floor trim to
    /// reconstruct down to.
    pub fn apply_reconstruct(&mut self, reference_mm: f64, shape: ReconstructShape) -> bool {
        if !self.reconstruct_available() {
            return false;
        }
        self.reconstruct = Some(AppliedReconstruct {
            reference_mm: reference_mm.max(0.0),
            shape,
        });
        true
    }

    /// The working mesh with the derived ops applied — centerline trim +
    /// floor reconstruction (or flat cap of the cut) — but NOT the reorient
    /// (that's display-only, applied by [`display_mesh`]). Mirrors the
    /// cf-scan-prep tool's trim → weld → reconstruct/auto-cap composition.
    /// Returns a clone of `working` when no trim is applied (or there's no
    /// centerline to trim along).
    ///
    /// [`display_mesh`]: Self::display_mesh
    #[must_use]
    fn processed_mesh(&self) -> IndexedMesh {
        if self.centerline.len() < 2 || (self.trim_tip_mm <= 0.0 && self.trim_floor_mm <= 0.0) {
            return self.working.clone();
        }
        let mut trimmed = cf_scan_prep_core::trim_mesh_along_centerline(
            &self.working,
            &self.centerline,
            self.trim_tip_mm,
            self.trim_floor_mm,
        );
        // Weld the cut boundary's duplicate intersection vertices so the cut
        // forms ONE closed loop (not per-edge fragments) — the same fix the
        // Bevy tool applies before reconstruct / auto-cap.
        weld_vertices(&mut trimmed, WELD_EPSILON_M);
        match self.reconstruct {
            Some(ar) if self.trim_floor_mm > 0.0 => {
                let trimmed_centerline = cf_scan_prep_core::trim_centerline_polyline(
                    &self.centerline,
                    self.trim_tip_mm,
                    self.trim_floor_mm,
                );
                cf_scan_prep_core::apply_reconstruction(
                    trimmed,
                    &trimmed_centerline,
                    self.trim_floor_mm,
                    ar.reference_mm,
                    ar.shape,
                )
            }
            _ => {
                cf_scan_prep_core::auto_cap_open_boundaries(&mut trimmed);
                trimmed
            }
        }
    }

    /// What the viewport should render: the processed mesh (trim +
    /// reconstruct) with the unbaked reorient applied, pivoted about the
    /// UN-trimmed working centroid — the exact pivot the save bake
    /// (`build_cleaned_mesh`) uses, so display == what gets written. The
    /// reorient is baked into the cleaned mesh only at save; this is
    /// display-only.
    #[must_use]
    pub fn display_mesh(&self) -> IndexedMesh {
        let mut out = self.processed_mesh();
        if self.reorient_rotation != UnitQuaternion::identity() {
            let pivot = self.working.aabb().center();
            for v in &mut out.vertices {
                *v = cf_scan_prep_core::bake_vertex_with_pivot(
                    v,
                    self.reorient_rotation,
                    &pivot,
                    Vector3::zeros(),
                );
            }
        }
        out
    }

    /// Write the cleaned scan + `.prep.toml` to `output_dir` (named
    /// `{stem}.cleaned.stl` / `{stem}.prep.toml`) — the cleanup output the
    /// rest of the wizard (and the cast pipeline) consumes. Mirrors the
    /// cf-scan-prep tool's `handle_save_action`: bake the reorient + cap the
    /// detected loops (`build_cleaned_mesh`), trim + reconstruct/flat-cap
    /// along the baked centerline, run the disk-cleanup pass (incl.
    /// `smoothing_iters` Taubin smoothing), then emit the provenance TOML.
    /// Studio has no manual recenter, so the baked translation is zero.
    /// `stl_units_label` is recorded as provenance only.
    ///
    /// # Errors
    /// [`EngineError::Save`] on a non-finite transform, TOML serialization,
    /// or the atomic file write.
    pub fn save(
        &self,
        output_dir: &Path,
        stem: &str,
        stl_units_label: &'static str,
        smoothing_iters: usize,
    ) -> Result<SaveReport> {
        // The cast needs the centerline (cf-cap-planes / curve-following);
        // without it `build_cleaned_mesh` caps nothing → an open base, and
        // the prep.toml has no `[centerline]` → `accept_prep` would reject
        // it. Refuse here so we never write an invalid cast input.
        if self.centerline.len() < 2 {
            return Err(EngineError::Save(
                "no centerline yet — run cap detection (Find floor) first; the cast needs it"
                    .into(),
            ));
        }
        let rotation = self.reorient_rotation;
        let translation = Vector3::zeros();
        if !rotation.into_inner().coords.iter().all(|c| c.is_finite())
            || !self.trim_tip_mm.is_finite()
            || !self.trim_floor_mm.is_finite()
        {
            return Err(EngineError::Save(
                "non-finite transform / trim values".into(),
            ));
        }
        let pivot = self.working.aabb().center();

        // 1. Bake the reorient + cap the detected loops.
        let mut cleaned = cf_scan_prep_core::build_cleaned_mesh(
            &self.working,
            rotation,
            translation,
            &self.cap_loops,
        );

        // 2. Trim (+ reconstruct / flat-cap) in the baked world frame —
        //    bake the centerline through the same transform first.
        // ⚠ No `centerline.len() >= 2` here: the refusal at the top of `save`
        // already returned for that, and `&self` cannot have changed since. The
        // conjunct was always true — it read as a guard and gated nothing.
        let mut trim_capped = 0_usize;
        if self.trim_tip_mm > 0.0 || self.trim_floor_mm > 0.0 {
            let centerline_world: Vec<Point3<f64>> = self
                .centerline
                .iter()
                .map(|p| {
                    cf_scan_prep_core::bake_vertex_with_pivot(p, rotation, &pivot, translation)
                })
                .collect();
            cleaned = cf_scan_prep_core::trim_mesh_along_centerline(
                &cleaned,
                &centerline_world,
                self.trim_tip_mm,
                self.trim_floor_mm,
            );
            weld_vertices(&mut cleaned, WELD_EPSILON_M);
            match self.reconstruct {
                Some(ar) if self.trim_floor_mm > 0.0 => {
                    let trimmed_centerline = cf_scan_prep_core::trim_centerline_polyline(
                        &centerline_world,
                        self.trim_tip_mm,
                        self.trim_floor_mm,
                    );
                    cleaned = cf_scan_prep_core::apply_reconstruction(
                        cleaned,
                        &trimmed_centerline,
                        self.trim_floor_mm,
                        ar.reference_mm,
                        ar.shape,
                    );
                }
                _ => trim_capped = cf_scan_prep_core::auto_cap_open_boundaries(&mut cleaned),
            }
        }

        // 3. Disk-cleanup pass (incl. Taubin surface smoothing).
        cf_scan_prep_core::cleanup_cleaned_mesh_for_disk(&mut cleaned, smoothing_iters);

        // 4. Provenance TOML.
        let cleaned_stl_name = format!("{stem}.cleaned.stl");
        let cleaned_aabb = cleaned.aabb();
        let reconstructed_floor = cf_scan_prep_core::compute_reconstructed_floor_plane_physics(
            &self.centerline,
            self.trim_tip_mm,
            self.trim_floor_mm,
        );
        let euler = rotation.euler_angles();
        let toml = cf_scan_prep_core::build_prep_toml_string(
            &self.source_path,
            stl_units_label,
            self.auto_center_offset_m,
            self.auto_pca_quat,
            rotation,
            [
                euler.0.to_degrees(),
                euler.1.to_degrees(),
                euler.2.to_degrees(),
            ],
            translation,
            &self.centerline,
            &self.cap_loops,
            self.trim_tip_mm,
            self.trim_floor_mm,
            self.reconstruct,
            trim_capped,
            &cleaned_stl_name,
            rotation,
            translation,
            pivot,
            self.simplify_target,
            self.simplify_applied,
            self.original_face_count,
            cleaned.faces.len(),
            smoothing_iters,
            &cleaned_aabb,
            reconstructed_floor,
        )
        .map_err(|e| EngineError::Save(e.to_string()))?;

        // 5. Atomic write.
        let cleaned_stl = output_dir.join(&cleaned_stl_name);
        let prep_toml = output_dir.join(format!("{stem}.prep.toml"));
        cf_scan_prep_core::atomic_write_save(&cleaned, &cleaned_stl, &prep_toml, &toml)
            .map_err(|e| EngineError::Save(e.to_string()))?;

        Ok(SaveReport {
            cleaned_stl,
            prep_toml,
            face_count: cleaned.faces.len(),
        })
    }

    /// Restore the working mesh to the originally-loaded scan and clear
    /// all applied ops + accumulated transform provenance.
    pub fn reset(&mut self) {
        self.working = self.original.clone();
        self.simplify_applied = false;
        self.simplify_target = 0;
        self.auto_center_offset_m = Vector3::zeros();
        self.auto_pca_quat = None;
        self.reorient_rotation = UnitQuaternion::identity();
        // trim + reconstruct are cleared by clear_caps (centerline-derived).
        self.clear_caps();
    }

    /// Weld coincident vertices then drop the unreferenced ones. Raw STL
    /// is a vertex soup (one vertex set per triangle); welding shares
    /// indices so downstream ops see real adjacency. Returns
    /// `(vertices_before, vertices_after)`.
    pub fn weld(&mut self) -> (usize, usize) {
        let before = self.working.vertices.len();
        weld_vertices(&mut self.working, WELD_EPSILON_M);
        remove_unreferenced_vertices(&mut self.working);
        self.clear_caps();
        (before, self.working.vertices.len())
    }

    /// Decimate toward `target_faces` (boundary-preserving quadric edge
    /// collapse). A no-op if the mesh already has `<= target_faces`.
    /// Returns the decimation wall-clock seconds. Records `[simplify]`
    /// provenance. Synchronous — for headless callers; a GUI should run the
    /// heavy work off-thread via [`run_simplify`] + [`apply_simplified`].
    ///
    /// [`apply_simplified`]: Self::apply_simplified
    pub fn simplify(&mut self, target_faces: usize) -> f64 {
        let (mesh, secs) = run_simplify(&self.working, target_faces);
        self.apply_simplified(mesh, target_faces);
        secs
    }

    /// A clone of the current working mesh — what a frontend hands to a
    /// background [`run_simplify`] thread.
    #[must_use]
    pub fn working_clone(&self) -> IndexedMesh {
        self.working.clone()
    }

    /// Install a simplify result computed off-thread (via [`run_simplify`]
    /// on a clone of [`working_clone`]). Records the `[simplify]` provenance
    /// and clears stale cap/centerline/trim state, exactly like the
    /// synchronous [`simplify`].
    ///
    /// [`run_simplify`]: run_simplify
    /// [`working_clone`]: Self::working_clone
    /// [`simplify`]: Self::simplify
    pub fn apply_simplified(&mut self, simplified: IndexedMesh, target_faces: usize) {
        self.working = simplified;
        self.simplify_applied = true;
        self.simplify_target = target_faces;
        self.clear_caps();
    }
}

/// Run the heavy boundary-preserving decimation toward `target_faces`,
/// detached from any [`EditSession`] — so a frontend can run it on a
/// background thread (the working mesh is `Send`) and hand the result to
/// [`EditSession::apply_simplified`], keeping the UI responsive. Returns the
/// decimated mesh + the wall-clock seconds. No-op above the current face
/// count.
#[must_use]
pub fn run_simplify(original: &IndexedMesh, target_faces: usize) -> (IndexedMesh, f64) {
    let result = cf_scan_prep_core::simplify_mesh(original, target_faces);
    (result.mesh, result.elapsed_secs)
}

/// The rotation that levels a floor plane: aligns `normal` to the nearest
/// Z pole (`+Z` if it already points up-ish, else `−Z`) via the shortest
/// rotation. Mirrors the cf-scan-prep tool's `leveling_euler_deg` (in
/// quaternion form). `None` only for an unnormalizable input.
fn floor_leveling_rotation(normal: Vector3<f64>) -> Option<UnitQuaternion<f64>> {
    let n = normal.normalize();
    let target = if n.z >= 0.0 {
        Vector3::z()
    } else {
        -Vector3::z()
    };
    UnitQuaternion::rotation_between(&n, &target)
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::expect_used)]

    use cortenforge::mesh::types::Point3;

    use super::*;

    /// Two triangles sharing an edge, but stored as vertex soup (6 vertex
    /// slots, two pairs coincident) — what a raw STL looks like.
    fn soup_quad() -> IndexedMesh {
        IndexedMesh {
            vertices: vec![
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(1.0, 0.0, 0.0),
                Point3::new(0.0, 1.0, 0.0),
                Point3::new(1.0, 0.0, 0.0), // dup of #1
                Point3::new(1.0, 1.0, 0.0),
                Point3::new(0.0, 1.0, 0.0), // dup of #2
            ],
            faces: vec![[0, 1, 2], [3, 4, 5]],
        }
    }

    /// A welded quad (4 shared vertices, 2 triangles) — its perimeter is a
    /// single open-boundary loop.
    fn open_quad() -> IndexedMesh {
        IndexedMesh {
            vertices: vec![
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(1.0, 0.0, 0.0),
                Point3::new(1.0, 1.0, 0.0),
                Point3::new(0.0, 1.0, 0.0),
            ],
            faces: vec![[0, 1, 2], [0, 2, 3]],
        }
    }

    /// A closed (watertight) tetrahedron — every edge is shared by two
    /// faces, so it has no open boundary.
    fn tetra() -> IndexedMesh {
        IndexedMesh {
            vertices: vec![
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(1.0, 0.0, 0.0),
                Point3::new(0.0, 1.0, 0.0),
                Point3::new(0.0, 0.0, 1.0),
            ],
            faces: vec![[0, 2, 1], [0, 1, 3], [0, 3, 2], [1, 2, 3]],
        }
    }

    /// `n` disjoint triangles as vertex soup (3 unshared vertices each) —
    /// what a raw, unwelded STL of `n` faces looks like to boundary
    /// detection: every triangle is its own loop.
    fn soup_many(n: usize) -> IndexedMesh {
        let mut vertices = Vec::with_capacity(n * 3);
        let mut faces = Vec::with_capacity(n);
        for i in 0..n {
            let x = i as f64;
            let base = (i * 3) as u32;
            vertices.push(Point3::new(x, 0.0, 0.0));
            vertices.push(Point3::new(x + 0.5, 0.0, 0.0));
            vertices.push(Point3::new(x, 0.5, 0.0));
            faces.push([base, base + 1, base + 2]);
        }
        IndexedMesh { vertices, faces }
    }

    /// A welded square tube along +Z (`rings` levels, open at both ends),
    /// then tilted `tilt_rad` about X — two open-boundary loops + a spine,
    /// with the floor planes tilted off horizontal.
    fn open_tube(rings: usize, tilt_rad: f64) -> IndexedMesh {
        let mut vertices = Vec::new();
        for r in 0..rings {
            let z = r as f64 / (rings - 1) as f64;
            vertices.push(Point3::new(0.0, 0.0, z));
            vertices.push(Point3::new(1.0, 0.0, z));
            vertices.push(Point3::new(1.0, 1.0, z));
            vertices.push(Point3::new(0.0, 1.0, z));
        }
        let mut faces = Vec::new();
        for r in 0..rings - 1 {
            let b = (r * 4) as u32;
            let t = ((r + 1) * 4) as u32;
            for k in 0..4u32 {
                let k2 = (k + 1) % 4;
                faces.push([b + k, b + k2, t + k2]);
                faces.push([b + k, t + k2, t + k]);
            }
        }
        let q = UnitQuaternion::from_axis_angle(&Vector3::x_axis(), tilt_rad);
        for v in &mut vertices {
            *v = q.transform_point(v);
        }
        IndexedMesh { vertices, faces }
    }

    /// `mesh` scaled about the origin — the same shape at a second size, so a
    /// reported length can be checked for LINEARITY rather than against a
    /// re-implementation of the length maths (which would be a mirror).
    fn scaled(mesh: IndexedMesh, k: f64) -> IndexedMesh {
        IndexedMesh {
            vertices: mesh
                .vertices
                .iter()
                .map(|p| Point3::new(p.x * k, p.y * k, p.z * k))
                .collect(),
            faces: mesh.faces,
        }
    }

    /// A mesh translated far from the origin (for the auto-center test).
    fn offset_tri() -> IndexedMesh {
        IndexedMesh {
            vertices: vec![
                Point3::new(10.0, 10.0, 10.0),
                Point3::new(11.0, 10.0, 10.0),
                Point3::new(10.0, 11.0, 10.0),
            ],
            faces: vec![[0, 1, 2]],
        }
    }

    fn session(mesh: IndexedMesh) -> EditSession {
        EditSession::from_mesh(PathBuf::from("/tmp/scan.stl"), mesh)
    }

    #[test]
    fn from_mesh_reports_counts() {
        let s = session(soup_quad());
        assert_eq!(s.vertex_count(), 6);
        assert_eq!(s.face_count(), 2);
        assert_eq!(s.original_face_count(), 2);
        assert!(!s.simplify_applied());
    }

    #[test]
    fn weld_collapses_coincident_vertices() {
        let mut s = session(soup_quad());
        let (before, after) = s.weld();
        assert_eq!(before, 6);
        assert_eq!(after, 4, "the two shared-edge vertices weld together");
        assert_eq!(s.face_count(), 2, "faces unchanged by welding");
    }

    #[test]
    fn reset_restores_the_original_mesh() {
        let mut s = session(offset_tri());
        // Land an origin-centred mesh the way a finished background simplify
        // does — `reset` has to undo both the mesh and the flag.
        s.apply_simplified(
            IndexedMesh {
                vertices: vec![
                    Point3::new(-1.0, 0.0, 0.0),
                    Point3::new(1.0, 0.0, 0.0),
                    Point3::new(0.0, 1.0, 0.0),
                ],
                faces: vec![[0, 1, 2]],
            },
            1,
        );
        assert!(s.aabb().center().coords.norm() < 1.0);
        assert!(
            s.simplify_applied(),
            "the edit is in place before the reset"
        );

        s.reset();

        // Back near (10.5, 10.5, 10) — the original AABB centre.
        assert!(
            s.aabb().center().coords.norm() > 1.0,
            "reset restored the original mesh"
        );
        assert!(!s.simplify_applied());
    }

    #[test]
    fn simplify_noop_when_target_exceeds_face_count_but_marks_applied() {
        let mut s = session(soup_quad());
        let _ = s.simplify(1000); // target >> 2 faces
        assert_eq!(s.face_count(), 2, "no decimation when target exceeds faces");
        assert!(s.simplify_applied(), "the op still records as applied");
    }

    /// `load` prepares the scan like the Bevy tool: an off-origin scan
    /// comes back auto-centered, with the offset recorded for provenance.
    #[test]
    fn load_auto_centers_and_records_both_provenance_values() {
        // An off-center triangle (~(10, 10, 10)) as a tiny ASCII STL.
        let stl = "solid s\n\
            facet normal 0 0 1\n\
            outer loop\n\
            vertex 10 10 10\n\
            vertex 11 10 10\n\
            vertex 10 11 10\n\
            endloop\n\
            endfacet\n\
            endsolid s\n";
        let dir = std::env::temp_dir().join(format!("cf-edit-load-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let path = dir.join("offset.stl");
        std::fs::write(&path, stl).unwrap();

        let s = EditSession::load(&path, 1.0).unwrap(); // already "meters"
        // After centering (then a PCA rotation about the origin), the
        // mesh sits near the origin — not out at its original ~(10,10,10).
        // (The bbox center isn't exactly 0 because PCA rotates the AABB.)
        assert!(
            s.aabb().center().coords.norm() < 1.0,
            "load brings the off-origin scan to near the origin, got {:?}",
            s.aabb().center(),
        );
        assert!(
            s.auto_center_offset_m().norm() > 1.0,
            "the non-trivial centering offset (~17) is recorded for provenance",
        );
        // ⚠ The other half of the same provenance pair. `load` records both
        // side by side and `save` writes both into `[scan_prep]`, but only the
        // offset was gated — so a `load` that dropped the rotation was silent.
        // The verdict is "a real rotation was recorded", not its exact value:
        // pinning the quaternion would gate cf-scan-prep-core's PCA, not this.
        let pca = s
            .auto_pca_quat()
            .expect("a flat triangle has a dominant axis, so PCA is not degenerate");
        assert!(
            pca.angle() > 0.1,
            "the non-trivial PCA rotation (~90 deg) is recorded beside the offset, got {pca:?}",
        );

        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn detect_caps_finds_the_open_boundary_loop() {
        let mut s = session(open_quad());
        let scan = s.detect_caps();
        assert!(
            scan.loop_count >= 1,
            "the open quad's perimeter is a boundary loop"
        );
        assert_eq!(s.cap_loops().len(), scan.loop_count);
        assert!(!scan.looks_unwelded, "a single-loop mesh isn't soup");
    }

    #[test]
    fn detect_caps_on_a_closed_mesh_has_no_loops_or_centerline() {
        let mut s = session(tetra());
        let scan = s.detect_caps();
        assert_eq!(
            scan.loop_count, 0,
            "a closed tetrahedron has no open boundary"
        );
        assert!(s.centerline().is_empty());
        assert!(!s.has_centerline());
        assert_eq!(scan.centerline_segments, 0);
    }

    /// The `valid.len() > UNWELDED_LOOP_WARN` boundary, at exactly the
    /// constant.
    ///
    /// ⚠ `detect_caps_short_circuits_on_unwelded_soup` uses 101 loops — above
    /// the bound, where `>` and `>=` agree, so `>` -> `>=` survived it. 100 is
    /// the only count that separates them: the soup still looks unwelded
    /// (300 vertices >= 2 x 100 faces), but the loop count has not passed the
    /// warn threshold, so the scan must proceed rather than short-circuit.
    #[test]
    fn a_soup_at_exactly_the_warn_threshold_is_still_scanned() {
        let mesh = soup_many(UNWELDED_LOOP_WARN);
        // ⚠ Without this the gate goes VACUOUS if the heuristic stops firing:
        // `&&` would short-circuit on its first operand and the loop-count
        // comparison this test exists for would never be evaluated.
        assert!(
            cf_scan_prep_core::mesh_looks_unwelded(mesh.vertices.len(), mesh.faces.len()),
            "the soup does look unwelded, so only the loop count decides"
        );

        let mut s = session(mesh);
        let scan = s.detect_caps();
        assert!(
            !scan.looks_unwelded,
            "at exactly {UNWELDED_LOOP_WARN} loops the threshold is not passed"
        );
        assert_eq!(
            scan.loop_count, UNWELDED_LOOP_WARN,
            "so every loop is built rather than discarded"
        );
    }

    #[test]
    fn detect_caps_short_circuits_on_unwelded_soup() {
        // 101 disjoint triangles: unwelded (303 ≥ 2·101) AND > 100 loops.
        let mut s = session(soup_many(101));
        let scan = s.detect_caps();
        assert!(scan.looks_unwelded, "raw soup is flagged for welding");
        assert_eq!(
            scan.loop_count, 0,
            "no cap loops built from soup — weld first"
        );
        assert!(s.cap_loops().is_empty());
        assert!(s.centerline().is_empty());
    }

    #[test]
    fn display_centerline_empty_until_caps_then_populated() {
        let mut s = session(open_tube(6, 20f64.to_radians()));
        assert!(
            s.display_centerline().is_empty(),
            "no centerline to draw before detect_caps"
        );
        s.detect_caps();
        assert!(
            s.display_centerline().len() >= 2,
            "the centerline is drawable after detect_caps"
        );
    }

    #[test]
    fn a_mesh_edit_clears_stale_caps() {
        let mut s = session(open_quad());
        s.detect_caps();
        assert!(!s.cap_loops().is_empty(), "caps detected on the quad");
        s.reset();
        assert!(
            s.cap_loops().is_empty(),
            "reset drops the now-stale cap loops"
        );
        assert!(s.centerline().is_empty());
    }

    #[test]
    fn leveling_rotation_brings_a_tilted_normal_vertical() {
        // ⚠ Both poles, and the SIGN — not `leveled.z.abs()`. Levelling to the
        // FAR pole is also "vertical", and it stands the scan on its head; the
        // previous gate could not tell the two apart, so a flipped target and a
        // dropped negation both passed it. `rotation_between` takes the short
        // way round, so each normal must land on the pole it started nearest.
        let tilt = UnitQuaternion::from_axis_angle(&Vector3::x_axis(), 30f64.to_radians());

        let up = tilt * Vector3::z();
        assert!(up.z > 0.0, "fixture: this normal points up");
        let leveled = floor_leveling_rotation(up).unwrap() * up;
        assert!(
            (leveled.z - 1.0).abs() < 1e-9,
            "an up-facing floor levels to +Z, not upside down: {leveled:?}"
        );

        let down = -up;
        assert!(down.z < 0.0, "fixture: this normal points down");
        let leveled = floor_leveling_rotation(down).unwrap() * down;
        assert!(
            (leveled.z + 1.0).abs() < 1e-9,
            "a down-facing floor levels to -Z, the pole it was nearer: {leveled:?}"
        );
    }

    #[test]
    fn level_to_floor_needs_a_centerline() {
        let mut s = session(open_quad()); // flat → no centerline
        s.detect_caps();
        assert!(
            s.level_to_floor().is_none(),
            "can't level without a centerline"
        );
        assert_eq!(s.reorient_rotation(), UnitQuaternion::identity());
    }

    #[test]
    fn level_to_floor_uprights_a_tilted_tube() {
        let mut s = session(open_tube(6, 20f64.to_radians()));
        let scan = s.detect_caps();
        assert!(scan.loop_count >= 2, "the tube has two open ends");
        assert!(s.has_centerline(), "the tube has a spine");
        let tilt = s.level_to_floor().expect("a tilted tube can be leveled");
        assert!(tilt > 5.0, "the ~20deg tilt is detected, got {tilt}");
        let n = s.cap_loops()[0].plane_normal.normalize();
        let leveled = s.reorient_rotation() * n;
        assert!(
            (leveled.z.abs() - 1.0).abs() < 1e-6,
            "floor normal is vertical after leveling: {leveled:?}"
        );
    }

    #[test]
    fn display_mesh_applies_the_reorient() {
        let mut s = session(open_tube(6, 20f64.to_radians()));
        assert_eq!(
            s.display_mesh().vertices,
            s.working().vertices,
            "display == working before leveling",
        );
        s.detect_caps();
        s.level_to_floor().expect("tube levels");
        assert_ne!(
            s.display_mesh().vertices,
            s.working().vertices,
            "display is rotated after leveling",
        );
    }

    #[test]
    fn apply_trim_changes_the_displayed_mesh() {
        let mut s = session(open_tube(6, 20f64.to_radians()));
        s.detect_caps();
        assert!(s.has_centerline());
        let before = s.working().faces.len();
        s.apply_trim(0.0, 300.0); // chop 300 mm off the ~1 m-tall tube's floor
        assert_ne!(
            s.display_mesh().faces.len(),
            before,
            "the floor trim chopped (+ re-capped) the mesh",
        );
    }

    #[test]
    fn reconstruct_needs_a_floor_trim() {
        let mut s = session(open_tube(6, 20f64.to_radians()));
        s.detect_caps();
        assert!(
            !s.apply_reconstruct(25.0, ReconstructShape::Constant),
            "no floor trim → reconstruct unavailable",
        );
        s.apply_trim(0.0, 200.0);
        assert!(
            s.apply_reconstruct(25.0, ReconstructShape::Constant),
            "floor trim → reconstruct available",
        );
        assert!(s.reconstruct().is_some());
    }

    #[test]
    fn changing_the_floor_trim_drops_reconstruct() {
        let mut s = session(open_tube(6, 20f64.to_radians()));
        s.detect_caps();
        s.apply_trim(0.0, 200.0);
        s.apply_reconstruct(25.0, ReconstructShape::Taper);
        assert!(s.reconstruct().is_some());
        s.apply_trim(0.0, 250.0); // floor cut moved → old reconstruction is stale
        assert!(
            s.reconstruct().is_none(),
            "moving the floor cut drops the now-stale reconstruction",
        );
    }

    #[test]
    fn a_mesh_edit_clears_pending_trim() {
        let mut s = session(open_tube(6, 20f64.to_radians()));
        s.detect_caps();
        s.apply_trim(0.0, 200.0);
        s.apply_reconstruct(25.0, ReconstructShape::Constant);
        assert_eq!(s.trim_floor_mm(), 200.0);
        s.weld(); // a mesh change invalidates the centerline-relative trim
        assert_eq!(s.trim_floor_mm(), 0.0, "welding drops the stale trim");
        assert!(s.reconstruct().is_none());
        assert!(!s.has_centerline());
    }

    #[test]
    fn reset_clears_trim_and_reconstruct() {
        let mut s = session(open_tube(6, 20f64.to_radians()));
        s.detect_caps();
        s.apply_trim(10.0, 200.0);
        s.apply_reconstruct(25.0, ReconstructShape::Constant);

        // ⚠ Assert the trims are SET before asserting `reset` clears them.
        // Checking only the zeros afterwards is satisfied by a getter that
        // always returns 0.0, which is exactly the mutant that survived.
        assert_eq!(s.trim_tip_mm(), 10.0, "the tip trim is in place");
        assert_eq!(s.trim_floor_mm(), 200.0, "and so is the floor trim");

        s.reset();
        assert_eq!(s.trim_tip_mm(), 0.0);
        assert_eq!(s.trim_floor_mm(), 0.0);
        assert!(s.reconstruct().is_none());
    }

    /// Integration probe: run the GUI's Save pipeline (load → weld →
    /// find-floor → save) on the real `~/scans/base_mold.stl` and write to a
    /// temp dir (never `~/scans`). Manual: `cargo test -p cf-studio-engine
    /// save_real_base_mold --ignored -- --nocapture`. Mirrors the base_mold
    /// mold-gen integration probe.
    #[test]
    #[ignore = "needs ~/scans/base_mold.stl; writes to a temp dir"]
    fn save_real_base_mold_to_tempdir() {
        let scan =
            std::path::PathBuf::from(std::env::var("HOME").unwrap()).join("scans/base_mold.stl");
        // ⚠ Panics rather than skipping, matching `mold.rs`'s
        // `isolated_base_mold_fixture`. This line set the precedent the mold
        // gates copied, and the precedent was wrong: an `#[ignore]`d gate is
        // only ever run deliberately, so a missing fixture means the requested
        // run cannot happen — reporting green is worse than failing.
        assert!(
            scan.exists(),
            "MISSING FIXTURE: {} is required by this #[ignore]d gate. It runs \
             only when explicitly asked for, so this is an error, not a skip.",
            scan.display(),
        );
        let mut s = EditSession::load(&scan, 0.001).unwrap();
        eprintln!(
            "loaded: {} faces, {} verts",
            s.face_count(),
            s.vertex_count()
        );
        let (vb, va) = s.weld();
        eprintln!("welded: {vb} -> {va} verts");
        let scan_result = s.detect_caps();
        eprintln!("detect_caps: {scan_result:?}");
        let tilt = s.level_to_floor();
        eprintln!("level_to_floor tilt: {tilt:?}");

        // ⚠ PID-scoped like every other temp dir in this file (`cf-edit-load-`,
        // `cf-edit-save-`, `cf-edit-nosave-` all do this). This one did NOT,
        // and it is the gate whose doc line — "writes to a temp dir (never
        // ~/scans)" — the mold gates were rewritten to follow. It was the least
        // isolated test in its own file: a fixed path shared by every
        // concurrent process, holding 10 MB of the previous run's output
        // indefinitely (measured 2026-08-27).
        let dir = std::env::temp_dir().join(format!("cf-studio-save-check-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let report = s.save(&dir, "base_mold", "mm", 8).unwrap();
        let stl_bytes = std::fs::metadata(&report.cleaned_stl).unwrap().len();
        eprintln!(
            "SAVED: {:?}\n  {} faces, {stl_bytes} bytes\n  prep: {:?}",
            report.cleaned_stl, report.face_count, report.prep_toml,
        );
        let reloaded = cortenforge::mesh::io::load_stl(&report.cleaned_stl).unwrap();
        eprintln!(
            "cleaned STL reloads: {} faces, {} verts",
            reloaded.faces.len(),
            reloaded.vertices.len()
        );
        let toml = std::fs::read_to_string(&report.prep_toml).unwrap();
        eprintln!("----- base_mold.prep.toml -----\n{toml}");

        // After the assertions, never on the failure path — a red gate keeps
        // its output for inspection. Matches `discard_fixture` in `mold.rs`.
        if let Err(err) = std::fs::remove_dir_all(&dir) {
            eprintln!("WARN: could not remove {}: {err}", dir.display());
        }
    }

    #[test]
    fn run_simplify_then_apply_matches_sync_simplify() {
        // The off-thread split (run_simplify → apply_simplified) lands the
        // same working mesh + provenance as the synchronous simplify.
        let mut sync = session(soup_quad());
        let _ = sync.simplify(1000); // no-op target, but marks applied

        let mut split = session(soup_quad());
        let (mesh, _secs) = run_simplify(&split.working_clone(), 1000);
        split.apply_simplified(mesh, 1000);

        assert_eq!(split.face_count(), sync.face_count());
        assert!(split.simplify_applied());
    }

    #[test]
    fn save_without_a_centerline_is_refused() {
        let s = session(open_quad()); // no detect_caps → no centerline
        let dir = std::env::temp_dir().join(format!("cf-edit-nosave-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        let err = s.save(&dir, "q", "mm", 0).unwrap_err();
        assert!(
            matches!(err, EngineError::Save(_)),
            "save refuses without a centerline (would write an open base): {err}"
        );
        assert!(
            !dir.join("q.cleaned.stl").exists(),
            "nothing written when refused"
        );
        let _ = std::fs::remove_dir_all(&dir);
    }

    /// The guard that short-circuits `processed_mesh` to a plain clone.
    ///
    /// ⚠ Each half of `tip <= 0.0 && floor <= 0.0` has to close the gate ALONE,
    /// or `<=` -> `>` survives on the other side. The untrimmed arm matters as
    /// much as the trimmed ones: it is what fails `||` -> `&&`, which otherwise
    /// sends an untrimmed mesh through trim + weld + auto-cap and silently
    /// closes the scan's open ends.
    #[test]
    fn processed_mesh_is_the_working_mesh_until_a_trim_is_set() {
        let mut s = session(open_tube(6, 20f64.to_radians()));
        s.detect_caps();
        let working_faces = s.working().faces.len();

        let untouched = s.processed_mesh();
        assert_eq!(
            untouched.faces,
            s.working().faces,
            "with no trim the working mesh is returned as-is, not capped"
        );

        let arc = s.centerline_arc_length_mm();
        s.apply_trim(arc * 0.15, 0.0);
        assert_ne!(
            s.processed_mesh().faces.len(),
            working_faces,
            "a tip-only trim alone opens the gate"
        );

        s.apply_trim(0.0, arc * 0.15);
        assert_ne!(
            s.processed_mesh().faces.len(),
            working_faces,
            "a floor-only trim alone opens the gate"
        );
    }

    /// The `Some(ar) if trim_floor_mm > 0.0` arm against the `_` auto-cap arm.
    ///
    /// ⚠ Needs the CURVED tube. On a straight one both arms return the same
    /// face and vertex counts and the same extent — `save`'s equivalent gate
    /// has to read `capped_loops` out of the written TOML because of it. Curved,
    /// the reconstruction rebuilds the floor instead of flat-capping the cut,
    /// which is plainly visible in the mesh.
    #[test]
    fn processed_mesh_reconstructs_the_floor_instead_of_capping_it() {
        let mut s = session(curved_tube(12, 1.2));
        s.detect_caps();
        let arc = s.centerline_arc_length_mm();
        s.apply_trim(arc * 0.15, arc * 0.15);

        let capped = s.processed_mesh();

        assert!(
            s.apply_reconstruct(arc * 0.10, ReconstructShape::Constant),
            "a floor trim makes reconstruction available"
        );
        let rebuilt = s.processed_mesh();

        // ⚠ Face count, NOT extent. `processed_mesh` is not deterministic:
        // over 40 runs the capped extent spans 0.787-0.921 and the rebuilt one
        // 0.868-0.952, so `rebuilt - capped` ranges -0.054..+0.164 — it goes
        // NEGATIVE, and no threshold on it can hold. An extent assertion here
        // failed 3 runs in 12. Face counts are tight by comparison: capped
        // 110-112, rebuilt 334-368 over 40 runs, a minimum ratio of 3.04
        // against the 2.0 asserted.
        assert!(
            rebuilt.faces.len() > capped.faces.len() * 2,
            "the reconstruction rebuilds the floor rather than flat-capping it: \
             {} faces vs {}",
            rebuilt.faces.len(),
            capped.faces.len()
        );
    }

    /// A scratch dir for the `load` gates, which need real files on disk.
    fn load_dir(tag: &str) -> std::path::PathBuf {
        let dir = std::env::temp_dir().join(format!("cf-edit-{tag}-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        dir
    }

    const UNIT_TRI_STL: &str = "solid s\nfacet normal 0 0 1\nouter loop\n\
         vertex 0 0 0\nvertex 10 0 0\nvertex 0 10 0\n\
         endloop\nendfacet\nendsolid s\n";

    /// ⚠ Vertices WITHOUT faces, not an empty file. `vertices.is_empty() ||
    /// faces.is_empty()` needs a mesh where exactly one side is empty, or
    /// `||` -> `&&` survives: with both empty the mutant refuses too. An OBJ
    /// carrying only `v` lines is the one input that separates them.
    #[test]
    fn load_rejects_a_point_cloud_with_no_surface() {
        let dir = load_dir("vonly");
        let obj = dir.join("cloud.obj");
        std::fs::write(&obj, "v 0 0 0\nv 1 0 0\nv 0 1 0\n").unwrap();

        let loaded = cortenforge::mesh::io::load_mesh(&obj).expect("the OBJ itself parses");
        assert!(
            !loaded.vertices.is_empty() && loaded.faces.is_empty(),
            "fixture: vertices without faces, so exactly one side is empty"
        );

        let err = EditSession::load(&obj, 1.0).unwrap_err();
        assert!(
            matches!(err, EngineError::EmptyScan { .. }),
            "a point cloud is refused as a scan, not carried as an empty session: {err}"
        );
        let _ = std::fs::remove_dir_all(&dir);
    }

    /// ⚠ The suite only ever loaded at `scale_to_m == 1.0`, which SKIPS the
    /// scaling branch entirely — that is why every mutant in it survived.
    /// The verdict is the AABB extent, which is invariant under the centring
    /// and PCA rotation `load` also applies, so it isolates the scale.
    #[test]
    fn load_applies_the_unit_scale() {
        let dir = load_dir("scale");
        let stl = dir.join("tri.stl");
        std::fs::write(&stl, UNIT_TRI_STL).unwrap();

        let extent = |s: &EditSession| {
            let a = s.aabb();
            a.max.z - a.min.z
        };
        let as_is = extent(&EditSession::load(&stl, 1.0).unwrap());
        let millimetres = extent(&EditSession::load(&stl, 0.001).unwrap());

        assert!(as_is > 1.0, "fixture: the unscaled scan is order 10 units");
        assert!(
            (as_is - millimetres * 1000.0).abs() < 1e-9 * as_is,
            "loading at 0.001 shrinks the scan by exactly 1000x: {as_is} vs {millimetres}"
        );
        let _ = std::fs::remove_dir_all(&dir);
    }

    /// The other side of `changing_the_floor_trim_drops_reconstruct`.
    ///
    /// ⚠ That gate only shows a CHANGED floor drops the fit. Nothing showed an
    /// UNCHANGED one keeps it, so both `-` swaps in the comparison survived:
    /// `+` makes `(200 + 200)` read as a change, `/` makes `(200 / 200)` read
    /// as one. Under either, the user loses a floor reconstruction they spent
    /// time fitting the moment any slider is touched again.
    ///
    /// The second arm gates which FIELD the comparison reads: moving the tip
    /// must not disturb a floor fit.
    #[test]
    fn re_applying_the_same_floor_trim_keeps_the_reconstruction() {
        let mut s = session(open_tube(6, 20f64.to_radians()));
        s.detect_caps();
        s.apply_trim(0.0, 200.0);
        s.apply_reconstruct(25.0, ReconstructShape::Taper);
        assert!(
            s.reconstruct().is_some(),
            "fixture: a reconstruction is fitted"
        );

        s.apply_trim(0.0, 200.0);
        assert!(
            s.reconstruct().is_some(),
            "re-applying the SAME floor trim is not a change, so the fit stands"
        );

        s.apply_trim(60.0, 200.0);
        assert!(
            s.reconstruct().is_some(),
            "the guard reads the FLOOR: moving only the tip leaves the fit alone"
        );
    }

    /// ⚠ `centerline.len()` is only ever 0 or `CENTERLINE_SLICES` (30).
    /// `compute_centerline_polyline` returns exactly `n_slices` points when it
    /// succeeds and none when it fails, and `detect_caps` always passes 30 — no
    /// mesh yields 1 or 2 (checked across tetra, quads, and every tube and
    /// curved-tube shape here). So the `< 2` boundary is UNREACHABLE, and
    /// `<` -> `<=` is an equivalent mutant in this guard, in `save`'s, and in
    /// `display_centerline`'s. No fixture can gate it.
    ///
    /// What IS reachable is len == 0 with a trim ALREADY SET — the user moves
    /// the sliders, then a mesh edit clears the caps. The guard has to still
    /// short-circuit: `<` -> `==` otherwise falls through and auto-caps a scan,
    /// silently closing the open ends of a mesh the user has not trimmed.
    #[test]
    fn processed_mesh_short_circuits_without_a_centerline_even_when_trimmed() {
        let mut s = session(open_tube(6, 20f64.to_radians()));
        assert!(
            s.centerline().is_empty(),
            "fixture: no detect_caps, so there is no centerline"
        );

        s.apply_trim(10.0, 5.0);
        // ⚠ Without this the gate goes VACUOUS if `apply_trim` ever stops
        // setting the trim: the guard's second disjunct would fire instead and
        // the assertion below would still hold, testing nothing.
        assert!(
            s.trim_tip_mm() > 0.0 && s.trim_floor_mm() > 0.0,
            "the trim is really set, so only the centerline half can short-circuit"
        );

        assert_eq!(
            s.processed_mesh().faces,
            s.working().faces,
            "with no centerline to trim along, the working mesh comes back \
             untouched — not welded and auto-capped"
        );
    }

    /// A tube bent through `bend_rad` in the x-z plane, scaled so its arc is
    /// ~1 unit long.
    ///
    /// ⚠ Exists because a STRAIGHT tube cannot separate `floor_normal`'s tiers:
    /// its centerline tangent and its cap-plane normal are the same direction
    /// (they agree to 1e-15), and its tangent is constant along the arc. Every
    /// mutant in that function is equivalent under `open_tube`. Curving it makes
    /// the cap normal and the tangent ~45 deg apart.
    fn curved_tube(rings: usize, bend_rad: f64) -> IndexedMesh {
        let r_curve = 1.0 / bend_rad;
        let mut vertices = Vec::new();
        for i in 0..rings {
            let t = i as f64 / (rings - 1) as f64;
            let ang = t * bend_rad;
            let c = Point3::new(r_curve * (1.0 - ang.cos()), 0.0, r_curve * ang.sin());
            let rot = UnitQuaternion::from_axis_angle(&Vector3::y_axis(), ang);
            for (dx, dy) in [(-0.2, -0.2), (0.2, -0.2), (0.2, 0.2), (-0.2, 0.2)] {
                vertices.push(c + rot * Vector3::new(dx, dy, 0.0));
            }
        }
        let mut faces = Vec::new();
        for r in 0..rings - 1 {
            let b = (r * 4) as u32;
            let t = ((r + 1) * 4) as u32;
            for k in 0..4u32 {
                let k2 = (k + 1) % 4;
                faces.push([b + k, b + k2, t + k2]);
                faces.push([b + k, t + k2, t + k]);
            }
        }
        IndexedMesh { vertices, faces }
    }

    /// `floor_normal`'s tier 2 — the centerline tangent at the predicted cut —
    /// against tier 3, the raw cap normal.
    ///
    /// ⚠ Tier 2 is reachable ONLY when the trim consumes the centerline: tier 1
    /// (the reconstructed plane) fires for any floor trim EXCEPT when the
    /// trimmed polyline drops below 2 points. That is the whole live window,
    /// and nothing exercised it.
    ///
    /// ⚠⚠ The oracle is the VALUE, not "differs from the cap". This gate first
    /// asserted only `(over - cap).norm() > 0.5`, which passes for any wrong
    /// answer that also happens to sit far from the cap — and all four of this
    /// function's arithmetic mutants are exactly that. It appeared to catch two
    /// of them only while `cap_loops[0]` was still flipping at random, i.e. by
    /// luck; pinning that order in mesh-repair exposed the gate as weak.
    /// A negative assertion cannot gate a computation.
    ///
    /// The literal is measured and stable over 30 runs, and each mutant lands
    /// far from it: the `-` swaps push the cut past the polyline's end and fall
    /// through to the cap normal, while the `*` swaps collapse it to zero — the
    /// tangent at the START, about (-0.361, -0.788, -0.500).
    #[test]
    fn an_over_trimmed_floor_normal_follows_the_centerline_not_the_cap() {
        let mut s = session(curved_tube(12, 1.2));
        s.detect_caps();
        let cap = s.cap_loops()[0].plane_normal.normalize();

        let untrimmed = s.floor_normal().expect("a capped tube has a floor normal");
        assert!(
            (untrimmed - cap).norm() < 1e-9,
            "with no trim the raw cap normal is used: {untrimmed:?} vs {cap:?}"
        );

        let arc = s.centerline_arc_length_mm();
        s.apply_trim(arc * 0.6, arc * 0.6);
        let over = s
            .floor_normal()
            .expect("over-trimming still yields a normal");

        let expected = Vector3::new(-0.744_117, 0.002_703, -0.668_044);
        assert!(
            (over - expected).norm() < 1e-4,
            "the tangent at the predicted cut — not the cap, not the start: \
             {over:?} vs {expected:?}"
        );
        assert!(
            (over - cap).norm() > 0.5,
            "and nowhere near the cap normal: {over:?} vs {cap:?}"
        );
    }

    /// ⚠ Each half of `trim_tip_mm > 0.0 || trim_floor_mm > 0.0` must open the
    /// branch ALONE, or `||` -> `&&` survives — the same shape as the `save`
    /// gates. The verdict is WHICH END moved: a tip trim cuts from
    /// `centerline[0]` and must leave the far end alone, and vice versa. Point
    /// count alone would not catch the two being swapped.
    #[test]
    fn a_tip_trim_shortens_the_drawn_centerline_from_the_tip_end() {
        let mut s = session(open_tube(6, 20f64.to_radians()));
        s.detect_caps();
        let full = s.display_centerline();

        s.apply_trim(s.centerline_arc_length_mm() * 0.2, 0.0);
        let cut = s.display_centerline();

        assert!(
            cut.len() < full.len(),
            "the drawn line loses points: {} -> {}",
            full.len(),
            cut.len()
        );
        assert!((cut[0] - full[0]).norm() > 1e-9, "the TIP end moved inward");
        assert!(
            (cut[cut.len() - 1] - full[full.len() - 1]).norm() < 1e-9,
            "and the floor end did not"
        );
    }

    /// The other half of the same disjunction — see the tip gate above.
    #[test]
    fn a_floor_trim_shortens_the_drawn_centerline_from_the_floor_end() {
        let mut s = session(open_tube(6, 20f64.to_radians()));
        s.detect_caps();
        let full = s.display_centerline();

        s.apply_trim(0.0, s.centerline_arc_length_mm() * 0.2);
        let cut = s.display_centerline();

        assert!(
            cut.len() < full.len(),
            "the drawn line loses points: {} -> {}",
            full.len(),
            cut.len()
        );
        assert!(
            (cut[cut.len() - 1] - full[full.len() - 1]).norm() > 1e-9,
            "the FLOOR end moved inward"
        );
        assert!((cut[0] - full[0]).norm() < 1e-9, "and the tip end did not");
    }

    /// The `reorient_rotation == identity` shortcut.
    ///
    /// ⚠ The identity case cannot catch `==` -> `!=`: the mutant then falls
    /// through and bakes with the identity rotation, which is the same answer.
    /// Only a REAL rotation separates them — the mutant returns the unbaked
    /// line, so the overlay would drift off the rendered mesh.
    #[test]
    fn the_drawn_centerline_is_baked_through_the_reorient() {
        let mut s = session(open_tube(6, 20f64.to_radians()));
        s.detect_caps();
        let raw = s.centerline().to_vec();
        assert_eq!(
            s.display_centerline(),
            raw,
            "with no reorient the drawn line is the centerline itself"
        );

        s.level_to_floor().expect("a tilted tube can be leveled");
        let drawn = s.display_centerline();
        let moved = drawn
            .iter()
            .zip(&raw)
            .map(|(a, b)| (a - b).norm())
            .fold(0.0_f64, f64::max);
        assert!(
            moved > 1e-3,
            "after leveling the drawn line is baked through the reorient, \
             so it tracks the rendered mesh; max point shift {moved}"
        );
    }

    /// ⚠ The verdict is the UNIT (millimetres), not the polyline maths —
    /// `polyline_arc_length_m` belongs to cf-scan-prep-core and is gated there.
    /// Re-summing the centerline here would mirror the function under test.
    ///
    /// The bound is physical, not fitted: `open_tube` spans z 0..=1 exactly and
    /// the tilt is a rotation, which preserves length, so the tube's own axis is
    /// 1.0 unit = 1000 mm and its centerline cannot be longer. That alone fails
    /// the three constant mutants and `* 1000.0` -> `/ 1000.0`.
    ///
    /// ⚠ The half-scale arm exists for `* 1000.0` -> `+ 1000.0`, which the bound
    /// would catch only by 1000.97 vs 1000 — a 0.1% margin. Halving the mesh has
    /// to halve the report; an added constant breaks that by ~1000 mm.
    #[test]
    fn the_centerline_arc_length_is_reported_in_millimetres() {
        let mut s = session(open_tube(6, 20f64.to_radians()));
        assert_eq!(
            s.centerline_arc_length_mm(),
            0.0,
            "no centerline yet, so there is no trim bound to offer"
        );

        s.detect_caps();
        let full = s.centerline_arc_length_mm();
        assert!(
            full > 900.0 && full < 1000.0,
            "the centerline of a 1.0-unit tube is just under 1000 mm, got {full}"
        );

        let mut half = session(scaled(open_tube(6, 20f64.to_radians()), 0.5));
        half.detect_caps();
        let half_mm = half.centerline_arc_length_mm();
        assert!(
            (half_mm * 2.0 - full).abs() < 1.0,
            "halving the mesh halves the reported length: {half_mm} * 2 vs {full}"
        );
    }

    /// The axial (z) and radial (x) extents of a saved cleaned STL.
    ///
    /// ⚠ Read back off DISK, not off the session: `save`'s whole job is the
    /// pair it writes, and the trim happens after the mesh leaves `working`.
    fn saved_extents(report: &SaveReport) -> (f64, f64) {
        let m = cortenforge::mesh::io::load_stl(&report.cleaned_stl).unwrap();
        let a = m.aabb();
        (a.max.z - a.min.z, a.max.x - a.min.x)
    }

    /// A capped tube session plus a scratch dir, for the save gates below.
    fn tube_ready_to_save(tag: &str) -> (EditSession, std::path::PathBuf) {
        let mut s = session(open_tube(6, 20f64.to_radians()));
        s.detect_caps();
        let dir = std::env::temp_dir().join(format!("cf-edit-{tag}-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();
        (s, dir)
    }

    /// ⚠ Each half of `trim_tip_mm > 0.0 || trim_floor_mm > 0.0` must open the
    /// gate ALONE. Both-set would leave `||` -> `&&` alive, since both
    /// disjuncts are true either way — the suite trimmed nothing at all before
    /// this, so the whole block was unexecuted.
    #[test]
    fn save_applies_a_tip_only_trim() {
        let (mut s, dir) = tube_ready_to_save("tiptrim");
        let (plain_z, plain_x) = saved_extents(&s.save(&dir, "plain", "mm", 0).unwrap());

        s.apply_trim(s.centerline_arc_length_mm() * 0.2, 0.0);
        let (trim_z, trim_x) = saved_extents(&s.save(&dir, "tip", "mm", 0).unwrap());

        assert!(
            trim_z < plain_z * 0.95,
            "a tip-only trim shortens the saved mesh along the centerline: \
             {plain_z} -> {trim_z}"
        );
        assert!(
            (trim_x - plain_x).abs() < 1e-6,
            "and does not touch the radius: {plain_x} -> {trim_x}"
        );
        let _ = std::fs::remove_dir_all(&dir);
    }

    /// The other half of the same disjunction — see the tip gate above.
    #[test]
    fn save_applies_a_floor_only_trim() {
        let (mut s, dir) = tube_ready_to_save("floortrim");
        let (plain_z, plain_x) = saved_extents(&s.save(&dir, "plain", "mm", 0).unwrap());

        s.apply_trim(0.0, s.centerline_arc_length_mm() * 0.2);
        let (trim_z, trim_x) = saved_extents(&s.save(&dir, "floor", "mm", 0).unwrap());

        assert!(
            trim_z < plain_z * 0.95,
            "a floor-only trim shortens the saved mesh along the centerline: \
             {plain_z} -> {trim_z}"
        );
        assert!(
            (trim_x - plain_x).abs() < 1e-6,
            "and does not touch the radius: {plain_x} -> {trim_x}"
        );
        let _ = std::fs::remove_dir_all(&dir);
    }

    /// The `Some(ar) if trim_floor_mm > 0.0` arm vs the `_` auto-cap arm.
    ///
    /// ⚠ The oracle is `capped_loops`, which ONLY the auto-cap arm sets
    /// (`trim_capped`); the reconstruct arm leaves it 0. Both arms close the
    /// same opening with the same face count, and the `[centerline_trim.
    /// reconstruct]` block is written from `self.reconstruct` — the SETTING —
    /// so it appears whichever arm ran. Gating on that block passed happily
    /// with the guard forced to `false`.
    #[test]
    fn save_reconstructs_the_floor_instead_of_auto_capping_it() {
        let (mut s, dir) = tube_ready_to_save("reconstruct");
        let arc = s.centerline_arc_length_mm();
        s.apply_trim(arc * 0.2, arc * 0.2);

        let capped =
            std::fs::read_to_string(&s.save(&dir, "capped", "mm", 0).unwrap().prep_toml).unwrap();
        assert!(
            capped.contains("capped_loops = 2"),
            "with no reconstruction the auto-cap arm closes both trimmed ends: {capped}"
        );

        assert!(
            s.apply_reconstruct(arc * 0.1, ReconstructShape::Constant),
            "a floor trim makes reconstruction available"
        );
        let rebuilt =
            std::fs::read_to_string(&s.save(&dir, "rebuilt", "mm", 0).unwrap().prep_toml).unwrap();
        assert!(
            rebuilt.contains("capped_loops = 0"),
            "the reconstruct arm runs INSTEAD of the auto-cap, so nothing is auto-capped: {rebuilt}"
        );
        assert!(
            rebuilt.contains("[centerline_trim.reconstruct]"),
            "and the reconstruction it was given is recorded for the consumer"
        );

        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn save_writes_cleaned_stl_and_prep_toml() {
        let mut s = session(open_tube(6, 20f64.to_radians()));
        s.detect_caps();
        let dir = std::env::temp_dir().join(format!("cf-edit-save-{}", std::process::id()));
        std::fs::create_dir_all(&dir).unwrap();

        let report = s.save(&dir, "tube", "mm", 0).unwrap();
        assert!(report.cleaned_stl.exists(), "cleaned STL written");
        assert!(report.prep_toml.exists(), "prep.toml written");
        assert!(report.face_count > 0);

        let toml_str = std::fs::read_to_string(&report.prep_toml).unwrap();
        assert!(
            toml_str.contains("[scan_prep]"),
            "prep has the scan_prep block"
        );
        assert!(
            toml_str.contains("cleaned_stl"),
            "prep records the cleaned STL"
        );

        let reloaded = cortenforge::mesh::io::load_stl(&report.cleaned_stl).unwrap();
        assert!(
            !reloaded.faces.is_empty(),
            "the cleaned STL reloads as a mesh"
        );

        // ⚠ The seam the GUI's Save sits on: cleanup hands this pair straight to
        // `accept_prep`, and a pair that fails there is one the user cannot
        // finish the step with. Writing both files and reloading the mesh does
        // not prove it — `accept_prep` also requires >= 2 centerline points.
        crate::accept_prep(&report.cleaned_stl, &report.prep_toml)
            .expect("save's own output passes the engine's acceptance check");

        let _ = std::fs::remove_dir_all(&dir);
    }

    #[test]
    fn level_to_floor_uses_the_trim_floor() {
        let mut s = session(open_tube(6, 20f64.to_radians()));
        s.detect_caps();
        s.apply_trim(0.0, 200.0);
        // With a floor trim, leveling uses the predicted/reconstructed cut
        // floor (not the raw cap) — still returns a tilt + sets the reorient.
        assert!(s.level_to_floor().is_some());
        assert_ne!(s.reorient_rotation(), UnitQuaternion::identity());
    }
}
