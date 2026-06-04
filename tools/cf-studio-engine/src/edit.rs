//! Step-2 scan-editing session — the headless engine boundary for
//! interactive scan cleanup.
//!
//! Holds the working mesh + the original (for reset) + accumulated
//! edit/provenance state, and applies [`cf_scan_prep_core`] ops. The Bevy
//! `cf-scan-prep` tool and CortenForge Studio's step-2 editor are two
//! frontends over this same logic — the goal is identical function. The
//! session ultimately produces the cleaned STL + `.prep.toml` the cast
//! pipeline consumes.
//!
//! Built phase by phase, per the agreed step-2 op order. **Phase 1:** load
//! (auto-center + auto-orient at load) / weld / simplify / reset. **Phase
//! 2a:** cap detection + the interior centerline. Leveling, trim /
//! reconstruct, and the save path land in later phases.

use std::path::{Path, PathBuf};

use cf_scan_prep_core::{AppliedReconstruct, DetectedCapLoop, ReconstructShape};
use mesh_repair::{remove_unreferenced_vertices, weld_vertices};
use mesh_types::{Aabb, Bounded, IndexedMesh, Point3};
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
        let mut mesh = mesh_io::load_mesh(path).map_err(|e| EngineError::ScanLoad {
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

    /// The auto-center offset applied (at load + any later `auto_center`
    /// calls), in meters — the `[scan_prep].auto_center_offset_m` provenance.
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

    /// Clear the trim (and any reconstruction).
    pub fn clear_trim(&mut self) {
        self.trim_tip_mm = 0.0;
        self.trim_floor_mm = 0.0;
        self.reconstruct = None;
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

    /// Drop the floor reconstruction (revert to a flat cap).
    pub fn clear_reconstruct(&mut self) {
        self.reconstruct = None;
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
    /// provenance.
    pub fn simplify(&mut self, target_faces: usize) -> f64 {
        let result = cf_scan_prep_core::simplify_mesh(&self.working, target_faces);
        self.working = result.mesh;
        self.simplify_applied = true;
        self.simplify_target = target_faces;
        self.clear_caps();
        result.elapsed_secs
    }

    /// Recenter the working mesh's AABB centroid onto the origin; returns
    /// (and accumulates) the applied offset in meters.
    pub fn auto_center(&mut self) -> Vector3<f64> {
        let offset = cf_scan_prep_core::auto_center_in_place(&mut self.working);
        self.auto_center_offset_m += offset;
        self.clear_caps();
        offset
    }

    /// PCA-orient the working mesh toward the cast frame (+Z up); returns
    /// (and records) the applied rotation, or `None` if PCA was
    /// degenerate (e.g. a near-spherical mesh with no dominant axis).
    pub fn auto_orient_pca(&mut self) -> Option<UnitQuaternion<f64>> {
        let q = cf_scan_prep_core::auto_pca_in_place(&mut self.working);
        if let Some(rot) = q {
            self.auto_pca_quat = Some(match self.auto_pca_quat {
                Some(prev) => rot * prev,
                None => rot,
            });
        }
        self.clear_caps();
        q
    }
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

    use mesh_types::Point3;

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
    fn auto_center_moves_centroid_to_origin() {
        let mut s = session(offset_tri());
        let offset = s.auto_center();
        // The offset is non-trivial (mesh was near (10, 10, 10)).
        assert!(offset.norm() > 1.0);
        let c = s.aabb().center();
        assert!(
            c.coords.norm() < 1e-9,
            "centroid recentered to origin: {c:?}"
        );
    }

    #[test]
    fn reset_restores_the_original_mesh() {
        let mut s = session(offset_tri());
        s.auto_center();
        assert!(s.aabb().center().coords.norm() < 1e-9);
        s.reset();
        // Back near (10.33, 10.33, 10) — the original centroid.
        assert!(
            s.aabb().center().coords.norm() > 1.0,
            "reset undid auto-center"
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
    fn load_auto_centers_and_records_offset() {
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
        let tilt = UnitQuaternion::from_axis_angle(&Vector3::x_axis(), 30f64.to_radians());
        let n = tilt * Vector3::z();
        let q = floor_leveling_rotation(n).unwrap();
        let leveled = q * n;
        assert!(
            (leveled.z.abs() - 1.0).abs() < 1e-9,
            "normal is vertical after leveling: {leveled:?}"
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
        s.reset();
        assert_eq!(s.trim_tip_mm(), 0.0);
        assert_eq!(s.trim_floor_mm(), 0.0);
        assert!(s.reconstruct().is_none());
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
