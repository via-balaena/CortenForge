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
//! Built phase by phase, per the agreed step-2 op order. **Phase 1 (here):**
//! load / simplify / weld / reset + auto-center / auto-orient. Capping,
//! centerline + trim / reconstruct, and the save path land in later phases.

use std::path::{Path, PathBuf};

use mesh_repair::{remove_unreferenced_vertices, weld_vertices};
use mesh_types::{Aabb, Bounded, IndexedMesh};
use nalgebra::{UnitQuaternion, Vector3};

use crate::error::{EngineError, Result};

/// Weld tolerance in meters — matches cf-scan-prep's
/// `SIMPLIFY_WELD_EPSILON_M` so the two tools weld identically.
const WELD_EPSILON_M: f64 = 1e-6;

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

    /// Restore the working mesh to the originally-loaded scan and clear
    /// all applied ops + accumulated transform provenance.
    pub fn reset(&mut self) {
        self.working = self.original.clone();
        self.simplify_applied = false;
        self.simplify_target = 0;
        self.auto_center_offset_m = Vector3::zeros();
        self.auto_pca_quat = None;
    }

    /// Weld coincident vertices then drop the unreferenced ones. Raw STL
    /// is a vertex soup (one vertex set per triangle); welding shares
    /// indices so downstream ops see real adjacency. Returns
    /// `(vertices_before, vertices_after)`.
    pub fn weld(&mut self) -> (usize, usize) {
        let before = self.working.vertices.len();
        weld_vertices(&mut self.working, WELD_EPSILON_M);
        remove_unreferenced_vertices(&mut self.working);
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
        result.elapsed_secs
    }

    /// Recenter the working mesh's AABB centroid onto the origin; returns
    /// (and accumulates) the applied offset in meters.
    pub fn auto_center(&mut self) -> Vector3<f64> {
        let offset = cf_scan_prep_core::auto_center_in_place(&mut self.working);
        self.auto_center_offset_m += offset;
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
        q
    }
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
}
