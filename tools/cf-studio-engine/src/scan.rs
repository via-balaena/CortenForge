//! The scan-load boundary (workflow step "Add scan"): load the chosen
//! file as a mesh, confirm it actually has geometry, and report quick
//! stats. Failing here — on a missing file, an unreadable format, or an
//! empty mesh — surfaces the problem at step 1 instead of deep in the
//! cast pipeline.

use std::path::{Path, PathBuf};

use cf_studio_core::ScanInput;
use cortenforge::mesh::io::load_mesh;

use crate::error::{EngineError, Result};

/// A loaded, validated scan plus quick stats for display. Intentionally
/// does not retain the mesh: step 1 only needs to *validate* the file
/// and record its path; downstream steps reload from the cleaned scan.
#[derive(Debug, Clone, PartialEq, Eq)]
pub struct LoadedScan {
    /// The raw scan file that was loaded.
    pub source_path: PathBuf,
    /// Number of vertices in the loaded mesh.
    pub vertex_count: usize,
    /// Number of triangle faces in the loaded mesh.
    pub face_count: usize,
}

impl LoadedScan {
    /// The spine artifact to hand to [`Project::set_scan`], recording
    /// which file this project's scan came from.
    ///
    /// [`Project::set_scan`]: cf_studio_core::Project::set_scan
    #[must_use]
    pub fn artifact(&self) -> ScanInput {
        ScanInput {
            source_path: self.source_path.clone(),
        }
    }
}

/// Load + validate a scan file, auto-detecting the format (STL / OBJ /
/// PLY / 3MF) by extension (case-insensitively). 3MF is enabled via
/// mesh-io's `threemf` feature (declared in the `cortenforge` facade's
/// Cargo.toml, through which this crate uses mesh-io). STEP stays opt-out
/// (heavy CAD deps).
///
/// # Errors
/// - [`EngineError::ScanLoad`] if the file is missing or can't be parsed.
/// - [`EngineError::EmptyScan`] if it parses but has no vertices or no faces.
pub fn load_scan(path: &Path) -> Result<LoadedScan> {
    let mesh = load_mesh(path).map_err(|e| EngineError::ScanLoad {
        path: path.display().to_string(),
        reason: e.to_string(),
    })?;
    if mesh.vertices.is_empty() || mesh.faces.is_empty() {
        return Err(EngineError::EmptyScan {
            path: path.display().to_string(),
        });
    }
    Ok(LoadedScan {
        source_path: path.to_path_buf(),
        vertex_count: mesh.vertices.len(),
        face_count: mesh.faces.len(),
    })
}

#[cfg(test)]
mod tests {
    #![allow(clippy::unwrap_used, clippy::expect_used)]

    use super::*;

    /// A minimal valid ASCII STL — one triangle.
    const ONE_TRIANGLE_STL: &str = "\
solid t
facet normal 0 0 1
  outer loop
    vertex 0 0 0
    vertex 1 0 0
    vertex 0 1 0
  endloop
endfacet
endsolid t
";

    /// An STL that parses but contains no facets.
    const EMPTY_STL: &str = "solid empty\nendsolid empty\n";

    /// A per-test temp subdir (keyed by PID **and** a unique label) so
    /// tests running in parallel never share a directory — otherwise one
    /// test's `remove_dir` teardown can race a sibling's write.
    fn temp_dir(label: &str) -> PathBuf {
        let dir = std::env::temp_dir().join(format!(
            "cf-studio-engine-scan-test-{}-{label}",
            std::process::id()
        ));
        std::fs::create_dir_all(&dir).unwrap();
        dir
    }

    #[test]
    fn loads_a_valid_stl_and_reports_stats() {
        let dir = temp_dir("valid");
        let path = dir.join("tri.stl");
        std::fs::write(&path, ONE_TRIANGLE_STL).unwrap();

        let loaded = load_scan(&path).unwrap();
        assert_eq!(loaded.face_count, 1);
        assert_eq!(loaded.vertex_count, 3);
        assert_eq!(loaded.source_path, path);
        assert_eq!(loaded.artifact().source_path, path);

        let _ = std::fs::remove_file(&path);
        let _ = std::fs::remove_dir(&dir);
    }

    #[test]
    fn missing_file_is_a_scan_load_error() {
        let err = load_scan(Path::new("/no/such/scan.stl")).unwrap_err();
        assert!(matches!(err, EngineError::ScanLoad { .. }), "got: {err:?}");
    }

    #[test]
    fn empty_mesh_is_rejected() {
        let dir = temp_dir("empty");
        let path = dir.join("empty.stl");
        std::fs::write(&path, EMPTY_STL).unwrap();

        let err = load_scan(&path).unwrap_err();
        assert!(matches!(err, EngineError::EmptyScan { .. }), "got: {err:?}");

        let _ = std::fs::remove_file(&path);
        let _ = std::fs::remove_dir(&dir);
    }
}
