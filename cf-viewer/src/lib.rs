//! `cf-viewer` library — PLY loading + the [`ViewerInput`] data type.
//!
//! Wraps [`mesh_io::load_ply_attributed`] in a Bevy-friendly resource that
//! carries the loaded geometry plus a deterministically-ordered list of
//! per-vertex scalar names. Submodules:
//!
//! - [`mesh`] — `AttributedMesh → Bevy Mesh` conversion (face case) and
//!   the point-cloud sizing constant.
//! - [`colormap`] — Q5 distribution detection + per-vertex RGBA mapping
//!   (commit 4).
//! - [`ui`] — scalar + colormap dropdowns via `bevy_egui` (commit 5).
//! - [`cli`] — `clap`-derived `--scalar` / `--colormap` / `--up` flags
//!   that seed [`ui::Selection`] + [`UpAxis`] before the dropdowns see
//!   them (commit 6).
//!
//! The orbit camera + up-axis convention previously housed in this crate
//! moved to [`cf_bevy_common`] at sim-soft PR2 C2b so sim-bevy-soft and
//! sim-bevy can share the same controller. [`UpAxis`] is re-exported here
//! for back-compat with cf-viewer's existing call sites.

pub mod cli;
pub mod colormap;
pub mod mesh;
pub mod ui;

use std::path::{Path, PathBuf};

use anyhow::{Context, Result, anyhow, bail};
use bevy::prelude::Resource;
pub use cf_bevy_common::axis::UpAxis;
use mesh_io::load_ply_attributed;
use mesh_types::AttributedMesh;

/// Loaded artifact ready for visualization.
///
/// `scalar_names` is the alphabetically-ordered list of per-vertex
/// `extras` keys (Q4 lock — see `docs/VIEWER_DESIGN.md`). Auto-selection
/// picks the first; the [`ui::scalar_and_colormap_panel`] dropdown
/// (commit 5) and the `--scalar=<name>` CLI flag (commit 6) override.
///
/// [`ui::scalar_and_colormap_panel`]: crate::ui::scalar_and_colormap_panel
#[derive(Resource)]
pub struct ViewerInput {
    /// Geometry plus per-vertex attributes loaded from the PLY.
    pub mesh: AttributedMesh,

    /// Sorted names of per-vertex scalar extras (`mesh.extras` keys).
    pub scalar_names: Vec<String>,
}

/// Load a PLY file into a [`ViewerInput`].
///
/// Wraps [`mesh_io::load_ply_attributed`] and projects the `extras` map
/// keys into `scalar_names`. `BTreeMap` iteration is alphabetical, so
/// the order is deterministic across loads.
///
/// # Errors
///
/// Returns an error if the file cannot be read or the PLY is malformed.
pub fn load_input(path: &Path) -> Result<ViewerInput> {
    let mesh = load_ply_attributed(path)
        .with_context(|| format!("loading PLY from {}", path.display()))?;
    let scalar_names: Vec<String> = mesh.extras.keys().cloned().collect();
    Ok(ViewerInput { mesh, scalar_names })
}

/// Lex-sorted list of `*_step_<digits>.ply` files under `dir`.
///
/// The sim-soft `viz::design_surface_deformed` PLY series writes
/// zero-padded step indices (e.g. `sleeve_design_surface_deformed_step_01.ply`),
/// so lex order matches step order. Other files in the directory
/// (final-state PLYs, JSON readouts, slab cuts) are silently skipped.
///
/// This is the D1 slice's input-mode detector for `cf-view <dir>` mode
/// (per `docs/SIM_SOFT_ROADMAP.md` Track D, leaf D1).
///
/// # Errors
///
/// - `dir` cannot be read.
/// - no entries match the `*_step_<digits>.ply` pattern.
pub fn discover_ply_sequence(dir: &Path) -> Result<Vec<PathBuf>> {
    let mut frames: Vec<PathBuf> = Vec::new();
    let read_dir =
        std::fs::read_dir(dir).with_context(|| format!("reading directory {}", dir.display()))?;
    for entry in read_dir {
        let entry = entry.with_context(|| format!("reading dir entry under {}", dir.display()))?;
        let path = entry.path();
        if !path.is_file() {
            continue;
        }
        let Some(name) = path.file_name().and_then(|n| n.to_str()) else {
            continue;
        };
        if is_sequence_frame_name(name) {
            frames.push(path);
        }
    }
    if frames.is_empty() {
        bail!(
            "no PLY sequence frames found in {} (looking for *_step_<digits>.ply)",
            dir.display(),
        );
    }
    frames.sort();
    Ok(frames)
}

/// Does the filename match the `*_step_<digits>.ply` sequence-frame
/// convention? `_step_` must appear in the stem, followed by at least
/// one ASCII digit before the `.ply` extension.
fn is_sequence_frame_name(name: &str) -> bool {
    let Some(stem) = name.strip_suffix(".ply") else {
        return false;
    };
    let Some(idx) = stem.rfind("_step_") else {
        return false;
    };
    let digits = &stem[idx + "_step_".len()..];
    !digits.is_empty() && digits.chars().all(|c| c.is_ascii_digit())
}

/// Resolve the CLI path to the single PLY file we'll load for the
/// initial frame: pass through for file input; pick the first frame of
/// the lex-sorted sequence for directory input.
///
/// Prints a one-line stdout summary in sequence mode so the user can
/// see frame-count + first-frame name before the Bevy window opens.
///
/// # Errors
///
/// Bubbles up [`discover_ply_sequence`]'s errors when the input is a
/// directory.
pub fn resolve_initial_frame(path: &Path) -> Result<PathBuf> {
    if path.is_dir() {
        let frames = discover_ply_sequence(path)?;
        let first = frames
            .first()
            .cloned()
            .ok_or_else(|| anyhow!("discover_ply_sequence returned empty"))?;
        println!(
            "detected PLY sequence: {} frame(s) in {}; rendering frame 1/{} ({})",
            frames.len(),
            path.display(),
            frames.len(),
            first.display(),
        );
        Ok(first)
    } else {
        Ok(path.to_path_buf())
    }
}

#[cfg(test)]
mod tests {
    use super::*;

    use std::fs::File;

    use mesh_io::save_ply_attributed;
    use mesh_types::{IndexedMesh, Point3};

    #[test]
    fn sequence_frame_name_accepts_zero_padded_step_suffix() {
        assert!(is_sequence_frame_name(
            "sleeve_design_surface_deformed_step_01.ply"
        ));
        assert!(is_sequence_frame_name(
            "sleeve_design_surface_deformed_step_16.ply"
        ));
        // Single-digit + multi-digit both legal.
        assert!(is_sequence_frame_name("frame_step_0.ply"));
        assert!(is_sequence_frame_name("frame_step_1234.ply"));
    }

    #[test]
    fn sequence_frame_name_rejects_non_matching() {
        // No `_step_` marker.
        assert!(!is_sequence_frame_name("sleeve_design_surface_final.ply"));
        // Not a PLY.
        assert!(!is_sequence_frame_name("sleeve_step_01.json"));
        // Marker present but no digits after.
        assert!(!is_sequence_frame_name("frame_step_.ply"));
        // Non-digit suffix (alpha).
        assert!(!is_sequence_frame_name("frame_step_final.ply"));
        // Mixed alpha+digit suffix.
        assert!(!is_sequence_frame_name("frame_step_01a.ply"));
        // Slab-cut convention (no step suffix).
        assert!(!is_sequence_frame_name("sleeve_slab_cut_x0_final.ply"));
    }

    #[test]
    fn discover_ply_sequence_filters_and_sorts() -> Result<()> {
        let dir = tempfile::tempdir()?;
        // Mix of matching + non-matching files; deliberately create out
        // of lex order so the sort step is meaningful.
        for name in [
            "sleeve_design_surface_deformed_step_03.ply",
            "sleeve_design_surface_deformed_step_01.ply",
            "sleeve_design_surface_deformed_step_02.ply",
            "sleeve_design_surface_final.ply", // skipped: no step
            "sleeve_slab_cut_x0_final.ply",    // skipped: no step
            "ramp_curve.json",                 // skipped: not .ply
        ] {
            File::create(dir.path().join(name))?;
        }
        let frames = discover_ply_sequence(dir.path())?;
        let names: Vec<String> = frames
            .iter()
            .filter_map(|p| p.file_name().and_then(|n| n.to_str()).map(String::from))
            .collect();
        assert_eq!(
            names,
            vec![
                "sleeve_design_surface_deformed_step_01.ply".to_string(),
                "sleeve_design_surface_deformed_step_02.ply".to_string(),
                "sleeve_design_surface_deformed_step_03.ply".to_string(),
            ],
        );
        Ok(())
    }

    #[test]
    fn discover_ply_sequence_errors_on_no_matches() -> Result<()> {
        let dir = tempfile::tempdir()?;
        // Files present, but none match the step pattern.
        for name in ["body_final.ply", "readout.json", "slab_cut_z0.ply"] {
            File::create(dir.path().join(name))?;
        }
        let err = discover_ply_sequence(dir.path()).err();
        assert!(err.is_some(), "expected error for no matching frames");
        let msg = err.map(|e| e.to_string()).unwrap_or_default();
        assert!(
            msg.contains("no PLY sequence frames"),
            "error should mention missing frames: {msg}",
        );
        Ok(())
    }

    #[test]
    fn discover_ply_sequence_errors_on_missing_dir() {
        let result = discover_ply_sequence(Path::new("/nonexistent/path/for/test"));
        assert!(result.is_err(), "expected error for unreadable dir");
    }

    #[test]
    fn resolve_initial_frame_passes_through_file_input() -> Result<()> {
        let dir = tempfile::tempdir()?;
        let file_path = dir.path().join("one_off.ply");
        File::create(&file_path)?;
        let resolved = resolve_initial_frame(&file_path)?;
        assert_eq!(resolved, file_path);
        Ok(())
    }

    #[test]
    fn resolve_initial_frame_picks_first_sequence_frame() -> Result<()> {
        let dir = tempfile::tempdir()?;
        for name in [
            "body_step_02.ply",
            "body_step_00.ply",
            "body_step_01.ply",
            "body_final.ply",
        ] {
            File::create(dir.path().join(name))?;
        }
        let resolved = resolve_initial_frame(dir.path())?;
        assert_eq!(
            resolved.file_name().and_then(|n| n.to_str()),
            Some("body_step_00.ply"),
        );
        Ok(())
    }

    /// Round-trip a synthetic point-cloud PLY (faces empty, two scalars)
    /// through `save_ply_attributed` → [`load_input`] and verify shape.
    ///
    /// Synthetic rather than fixture-based: the natural sphere-sdf-eval
    /// fixture under `examples/sim-soft/sphere-sdf-eval/out/` is gitignored
    /// and regenerated by `cargo run -p sphere-sdf-eval`. The user-facing
    /// fixture smoke check happens via the stdout vertex/scalar print in
    /// `main.rs` during the visual-review cadence step.
    #[test]
    fn round_trip_point_cloud_with_scalars() -> Result<()> {
        let geometry = IndexedMesh::from_parts(
            vec![
                Point3::new(0.0, 0.0, 0.0),
                Point3::new(1.0, 0.0, 0.0),
                Point3::new(0.0, 1.0, 0.0),
                Point3::new(0.0, 0.0, 1.0),
            ],
            Vec::new(),
        );
        let mut mesh = AttributedMesh::new(geometry);
        // Two extras inserted in reverse-alphabetical order; the loader
        // should still surface them sorted via the `BTreeMap` projection.
        mesh.insert_extra("zeta", vec![-1.0, -2.0, -3.0, -4.0])?;
        mesh.insert_extra("phi", vec![1.0, 2.0, 3.0, 4.0])?;

        let dir = tempfile::tempdir()?;
        let out_path = dir.path().join("test.ply");
        save_ply_attributed(&mesh, &out_path, true)?;

        let input = load_input(&out_path)?;
        assert_eq!(input.mesh.vertex_count(), 4);
        assert_eq!(input.mesh.face_count(), 0);
        assert_eq!(
            input.scalar_names,
            vec!["phi".to_string(), "zeta".to_string()],
        );
        Ok(())
    }
}
