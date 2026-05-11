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
pub mod sequence;
pub mod ui;

use std::path::{Path, PathBuf};

use anyhow::{Context, Result, bail};
use bevy::prelude::Resource;
pub use cf_bevy_common::axis::UpAxis;
use mesh_io::load_ply_attributed;
use mesh_types::AttributedMesh;

pub use sequence::Sequence;

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

/// Detected CLI-input form: either a single PLY file or a multi-frame
/// sequence. Sequence variant carries the populated [`Sequence`]
/// resource ready to insert into the Bevy app; `main.rs` only inserts
/// it in directory-input mode so single-file consumers pay nothing.
#[derive(Debug, Clone)]
pub enum InputMode {
    /// Single static PLY — the existing pre-D1 viewer behavior.
    Single(PathBuf),
    /// Lex-sorted sequence of `*_step_<n>.ply` frames, with the initial
    /// `current` index set to 0.
    Sequence(Sequence),
}

impl InputMode {
    /// Path of the frame to load first. For [`InputMode::Single`] this
    /// is the CLI-supplied path; for [`InputMode::Sequence`] it is
    /// frame 0 of the sorted series.
    ///
    /// # Errors
    ///
    /// Returns an error only in the (logically unreachable) case where
    /// a [`Sequence`] was constructed with an empty `frames` vec.
    pub fn initial_frame(&self) -> Result<PathBuf> {
        match self {
            Self::Single(p) => Ok(p.clone()),
            Self::Sequence(s) => s
                .current_path()
                .map(Path::to_path_buf)
                .ok_or_else(|| anyhow::anyhow!("sequence has no current frame")),
        }
    }
}

/// Detect whether the CLI path resolves to a single PLY or a directory
/// of sequence frames.
///
/// Pure detection — no stdout side effects (see `main.rs` for the
/// human-facing detection log).
///
/// # Errors
///
/// Bubbles up [`discover_ply_sequence`]'s errors when the input is a
/// directory containing no matching frames.
pub fn detect_input_mode(path: &Path) -> Result<InputMode> {
    if path.is_dir() {
        let frames = discover_ply_sequence(path)?;
        Ok(InputMode::Sequence(Sequence::new(frames)))
    } else {
        Ok(InputMode::Single(path.to_path_buf()))
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
    fn detect_input_mode_returns_single_for_file_input() -> Result<()> {
        let dir = tempfile::tempdir()?;
        let file_path = dir.path().join("one_off.ply");
        File::create(&file_path)?;
        let mode = detect_input_mode(&file_path)?;
        assert!(
            matches!(&mode, InputMode::Single(p) if p == &file_path),
            "expected Single({}); got {mode:?}",
            file_path.display(),
        );
        assert_eq!(mode.initial_frame()?, file_path);
        Ok(())
    }

    #[test]
    fn detect_input_mode_returns_sequence_for_dir_input() -> Result<()> {
        let dir = tempfile::tempdir()?;
        for name in [
            "body_step_02.ply",
            "body_step_00.ply",
            "body_step_01.ply",
            "body_final.ply",
        ] {
            File::create(dir.path().join(name))?;
        }
        let mode = detect_input_mode(dir.path())?;
        let seq = match &mode {
            InputMode::Sequence(s) => s,
            InputMode::Single(_) => {
                return Err(anyhow::anyhow!(
                    "expected Sequence variant for dir input; got Single",
                ));
            }
        };
        assert_eq!(seq.len(), 3, "should see 3 step frames + 1 skipped");
        assert_eq!(seq.current, 0);
        let initial = mode.initial_frame()?;
        assert_eq!(
            initial.file_name().and_then(|n| n.to_str()),
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
