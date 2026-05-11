//! Multi-frame PLY sequence support — Track D in `docs/SIM_SOFT_ROADMAP.md`.
//!
//! When `cf-view <path>` is called with a directory of
//! `*_step_<n>.ply` files (detected by [`crate::discover_ply_sequence`]),
//! `main.rs` inserts a [`Sequence`] resource. The
//! [`sequence_info_panel`] egui system reads it and renders a one-line
//! "Frame N/M" status bar at the bottom of the viewport. Frame-swap and
//! scrub controls (D1.3 / D2) are layered on top of this resource in
//! subsequent commits.

use std::path::{Path, PathBuf};

use bevy::prelude::*;
use bevy_egui::{EguiContexts, egui};

/// Active multi-frame PLY sequence — the lex-sorted frame list plus
/// the index of the frame currently rendered. Inserted by `main.rs`
/// only when the CLI input is a directory; absent in single-file mode.
#[derive(Resource, Debug, Clone)]
pub struct Sequence {
    /// Lex-sorted list of frame paths from [`crate::discover_ply_sequence`].
    /// Guaranteed non-empty at construction (the discoverer errors out
    /// before any [`Sequence`] is constructed).
    pub frames: Vec<PathBuf>,
    /// Zero-based index into [`Self::frames`] for the frame currently
    /// loaded into [`crate::ViewerInput`]. Mutated by future scrub UI
    /// (D2); D1.2 only reads it for the info panel display.
    pub current: usize,
}

impl Sequence {
    /// Build a fresh sequence starting on frame 0.
    #[must_use]
    pub fn new(frames: Vec<PathBuf>) -> Self {
        Self { frames, current: 0 }
    }

    /// Number of frames in the sequence.
    #[must_use]
    pub fn len(&self) -> usize {
        self.frames.len()
    }

    /// `true` if the sequence carries no frames. Logically unreachable
    /// for live `Sequence` instances (the discoverer errors on empty),
    /// retained for `Vec`-mirroring API completeness + clippy's
    /// `len_without_is_empty` lint.
    #[must_use]
    pub fn is_empty(&self) -> bool {
        self.frames.is_empty()
    }

    /// Path of the currently-selected frame. Returns `None` only in the
    /// unreachable empty-sequence case (`Sequence` is never constructed
    /// empty by the discoverer, but `current` could be ill-clamped by a
    /// future caller, so the accessor stays defensive).
    #[must_use]
    pub fn current_path(&self) -> Option<&Path> {
        self.frames.get(self.current).map(PathBuf::as_path)
    }

    /// Filename of the currently-selected frame, decoded as UTF-8.
    /// `None` when the path lacks a filename (logically unreachable for
    /// frames produced by [`crate::discover_ply_sequence`]) or its
    /// filename is non-UTF-8.
    #[must_use]
    pub fn current_name(&self) -> Option<&str> {
        self.current_path()
            .and_then(Path::file_name)
            .and_then(|n| n.to_str())
    }
}

/// egui bottom panel: one-line `Frame N/M — <filename>` sequence
/// status. Runs in [`bevy_egui::EguiPrimaryContextPass`]; gated on
/// the presence of the [`Sequence`] resource so single-file mode pays
/// nothing.
#[allow(clippy::needless_pass_by_value)] // Bevy systems take resources by value
pub fn sequence_info_panel(mut contexts: EguiContexts, sequence: Option<Res<Sequence>>) -> Result {
    let Some(sequence) = sequence else {
        return Ok(());
    };
    let ctx = contexts.ctx_mut()?;
    let len = sequence.len();
    // 1-indexed display so "Frame 1/8" reads naturally; storage stays
    // 0-indexed for `Vec` access.
    let current_human = sequence.current.saturating_add(1);
    let name = sequence.current_name().unwrap_or("?");
    // `min_height` guards against the single-row content collapsing to
    // an invisible strip against the dark Bevy clear color; `.frame()`
    // gives the panel an explicit fill so the divide between geometry
    // and status reads clearly.
    let frame = egui::Frame::default()
        .fill(egui::Color32::from_rgb(28, 28, 32))
        .inner_margin(egui::Margin::symmetric(8, 6));
    egui::TopBottomPanel::bottom("cf-view-sequence")
        .resizable(false)
        .min_height(28.0)
        .frame(frame)
        .show(ctx, |ui| {
            ui.horizontal(|ui| {
                ui.label(egui::RichText::new(format!("Frame {current_human}/{len}")).strong());
                ui.separator();
                ui.label(egui::RichText::new(name).monospace());
            });
        });
    Ok(())
}

#[cfg(test)]
mod tests {
    use super::*;

    fn sample_frames() -> Vec<PathBuf> {
        vec![
            PathBuf::from("/tmp/body_step_00.ply"),
            PathBuf::from("/tmp/body_step_01.ply"),
            PathBuf::from("/tmp/body_step_02.ply"),
        ]
    }

    #[test]
    fn new_starts_on_frame_zero() {
        let s = Sequence::new(sample_frames());
        assert_eq!(s.current, 0);
        assert_eq!(s.len(), 3);
        assert!(!s.is_empty());
    }

    #[test]
    fn current_path_tracks_current_index() {
        let mut s = Sequence::new(sample_frames());
        assert_eq!(
            s.current_path().and_then(Path::file_name),
            Some(std::ffi::OsStr::new("body_step_00.ply")),
        );
        s.current = 2;
        assert_eq!(
            s.current_path().and_then(Path::file_name),
            Some(std::ffi::OsStr::new("body_step_02.ply")),
        );
    }

    #[test]
    fn current_name_returns_utf8_filename() {
        let s = Sequence::new(sample_frames());
        assert_eq!(s.current_name(), Some("body_step_00.ply"));
    }

    #[test]
    fn current_path_returns_none_when_index_out_of_range() {
        let mut s = Sequence::new(sample_frames());
        s.current = 99;
        assert!(s.current_path().is_none());
        assert!(s.current_name().is_none());
    }

    #[test]
    fn is_empty_reflects_frame_count() {
        let empty = Sequence::new(Vec::new());
        assert!(empty.is_empty());
        let nonempty = Sequence::new(sample_frames());
        assert!(!nonempty.is_empty());
    }
}
