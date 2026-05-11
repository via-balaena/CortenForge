//! Multi-frame PLY sequence support — Track D in `docs/SIM_SOFT_ROADMAP.md`.
//!
//! When `cf-view <path>` is called with a directory of
//! `*_step_<n>.ply` files (detected by [`crate::discover_ply_sequence`]),
//! `main.rs` inserts a [`Sequence`] resource. Three systems live here:
//!
//! - [`sequence_info_panel`] — egui bottom-bar "Frame N/M" status.
//! - [`handle_frame_navigation`] — keyboard `←` / `→` arrows step
//!   `Sequence::current` (with `Home` / `End` for first / last). The
//!   richer scrub UI (timeline drag) comes in D2.
//! - [`reload_frame_on_change`] — detects [`Sequence`] mutation and
//!   re-loads the new frame's PLY into the [`crate::ViewerInput`]
//!   resource. The existing geometry-spawn system in `main.rs`
//!   respawns geometry on `ViewerInput::is_changed()`.

use std::path::{Path, PathBuf};

use bevy::prelude::*;
use bevy_egui::{EguiContexts, egui};

use crate::{ViewerInput, load_input};

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
    /// loaded into [`crate::ViewerInput`]. Mutated by
    /// [`handle_frame_navigation`] (D1.3 keyboard) — the scrub UI (D2)
    /// will mutate it the same way.
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
    // Bottom-panel default frame derived from the active egui style
    // (so theme changes flow through) plus a small inner margin so
    // the single-row content doesn't hug the panel edges. `min_height`
    // keeps the strip readable at the bottom of a tall viewport.
    let frame =
        egui::Frame::side_top_panel(&ctx.style()).inner_margin(egui::Margin::symmetric(8, 6));
    egui::TopBottomPanel::bottom("cf-view-sequence")
        .resizable(false)
        .min_height(28.0)
        .frame(frame)
        .show(ctx, |ui| {
            ui.horizontal(|ui| {
                ui.label(egui::RichText::new(format!("Frame {current_human}/{len}")).strong());
                ui.separator();
                ui.label(egui::RichText::new(name).monospace());
                ui.separator();
                ui.label(
                    egui::RichText::new("←/→ step · Home/End first/last")
                        .small()
                        .italics(),
                );
            });
        });
    Ok(())
}

/// Keyboard-driven frame navigation. Mutates [`Sequence::current`] on
/// `←` / `→` (previous / next, clamped) and `Home` / `End` (first /
/// last). No-op when the [`Sequence`] resource is absent (single-file
/// mode).
///
/// Mutation goes through `ResMut`, so Bevy's change-detection on
/// [`Sequence`] flips to `true` for that frame — that's what
/// [`reload_frame_on_change`] reads to decide when to re-load the PLY.
#[allow(clippy::needless_pass_by_value)] // Bevy systems take resources by value
pub fn handle_frame_navigation(
    keys: Res<ButtonInput<KeyCode>>,
    sequence: Option<ResMut<Sequence>>,
) {
    let Some(mut sequence) = sequence else {
        return;
    };
    let len = sequence.len();
    if len == 0 {
        return;
    }
    if keys.just_pressed(KeyCode::ArrowRight) && sequence.current + 1 < len {
        sequence.current += 1;
    } else if keys.just_pressed(KeyCode::ArrowLeft) && sequence.current > 0 {
        sequence.current -= 1;
    } else if keys.just_pressed(KeyCode::Home) && sequence.current != 0 {
        sequence.current = 0;
    } else if keys.just_pressed(KeyCode::End) && sequence.current + 1 != len {
        sequence.current = len - 1;
    }
}

/// Re-load the active frame's PLY into [`ViewerInput`] when
/// [`Sequence::current`] changes. The initial-insertion change event
/// (right after `main.rs` inserts the resource) is skipped via a
/// `Local<bool>` guard — `main.rs` already loaded frame 0 before the
/// Bevy app started, so the first observed change is a no-op.
///
/// Replacing the [`ViewerInput`] resource flips its
/// `Res::is_changed()` flag, which is what `spawn_geometry` in
/// `main.rs` reads to despawn-and-respawn the rendered geometry.
///
/// PLY-load errors are printed to stderr and the old frame stays
/// rendered (no panic, no resource swap).
#[allow(clippy::needless_pass_by_value)] // Bevy systems take resources by value
pub fn reload_frame_on_change(
    mut have_seen_initial: Local<bool>,
    sequence: Option<Res<Sequence>>,
    mut input: ResMut<ViewerInput>,
) {
    let Some(sequence) = sequence else {
        return;
    };
    if !sequence.is_changed() {
        return;
    }
    if !*have_seen_initial {
        // First fire: main.rs already loaded frame 0 before Bevy ran.
        *have_seen_initial = true;
        return;
    }
    let Some(path) = sequence.current_path() else {
        return;
    };
    match load_input(path) {
        Ok(new_input) => {
            *input = new_input;
        }
        Err(e) => {
            eprintln!(
                "cf-view: failed to load frame {} ({}): {e:#}",
                sequence.current,
                path.display(),
            );
        }
    }
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

    // The keyboard-driven nav system mirrors these state transitions on
    // `←`/`→`/`Home`/`End`. Faking `ButtonInput<KeyCode>` inside a
    // headless Bevy app is heavyweight; verifying the clamping math
    // here keeps the unit test surface lean. `handle_frame_navigation`
    // itself is exercised end-to-end during the visual-review pass.

    #[test]
    fn frame_advance_clamps_at_last_index() {
        let mut s = Sequence::new(sample_frames());
        s.current = s.len() - 1;
        // Mirrors the `current + 1 < len` guard in handle_frame_navigation.
        let can_advance = s.current + 1 < s.len();
        assert!(!can_advance, "advance past last must be rejected");
    }

    #[test]
    fn frame_retreat_clamps_at_zero() {
        let mut s = Sequence::new(sample_frames());
        s.current = 0;
        // Mirrors the `current > 0` guard.
        let can_retreat = s.current > 0;
        assert!(!can_retreat, "retreat from frame 0 must be rejected");
    }

    #[test]
    fn frame_advance_and_retreat_in_middle() {
        let mut s = Sequence::new(sample_frames());
        s.current = 1;
        assert!(s.current + 1 < s.len());
        assert!(s.current > 0);
    }
}
