//! Multi-frame PLY sequence support — Track D in `docs/SIM_SOFT_ROADMAP.md`.
//!
//! When `cf-view <path>` is called with a directory of
//! `*_step_<n>.ply` files (detected by [`crate::discover_ply_sequence`]),
//! `main.rs` inserts a [`Sequence`] resource (plus a [`Playback`]
//! resource alongside it). Five systems live here:
//!
//! - [`sequence_info_panel`] — egui bottom-bar with a status row
//!   ("Frame N/M" + filename + hint) plus a control row (D2.2:
//!   play/pause button + frame scrub slider). Mutates `Sequence` and
//!   `Playback` through `ResMut`; single-file mode still gates the
//!   panel out entirely.
//! - [`handle_frame_navigation`] — keyboard `←` / `→` arrows step
//!   `Sequence::current` (with `Home` / `End` for first / last).
//! - [`handle_playback_input`] (D2.1) — keyboard `Space` toggles
//!   `Playback::playing`. The scrub UI (D2.2+) mutates the same
//!   `Playback` resource.
//! - [`advance_playback_on_clock`] (D2.1) — when `Playback::playing`,
//!   accumulates `Time::delta` and advances `Sequence::current` at
//!   the configured fps. Wraps to 0 on `Playback::loop_mode`, else
//!   clamps + auto-pauses at the last frame.
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
    /// [`handle_frame_navigation`] (keyboard arrows / Home / End),
    /// [`advance_playback_on_clock`] (auto-advance when
    /// [`Playback::playing`]), and [`sequence_info_panel`] (D2.2
    /// scrub slider).
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

/// Playback state for the D2 timeline UI — auto-advance + speed + loop
/// behaviour driving [`Sequence::current`] alongside the user's manual
/// keyboard / scrub-bar input. Inserted by `main.rs` next to [`Sequence`]
/// only when the CLI input is a directory; absent in single-file mode.
///
/// `playing` and `loop_mode` and `fps` are the user-facing controls
/// (D2.1 ships keyboard play/pause; D2.2+ ships the on-screen widgets).
/// `accumulator` is internal bookkeeping for the clock advance system —
/// seconds of `Time::delta` accumulated since the last frame advance.
#[derive(Resource, Debug, Clone)]
pub struct Playback {
    /// `true` when [`advance_playback_on_clock`] is auto-advancing
    /// [`Sequence::current`]; `false` when paused.
    pub playing: bool,
    /// Target playback rate in frames per second. The clock system
    /// advances exactly one frame whenever the accumulator passes
    /// `1.0 / fps` seconds. Non-positive values are treated as "no-op"
    /// by the clock — the system holds the current frame.
    pub fps: f64,
    /// `true` = wrap to frame 0 after the last frame; `false` = clamp
    /// at the last frame and auto-pause.
    pub loop_mode: bool,
    /// Internal accumulator (seconds since last frame advance). Reset
    /// on play/pause toggle so resuming doesn't auto-advance until a
    /// fresh `1.0 / fps` has elapsed.
    accumulator: f64,
}

impl Default for Playback {
    fn default() -> Self {
        Self {
            playing: false,
            fps: 10.0,
            loop_mode: true,
            accumulator: 0.0,
        }
    }
}

impl Playback {
    /// Flip [`Self::playing`] and drain the internal clock-accumulator
    /// to `0.0` on the pause edge.
    ///
    /// Accumulator drain on pause matters because the clock-advance
    /// system carries the accumulator across paused frames — without
    /// the drain, resuming after a pause would fire the next advance
    /// using whatever sub-period time was banked at pause-time.
    /// Draining gives every play→pause→play cycle a fresh `1/fps`
    /// before the first auto-advance.
    ///
    /// Resuming from pause (false → true) leaves the accumulator at
    /// `0.0` (set on the prior pause-edge), which is the same starting
    /// point the clock system uses on first-ever play.
    pub fn toggle_playing(&mut self) {
        self.playing = !self.playing;
        if !self.playing {
            self.accumulator = 0.0;
        }
    }
}

/// One step of the clock-advance state machine. Pure function over the
/// previous `(accumulator, current, playing)` plus the inputs
/// `(delta, fps, len, loop_mode)`; returns the next state.
///
/// Degenerate inputs (`fps <= 0`, `len == 0`, `delta < 0`, non-finite)
/// pass through as no-ops — the clock holds the current frame without
/// touching the play state.
///
/// At end-of-sequence:
/// - `loop_mode = true` → wraps `current` to `0` and continues at the
///   target rate (accumulator drains across the wrap).
/// - `loop_mode = false` → clamps `current` to `len - 1`, drops the
///   accumulator to `0.0`, and returns `playing = false` so the next
///   frame the system idles until the user resumes.
fn advance_step(
    accumulator: f64,
    delta: f64,
    fps: f64,
    current: usize,
    len: usize,
    loop_mode: bool,
) -> (usize, f64, bool) {
    if len == 0 || fps <= 0.0 || !fps.is_finite() || !delta.is_finite() || delta < 0.0 {
        return (current, accumulator, true);
    }
    let period = 1.0 / fps;
    let mut acc = accumulator + delta;
    let mut cur = current;
    let mut playing = true;
    while acc >= period {
        acc -= period;
        let next = cur + 1;
        if next < len {
            cur = next;
        } else if loop_mode {
            cur = 0;
        } else {
            cur = len - 1;
            acc = 0.0;
            playing = false;
            break;
        }
    }
    (cur, acc, playing)
}

/// Spacebar toggles [`Playback::playing`] via
/// [`Playback::toggle_playing`] (which also drains the accumulator on
/// pause). Gated on `!egui_ctx.wants_keyboard_input()` so spacebar
/// presses targeting a focused egui widget (e.g. the on-screen
/// play/pause button after a click, or any future text-input field)
/// don't double-fire — the focused widget handles the input itself.
///
/// No-op when the [`Playback`] resource is absent (single-file mode).
#[allow(clippy::needless_pass_by_value)] // Bevy systems take resources by value
pub fn handle_playback_input(
    keys: Res<ButtonInput<KeyCode>>,
    playback: Option<ResMut<Playback>>,
    mut contexts: EguiContexts,
) -> Result {
    let Some(mut playback) = playback else {
        return Ok(());
    };
    // If egui has keyboard focus (button, slider, text field, ...),
    // let the focused widget handle the input — otherwise spacebar
    // fires both Bevy's path and egui's focus-activation, netting two
    // toggles in one frame.
    if contexts.ctx_mut()?.wants_keyboard_input() {
        return Ok(());
    }
    if keys.just_pressed(KeyCode::Space) {
        playback.toggle_playing();
    }
    Ok(())
}

/// Clock-driven advance of [`Sequence::current`] when
/// [`Playback::playing`] is `true`. Accumulates `Time::delta_secs_f64`;
/// when the accumulator passes `1.0 / Playback::fps`, advances the
/// frame index through the private `advance_step` state machine
/// (handles wrap / clamp / degenerate-input cases).
///
/// Only writes to `sequence.current` when the index actually changes,
/// so change-detection on [`Sequence`] (read by
/// [`reload_frame_on_change`]) doesn't fire on every paused frame.
///
/// No-op when either resource is absent (single-file mode) or when
/// playback is paused.
#[allow(clippy::needless_pass_by_value)] // Bevy systems take resources by value
pub fn advance_playback_on_clock(
    time: Res<Time>,
    sequence: Option<ResMut<Sequence>>,
    playback: Option<ResMut<Playback>>,
) {
    let (Some(mut sequence), Some(mut playback)) = (sequence, playback) else {
        return;
    };
    if !playback.playing {
        return;
    }
    let (new_current, new_acc, keep_playing) = advance_step(
        playback.accumulator,
        time.delta_secs_f64(),
        playback.fps,
        sequence.current,
        sequence.len(),
        playback.loop_mode,
    );
    playback.accumulator = new_acc;
    playback.playing = keep_playing;
    if sequence.current != new_current {
        sequence.current = new_current;
    }
}

/// egui bottom panel — three-row timeline + playback UI. Runs in
/// [`bevy_egui::EguiPrimaryContextPass`]; gated on the presence of the
/// [`Sequence`] resource so single-file mode pays nothing.
///
/// # Layout
///
/// 1. **Status row** — `Frame N/M | filename | keyboard hint`.
/// 2. **Playback row (D2.2)** — play/pause button + frame scrub
///    slider. The button calls [`Playback::toggle_playing`]; the
///    scrub slider mutates [`Sequence::current`] (1-indexed display,
///    0-indexed storage). Slider hidden when `len < 2` (the range
///    would collapse otherwise).
/// 3. **Speed / loop row (D2.3)** — fps slider (`1..=30`, integer
///    display, stored as `f64`) bound to [`Playback::fps`] + loop
///    checkbox bound to [`Playback::loop_mode`].
///
/// # Change detection
///
/// The scrub slider's write to `sequence.current` is gated on
/// `response.changed()` (egui's "user actually moved it" signal) and
/// the inner `if sequence.current != new_current` equality check, so
/// a steady-state re-render at frame N doesn't flag the resource as
/// changed.
///
/// The fps and loop_mode mutations go through `&mut` slider/checkbox
/// bindings — egui only writes back when the user interacts, so
/// these likewise don't fire change-detection on quiescent frames.
#[allow(clippy::needless_pass_by_value)] // Bevy systems take resources by value
pub fn sequence_info_panel(
    mut contexts: EguiContexts,
    sequence: Option<ResMut<Sequence>>,
    mut playback: Option<ResMut<Playback>>,
) -> Result {
    let Some(mut sequence) = sequence else {
        return Ok(());
    };
    let ctx = contexts.ctx_mut()?;
    let len = sequence.len();
    // 1-indexed display so "Frame 1/8" reads naturally; storage stays
    // 0-indexed for `Vec` access.
    let current_human = sequence.current.saturating_add(1);
    let name = sequence.current_name().unwrap_or("?").to_string();
    // Bottom-panel default frame derived from the active egui style
    // (so theme changes flow through) plus a small inner margin so
    // the single-row content doesn't hug the panel edges. `min_height`
    // keeps the two-row layout readable at the bottom of a tall
    // viewport.
    let frame =
        egui::Frame::side_top_panel(&ctx.style()).inner_margin(egui::Margin::symmetric(8, 6));
    egui::TopBottomPanel::bottom("cf-view-sequence")
        .resizable(false)
        .min_height(76.0)
        .frame(frame)
        .show(ctx, |ui| {
            // Row 1 — status: Frame N/M + filename + hint row.
            ui.horizontal(|ui| {
                ui.label(egui::RichText::new(format!("Frame {current_human}/{len}")).strong());
                ui.separator();
                ui.label(egui::RichText::new(name).monospace());
                ui.separator();
                ui.label(
                    egui::RichText::new(
                        "Space play/pause · ←/→ step · Home/End first/last · drag to scrub",
                    )
                    .small()
                    .italics(),
                );
            });
            ui.add_space(2.0);
            // Row 2 — controls: play/pause button + frame scrub slider.
            //
            // The scrub mutation goes through `response.changed()` so a
            // pristine render (slider re-built from the current value
            // each frame) doesn't fake a change-detection fire. The
            // play/pause button mirrors `handle_playback_input`'s
            // accumulator-drain on pause.
            ui.horizontal(|ui| {
                if let Some(playback) = playback.as_deref_mut() {
                    let label = if playback.playing {
                        "⏸ Pause"
                    } else {
                        "▶ Play"
                    };
                    if ui.button(label).clicked() {
                        playback.toggle_playing();
                    }
                    ui.separator();
                }
                // Scrub slider — only meaningful when there's something
                // to scrub through. For len <= 1 the range collapses and
                // egui would draw a degenerate slider; skip it.
                if len >= 2 {
                    let mut scrub_human = current_human;
                    let response = ui.add_sized(
                        [ui.available_width(), 20.0],
                        egui::Slider::new(&mut scrub_human, 1..=len).show_value(false),
                    );
                    if response.changed() {
                        let new_current = scrub_human.saturating_sub(1).min(len.saturating_sub(1));
                        if sequence.current != new_current {
                            sequence.current = new_current;
                        }
                    }
                }
            });
            ui.add_space(2.0);
            // Row 3 (D2.3) — speed + loop controls. Linear FPS slider
            // 1..=30 covers slow-frame inspection (1 fps = 1s/frame) up
            // through fast playback (30 fps = ~33 ms/frame). Default 10
            // fps gives row 25's 8-frame ramp an ~0.8s end-to-end run.
            // egui's `Slider::clamp_to_range` (default true) keeps the
            // value within the band even when typed.
            //
            // Loop checkbox flips `Playback::loop_mode`; the clock
            // system reads it at next advance, so the toggle takes
            // effect immediately on the next frame boundary.
            if let Some(playback) = playback.as_deref_mut() {
                ui.horizontal(|ui| {
                    ui.label("Speed");
                    ui.add(
                        egui::Slider::new(&mut playback.fps, 1.0..=30.0)
                            .suffix(" fps")
                            .integer(),
                    );
                    ui.separator();
                    ui.checkbox(&mut playback.loop_mode, "Loop");
                });
            }
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

    // advance_step (D2.1): clock-driven frame advance. Pure-function
    // unit tests on the state machine; the Bevy system wrapper is
    // exercised end-to-end during the visual-review pass (same
    // rationale as `handle_frame_navigation` above).

    /// Zero delta with a sub-period accumulator → no advance,
    /// accumulator unchanged. (A multi-period seeded accumulator would
    /// still drain — see `advance_step_drains_seeded_accumulator`.)
    #[test]
    fn advance_step_zero_delta_sub_period_no_op() {
        // fps=10 → period=0.1; accumulator=0.05 < 0.1 → no advance.
        let (cur, acc, playing) = advance_step(0.05, 0.0, 10.0, 2, 8, true);
        assert_eq!(cur, 2);
        assert!((acc - 0.05).abs() < 1e-12);
        assert!(playing);
    }

    /// Zero delta with a multi-period seeded accumulator drains across
    /// the accumulated time — covers the case where a future caller
    /// (e.g. scrub-then-resume) seeds accumulator from elsewhere.
    #[test]
    fn advance_step_drains_seeded_accumulator() {
        // fps=10 → period=0.1; accumulator=0.5 → drains across 5
        // periods, advancing 5 frames from cur=2 to cur=7.
        let (cur, acc, playing) = advance_step(0.5, 0.0, 10.0, 2, 8, true);
        assert_eq!(cur, 7);
        assert!(acc.abs() < 1e-9, "expected ~0, got {acc}");
        assert!(playing);
    }

    /// Small delta below the period → accumulator grows, no advance.
    #[test]
    fn advance_step_subperiod_delta_accumulates_without_advance() {
        // fps=10 → period=0.1; 0.05 < 0.1 → no advance.
        let (cur, acc, playing) = advance_step(0.0, 0.05, 10.0, 3, 8, true);
        assert_eq!(cur, 3);
        assert!((acc - 0.05).abs() < 1e-12);
        assert!(playing);
    }

    /// Delta crosses one period → advance by 1, accumulator drains.
    #[test]
    fn advance_step_one_period_advances_once() {
        // fps=10 → period=0.1; accumulator starts at 0.06, delta=0.05 →
        // total=0.11 → crosses one period, advance once, new acc=0.01.
        let (cur, acc, playing) = advance_step(0.06, 0.05, 10.0, 3, 8, true);
        assert_eq!(cur, 4);
        assert!((acc - 0.01).abs() < 1e-12, "expected ~0.01, got {acc}");
        assert!(playing);
    }

    /// Large delta crosses multiple periods → multi-frame advance.
    #[test]
    fn advance_step_multi_period_advances_multiple_frames() {
        // fps=10 → period=0.1; accumulator=0, delta=0.35 → advance 3
        // frames, new acc=0.05.
        let (cur, acc, playing) = advance_step(0.0, 0.35, 10.0, 0, 8, true);
        assert_eq!(cur, 3);
        assert!((acc - 0.05).abs() < 1e-9, "expected ~0.05, got {acc}");
        assert!(playing);
    }

    /// End-of-sequence with loop_mode=true → wraps to 0, keeps playing.
    #[test]
    fn advance_step_wraps_at_end_when_loop() {
        // fps=10, period=0.1; at frame 7 of 8 (last); delta=0.1 → advance,
        // next index would be 8 → wraps to 0.
        let (cur, acc, playing) = advance_step(0.0, 0.1, 10.0, 7, 8, true);
        assert_eq!(cur, 0);
        assert!(acc.abs() < 1e-9);
        assert!(playing);
    }

    /// End-of-sequence with loop_mode=false → clamps + auto-pauses.
    #[test]
    fn advance_step_clamps_and_pauses_at_end_when_no_loop() {
        let (cur, acc, playing) = advance_step(0.0, 0.1, 10.0, 7, 8, false);
        assert_eq!(cur, 7);
        assert!(acc.abs() < 1e-12);
        assert!(!playing, "should auto-pause at end when loop_mode=false");
    }

    /// Multi-period delta + no loop → clamps at the first end crossing,
    /// dropping the remaining accumulator so resume doesn't immediately
    /// re-advance past the boundary.
    #[test]
    fn advance_step_no_loop_drains_overshoot() {
        // fps=10, period=0.1; start at frame 6 of 8 with delta=1.0 → would
        // advance 10 frames if unconstrained; clamps at 7 and pauses.
        let (cur, acc, playing) = advance_step(0.0, 1.0, 10.0, 6, 8, false);
        assert_eq!(cur, 7);
        assert!(acc.abs() < 1e-12);
        assert!(!playing);
    }

    /// Degenerate fps (zero, negative, NaN) → no-op.
    #[test]
    fn advance_step_degenerate_fps_no_op() {
        for &bad_fps in &[0.0, -10.0, f64::NAN, f64::INFINITY] {
            let (cur, acc, playing) = advance_step(0.5, 0.1, bad_fps, 2, 8, true);
            assert_eq!(cur, 2, "bad fps {bad_fps} must not advance frame");
            assert!((acc - 0.5).abs() < 1e-12);
            assert!(playing);
        }
    }

    /// Empty sequence → no-op (defensive — the discoverer guarantees
    /// non-empty, but advance_step stays robust if a future caller
    /// breaks the invariant).
    #[test]
    fn advance_step_empty_sequence_no_op() {
        let (cur, acc, playing) = advance_step(0.0, 1.0, 10.0, 0, 0, true);
        assert_eq!(cur, 0);
        assert!(acc.abs() < 1e-12);
        assert!(playing);
    }

    /// Negative or non-finite delta → no-op (defensive).
    #[test]
    fn advance_step_degenerate_delta_no_op() {
        for &bad_delta in &[-1.0, f64::NAN, f64::INFINITY] {
            let (cur, acc, _) = advance_step(0.0, bad_delta, 10.0, 2, 8, true);
            assert_eq!(cur, 2, "bad delta {bad_delta} must not advance frame");
            assert!(acc.abs() < 1e-12);
        }
    }

    /// Accumulator value exactly at the period boundary advances once
    /// — the `while acc >= period` (not `>`) comparison takes effect
    /// at the boundary.
    #[test]
    fn advance_step_at_period_boundary_advances_once() {
        // fps=10 → period=0.1; accumulator=0.1 exactly, delta=0 → acc
        // hits the >= period boundary on the first iteration, advances
        // once, then acc=0 fails the >= check.
        let (cur, acc, playing) = advance_step(0.1, 0.0, 10.0, 2, 8, true);
        assert_eq!(cur, 3);
        assert!(acc.abs() < 1e-12);
        assert!(playing);
    }

    /// At end of sequence with `loop_mode=true` and a multi-period
    /// delta, the wrap is followed by continued advance from frame 0
    /// — the `while acc >= period` loop doesn't break on wrap.
    #[test]
    fn advance_step_wraps_and_continues_advancing() {
        // fps=10 → period=0.1; start at frame 6 of 8 with delta=0.5
        // (= 5 periods). Path: 6 → 7 → wrap to 0 → 1 → 2 → 3, end at
        // frame 3 with accumulator drained.
        let (cur, acc, playing) = advance_step(0.0, 0.5, 10.0, 6, 8, true);
        assert_eq!(cur, 3, "wrap should be followed by continued advance");
        assert!(
            acc.abs() < 1e-9,
            "accumulator should drain across 5 periods"
        );
        assert!(playing);
    }

    /// Sequential calls thread the accumulator across invocations:
    /// two consecutive sub-period deltas accumulate to trigger an
    /// advance on the second call. Documents the threading contract
    /// the Bevy system relies on.
    #[test]
    fn advance_step_sequential_calls_thread_accumulator() {
        // fps=10 → period=0.1.
        // Call 1: delta=0.06, acc=0+0.06=0.06 → below period, no advance.
        let (cur1, acc1, playing1) = advance_step(0.0, 0.06, 10.0, 2, 8, true);
        assert_eq!(cur1, 2);
        assert!((acc1 - 0.06).abs() < 1e-12);
        assert!(playing1);
        // Call 2: delta=0.05, acc=0.06+0.05=0.11 → crosses period,
        // advances once, leaves acc=0.01.
        let (cur2, acc2, playing2) = advance_step(acc1, 0.05, 10.0, cur1, 8, true);
        assert_eq!(cur2, 3);
        assert!((acc2 - 0.01).abs() < 1e-12);
        assert!(playing2);
    }

    // Playback resource defaults are part of the public contract — the
    // UI sliders in D2.3 read these as initial values.

    #[test]
    fn playback_default_starts_paused_at_ten_fps_with_loop() {
        let p = Playback::default();
        assert!(!p.playing);
        assert!((p.fps - 10.0).abs() < 1e-12);
        assert!(p.loop_mode);
        assert!(p.accumulator.abs() < 1e-12);
    }

    // Playback::toggle_playing — the shared toggle path called by
    // both the spacebar handler and the on-screen play/pause button.

    /// Toggling a paused Playback to playing leaves the accumulator
    /// where it was (the resume side of the cycle starts from a
    /// fresh-on-pause `0.0` set by the prior pause-edge).
    #[test]
    fn toggle_playing_paused_to_playing_preserves_accumulator() {
        let mut p = Playback {
            playing: false,
            accumulator: 0.0, // freshly-paused state
            ..Playback::default()
        };
        p.toggle_playing();
        assert!(p.playing);
        assert!(p.accumulator.abs() < 1e-12);
    }

    /// Toggling a playing Playback to paused drains the accumulator
    /// to `0.0` so resuming requires a fresh `1/fps` before the next
    /// auto-advance.
    #[test]
    fn toggle_playing_playing_to_paused_drains_accumulator() {
        let mut p = Playback {
            playing: true,
            accumulator: 0.07, // mid-period when paused
            ..Playback::default()
        };
        p.toggle_playing();
        assert!(!p.playing);
        assert!(
            p.accumulator.abs() < 1e-12,
            "pause-edge must drain accumulator, found {}",
            p.accumulator
        );
    }

    /// Double-toggle (play → pause → play) lands back at playing with
    /// the accumulator drained — proves the drain happens on every
    /// pause edge, not just first.
    #[test]
    fn toggle_playing_round_trip_drains_accumulator() {
        let mut p = Playback {
            playing: true,
            accumulator: 0.07,
            ..Playback::default()
        };
        p.toggle_playing(); // playing → paused (drains)
        p.toggle_playing(); // paused → playing (no drain, but already 0)
        assert!(p.playing);
        assert!(p.accumulator.abs() < 1e-12);
    }
}
