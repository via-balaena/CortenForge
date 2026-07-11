//! The Studio's top-level modes and the input that moves between them.
//!
//! [`StudioState::Design`] (the default) paints the two endplate patches (the
//! [`cf_mesh_paint`] brush); `Enter` lofts + **builds** them ([`StudioState::Building`],
//! the ~5 s background assembly) and shows the conformed disc ([`StudioState::Preview`]).
//! `S` in Preview runs the ~85 s **capture** ([`StudioState::Solving`]) and replays it
//! ([`StudioState::Simulate`]). Splitting build from capture catches a bad painting in
//! seconds (see [`crate::solve`]). The camera persists; the paint bodies (Design →
//! Solving) and the bones+disc (Simulate) are mode-scoped, with the Preview disc additive
//! (see `run_app` in `main`).

use bevy::prelude::*;
use bevy_egui::{EguiContexts, egui};
use cf_mesh_paint::prelude::{Brush, BrushMode, NormalFilter, PaintBody, PaintTargets};

use crate::solve::{HeldBuildSlot, SolveError};

/// The Studio's top-level modes, in the order the tweak loop walks them.
#[derive(States, Default, Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub(crate) enum StudioState {
    /// Paint the two endplate patches; `Enter` lofts + builds them.
    #[default]
    Design,
    /// The background ~5 s build phase (tet-mesh + `k_disc` probe + guards) is in flight.
    Building,
    /// The build succeeded: inspect the conformed disc. `S` captures, `Esc`/`D` repaints.
    Preview,
    /// The background ~85 s capture phase (the moment ramp) is in flight.
    Solving,
    /// Replay the solved moment-driven coupled FSU.
    Simulate,
}

/// `Esc` quits — wired for the top-level background states Design, Building, and Solving
/// (Preview and Simulate's `Esc` return to Design instead). Quitting mid-build/-solve is
/// safe: the paint bodies keep the 3D world non-empty and the background task is abandoned
/// at process exit.
#[allow(clippy::needless_pass_by_value)] // Bevy systems take resources by value.
pub(crate) fn quit_on_esc(keys: Res<ButtonInput<KeyCode>>, mut exit: MessageWriter<AppExit>) {
    if keys.just_pressed(KeyCode::Escape) {
        exit.write(AppExit::Success);
    }
}

/// `Esc` / `D` returns to Design — from Preview (to repaint, discarding the held build) or
/// from Simulate (to re-solve). The held build (and any in-flight task) is dropped by
/// `discard_pending_work` `OnEnter(Design)`.
#[allow(clippy::needless_pass_by_value)] // Bevy systems take resources by value.
pub(crate) fn back_to_design(
    keys: Res<ButtonInput<KeyCode>>,
    mut next: ResMut<NextState<StudioState>>,
) {
    if keys.just_pressed(KeyCode::Escape) || keys.just_pressed(KeyCode::KeyD) {
        next.set(StudioState::Design);
    }
}

/// The Design-mode paint HUD: live brush status + controls + the `S` prompt,
/// read from the [`cf_mesh_paint`] plugin's public state. A right side panel, so
/// the vertebrae stay visible + paintable. Shows the last solve's error if it
/// failed.
#[allow(clippy::needless_pass_by_value)] // Bevy systems take resources by value.
pub(crate) fn design_panel(
    mut contexts: EguiContexts,
    mode: Res<BrushMode>,
    brush: Res<Brush>,
    filter: Res<NormalFilter>,
    targets: Res<PaintTargets>,
    error: Res<SolveError>,
    q_bodies: Query<&PaintBody>,
) -> bevy::ecs::error::Result {
    let ctx = contexts.ctx_mut()?;
    egui::SidePanel::right("design-panel")
        .resizable(false)
        .default_width(260.0)
        .show(ctx, |ui| {
            ui.heading("Design — paint the disc");
            ui.label("Paint both endplate patches, then press Enter to build.");
            ui.separator();

            let active_entity = targets.active_entity();
            let active_name = active_entity
                .and_then(|e| q_bodies.get(e).ok())
                .map_or("?", PaintBody::name);
            let mode = match *mode {
                BrushMode::Paint => "PAINT",
                BrushMode::Erase => "ERASE",
            };
            let filter_status = if filter.enabled {
                format!("on ({:.0} deg)", filter.max_angle_deg)
            } else {
                "off".to_string()
            };
            ui.label(format!("active   {active_name}"));
            ui.label(format!("mode     {mode}"));
            ui.label(format!("brush    {:.1} mm", brush.radius));
            ui.label(format!("filter   {filter_status}"));
            ui.separator();

            ui.label("painted faces");
            for &e in targets.entities() {
                if let Ok(b) = q_bodies.get(e) {
                    let row = format!("  {:<8}{}", b.name(), b.painted_count());
                    // The active body (the one being rendered/painted) in white; the
                    // hidden one stays the default grey.
                    if Some(e) == active_entity {
                        ui.colored_label(egui::Color32::WHITE, row);
                    } else {
                        ui.label(row);
                    }
                }
            }
            ui.separator();

            for (action, key) in [
                ("orbit", "LMB drag"),
                ("pan", "RMB drag"),
                ("zoom", "scroll"),
                ("paint", "Shift + drag"),
                ("switch body", "Tab"),
                ("erase", "E"),
                ("normal filter", "N"),
                ("tolerance", "- / ="),
                ("brush size", "[ / ]"),
                ("undo", "Ctrl + Z"),
                ("clear body", "C"),
            ] {
                ui.monospace(format!("{action:<14}{key}"));
            }
            ui.separator();
            ui.monospace(format!("{:<14}{}", "build disc", "Enter  (~5 s)"));
            ui.monospace(format!("{:<14}{}", "quit", "Esc"));
            if let Some(msg) = &error.0 {
                ui.separator();
                ui.colored_label(egui::Color32::from_rgb(230, 120, 90), format!("⚠ {msg}"));
            }
        });
    Ok(())
}

/// A centred spinner overlay while a background phase runs — shared shape for Building and
/// Solving, differing only in `heading` and `detail` (each phase stays responsive; `Esc`
/// quits). `Esc`'s quit is wired by [`quit_on_esc`] for both states.
fn spinner_panel(
    contexts: &mut EguiContexts,
    heading: &str,
    detail: &str,
) -> bevy::ecs::error::Result {
    let ctx = contexts.ctx_mut()?;
    egui::CentralPanel::default().show(ctx, |ui| {
        ui.add_space(48.0);
        ui.vertical_centered(|ui| {
            ui.heading(heading);
            ui.add_space(8.0);
            ui.spinner();
            ui.add_space(8.0);
            ui.label(detail);
            ui.add_space(8.0);
            ui.label("Esc  quit");
        });
    });
    Ok(())
}

/// The Building-mode overlay: a spinner while the ~5 s build phase runs.
pub(crate) fn building_panel(mut contexts: EguiContexts) -> bevy::ecs::error::Result {
    spinner_panel(
        &mut contexts,
        "Building…",
        "Tet-meshing + bonding the disc (~5 s). This window stays responsive.",
    )
}

/// The Solving-mode overlay: a spinner while the ~85 s capture phase runs.
pub(crate) fn solving_panel(mut contexts: EguiContexts) -> bevy::ecs::error::Result {
    spinner_panel(
        &mut contexts,
        "Solving…",
        "Capturing the coupled FSU moment ramp (~85 s). This window stays responsive.",
    )
}

/// The Preview-mode panel: the conformed disc is on screen (a static teal lens seated on
/// the painted vertebrae). `S` runs the ~85 s capture; `Esc`/`D` returns to Design to
/// repaint. Any co-registration warnings from the build are surfaced here.
#[allow(clippy::needless_pass_by_value)] // Bevy systems take resources by value.
pub(crate) fn preview_panel(
    mut contexts: EguiContexts,
    held: NonSend<HeldBuildSlot>,
) -> bevy::ecs::error::Result {
    let ctx = contexts.ctx_mut()?;
    egui::SidePanel::right("preview-panel")
        .resizable(false)
        .default_width(260.0)
        .show(ctx, |ui| {
            ui.heading("Preview — the built disc");
            ui.label("The conformed disc is seated on the vertebrae.");
            ui.separator();
            ui.monospace(format!("{:<14}{}", "simulate", "S  (~85 s)"));
            ui.monospace(format!("{:<14}{}", "repaint", "Esc / D"));
            if let Some(build) = held.0.as_ref() {
                if !build.warnings.is_empty() {
                    ui.separator();
                    ui.label("co-registration warnings:");
                    for w in &build.warnings {
                        ui.colored_label(egui::Color32::from_rgb(230, 200, 90), format!("• {w}"));
                    }
                }
            }
        });
    Ok(())
}
