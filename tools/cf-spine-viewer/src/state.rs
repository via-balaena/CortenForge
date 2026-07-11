//! The Studio's top-level modes and the input that moves between them.
//!
//! [`StudioState::Design`] (the default) paints the two endplate patches (the
//! [`cf_mesh_paint`] brush) and, on `S`, lofts + solves them.
//! [`StudioState::Solving`] is the background coupled-FSU assembly (see
//! [`crate::solve`]). [`StudioState::Simulate`] is the moment-driven replay. The
//! camera persists; the paint bodies (Design) and the bones+disc (Simulate) are
//! mode-scoped (see `run_app` in `main`).

use bevy::prelude::*;
use bevy_egui::{EguiContexts, egui};
use cf_mesh_paint::prelude::{Brush, BrushMode, NormalFilter, PaintBody, PaintTargets};

use crate::solve::SolveError;

/// The Studio's three top-level modes.
#[derive(States, Default, Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub(crate) enum StudioState {
    /// Paint the two endplate patches; `S` lofts + solves them.
    #[default]
    Design,
    /// The background coupled-FSU assembly is in flight.
    Solving,
    /// Replay the solved moment-driven coupled FSU.
    Simulate,
}

/// `Esc` quits — wired for Design and Solving (Simulate's `Esc` returns to
/// Design instead). Quitting mid-solve is safe: the bones keep the 3D world
/// non-empty and the background task is abandoned at process exit.
#[allow(clippy::needless_pass_by_value)] // Bevy systems take resources by value.
pub(crate) fn quit_on_esc(keys: Res<ButtonInput<KeyCode>>, mut exit: MessageWriter<AppExit>) {
    if keys.just_pressed(KeyCode::Escape) {
        exit.write(AppExit::Success);
    }
}

/// `Esc` / `D` returns from Simulate to Design (to re-solve).
#[allow(clippy::needless_pass_by_value)] // Bevy systems take resources by value.
pub(crate) fn simulate_back(
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
            ui.label("Paint both endplate patches, then press S.");
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
            ui.monospace(format!("{:<14}{}", "loft + solve", "S  (~90 s)"));
            ui.monospace(format!("{:<14}{}", "quit", "Esc"));
            if let Some(msg) = &error.0 {
                ui.separator();
                ui.colored_label(
                    egui::Color32::from_rgb(230, 120, 90),
                    format!("solve failed: {msg}"),
                );
            }
        });
    Ok(())
}

/// The Solving-mode overlay: a spinner while the background solve runs.
pub(crate) fn solving_panel(mut contexts: EguiContexts) -> bevy::ecs::error::Result {
    let ctx = contexts.ctx_mut()?;
    egui::CentralPanel::default().show(ctx, |ui| {
        ui.add_space(48.0);
        ui.vertical_centered(|ui| {
            ui.heading("Solving…");
            ui.add_space(8.0);
            ui.spinner();
            ui.add_space(8.0);
            ui.label("Assembling + solving the coupled FSU (~90 s). This window stays responsive.");
            ui.add_space(8.0);
            ui.label("Esc  quit");
        });
    });
    Ok(())
}
