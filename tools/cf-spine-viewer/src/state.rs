//! The Studio's top-level modes and the input that moves between them.
//!
//! [`StudioState::Design`] (the default) is where the disc will be painted — for
//! now a prompt that dispatches the solve on `S`. [`StudioState::Solving`] is the
//! background coupled-FSU assembly (see [`crate::solve`]). [`StudioState::Simulate`]
//! is the moment-driven replay. The L4/L5 bones + camera are spawned once at
//! startup and persist; only the disc + the replay/overlay/panel systems are
//! mode-scoped (see `run_app` in `main`).

use bevy::prelude::*;
use bevy_egui::{EguiContexts, egui};

use crate::solve::SolveError;

/// The Studio's three top-level modes.
#[derive(States, Default, Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub(crate) enum StudioState {
    /// Prepare the disc (paint front-end lands in the next rung); `S` solves.
    #[default]
    Design,
    /// The background coupled-FSU assembly is in flight.
    Solving,
    /// Replay the solved moment-driven coupled FSU.
    Simulate,
}

/// `Esc` quits from Design (the top-level mode). `S`→solve is handled by
/// [`crate::solve::start_solve`].
#[allow(clippy::needless_pass_by_value)] // Bevy systems take resources by value.
pub(crate) fn design_quit(keys: Res<ButtonInput<KeyCode>>, mut exit: MessageWriter<AppExit>) {
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

/// The Design-mode prompt: dispatch the solve, plus the last solve's error if it
/// failed. (Becomes the paint front-end in the next rung.)
pub(crate) fn design_panel(
    mut contexts: EguiContexts,
    error: Res<SolveError>,
) -> bevy::ecs::error::Result {
    let ctx = contexts.ctx_mut()?;
    egui::CentralPanel::default().show(ctx, |ui| {
        ui.add_space(48.0);
        ui.vertical_centered(|ui| {
            ui.heading("Design");
            ui.label("Painting the endplate patches lands next rung.");
            ui.add_space(8.0);
            ui.label("S  assemble + solve the coupled FSU (~90 s)    ·    Esc  quit");
            if let Some(msg) = &error.0 {
                ui.add_space(16.0);
                ui.colored_label(
                    egui::Color32::from_rgb(230, 120, 90),
                    format!("solve failed: {msg}"),
                );
            }
        });
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
        });
    });
    Ok(())
}
