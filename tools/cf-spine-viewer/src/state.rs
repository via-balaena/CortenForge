//! The Studio's two top-level modes and the input that moves between them.
//!
//! [`StudioState::Simulate`] is the coupled-FSU replay (the current viewer);
//! [`StudioState::Design`] is where the disc gets painted — a placeholder in this
//! rung, wired to the `cf-mesh-paint` front-end in the next. The FSU scene is
//! spawned once at startup and persists; only the replay/overlay/panel systems
//! are gated to Simulate (see `run_app` in `main`), so in Design the FSU freezes
//! behind the opaque Design panel. (Per-state spawn/despawn waits for PR3d, where
//! Design gets its own paint entities — and it avoids a macOS quit-deadlock that
//! tearing the whole 3D scene down would otherwise cause.)

use bevy::prelude::*;
use bevy_egui::{EguiContexts, egui};

/// The Studio's two top-level modes.
#[derive(States, Default, Debug, Clone, Copy, PartialEq, Eq, Hash)]
pub(crate) enum StudioState {
    /// Paint the endplate patches to loft the disc (placeholder until PR3d).
    Design,
    /// Replay the moment-driven coupled FSU — the current viewer.
    #[default]
    Simulate,
}

/// Move between modes: `S` enters Simulate from Design; `Esc` / `D` returns to
/// Design from Simulate; `Esc` from Design quits. (In the next rung `S` triggers
/// the async solve from the painted disc; here it just re-enters the pre-built
/// scene.)
#[allow(clippy::needless_pass_by_value)] // Bevy systems take resources by value.
pub(crate) fn handle_state_transitions(
    keys: Res<ButtonInput<KeyCode>>,
    state: Res<State<StudioState>>,
    mut next: ResMut<NextState<StudioState>>,
    mut exit: MessageWriter<AppExit>,
) {
    match state.get() {
        StudioState::Simulate => {
            if keys.just_pressed(KeyCode::Escape) || keys.just_pressed(KeyCode::KeyD) {
                next.set(StudioState::Design);
            }
        }
        StudioState::Design => {
            if keys.just_pressed(KeyCode::KeyS) {
                next.set(StudioState::Simulate);
            } else if keys.just_pressed(KeyCode::Escape) {
                exit.write(AppExit::Success);
            }
        }
    }
}

/// The Design-mode placeholder: a centered prompt until the paint front-end
/// lands. Only runs in [`StudioState::Design`].
pub(crate) fn design_panel(mut contexts: EguiContexts) -> bevy::ecs::error::Result {
    let ctx = contexts.ctx_mut()?;
    egui::CentralPanel::default().show(ctx, |ui| {
        ui.add_space(48.0);
        ui.vertical_centered(|ui| {
            ui.heading("Design");
            ui.label("Paint the endplate patches to loft the disc — coming next rung.");
            ui.add_space(8.0);
            ui.label("S  simulate    ·    Esc  quit");
        });
    });
    Ok(())
}
