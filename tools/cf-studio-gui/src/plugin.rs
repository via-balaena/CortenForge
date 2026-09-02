//! `StudioPlugin` — every system this app owns, wired in one place.
//!
//! ★ The wiring is a plugin rather than inline `App` calls so a test can build
//! a headless `App`, add this, and drive the state machine without a window.
//! `cf-spine-studio` wires inline in its `run_app` and is untestable for
//! exactly that reason; this is the one structural choice that avoids it.

use bevy::prelude::*;
use bevy_egui::{EguiContexts, EguiPrimaryContextPass, egui};
use cf_bevy_common::camera::OrbitCameraPlugin;

use crate::dialogs::PendingDialog;
use crate::input::arbitrate_pointer_over_egui;
use crate::jobs::{PrintJob, poll_dialogs, poll_print_job};
use crate::panel::wizard_screen;
use crate::scene::setup_scene;
use crate::state::{Screen, Studio};
use crate::waiver::waiver_screen;

/// The window background the app was designed against.
const BACKGROUND: Color = Color::srgb(0.863, 0.878, 0.898);

pub(crate) struct StudioPlugin;

impl Plugin for StudioPlugin {
    fn build(&self, app: &mut App) {
        app.init_state::<Screen>()
            .init_resource::<Studio>()
            .init_resource::<PendingDialog>()
            .init_resource::<PrintJob>()
            .insert_resource(ClearColor(BACKGROUND))
            .add_plugins(OrbitCameraPlugin)
            .add_systems(Startup, setup_scene)
            .add_systems(
                EguiPrimaryContextPass,
                (
                    force_light_theme,
                    waiver_screen.run_if(in_state(Screen::Waiver)),
                    wizard_screen.run_if(in_state(Screen::Wizard)),
                )
                    .chain(),
            )
            .add_systems(
                Update,
                (arbitrate_pointer_over_egui, poll_dialogs, poll_print_job),
            );
    }
}

/// Pin the light theme once, on the first frame that has a context.
///
/// ⚠ Load-bearing, not cosmetic. This app is designed light — light body panel,
/// dark text — and egui defaults to **dark**. The pre-port UI forced light for
/// the same reason: under macOS dark mode its off-panel text went dark-on-dark
/// and vanished.
fn force_light_theme(
    mut contexts: EguiContexts,
    mut done: Local<bool>,
) -> bevy::ecs::error::Result {
    if *done {
        return Ok(());
    }
    contexts.ctx_mut()?.set_visuals(egui::Visuals::light());
    *done = true;
    Ok(())
}
