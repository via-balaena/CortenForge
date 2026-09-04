//! `StudioPlugin` — every system this app owns, wired in one place.
//!
//! ★ The wiring is a plugin rather than inline `App` calls so a test can build
//! a headless `App`, add this, and drive the state machine without a window.
//! `cf-spine-studio` wires inline in its `run_app` and is untestable for
//! exactly that reason; this is the one structural choice that avoids it.

use bevy::prelude::*;
use bevy_egui::{EguiContexts, EguiGlobalSettings, EguiPrimaryContextPass, egui};
use cf_bevy_common::camera::OrbitCameraPlugin;

use crate::dialogs::PendingDialog;
use crate::edit::EditControls;
use crate::input::arbitrate_pointer_over_egui;
use crate::jobs::{PrintJob, poll_dialogs, poll_print_job};
use crate::panel::wizard_screen;
use crate::scan::ScanEdit;
use crate::scene::{draw_centerline, fit_viewport_to_free_space, setup_scene, show_scan};
use crate::state::{Screen, Studio};
use crate::waiver::waiver_screen;

/// The window background the app was designed against.
const BACKGROUND: Color = Color::srgb(0.863, 0.878, 0.898);

pub(crate) struct StudioPlugin;

impl Plugin for StudioPlugin {
    fn build(&self, app: &mut App) {
        // ⚠ Off, because the default is "attach the primary context to the
        // first camera created" — which would be the 3D camera, the one whose
        // viewport `fit_viewport_to_free_space` shrinks. `scene.rs` spawns a
        // dedicated UI camera carrying `PrimaryEguiContext` instead; the comment
        // there has the measurement.
        app.world_mut()
            .get_resource_or_init::<EguiGlobalSettings>()
            .auto_create_primary_context = false;

        app.init_state::<Screen>()
            .init_resource::<Studio>()
            .init_resource::<PendingDialog>()
            .init_resource::<PrintJob>()
            .init_resource::<ScanEdit>()
            .init_resource::<EditControls>()
            .insert_resource(ClearColor(BACKGROUND))
            // The centerline runs *inside* the scan, so at the default
            // `depth_bias` of 0 it is hidden by the surface it describes and the
            // overlay is invisible exactly when it matters. −1 draws it in
            // front, matching the pre-port viewer's `depth_compare: Always`.
            .insert_gizmo_config(
                DefaultGizmoConfigGroup,
                GizmoConfig {
                    depth_bias: -1.0,
                    ..default()
                },
            )
            .add_plugins(OrbitCameraPlugin)
            .add_systems(Startup, setup_scene)
            .add_systems(
                EguiPrimaryContextPass,
                (
                    pin_theme_and_fonts,
                    waiver_screen.run_if(in_state(Screen::Waiver)),
                    wizard_screen.run_if(in_state(Screen::Wizard)),
                    // Last: it measures what the panels above left uncovered.
                    fit_viewport_to_free_space,
                )
                    .chain(),
            )
            .add_systems(
                Update,
                (
                    arbitrate_pointer_over_egui,
                    poll_dialogs,
                    poll_print_job,
                    show_scan
                        .after(poll_dialogs)
                        .run_if(resource_changed::<ScanEdit>),
                    // Immediate mode: gizmos are re-emitted every frame, so
                    // this one is NOT gated on the resource changing.
                    draw_centerline,
                ),
            );
    }
}

/// Pin the light theme and the font fallback once, on the first frame that has a
/// context.
///
/// ⚠ The theme is load-bearing, not cosmetic. This app is designed light — light
/// body panel, dark text — and egui defaults to **dark**. The pre-port UI forced
/// light for the same reason: under macOS dark mode its off-panel text went
/// dark-on-dark and vanished.
///
/// ⚠⚠ So is the font fallback. egui puts `Hack` in the **monospace** family only,
/// and `Ubuntu-Light` — the proportional face — has no `←`, `→` or `↺`. Every
/// arrow in this UI (`← Back`, `Next →`, `← you are here`, the weld report's
/// `before → after`, `↺ Reset`) is proportional text, so without this they all
/// render as tofu boxes. Measured against the bundled `.ttf`s: those codepoints
/// exist in `Hack` and nowhere else in the default set.
fn pin_theme_and_fonts(
    mut contexts: EguiContexts,
    mut done: Local<bool>,
) -> bevy::ecs::error::Result {
    if *done {
        return Ok(());
    }
    let ctx = contexts.ctx_mut()?;
    ctx.set_visuals(egui::Visuals::light());

    let mut fonts = egui::FontDefinitions::default();
    if let Some(proportional) = fonts.families.get_mut(&egui::FontFamily::Proportional) {
        proportional.push("Hack".to_owned());
    }
    ctx.set_fonts(fonts);

    *done = true;
    Ok(())
}

#[cfg(test)]
mod tests {
    use bevy::state::app::StatesPlugin;

    use super::*;

    /// The other half of the viewport fix, and the half `scene.rs` cannot see:
    /// `setup_scene` spawning a dedicated context camera only helps while
    /// `bevy_egui` is told to stop attaching one of its own. Left on, it would
    /// mark the 3D camera too — two primary contexts, and the feedback loop back
    /// through whichever one egui picked.
    #[test]
    fn bevy_egui_is_told_not_to_claim_the_first_camera() {
        let mut app = App::new();
        app.add_plugins((MinimalPlugins, StatesPlugin, StudioPlugin));

        assert!(
            !app.world()
                .resource::<EguiGlobalSettings>()
                .auto_create_primary_context,
            "the context belongs to scene.rs's UI camera, not to whichever camera spawns first"
        );
    }
}
