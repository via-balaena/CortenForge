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
use crate::jobs::{PrintJob, SimplifyJob, poll_dialogs, poll_print_job, poll_simplify_job};
use crate::panel::wizard_screen;
use crate::scan::ScanEdit;
use crate::scene::{draw_centerline, fit_viewport_to_free_space, setup_scene, show_scan};
use crate::shape::ShapeControls;
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
            .init_resource::<SimplifyJob>()
            .init_resource::<ScanEdit>()
            .init_resource::<EditControls>()
            .init_resource::<ShapeControls>()
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
                    poll_simplify_job,
                    // After both writers, so a landing scan or a landed
                    // Simplify reaches the viewport on its own frame rather
                    // than the next one.
                    show_scan
                        .after(poll_dialogs)
                        .after(poll_simplify_job)
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
    ctx.set_fonts(font_definitions());

    *done = true;
    Ok(())
}

/// egui's defaults, with `Hack` reachable from proportional text.
///
/// Split out from the system so the one decision in it can be tested: reaching
/// it through [`pin_theme_and_fonts`] would need a live egui context, and the
/// arrows breaking again is not something to find out by looking.
pub(crate) fn font_definitions() -> egui::FontDefinitions {
    let mut fonts = egui::FontDefinitions::default();
    if let Some(proportional) = fonts.families.get_mut(&egui::FontFamily::Proportional) {
        proportional.push("Hack".to_owned());
    }
    fonts
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
    /// The system itself, run for real — no window, no `EguiPlugin`, no render
    /// stack. `EguiContexts` needs only a component query plus
    /// `EguiUserTextures`, and `PrimaryEguiContext` pulls in `EguiContext`, so a
    /// bare `App` is enough to prove the theme actually reaches the context.
    #[test]
    fn the_theme_reaches_the_context() {
        use bevy_egui::{EguiContext, EguiUserTextures, PrimaryEguiContext};

        let mut app = App::new();
        app.init_resource::<EguiUserTextures>()
            .add_systems(Update, pin_theme_and_fonts);
        app.world_mut().spawn(PrimaryEguiContext);
        app.update();

        let mut q = app.world_mut().query::<&mut EguiContext>();
        let dark = q
            .iter_mut(app.world_mut())
            .next()
            .map(|mut c| c.get_mut().style().visuals.dark_mode);

        assert_eq!(
            dark,
            Some(false),
            "this app is designed light; egui defaults dark"
        );
    }

    /// ⚠ Two separate failures, one assertion each.
    ///
    /// Losing the `Hack` push tofus every arrow in the UI. Losing the bundled
    /// faces is worse and quieter to cause: `set_fonts` REPLACES the font set,
    /// so if `default_fonts` were ever dropped from the `bevy_egui` features,
    /// `FontDefinitions::default()` would hand over an empty set and the app
    /// would render no text at all.
    #[test]
    fn proportional_text_can_reach_the_arrow_glyphs() {
        let fonts = font_definitions();
        let proportional = fonts.families.get(&egui::FontFamily::Proportional);

        assert!(proportional.is_some(), "the proportional family must exist");
        if let Some(family) = proportional {
            assert!(
                family.iter().any(|f| f == "Hack"),
                "← → ↺ live only in Hack; without it they render as tofu: {family:?}"
            );
            assert!(
                family.len() > 1,
                "Hack is a fallback, not a replacement for the bundled faces: {family:?}"
            );
        }
    }

    /// The centerline runs *inside* the scan, so it is visible only because the
    /// gizmo group ignores depth. At the default bias of 0 the surface it
    /// describes hides it, and the overlay silently stops existing — no error,
    /// no failing test, just an empty viewport where the whole point of this
    /// screen used to be.
    ///
    /// ⚠ Asserts the sign, not `-1.0`. "In front" is the invariant; the exact
    /// magnitude is free to change.
    #[test]
    fn the_centerline_gizmo_is_configured_to_draw_in_front() {
        let mut app = App::new();
        app.add_plugins((MinimalPlugins, StatesPlugin, StudioPlugin));

        let (config, _) = app
            .world()
            .resource::<GizmoConfigStore>()
            .config::<DefaultGizmoConfigGroup>();

        assert!(
            config.depth_bias < 0.0,
            "the overlay must draw in front of the scan, not inside it: depth_bias={}",
            config.depth_bias
        );
    }

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
