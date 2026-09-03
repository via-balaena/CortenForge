//! `cf-studio-gui` (Cendrillon) — the Bevy + `bevy_egui` app shell.
//!
//! Wiring only. Every system lives in [`plugin::StudioPlugin`]; the decisions
//! those systems make are plain functions in the lib, which is where the tests
//! are.

mod dialogs;
mod input;
mod jobs;
mod panel;
mod plugin;
mod scan;
mod scene;
mod state;
mod waiver;
mod widgets;

use bevy::prelude::*;
use bevy::render::pipelined_rendering::PipelinedRenderingPlugin;
use bevy::window::WindowResizeConstraints;
use bevy_egui::EguiPlugin;

use crate::panel::PANEL_WIDTH;

/// The 3D view the window opens with, beside the panels.
const VIEWPORT_WIDTH: f32 = 600.0;
/// The narrowest 3D view still worth showing — the resize floor.
const MIN_VIEWPORT_WIDTH: f32 = 220.0;

fn main() {
    // `WindowResolution` is whole pixels; the panel widths are egui's points.
    #[allow(clippy::cast_possible_truncation, clippy::cast_sign_loss)]
    let default_width = (PANEL_WIDTH + VIEWPORT_WIDTH) as u32;

    let plugins = DefaultPlugins.set(WindowPlugin {
        primary_window: Some(Window {
            title: "Cendrillon".into(),
            resolution: (default_width, 900u32).into(),
            // Width is measured from the panels outwards: they cover the same
            // strip whatever the window's width, and everything left over is
            // the 3D view. The pre-port floor of 640 predates the viewport and
            // was narrower than the two panels it sat beside. Height is
            // unchanged — enough that the footer nav stays on screen if the OS
            // restores a shorter window.
            resize_constraints: WindowResizeConstraints {
                min_width: PANEL_WIDTH + MIN_VIEWPORT_WIDTH,
                min_height: 850.0,
                ..default()
            },
            ..default()
        }),
        ..default()
    });

    App::new()
        // ⚠⚠ `PipelinedRenderingPlugin` is DISABLED on purpose, and this is the
        // one line that keeps the app from hanging on quit.
        //
        // Measured on macOS with bevy 0.18.1 / wgpu 27.0.1: **5 of 16 quits
        // deadlocked** with it on, **0 of 16** with it off. The sampled stack
        // shows the main thread parked forever in
        // `Drop for RenderAppChannels`, waiting for a render world that a
        // now-exited render thread will never send back.
        //
        // ⚠ It is NOT about empty render worlds — it reproduces with a mesh on
        // screen. Do not "fix" this by constraining what the scene may despawn.
        //
        // The cost is that render and main no longer overlap. For a wizard UI
        // that is free; for a simulation viewer it would not be, so do not copy
        // this line into one without measuring first.
        .add_plugins(plugins.build().disable::<PipelinedRenderingPlugin>())
        .add_plugins(EguiPlugin::default())
        .add_plugins(plugin::StudioPlugin)
        .run();
}
