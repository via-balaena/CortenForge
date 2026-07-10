//! Input arbitration: suppress orbit-camera motion while the pointer is over the
//! egui panel, and quit on `Esc`.

use bevy::prelude::*;
use bevy_egui::EguiContexts;

/// Suppress orbit-camera input while the pointer is over the egui panel.
pub(crate) fn block_orbit_input_when_over_egui(
    mut contexts: EguiContexts,
    mut motion: ResMut<bevy::input::mouse::AccumulatedMouseMotion>,
    mut scroll: ResMut<bevy::input::mouse::AccumulatedMouseScroll>,
) -> bevy::ecs::error::Result {
    let ctx = contexts.ctx_mut()?;
    if ctx.wants_pointer_input() || ctx.is_pointer_over_area() {
        motion.delta = Vec2::ZERO;
        scroll.delta = Vec2::ZERO;
    }
    Ok(())
}

/// Esc quits the window (mirrors the other `tools/` viewers).
#[allow(clippy::needless_pass_by_value)] // Bevy systems take resources by value.
pub(crate) fn exit_on_esc(keys: Res<ButtonInput<KeyCode>>, mut exit: MessageWriter<AppExit>) {
    if keys.just_pressed(KeyCode::Escape) {
        exit.write(AppExit::Success);
    }
}
